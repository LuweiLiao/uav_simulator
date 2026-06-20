#include <algorithm>
#include <atomic>
#include <thread>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/PointStamped.h>
#include <ignition/math/Pose3.hh>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

namespace gazebo {

class PoleHoldReleasePlugin : public ModelPlugin {
public:
  PoleHoldReleasePlugin() = default;

  ~PoleHoldReleasePlugin() override
  {
    if (ros_node_) {
      ros_queue_.clear();
      ros_queue_.disable();
      ros_node_->shutdown();
    }
    if (ros_queue_thread_.joinable()) {
      ros_queue_thread_.join();
    }
  }

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    world_ = model_->GetWorld();

    const std::string link_name =
        sdf->HasElement("linkName") ? sdf->Get<std::string>("linkName") : "pole";
    link_ = model_->GetLink(link_name);
    if (!link_) {
      gzerr << "[PoleHoldReleasePlugin] Link [" << link_name
            << "] not found in model [" << model_->GetName() << "]\n";
      return;
    }

    unlock_topic_ = sdf->HasElement("unlockTopic")
                        ? sdf->Get<std::string>("unlockTopic")
                        : "/pendulum_pole/unlock";
    set_position_topic_ = sdf->HasElement("setPositionTopic")
                              ? sdf->Get<std::string>("setPositionTopic")
                              : "/pendulum_pole/set_position_ned";
    pos_p_ = sdf->HasElement("posP") ? sdf->Get<double>("posP") : pos_p_;
    pos_d_ = sdf->HasElement("posD") ? sdf->Get<double>("posD") : pos_d_;
    rot_p_ = sdf->HasElement("rotP") ? sdf->Get<double>("rotP") : rot_p_;
    rot_d_ = sdf->HasElement("rotD") ? sdf->Get<double>("rotD") : rot_d_;
    max_force_ = sdf->HasElement("maxForce") ? sdf->Get<double>("maxForce") : max_force_;
    max_torque_ = sdf->HasElement("maxTorque") ? sdf->Get<double>("maxTorque") : max_torque_;

    hold_pose_ = link_->WorldPose();
    mass_ = link_->GetInertial()->Mass();

    if (!ros::isInitialized()) {
      int argc = 0;
      char** argv = nullptr;
      ros::init(argc, argv, "pole_hold_release_plugin",
                ros::init_options::NoSigintHandler);
    }

    ros_node_.reset(new ros::NodeHandle(""));
    ros::SubscribeOptions unlock_so =
        ros::SubscribeOptions::create<std_msgs::Bool>(
            unlock_topic_, 1,
            boost::bind(&PoleHoldReleasePlugin::UnlockCallback, this, _1),
            ros::VoidPtr(), &ros_queue_);
    unlock_sub_ = ros_node_->subscribe(unlock_so);
    ros::SubscribeOptions set_position_so =
        ros::SubscribeOptions::create<geometry_msgs::PointStamped>(
            set_position_topic_, 1,
            boost::bind(&PoleHoldReleasePlugin::SetPositionCallback, this, _1),
            ros::VoidPtr(), &ros_queue_);
    set_position_sub_ = ros_node_->subscribe(set_position_so);
    ros_queue_thread_ = std::thread([this]() {
      while (ros_node_ && ros_node_->ok()) {
        ros_queue_.callAvailable(ros::WallDuration(0.01));
      }
    });

    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&PoleHoldReleasePlugin::OnUpdate, this));

    gzmsg << "[PoleHoldReleasePlugin] Holding [" << model_->GetName()
          << "::" << link_name << "] until " << unlock_topic_
          << " receives true\n";
    gzmsg << "[PoleHoldReleasePlugin] Updating hold position from ROS topic ["
          << set_position_topic_ << "]\n";
  }

  void Reset() override
  {
    if (!link_) {
      return;
    }

    hold_pose_ = link_->WorldPose();
    LockPole();
  }

private:
  void LockPole()
  {
    unlocked_.store(false);
    if (link_) {
      link_->SetLinearVel(ignition::math::Vector3d::Zero);
      link_->SetAngularVel(ignition::math::Vector3d::Zero);
    }
  }

  void UnlockCallback(const std_msgs::BoolConstPtr& msg)
  {
    unlocked_.store(msg->data);
    if (msg->data && link_) {
      link_->SetLinearVel(ignition::math::Vector3d::Zero);
      link_->SetAngularVel(ignition::math::Vector3d::Zero);
    }
  }

  static ignition::math::Vector3d NedToWorld(const ignition::math::Vector3d& ned)
  {
    // Match the ArduPilot Gazebo plugins: Gazebo world xyz is N, -E, -D.
    return ignition::math::Vector3d(ned.X(), -ned.Y(), -ned.Z());
  }

  void SetPositionCallback(const geometry_msgs::PointStampedConstPtr& msg)
  {
    const ignition::math::Vector3d target_ned(msg->point.x, msg->point.y, msg->point.z);
    const ignition::math::Vector3d target_enu = NedToWorld(target_ned);

    hold_pose_.Pos() = target_enu;
    LockPole();
    if (model_) {
      model_->SetWorldPose(hold_pose_);
      model_->SetLinearVel(ignition::math::Vector3d::Zero);
      model_->SetAngularVel(ignition::math::Vector3d::Zero);
    }

    gzmsg << "[PoleHoldReleasePlugin] Hold position updated from NED "
          << target_ned << " to Gazebo world " << target_enu << "\n";
  }

  void OnUpdate()
  {
    if (!link_ || unlocked_.load()) {
      return;
    }

    const ignition::math::Pose3d pose = link_->WorldPose();
    const ignition::math::Vector3d pos_error = hold_pose_.Pos() - pose.Pos();
    const ignition::math::Vector3d vel = link_->WorldLinearVel();
    ignition::math::Vector3d force = pos_p_ * pos_error - pos_d_ * vel;

    // Cancel gravity while locked, then let the PD term remove drift.
    force += -mass_ * world_->Gravity();
    link_->AddForce(LimitVector(force, max_force_));

    const ignition::math::Vector3d hold_rpy = hold_pose_.Rot().Euler();
    const ignition::math::Vector3d rpy = pose.Rot().Euler();
    ignition::math::Vector3d rot_error(
        NormalizeAngle(hold_rpy.X() - rpy.X()),
        NormalizeAngle(hold_rpy.Y() - rpy.Y()),
        NormalizeAngle(hold_rpy.Z() - rpy.Z()));
    ignition::math::Vector3d torque =
        rot_p_ * rot_error - rot_d_ * link_->WorldAngularVel();
    link_->AddTorque(LimitVector(torque, max_torque_));
  }

  static double NormalizeAngle(double angle)
  {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  static ignition::math::Vector3d LimitVector(
      const ignition::math::Vector3d& vector, double limit)
  {
    if (limit <= 0.0 || vector.Length() <= limit) {
      return vector;
    }
    return vector.Normalized() * limit;
  }

  physics::ModelPtr model_;
  physics::WorldPtr world_;
  physics::LinkPtr link_;
  event::ConnectionPtr update_connection_;

  ignition::math::Pose3d hold_pose_;
  double mass_ = 0.0;
  double pos_p_ = 80.0;
  double pos_d_ = 18.0;
  double rot_p_ = 2.0;
  double rot_d_ = 0.4;
  double max_force_ = 200.0;
  double max_torque_ = 20.0;

  std::string unlock_topic_;
  std::string set_position_topic_;
  std::atomic<bool> unlocked_{false};

  std::unique_ptr<ros::NodeHandle> ros_node_;
  ros::Subscriber unlock_sub_;
  ros::Subscriber set_position_sub_;
  ros::CallbackQueue ros_queue_;
  std::thread ros_queue_thread_;
};

GZ_REGISTER_MODEL_PLUGIN(PoleHoldReleasePlugin)

}  // namespace gazebo
