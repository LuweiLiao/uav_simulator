#include <arpa/inet.h>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/PointStamped.h>
#include <ignition/math/Pose3.hh>
#include <mavlink/v2.0/common/mavlink.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

namespace gazebo {

class PoleMavlinkPosePlugin : public ModelPlugin {
public:
  PoleMavlinkPosePlugin() = default;

  ~PoleMavlinkPosePlugin() override
  {
    if (socket_fd_ >= 0) {
      close(socket_fd_);
    }
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

    const std::string link_name =
        sdf->HasElement("linkName") ? sdf->Get<std::string>("linkName") : "pole";
    link_ = model_->GetLink(link_name);
    if (!link_) {
      gzerr << "[PoleMavlinkPosePlugin] Link [" << link_name
            << "] not found in model [" << model_->GetName() << "]\n";
      return;
    }

    tcp_addr_ = sdf->HasElement("tcpAddr") ? sdf->Get<std::string>("tcpAddr") : tcp_addr_;
    tcp_port_ = sdf->HasElement("tcpPort") ? sdf->Get<int>("tcpPort") : tcp_port_;
    send_rate_hz_ = sdf->HasElement("sendRateHz") ? sdf->Get<double>("sendRateHz") : send_rate_hz_;
    ros_target_topic_ = sdf->HasElement("rosTargetTopic")
                            ? sdf->Get<std::string>("rosTargetTopic")
                            : ros_target_topic_;
    ros_frame_id_ = sdf->HasElement("rosFrameId")
                        ? sdf->Get<std::string>("rosFrameId")
                        : ros_frame_id_;
    system_id_ = static_cast<uint8_t>(
        sdf->HasElement("systemId") ? sdf->Get<int>("systemId") : system_id_);
    component_id_ = static_cast<uint8_t>(
        sdf->HasElement("componentId") ? sdf->Get<int>("componentId") : component_id_);

    if (tcp_port_ <= 0) {
      gzerr << "[PoleMavlinkPosePlugin] Missing or invalid <tcpPort> in SDF\n";
      return;
    }

    OpenSocket();
    InitRosSubscriber();

    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&PoleMavlinkPosePlugin::OnUpdate, this, std::placeholders::_1));

    gzmsg << "[PoleMavlinkPosePlugin] Sending " << model_->GetName()
          << "::" << link_name << " pose as MAVLink ODOMETRY to "
          << tcp_addr_ << ":" << tcp_port_ << " via TCP at " << send_rate_hz_ << " Hz\n";
    gzmsg << "[PoleMavlinkPosePlugin] Listening for ROS pole position commands on ["
          << ros_target_topic_ << "] frame [" << ros_frame_id_
          << "]. Commands are ArduPilot local NED; ODOMETRY uses real pole pose converted to NED.\n";
  }

private:
  void InitRosSubscriber()
  {
    if (!ros::isInitialized()) {
      int argc = 0;
      char** argv = nullptr;
      ros::init(argc, argv, "pole_mavlink_pose_plugin",
                ros::init_options::NoSigintHandler);
    }

    ros_node_.reset(new ros::NodeHandle(""));
    ros_node_->setCallbackQueue(&ros_queue_);
    target_position_sub_ = ros_node_->subscribe(
        ros_target_topic_, 1, &PoleMavlinkPosePlugin::TargetPositionCallback, this);
    ros_queue_thread_ = std::thread([this]() {
      while (ros_node_ && ros_node_->ok()) {
        ros_queue_.callAvailable(ros::WallDuration(0.01));
      }
    });
  }

  void TargetPositionCallback(const geometry_msgs::PointStampedConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    pending_position_ned_.X() = msg->point.x;
    pending_position_ned_.Y() = msg->point.y;
    pending_position_ned_.Z() = msg->point.z;
    has_pending_position_ = true;
    gzmsg << "[PoleMavlinkPosePlugin] Received ROS pole position command NED: "
          << pending_position_ned_ << "\n";
  }

  bool OpenSocket()
  {
    if (socket_fd_ >= 0) {
      return true;
    }

    socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd_ < 0) {
      gzerr << "[PoleMavlinkPosePlugin] Failed to create TCP socket\n";
      return false;
    }

    std::memset(&remote_addr_, 0, sizeof(remote_addr_));
    remote_addr_.sin_family = AF_INET;
    remote_addr_.sin_port = htons(static_cast<uint16_t>(tcp_port_));
    if (inet_aton(tcp_addr_.c_str(), &remote_addr_.sin_addr) == 0) {
      gzerr << "[PoleMavlinkPosePlugin] Invalid TCP address [" << tcp_addr_ << "]\n";
      close(socket_fd_);
      socket_fd_ = -1;
      return false;
    }

    if (connect(socket_fd_, reinterpret_cast<sockaddr*>(&remote_addr_), sizeof(remote_addr_)) != 0) {
      gzerr << "[PoleMavlinkPosePlugin] TCP connect failed to "
            << tcp_addr_ << ":" << tcp_port_ << ", will retry\n";
      close(socket_fd_);
      socket_fd_ = -1;
      return false;
    }

    gzmsg << "[PoleMavlinkPosePlugin] TCP connected to "
          << tcp_addr_ << ":" << tcp_port_ << "\n";
    return true;
  }

  void OnUpdate(const common::UpdateInfo& info)
  {
    if (!link_ || send_rate_hz_ <= 0.0) {
      return;
    }

    const double now = info.simTime.Double();
    ApplyPendingPositionCommand();

    if (socket_fd_ < 0) {
      if (last_connect_attempt_time_ < 0.0 || now - last_connect_attempt_time_ > 1.0) {
        last_connect_attempt_time_ = now;
        OpenSocket();
      }
      if (socket_fd_ < 0) {
        return;
      }
    }

    if (last_send_time_ >= 0.0 && now - last_send_time_ < 1.0 / send_rate_hz_) {
      return;
    }
    last_send_time_ = now;

    const ignition::math::Vector3d pos = link_->WorldPose().Pos();
    const ignition::math::Vector3d vel = link_->WorldLinearVel();
    SendOdometry(static_cast<uint64_t>(now * 1.0e6), WorldToNed(pos), WorldToNed(vel));
  }

  static ignition::math::Vector3d WorldToNed(const ignition::math::Vector3d& world)
  {
    // Match the ArduPilot Gazebo plugins: Gazebo world xyz is N, -E, -D.
    return ignition::math::Vector3d(world.X(), -world.Y(), -world.Z());
  }

  static ignition::math::Vector3d NedToWorld(const ignition::math::Vector3d& ned)
  {
    return ignition::math::Vector3d(ned.X(), -ned.Y(), -ned.Z());
  }

  void ApplyPendingPositionCommand()
  {
    ignition::math::Vector3d target_ned;
    {
      std::lock_guard<std::mutex> lock(target_mutex_);
      if (!has_pending_position_) {
        return;
      }
      target_ned = pending_position_ned_;
      has_pending_position_ = false;
    }

    const ignition::math::Vector3d target_enu = NedToWorld(target_ned);
    ignition::math::Pose3d pose = model_->WorldPose();
    pose.Pos() = target_enu;

    model_->SetWorldPose(pose);
    model_->SetLinearVel(ignition::math::Vector3d::Zero);
    model_->SetAngularVel(ignition::math::Vector3d::Zero);
  }

  void SendOdometry(uint64_t time_usec,
                    const ignition::math::Vector3d& pos_ned,
                    const ignition::math::Vector3d& vel_ned)
  {
    mavlink_message_t msg;
    mavlink_odometry_t odom {};

    odom.time_usec = time_usec;
    odom.frame_id = MAV_FRAME_LOCAL_NED;
    odom.child_frame_id = MAV_FRAME_LOCAL_NED;

    odom.x = static_cast<float>(pos_ned.X());
    odom.y = static_cast<float>(pos_ned.Y());
    odom.z = static_cast<float>(pos_ned.Z());
    odom.vx = static_cast<float>(vel_ned.X());
    odom.vy = static_cast<float>(vel_ned.Y());
    odom.vz = static_cast<float>(vel_ned.Z());

    odom.q[0] = 1.0f;
    odom.q[1] = 0.0f;
    odom.q[2] = 0.0f;
    odom.q[3] = 0.0f;
    odom.rollspeed = 0.0f;
    odom.pitchspeed = 0.0f;
    odom.yawspeed = 0.0f;
    odom.pose_covariance[0] = std::numeric_limits<float>::quiet_NaN();
    odom.velocity_covariance[0] = std::numeric_limits<float>::quiet_NaN();
    odom.estimator_type = MAV_ESTIMATOR_TYPE_NAIVE;
    odom.quality = 100;

    mavlink_msg_odometry_encode(system_id_, component_id_, &msg, &odom);

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    const ssize_t sent = send(socket_fd_, buffer, len, MSG_NOSIGNAL);
    if (sent != static_cast<ssize_t>(len)) {
      gzerr << "[PoleMavlinkPosePlugin] TCP send failed, closing socket and retrying later\n";
      close(socket_fd_);
      socket_fd_ = -1;
    }
  }

  physics::ModelPtr model_;
  physics::LinkPtr link_;
  event::ConnectionPtr update_connection_;

  std::string tcp_addr_ = "127.0.0.1";
  int tcp_port_ = -1;
  double send_rate_hz_ = 50.0;
  double last_send_time_ = -1.0;
  double last_connect_attempt_time_ = -1.0;
  std::string ros_target_topic_ = "/pendulum_pole/set_position_ned";
  std::string ros_frame_id_ = "local_ned";
  uint8_t system_id_ = 42;
  uint8_t component_id_ = MAV_COMP_ID_ONBOARD_COMPUTER;

  int socket_fd_ = -1;
  sockaddr_in remote_addr_ {};
  std::unique_ptr<ros::NodeHandle> ros_node_;
  ros::Subscriber target_position_sub_;
  ros::CallbackQueue ros_queue_;
  std::thread ros_queue_thread_;
  std::mutex target_mutex_;
  bool has_pending_position_ = false;
  ignition::math::Vector3d pending_position_ned_{0, 0, 0};
};

GZ_REGISTER_MODEL_PLUGIN(PoleMavlinkPosePlugin)

}  // namespace gazebo
