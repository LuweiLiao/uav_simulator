#include <cerrno>
#include <arpa/inet.h>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <fcntl.h>
#include <memory>
#include <netinet/tcp.h>
#include <string>
#include <sys/socket.h>
#include <unistd.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ignition/math/Pose3.hh>
#include <mavlink/v2.0/common/mavlink.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

namespace gazebo {

class SawMavlinkPosePlugin : public ModelPlugin {
public:
  SawMavlinkPosePlugin() = default;

  ~SawMavlinkPosePlugin() override
  {
    if (socket_fd_ >= 0) {
      close(socket_fd_);
    }
    if (ros_node_) {
      ros_node_->shutdown();
    }
  }

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;

    const std::string link_name =
        sdf->HasElement("linkName") ? sdf->Get<std::string>("linkName")
                                    : "tsduav_t4_saw::saw";
    link_ = FindLink(link_name);
    if (!link_) {
      gzerr << "[SawMavlinkPosePlugin] Link [" << link_name
            << "] not found in model [" << model_->GetName() << "]\n";
      return;
    }

    const std::string joint_name =
        sdf->HasElement("jointName") ? sdf->Get<std::string>("jointName")
                                     : "link_base_joint";
    joint_ = model_->GetJoint(joint_name);

    tcp_addr_ = sdf->HasElement("tcpAddr") ? sdf->Get<std::string>("tcpAddr") : tcp_addr_;
    tcp_port_ = sdf->HasElement("tcpPort") ? sdf->Get<int>("tcpPort") : tcp_port_;
    send_rate_hz_ = sdf->HasElement("sendRateHz") ? sdf->Get<double>("sendRateHz") : send_rate_hz_;
    home_lat_deg_ = sdf->HasElement("homeLat") ? sdf->Get<double>("homeLat") : home_lat_deg_;
    home_lon_deg_ = sdf->HasElement("homeLon") ? sdf->Get<double>("homeLon") : home_lon_deg_;
    home_alt_m_ = sdf->HasElement("homeAlt") ? sdf->Get<double>("homeAlt") : home_alt_m_;
    pose_topic_ = sdf->HasElement("rosPoseTopic") ? sdf->Get<std::string>("rosPoseTopic")
                                                  : pose_topic_;
    vel_topic_ = sdf->HasElement("rosVelTopic") ? sdf->Get<std::string>("rosVelTopic")
                                                : vel_topic_;
    joint_topic_ = sdf->HasElement("rosJointTopic") ? sdf->Get<std::string>("rosJointTopic")
                                                    : joint_topic_;
    system_id_ = static_cast<uint8_t>(
        sdf->HasElement("systemId") ? sdf->Get<int>("systemId") : system_id_);
    component_id_ = static_cast<uint8_t>(
        sdf->HasElement("componentId") ? sdf->Get<int>("componentId") : component_id_);

    if (tcp_port_ <= 0) {
      gzerr << "[SawMavlinkPosePlugin] Missing or invalid <tcpPort> in SDF\n";
      return;
    }

    InitRos();
    OpenSocket();

    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&SawMavlinkPosePlugin::OnUpdate, this, std::placeholders::_1));

    gzmsg << "[SawMavlinkPosePlugin] Sending " << model_->GetName()
          << "::" << link_->GetName() << " as GLOBAL_POSITION_INT to "
          << tcp_addr_ << ":" << tcp_port_ << " via TCP at " << send_rate_hz_
          << " Hz sysid=" << static_cast<int>(system_id_) << "\n";
    gzmsg << "[SawMavlinkPosePlugin] Home "
          << home_lat_deg_ << "," << home_lon_deg_ << "," << home_alt_m_ << "\n";
  }

private:
  physics::LinkPtr FindLink(const std::string& name)
  {
    physics::LinkPtr link = model_->GetLink(name);
    if (link) {
      return link;
    }

    const auto links = model_->GetLinks();
    for (const auto& candidate : links) {
      if (candidate->GetName() == name ||
          candidate->GetScopedName().rfind(name) != std::string::npos) {
        return candidate;
      }
    }

    for (const auto& nested : model_->NestedModels()) {
      link = nested->GetLink(name);
      if (link) {
        return link;
      }
    }
    return physics::LinkPtr();
  }

  void InitRos()
  {
    if (!ros::isInitialized()) {
      int argc = 0;
      char** argv = nullptr;
      ros::init(argc, argv, "saw_mavlink_pose_plugin",
                ros::init_options::NoSigintHandler);
    }

    ros_node_.reset(new ros::NodeHandle(""));
    pose_pub_ = ros_node_->advertise<geometry_msgs::PoseStamped>(pose_topic_, 10);
    vel_pub_ = ros_node_->advertise<geometry_msgs::TwistStamped>(vel_topic_, 10);
    joint_pub_ = ros_node_->advertise<sensor_msgs::JointState>(joint_topic_, 10);
  }

  bool OpenSocket()
  {
    if (socket_fd_ >= 0) {
      return true;
    }

    socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd_ < 0) {
      gzerr << "[SawMavlinkPosePlugin] Failed to create TCP socket\n";
      return false;
    }

    std::memset(&remote_addr_, 0, sizeof(remote_addr_));
    remote_addr_.sin_family = AF_INET;
    remote_addr_.sin_port = htons(static_cast<uint16_t>(tcp_port_));
    if (inet_aton(tcp_addr_.c_str(), &remote_addr_.sin_addr) == 0) {
      gzerr << "[SawMavlinkPosePlugin] Invalid TCP address [" << tcp_addr_ << "]\n";
      close(socket_fd_);
      socket_fd_ = -1;
      return false;
    }

    if (connect(socket_fd_, reinterpret_cast<sockaddr*>(&remote_addr_),
                sizeof(remote_addr_)) != 0) {
      gzerr << "[SawMavlinkPosePlugin] TCP connect failed to " << tcp_addr_
            << ":" << tcp_port_ << ", will retry\n";
      close(socket_fd_);
      socket_fd_ = -1;
      return false;
    }

    const int yes = 1;
    setsockopt(socket_fd_, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));

    gzmsg << "[SawMavlinkPosePlugin] TCP connected to " << tcp_addr_ << ":"
          << tcp_port_ << "\n";
    return true;
  }

  void DrainSocket()
  {
    if (socket_fd_ < 0) {
      return;
    }
    uint8_t discard[1024];
    while (recv(socket_fd_, discard, sizeof(discard), MSG_DONTWAIT) > 0) {
    }
  }

  void OnUpdate(const common::UpdateInfo& info)
  {
    if (!link_ || send_rate_hz_ <= 0.0) {
      return;
    }

    const double now = info.simTime.Double();
    const ignition::math::Pose3d pose = link_->WorldPose();
    const ignition::math::Vector3d pos_world = pose.Pos();
    const ignition::math::Vector3d vel_world = link_->WorldLinearVel();
    const ignition::math::Vector3d pos_ned = WorldToNed(pos_world);
    const ignition::math::Vector3d vel_ned = WorldToNed(vel_world);

    if (last_ros_time_ < 0.0 || now - last_ros_time_ >= 1.0 / send_rate_hz_) {
      last_ros_time_ = now;
      PublishRos(now, pose, vel_world);
    }

    if (socket_fd_ < 0) {
      const double wall = gazebo::common::Time::GetWallTime().Double();
      if (last_connect_attempt_time_ < 0.0 || wall - last_connect_attempt_time_ > 1.0) {
        last_connect_attempt_time_ = wall;
        OpenSocket();
      }
      if (socket_fd_ < 0) {
        return;
      }
    }

    DrainSocket();

    if (last_heartbeat_time_ < 0.0 || now - last_heartbeat_time_ >= 1.0) {
      last_heartbeat_time_ = now;
      if (!SendHeartbeat()) {
        return;
      }
    }

    if (last_send_time_ >= 0.0 && now - last_send_time_ < 1.0 / send_rate_hz_) {
      return;
    }
    last_send_time_ = now;
    SendGlobalPosition(static_cast<uint32_t>(now * 1000.0), pos_ned, vel_ned);
  }

  void PublishRos(double now, const ignition::math::Pose3d& pose,
                  const ignition::math::Vector3d& vel_world)
  {
    if (!ros_node_) {
      return;
    }

    std_msgs::Header header;
    header.stamp = ros::Time(now);
    header.frame_id = "map";

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = header;
    pose_msg.pose.position.x = pose.Pos().X();
    pose_msg.pose.position.y = pose.Pos().Y();
    pose_msg.pose.position.z = pose.Pos().Z();
    pose_msg.pose.orientation.w = pose.Rot().W();
    pose_msg.pose.orientation.x = pose.Rot().X();
    pose_msg.pose.orientation.y = pose.Rot().Y();
    pose_msg.pose.orientation.z = pose.Rot().Z();
    pose_pub_.publish(pose_msg);

    geometry_msgs::TwistStamped vel_msg;
    vel_msg.header = header;
    vel_msg.twist.linear.x = vel_world.X();
    vel_msg.twist.linear.y = vel_world.Y();
    vel_msg.twist.linear.z = vel_world.Z();
    vel_pub_.publish(vel_msg);

    if (joint_) {
      sensor_msgs::JointState joint_msg;
      joint_msg.header = header;
      joint_msg.name.push_back(joint_->GetName());
      joint_msg.position.push_back(joint_->Position(0));
      joint_msg.velocity.push_back(joint_->GetVelocity(0));
      joint_pub_.publish(joint_msg);
    }
  }

  static ignition::math::Vector3d WorldToNed(const ignition::math::Vector3d& world)
  {
    // Match the ArduPilot Gazebo plugins: Gazebo world xyz is N, -E, -D.
    return ignition::math::Vector3d(world.X(), -world.Y(), -world.Z());
  }

  static int16_t ToCms(double vel_mps)
  {
    const int value = static_cast<int>(std::lround(vel_mps * 100.0));
    return static_cast<int16_t>(std::max(-32768, std::min(32767, value)));
  }

  void NedToGlobal(const ignition::math::Vector3d& pos_ned, int32_t& lat_e7,
                   int32_t& lon_e7, int32_t& alt_mm, int32_t& rel_alt_mm) const
  {
    // Same scaling as ArduPilot Location offset_ned().
    const double inv = 89.83204953368922;
    const double lat_rad = home_lat_deg_ * M_PI / 180.0;
    const double lon_scale = std::max(std::cos(lat_rad), 1.0e-7);
    lat_e7 = static_cast<int32_t>(
        std::lround(home_lat_deg_ * 1.0e7 + pos_ned.X() * inv));
    lon_e7 = static_cast<int32_t>(
        std::lround(home_lon_deg_ * 1.0e7 + pos_ned.Y() * inv / lon_scale));
    alt_mm = static_cast<int32_t>(std::lround((home_alt_m_ - pos_ned.Z()) * 1000.0));
    rel_alt_mm = static_cast<int32_t>(std::lround(-pos_ned.Z() * 1000.0));
  }

  bool SendBuffer(const mavlink_message_t& msg)
  {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    const ssize_t sent = send(socket_fd_, buffer, len, MSG_NOSIGNAL);
    if (sent < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      return false;
    }
    if (sent != static_cast<ssize_t>(len)) {
      gzerr << "[SawMavlinkPosePlugin] TCP send failed, closing socket and retrying later\n";
      close(socket_fd_);
      socket_fd_ = -1;
      return false;
    }
    return true;
  }

  bool SendHeartbeat()
  {
    mavlink_message_t msg;
    mavlink_heartbeat_t heartbeat {};
    heartbeat.type = MAV_TYPE_GENERIC;
    heartbeat.autopilot = MAV_AUTOPILOT_INVALID;
    heartbeat.mavlink_version = 3;
    mavlink_msg_heartbeat_encode(system_id_, component_id_, &msg, &heartbeat);
    return SendBuffer(msg);
  }

  void SendGlobalPosition(uint32_t time_boot_ms,
                          const ignition::math::Vector3d& pos_ned,
                          const ignition::math::Vector3d& vel_ned)
  {
    int32_t lat_e7 = 0;
    int32_t lon_e7 = 0;
    int32_t alt_mm = 0;
    int32_t rel_alt_mm = 0;
    NedToGlobal(pos_ned, lat_e7, lon_e7, alt_mm, rel_alt_mm);

    mavlink_message_t msg;
    mavlink_global_position_int_t gpi {};
    gpi.time_boot_ms = time_boot_ms;
    gpi.lat = lat_e7;
    gpi.lon = lon_e7;
    gpi.alt = alt_mm;
    gpi.relative_alt = rel_alt_mm;
    gpi.vx = ToCms(vel_ned.X());
    gpi.vy = ToCms(vel_ned.Y());
    gpi.vz = ToCms(vel_ned.Z());
    gpi.hdg = UINT16_MAX;
    mavlink_msg_global_position_int_encode(system_id_, component_id_, &msg, &gpi);
    SendBuffer(msg);
  }

  physics::ModelPtr model_;
  physics::LinkPtr link_;
  physics::JointPtr joint_;
  event::ConnectionPtr update_connection_;

  std::string tcp_addr_ = "127.0.0.1";
  int tcp_port_ = 14557;
  double send_rate_hz_ = 20.0;
  double last_send_time_ = -1.0;
  double last_ros_time_ = -1.0;
  double last_heartbeat_time_ = -1.0;
  double last_connect_attempt_time_ = -1.0;
  double home_lat_deg_ = -35.362734;
  double home_lon_deg_ = 149.165300;
  double home_alt_m_ = 586.0;
  std::string pose_topic_ = "/tsduav_t4/saw/pose";
  std::string vel_topic_ = "/tsduav_t4/saw/velocity";
  std::string joint_topic_ = "/tsduav_t4/link_base_joint";
  uint8_t system_id_ = 42;
  uint8_t component_id_ = MAV_COMP_ID_ONBOARD_COMPUTER;

  int socket_fd_ = -1;
  sockaddr_in remote_addr_ {};
  std::unique_ptr<ros::NodeHandle> ros_node_;
  ros::Publisher pose_pub_;
  ros::Publisher vel_pub_;
  ros::Publisher joint_pub_;
};

GZ_REGISTER_MODEL_PLUGIN(SawMavlinkPosePlugin)

}  // namespace gazebo
