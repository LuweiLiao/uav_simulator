#include "GazeboAdm002Plugin.hh"

#include "Adm002Protocol.hh"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cstring>
#include <std_msgs/Float64.h>

namespace gazebo {

struct GazeboAdm002Plugin::sockaddr_in_storage {
    sockaddr_in value{};
};

GZ_REGISTER_SENSOR_PLUGIN(GazeboAdm002Plugin)

GazeboAdm002Plugin::GazeboAdm002Plugin() = default;

GazeboAdm002Plugin::~GazeboAdm002Plugin()
{
    CloseSocket();
}

void GazeboAdm002Plugin::Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf)
{
    sensor_ = std::dynamic_pointer_cast<sensors::ForceTorqueSensor>(sensor);
    if (!sensor_) {
        gzerr << "GazeboAdm002Plugin requires a ForceTorqueSensor\n";
        return;
    }

    force_axis_ = sdf->Get("forceAxis", force_axis_).first;
    force_sign_ = sdf->Get("forceSign", force_sign_).first;
    force_scale_ = sdf->Get("forceScale", force_scale_).first;
    ros_topic_ = sdf->Get("rosTopic", ros_topic_).first;
    udp_bind_address_ = sdf->Get("udpBindAddress", udp_bind_address_).first;
    udp_bind_port_ = static_cast<uint16_t>(sdf->Get("udpBindPort", static_cast<uint32_t>(udp_bind_port_)).first);
    stream_rate_hz_ = sdf->Get("streamRateHz", stream_rate_hz_).first;
    device_address_ = static_cast<uint8_t>(sdf->Get("deviceAddress", static_cast<uint32_t>(device_address_)).first);

    if (force_axis_ != "x" && force_axis_ != "y" && force_axis_ != "z") {
        gzerr << "GazeboAdm002Plugin forceAxis must be x, y or z\n";
        return;
    }
    if (stream_rate_hz_ <= 0.0) {
        gzerr << "GazeboAdm002Plugin streamRateHz must be positive\n";
        return;
    }
    if (!ros::isInitialized()) {
        gzerr << "GazeboAdm002Plugin requires gazebo_ros to initialize ROS\n";
        return;
    }
    if (!OpenSocket()) {
        gzerr << "GazeboAdm002Plugin failed to bind " << udp_bind_address_
              << ":" << udp_bind_port_ << "\n";
        return;
    }

    ros_node_.reset(new ros::NodeHandle());
    force_publisher_ = ros_node_->advertise<std_msgs::Float64>(ros_topic_, 10);
    peer_address_.reset(new sockaddr_in_storage());
    last_stream_time_ = sensor_->LastUpdateTime();
    update_connection_ = sensor_->ConnectUpdated(
        std::bind(&GazeboAdm002Plugin::OnUpdate, this));
    sensor_->SetActive(true);

    ROS_INFO_STREAM("Gazebo ADM002 sensor:" << sensor_->ScopedName()
                    << " axis:" << force_axis_ << " sign:" << force_sign_
                    << " UDP:" << udp_bind_address_ << ":" << udp_bind_port_
                    << " stream_rate_hz:" << stream_rate_hz_);
}

bool GazeboAdm002Plugin::OpenSocket()
{
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
        return false;
    }
    const int reuse = 1;
    setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    fcntl(socket_fd_, F_SETFL, fcntl(socket_fd_, F_GETFL, 0) | O_NONBLOCK);

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(udp_bind_port_);
    if (inet_pton(AF_INET, udp_bind_address_.c_str(), &address.sin_addr) != 1 ||
        bind(socket_fd_, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) != 0) {
        CloseSocket();
        return false;
    }
    return true;
}

void GazeboAdm002Plugin::CloseSocket()
{
    if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
    }
}

double GazeboAdm002Plugin::SelectedForceNewton() const
{
    const ignition::math::Vector3d force = sensor_->Force();
    const double component = force_axis_ == "x" ? force.X() :
                             force_axis_ == "y" ? force.Y() : force.Z();
    return component * force_sign_;
}

void GazeboAdm002Plugin::OnUpdate()
{
    const double force_n = SelectedForceNewton();
    std_msgs::Float64 force_message;
    force_message.data = force_n;
    force_publisher_.publish(force_message);

    std::array<uint8_t, 256> request{};
    while (true) {
        sockaddr_in source{};
        socklen_t source_length = sizeof(source);
        const ssize_t received = recvfrom(socket_fd_, request.data(), request.size(), 0,
            reinterpret_cast<sockaddr*>(&source), &source_length);
        if (received < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                ROS_WARN_THROTTLE(1.0, "Gazebo ADM002 receive failed: %s", std::strerror(errno));
            }
            break;
        }
        if (!uav_gazebo_plugin::Adm002Protocol::IsEnableStreamCommand(
                request.data(), static_cast<std::size_t>(received), device_address_)) {
            continue;
        }
        peer_address_->value = source;
        stream_enabled_ = true;
        const auto ack = uav_gazebo_plugin::Adm002Protocol::EncodeEnableAck(device_address_);
        sendto(socket_fd_, ack.data(), ack.size(), 0,
            reinterpret_cast<const sockaddr*>(&peer_address_->value), sizeof(peer_address_->value));
    }

    if (!stream_enabled_) {
        return;
    }

    const common::Time now = sensor_->LastUpdateTime();
    if (now < last_stream_time_) {
        last_stream_time_ = now;
    }
    const double period_s = 1.0 / stream_rate_hz_;
    if ((now - last_stream_time_).Double() < period_s) {
        return;
    }
    last_stream_time_ = now;

    const auto frame = uav_gazebo_plugin::Adm002Protocol::EncodeForceNewton(force_n, force_scale_);
    sendto(socket_fd_, frame.data(), frame.size(), 0,
        reinterpret_cast<const sockaddr*>(&peer_address_->value), sizeof(peer_address_->value));
}

}  // namespace gazebo
