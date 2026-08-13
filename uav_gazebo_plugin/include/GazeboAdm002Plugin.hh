#ifndef UAV_GAZEBO_PLUGIN_GAZEBO_ADM002_PLUGIN_HH_
#define UAV_GAZEBO_PLUGIN_GAZEBO_ADM002_PLUGIN_HH_

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <ros/ros.h>
#include <sdf/sdf.hh>

#include <cstdint>
#include <memory>
#include <string>

namespace gazebo {

class GAZEBO_VISIBLE GazeboAdm002Plugin : public SensorPlugin {
public:
    GazeboAdm002Plugin();
    ~GazeboAdm002Plugin() override;
    void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf) override;

private:
    void OnUpdate();
    bool OpenSocket();
    void CloseSocket();
    double SelectedForceNewton() const;

    sensors::ForceTorqueSensorPtr sensor_;
    event::ConnectionPtr update_connection_;
    std::unique_ptr<ros::NodeHandle> ros_node_;
    ros::Publisher force_publisher_;

    std::string force_axis_ = "x";
    double force_sign_ = 1.0;
    double force_scale_ = 1.0;
    std::string ros_topic_ = "/tilt_quadcopter_fix/front_rod/contact_force_x";
    std::string udp_bind_address_ = "127.0.0.1";
    uint16_t udp_bind_port_ = 9024;
    double stream_rate_hz_ = 100.0;
    uint8_t device_address_ = 1;

    int socket_fd_ = -1;
    bool stream_enabled_ = false;
    struct sockaddr_in_storage;
    std::unique_ptr<sockaddr_in_storage> peer_address_;
    common::Time last_stream_time_;
};

}  // namespace gazebo

#endif
