#pragma once

#include <ros/ros.h>

#include <memory>
#include <mutex>
#include <thread>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vectortricopter/pos_controller.hpp>
#include <vectortricopter/att_controller.hpp>
#include <vectortricopter/rate_controller.hpp>
#include <vectortricopter/control_allocator.hpp>

#include <vectortricopter/msg/setpoints.hpp>

#include <gazebo_msgs/ModelStates.h>
#include <sensor_msgs/Imu.h>

namespace tilt {
namespace tricopter {

class Vehicle {
public:
    struct Parameter {
        TriPositionControl::Parameter pos_ctrl_param;
        TriAttitudeControl::Parameter att_ctrl_param;
        TriRateControl::Parameter rate_ctrl_param;
        TriControlAllocator::Parameter ctrl_alloc_param;
    };


    // Get the global instance of the vehicle
    static Vehicle& GetInstance();

    // Get the current position of the vehicle
    Eigen::Vector3d GetVehiclePosition() {
        std::unique_lock<std::mutex> lock(_odom_mutex);
        return _position;
    }

    // Get the current velocity of the vehicle
    Eigen::Vector3d GetVehicleVelocity() {
        std::unique_lock<std::mutex> lock(_odom_mutex);
        return _velocity;
    }

    // Get the current orientation of the vehicle
    Eigen::Quaterniond GetVehicleAttitude() {
        std::unique_lock<std::mutex> lock(_odom_mutex);
        return _attitude;
    }

    // Get the current acceleration of the vehicle
    Eigen::Vector3d GetVehicleAcceleration() {
        std::unique_lock<std::mutex> lock(_imu_mutex);
        return _acceleration;
    }

    // Get the current angular velocity of the vehicle
    Eigen::Vector3d GetAngularVelocity() {
        std::unique_lock<std::mutex> lock(_imu_mutex);
        return _omega;
    }

    double GetCurrentTime() const {
        return _current_time;
    }

    void SetTrajectorySetpoint(const TrajectorySetpoint& setpoint);

    void SetAttitudeSetpoint(const AttitudeSetpoint& setpoint);

    void SetRateSetpoint(const RateSetpoint& setpoint);

    void SetCASetpoint(const CASetpoint& setpoint);
private:
    // Thr global instance of the vehicle
    static Vehicle *globalVehicleInstance;

    TriPositionControl _pos_ctrl;
    TriAttitudeControl _att_ctrl;
    TriRateControl _rate_ctrl;
    TriControlAllocator _control_allocator;
private:
    // Disable construct and copy from output
    Vehicle();

    Vehicle(const Vehicle&) = delete;
    Vehicle(Vehicle&&) = delete;
    Vehicle& operator=(const Vehicle&) = delete;
    Vehicle& operator=(Vehicle&&) = delete;

    void LoadParametersFromNH();

    void LoadPosCtrlParameters(ros::NodeHandle& nh, TriPositionControl::Parameter& param);
    void LoadAttCtrlParameters(ros::NodeHandle& nh, TriAttitudeControl::Parameter& param);
    void LoadRateCtrlParameters(ros::NodeHandle& nh, TriRateControl::Parameter& param);
    void LoadCtrlAllocParameters(ros::NodeHandle& nh, TriControlAllocator::Parameter& param);

    void StartControlLoop();

private:
    ros::Subscriber _odom_sub;
    ros::Subscriber _imu_sub;

    // Odometry 
    std::mutex _odom_mutex;
    Eigen::Vector3d _position;
    Eigen::Vector3d _velocity;
    Eigen::Quaterniond _attitude;

    // IMU
    std::mutex _imu_mutex;
    Eigen::Vector3d _acceleration;
    Eigen::Vector3d _omega;
    double _current_time;

    void ModelStatesCallback(const gazebo_msgs::ModelStatesConstPtr msg);

    void IMUCallback(const sensor_msgs::ImuConstPtr msg);
};

}
}