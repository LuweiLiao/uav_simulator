#include <vectortricopter/vehicle.hpp>


#include <ros/ros.h>

#include <gazebo_msgs/ModelStates.h>
#include <sensor_msgs/Imu.h>

#include <glog/logging.h>

#include <vectortricopter/scheduler.hpp>

namespace tilt {
namespace tricopter {

Vehicle *Vehicle::globalVehicleInstance = nullptr;

Vehicle::Vehicle() {
    LoadParametersFromNH();

    ros::NodeHandle nh("~");
    _odom_sub = nh.subscribe("/gazebo/model_states", 10, &Vehicle::ModelStatesCallback, this);
    _imu_sub = nh.subscribe("/tilt_tricopter/imu", 10, &Vehicle::IMUCallback, this);

    StartControlLoop();
}

Vehicle& Vehicle::GetInstance() {
    if (!globalVehicleInstance) {
        globalVehicleInstance = new Vehicle();
    }

    CHECK(globalVehicleInstance != nullptr);

    return *globalVehicleInstance;
}

void Vehicle::LoadParametersFromNH() {
    ros::NodeHandle nh("~");

    Parameter param;

    LoadPosCtrlParameters(nh, param.pos_ctrl_param);
    LoadAttCtrlParameters(nh, param.att_ctrl_param);
    LoadRateCtrlParameters(nh, param.rate_ctrl_param);
    LoadCtrlAllocParameters(nh, param.ctrl_alloc_param);

    _pos_ctrl.Init(param.pos_ctrl_param);
    _att_ctrl.Init(param.att_ctrl_param);
    _rate_ctrl.Init(param.rate_ctrl_param);
    _control_allocator.Init(param.ctrl_alloc_param);
}

#define LOAD_PARAM_VALUE(namespace, name, default) \
    param.name = nh.param<double>(#namespace "/" #name, default); \
    CHECK_GE(param.name, 0)

void Vehicle::LoadPosCtrlParameters(ros::NodeHandle& nh, TriPositionControl::Parameter& param) {
    LOAD_PARAM_VALUE(pos_ctrl, kpx, 5);
    LOAD_PARAM_VALUE(pos_ctrl, kpy, 5);
    LOAD_PARAM_VALUE(pos_ctrl, kpz, 5);
    LOAD_PARAM_VALUE(pos_ctrl, kvx, 1.5);
    LOAD_PARAM_VALUE(pos_ctrl, kvy, 1.5);
    LOAD_PARAM_VALUE(pos_ctrl, kvz, 1.5);

    LOAD_PARAM_VALUE(pos_ctrl, hover_thrust, 0.5);
}

void Vehicle::LoadAttCtrlParameters(ros::NodeHandle& nh, TriAttitudeControl::Parameter& param){
    LOAD_PARAM_VALUE(att_ctrl, roll_gain, 1);
    LOAD_PARAM_VALUE(att_ctrl, pitch_gain, 1);
    LOAD_PARAM_VALUE(att_ctrl, yaw_gain, 1);
}

void Vehicle::LoadRateCtrlParameters(ros::NodeHandle& nh, TriRateControl::Parameter& param) {
    LOAD_PARAM_VALUE(rate_ctrl, kroll, 1);
    LOAD_PARAM_VALUE(rate_ctrl, kpitch, 1);
    LOAD_PARAM_VALUE(rate_ctrl, kyaw, 1);

    LOAD_PARAM_VALUE(rate_ctrl, ki_roll, 0.12);
    LOAD_PARAM_VALUE(rate_ctrl, ki_pitch, 0.12);
    LOAD_PARAM_VALUE(rate_ctrl, ki_yaw, 0.12);

    LOAD_PARAM_VALUE(rate_ctrl, kp_roll, 5);
    LOAD_PARAM_VALUE(rate_ctrl, kp_pitch, 5);
    LOAD_PARAM_VALUE(rate_ctrl, kp_yaw, 5);
}

void Vehicle::LoadCtrlAllocParameters(ros::NodeHandle& nh, TriControlAllocator::Parameter& param) {
    LOAD_PARAM_VALUE(control_allocator, alpha, 1);

    param.rotor_topic = nh.param<std::string>("control_allocator/rotor_topic", "/gazebo/command/prop_speed");
    param.servo_topic = nh.param<std::string>("control_allocator/servo_topic", "/gazebo/command/tilt1_pos");
}

#undef LOAD_PARAM_VALUE

void Vehicle::StartControlLoop() {
    std::vector<Scheduler::Task> tasks = {
        SCHED_TASK_CLASS(&_pos_ctrl, TriPositionControl, Run, 50),
        SCHED_TASK_CLASS(&_att_ctrl, TriAttitudeControl, Run, 100),
        SCHED_TASK_CLASS(&_rate_ctrl, TriRateControl, Run, 500),
        SCHED_TASK_CLASS(&_control_allocator, TriControlAllocator, Run, 500)
    };

    Scheduler& scheduler = Scheduler::GetInstance();

    scheduler.Init(tasks);
}

void Vehicle::SetTrajectorySetpoint(const TrajectorySetpoint& setpoint) {
    _pos_ctrl.SetTrajectorySetpoint(setpoint);
}

void Vehicle::SetAttitudeSetpoint(const AttitudeSetpoint& setpoint) {
    _att_ctrl.SetAttitudeSetpoint(setpoint);
}

void Vehicle::SetRateSetpoint(const RateSetpoint& setpoint) {
    _rate_ctrl.SetRateSetpoint(setpoint);
}

void Vehicle::SetCASetpoint(const CASetpoint& setpoint) {
    _control_allocator.SetCASetpoint(setpoint);
}

void Vehicle::ModelStatesCallback(const gazebo_msgs::ModelStatesConstPtr msg) {
    std::unique_lock<std::mutex> lock(_odom_mutex);

    _position.x() = msg->pose[1].position.x;
    _position.y() = msg->pose[1].position.y;
    _position.z() = msg->pose[1].position.z;

    _velocity.x() = msg->twist[1].linear.x;
    _velocity.y() = msg->twist[1].linear.y;
    _velocity.z() = msg->twist[1].linear.z;

    _attitude.w() = msg->pose[1].orientation.w;
    _attitude.x() = msg->pose[1].orientation.x;
    _attitude.y() = msg->pose[1].orientation.y;
    _attitude.z() = msg->pose[1].orientation.z;
}

void Vehicle::IMUCallback(const sensor_msgs::ImuConstPtr msg) {
    std::unique_lock<std::mutex> lock(_imu_mutex);

    _acceleration.x() = msg->linear_acceleration.x;
    _acceleration.y() = msg->linear_acceleration.y;
    _acceleration.z() = msg->linear_acceleration.z;

    _omega.x() = msg->angular_velocity.x;
    _omega.y() = msg->angular_velocity.y;
    _omega.z() = msg->angular_velocity.z;

    _current_time = msg->header.stamp.toSec();
}

}
}