#include <vectortricopter/pos_controller.hpp>
#include <vectortricopter/world.hpp>
#include <vectortricopter/vehicle.hpp>

#include <glog/logging.h>

namespace tilt {
namespace tricopter {

void TriPositionControl::Init(const TriPositionControl::Parameter& param) {
    _params = param;
    _trajectory_initialized.store(false);

    _kp = Eigen::Vector3d{
        _params.kpx,
        _params.kpy,
        _params.kpz
    };

    _kv = Eigen::Vector3d{
        _params.kvx,
        _params.kvy,
        _params.kvz
    };

    _hover_thrust = param.hover_thrust;
} 

void TriPositionControl::SetTrajectorySetpoint(const TrajectorySetpoint& setpoint) {
    std::unique_lock<std::mutex> lock(_mutex);

    _trajectory_setpoint = setpoint;
    _trajectory_initialized.store(true);
}

void TriPositionControl::Run() {
    if (! _trajectory_initialized.load()) return;

    Vehicle& vehicle = GetVehicle();
    
    Eigen::Vector3d pos_sp {
        _trajectory_setpoint.x,
        _trajectory_setpoint.y,
        _trajectory_setpoint.z
    };

    Eigen::Vector3d vel_sp {
        _trajectory_setpoint.vx,
        _trajectory_setpoint.vy,
        _trajectory_setpoint.vz
    };

    Eigen::Vector3d acc_sp {
        _trajectory_setpoint.ax,
        _trajectory_setpoint.ay,
        _trajectory_setpoint.az
    };

    Eigen::Vector3d pos_err = pos_sp - vehicle.GetVehiclePosition();
    Eigen::Vector3d vel_err = vel_sp - vehicle.GetVehicleVelocity();

    acc_sp += _kp.asDiagonal() * pos_err + _kv.asDiagonal() * vel_err;

    // Calculate the attitude setpoint
    CalculateAttitudeSetpoint(acc_sp);
}

void TriPositionControl::CalculateAttitudeSetpoint(const Eigen::Vector3d& acc_sp) {
    Eigen::Vector3d desired_force = acc_sp;
    desired_force.z() += 9.8;

    // Calculate the target attitude
    const double syaw = std::sin(_trajectory_setpoint.yaw);
    const double cyaw = std::cos(_trajectory_setpoint.yaw);

    const double sroll = desired_force.x() * syaw - desired_force.y() * cyaw;
    const double croll = desired_force.z();

    const double roll = std::atan2(sroll, croll);

    Eigen::Quaterniond qsp(
        Eigen::AngleAxisd(_trajectory_setpoint.yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(_trajectory_setpoint.pitch, Eigen::Vector3d::UnitY())
    );

    // Calaulate the target thrust
    Eigen::Vector3d thrust = _hover_thrust / 9.8 * desired_force;

    AttitudeSetpoint att_sp {
        .qw = qsp.w(),
        .qx = qsp.x(),
        .qy = qsp.y(),
        .qz = qsp.z(),
        .thrustx = thrust.x(),
        .thrusty = thrust.y(),
        .thrustz = thrust.z()
    };

    GetVehicle().SetAttitudeSetpoint(att_sp);
}

}
}