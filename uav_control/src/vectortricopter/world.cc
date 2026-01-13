#include <ros/ros.h>

#include <vectortricopter/world.hpp>
#include <vectortricopter/vehicle.hpp>

namespace tilt {
namespace tricopter {

double GetCurrentTimeSeconds() {
    return ros::Time::now().toSec();
}

Vehicle& GetVehicle() {
    return Vehicle::GetInstance();
}

void SetVehicleTrajectory(const TrajectorySetpoint& setpoint) {
    GetVehicle().SetTrajectorySetpoint(setpoint);
}

}
}