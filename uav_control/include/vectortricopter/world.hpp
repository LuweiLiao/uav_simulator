#pragma once

#include <vectortricopter/msg/setpoints.hpp>

namespace tilt {
namespace tricopter {

class Vehicle;

// Get the current time in seconds
double GetCurrentTimeSeconds();

// Get the current vehicle
Vehicle& GetVehicle();

// Set the current trajectory of the vehicle
void SetVehicleTrajectory(const TrajectorySetpoint& setpoint);

}
}