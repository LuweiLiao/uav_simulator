#include <vectortricopter/world.hpp>
#include <vectortricopter/msg/setpoints.hpp>

#include <mav_msgs/TrajectorySetpoint.h>

#include <ros/ros.h>

static ros::Subscriber trajectory_setpoint_sub;

void TrajectorySetpointCallback(const mav_msgs::TrajectorySetpointConstPtr traj) {
    tilt::tricopter::TrajectorySetpoint setpoint;

    setpoint.x = traj->x;
    setpoint.y = traj->y;
    setpoint.z = traj->z;

    setpoint.vx = traj->vx;
    setpoint.vy = traj->vy;
    setpoint.vz = traj->vz;

    setpoint.ax = traj->ax;
    setpoint.ay = traj->ay;
    setpoint.az = traj->az;

    setpoint.yaw = traj->yaw;
    setpoint.pitch = traj->pitch;

    tilt::tricopter::SetVehicleTrajectory(setpoint);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tricopter_control_node");

    ros::NodeHandle nh("~");

    trajectory_setpoint_sub = \
        nh.subscribe("/tilt_tricopter/trajectory_setpoint", 10, TrajectorySetpointCallback);

    tilt::tricopter::Vehicle& vehicle = tilt::tricopter::GetVehicle();

    ros::spin();
}