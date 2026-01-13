#pragma once

#include <ros/ros.h>
#include <string>

#include <Eigen/Dense>

#include <vectortricopter/msg/setpoints.hpp>

namespace tilt {
namespace tricopter {

class TriControlAllocator {
public:
    struct Parameter {
        double alpha;

        std::string rotor_topic;
        std::string servo_topic;
    };

    void Init(const Parameter& param);

    void Run();

    void SetCASetpoint(const CASetpoint& setpoint);

private:
    void CreateCAMatrix();

    void CreateSubsAndPubs();
private:
    Parameter _param;

    CASetpoint _ca_setpoint;
    bool _setpoint_initialized;

    ros::Publisher _rotor_speed_pub;
    ros::Publisher _servo_position_pub;

    Eigen::MatrixXd _ca_matrix;
};

}
}