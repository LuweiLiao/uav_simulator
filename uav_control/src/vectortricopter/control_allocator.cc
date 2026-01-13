#include <vectortricopter/control_allocator.hpp>

#include <mav_msgs/Actuators.h>

#include <glog/logging.h>

namespace tilt {
namespace tricopter {

void TriControlAllocator::Init(const Parameter& param) {
    _param = param;

    CreateCAMatrix();

    CreateSubsAndPubs();

    _setpoint_initialized = false;
}

void TriControlAllocator::CreateCAMatrix() {
    Eigen::Matrix<double, 5, 6> M;

    M << 0, 1, 0, 1, 0, 1,
         1, 0, 1, 0, 1, 0,
         1, 0, -1, 0, 0, 0,
         -1, 0, -1, 0, _param.alpha, 0,
         0, -1, 0, 1, 0, 0;

    _ca_matrix = M;
}

void TriControlAllocator::CreateSubsAndPubs() {
    ros::NodeHandle nh("~");

    _rotor_speed_pub = nh.advertise<mav_msgs::Actuators>(_param.rotor_topic, 10);
    _servo_position_pub = nh.advertise<mav_msgs::Actuators>(_param.servo_topic, 10);
}

void TriControlAllocator::SetCASetpoint(const CASetpoint& setpoint) {
    _ca_setpoint = setpoint;

    _setpoint_initialized = true;
}

void TriControlAllocator::Run() {
    if (!_setpoint_initialized) return;

    Eigen::Matrix<double, 5, 1> thrust_torque_setpoint;

    thrust_torque_setpoint(0, 0) = _ca_setpoint.tx;
    thrust_torque_setpoint(1, 0) = _ca_setpoint.tz;
    thrust_torque_setpoint(2, 0) = _ca_setpoint.mx;
    thrust_torque_setpoint(3, 0) = _ca_setpoint.my;
    thrust_torque_setpoint(4, 0) = _ca_setpoint.mz;

    Eigen::Matrix<double, 6, 1> controls = \
        _ca_matrix.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(thrust_torque_setpoint);

    // LOG(INFO) << "Thrust torque setpoints are: " << thrust_torque_setpoint.transpose();
    // LOG(INFO) << "Controls are: " << controls.transpose();
    // LOG(INFO) << "Inversed: " << (_ca_matrix * controls).transpose();

    auto ComputeThrustAndAlpha = [](const double fc, const double fs, double& f, double& alpha) {
        alpha = std::atan2(fs, fc);
        f = std::sqrt(fc * fc + fs * fs);
    };

    double f1, alpha1, f2, alpha2, f3, alpha3;

    ComputeThrustAndAlpha(
        controls(0, 0), controls(1, 0),
        f1, alpha1
    );

    ComputeThrustAndAlpha(
        controls(2, 0), controls(3, 0),
        f2, alpha2
    );

    ComputeThrustAndAlpha(
        controls(4, 0), controls(5, 0),
        f3, alpha3
    );

    // LOG(INFO) << "Thrusts are: "
    //     << f1 << ", " << f2 << ", " << f3 << ", "
    //     << alpha1 << ", " << alpha2 << ", " << alpha3;

    #define CONSTRAINT(v, minv, maxv) \
        std::min(maxv, std::max(minv, (v)))

    mav_msgs::Actuators rotor_msg;
    rotor_msg.angular_velocities = {
        1000 * CONSTRAINT(std::sqrt(3. * f2), 0., 1.),
        1000 * CONSTRAINT(std::sqrt(3. * f2), 0., 1.),
        1000 * CONSTRAINT(std::sqrt(3. * f1), 0., 1.),
        1000 * CONSTRAINT(std::sqrt(3. * f1), 0., 1.),
        1000 * CONSTRAINT(std::sqrt(3. * f3), 0., 1.),
        1000 * CONSTRAINT(std::sqrt(3. * f3), 0., 1.),
    };

    _rotor_speed_pub.publish(rotor_msg);

    mav_msgs::Actuators servo_msg;
    servo_msg.angular_velocities = {
        CONSTRAINT(-alpha2, - M_PI, M_PI),
        CONSTRAINT(alpha1, - M_PI, M_PI),
        CONSTRAINT(-alpha3, - M_PI, M_PI)
    };

    _servo_position_pub.publish(servo_msg);

    #undef CONSTRAINT
}

}
}