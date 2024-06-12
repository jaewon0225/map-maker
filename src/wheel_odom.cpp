#include "wheel_odom.hpp"

WheelOdom::WheelOdom()
: x(0), y(0), theta(0), b(1), kr(1), kl(1), covariance(Eigen::Matrix3d::Zero()) {}

WheelOdom::WheelOdom(double x, double y, double theta, double b, double kr, double kl, Eigen::Matrix3d covariance)
    : x(x), y(y), theta(theta), b(b), kr(kr), kl(kl), covariance(covariance) {}

void WheelOdom::update(const double& dsl, const double& dsr) {
    double dtheta = (dsr - dsl) / b;
    double ds     = (dsr + dsl) / 2;
    double s      = sin(theta + dtheta / 2);
    double c      = cos(theta + dtheta / 2);
    double dx     = ds * c;
    double dy     = ds * s;
    Eigen::Matrix3d pf;
    pf << 1, 0, -ds * s,
            0, 1,  ds * c,
            0, 0,       1;
    
    Eigen::Matrix<double, 3, 2> rlf;
    rlf << c / 2 - s * ds / (2 * b), c / 2 + s * ds / (2 * b),
            s / 2 + c * ds / (2 * b), s / 2 - c * ds / (2 * b),
            1 / b,                    -1 / b;

    Eigen::Matrix2d wheel_covariance;
    wheel_covariance << kr * abs(dsr), 0,
                        0, kl * abs(dsl);

    // Update variables
    x += dx;
    y += dy;
    theta += dtheta;
    covariance = pf * covariance * pf.transpose() + rlf * wheel_covariance * rlf.transpose();
}

void WheelOdom::updateManually(const double& dx, const double& dy, const double& dtheta, const Eigen::Matrix3d& new_covariance) {
    x += dx;
    y += dy;
    theta += dtheta;
    covariance = new_covariance;
}