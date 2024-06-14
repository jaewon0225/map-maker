#include "odom.hpp"

Odom::Odom()
: x(0), y(0), theta(0), b(1), kr(1), kl(1) {}

Odom::Odom(double x, double y, double theta, double b, double kr, double kl)
    : x(x), y(y), theta(theta), b(b), kr(kr), kl(kl) {}

Eigen::Matrix3d Odom::returnWheelOdomCovariance(const double& dsl, const double& dsr) {
    double dtheta = (dsr - dsl) / b;
    double ds     = (dsr + dsl) / 2;
    double s      = sin(theta + dtheta / 2);
    double c      = cos(theta + dtheta / 2);
    double dx     = ds * c;
    double dy     = ds * s;
    
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
    return (rlf * wheel_covariance * rlf.transpose());
}

void Odom::update(const double& dx, const double& dy, const double& dtheta) {
    x += dx;
    y += dy;
    theta += dtheta;
}