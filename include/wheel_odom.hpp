#include <cmath>
#include <Eigen/Dense>
#include <vector>

class WheelOdom {
public:
    WheelOdom();
    WheelOdom(double x, double y, double theta, double b, double kr, double kl, Eigen::Matrix3d covariance);

private:
    double x, y, theta, b, kr, kl;
    Eigen::Matrix3d covariance;

public:
    void update(const double& dsl, const double& dsr);
    void updateManually(const double& dx, const double& dy, const double& dtheta, const Eigen::Matrix3d& new_covariance);
    void fuseMeasurements(const double& other_x, const double& other_y, const Eigen::Matrix3d& other_covariance);
    Eigen::Matrix3d getInfoMatrix();

};