#include <cmath>
#include <Eigen/Dense>
#include <vector>
#include <memory>

class Odom {
public:
    Odom();
    Odom(double x, double y, double theta, double b, double kr, double kl);

private:
    double x, y, theta, b, kr, kl;

public:
    Eigen::Matrix3d returnWheelOdomCovariance(const double& dsl, const double& dsr);
    void update(const double& dx, const double& dy, const double& dtheta);
};

class OdomCollection {
public:
    OdomCollection();
};