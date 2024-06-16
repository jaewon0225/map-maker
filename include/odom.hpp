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

class Node {
public:
    Node(u_int32_t index, Eigen::Vector3d pose);
    Node(u_int32_t index, Eigen::Vector3d pose, Eigen::Matrix3Xd point_cloud);

public:
    u_int32_t index;
    Eigen::Vector3d pose;
    Eigen::Matrix3Xd point_cloud;
};

class Edge {
public:
    Edge(u_int32_t from, u_int32_t to, Eigen::Vector3d measurement, Eigen::Matrix3d information);

public:
    u_int32_t from;
    u_int32_t to;
    Eigen::Vector3d measurement;
    Eigen::Matrix3d information;
};

class OdomCollection {
public:
    OdomCollection();
};