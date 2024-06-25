#include <Eigen/Dense>
#include <iostream>
#include <limits>

class ICP2D {
public:
    ICP2D(const Eigen::MatrixXd& source_points, const Eigen::MatrixXd& target_points)
        : source_points(source_points), target_points(target_points) {}

    std::tuple<Eigen::MatrixXd, Eigen::Matrix2d, Eigen::Vector2d> runICP(int max_iterations = 100, double tolerance = 1e-6);

private:
    Eigen::MatrixXd source_points;
    Eigen::MatrixXd target_points;
    std::vector<int> edge_points;
    std::vector<int> plane_points;
    std::vector<int> picked_points;
    const int num_regions = 5;
    const int region_size = 5; // Number of points in half region
    const int max_edge_per_region = 2;
    const float plane_threshold = 0.1; // Curvature value above this is considered a corner and plane if below

    Eigen::MatrixXd findCorrespondences(const Eigen::MatrixXd& source, const Eigen::MatrixXd& target);
    std::pair<Eigen::Matrix2d, Eigen::Vector2d> computeTransformation(const Eigen::MatrixXd& P, const Eigen::MatrixXd& Q);
    double computeError(const Eigen::MatrixXd& source, const Eigen::MatrixXd& target);
    void extractFeatures();
    void extractUnreliablePoints();
};