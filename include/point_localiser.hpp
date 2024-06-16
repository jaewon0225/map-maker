#include <Eigen/Dense>
#include <iostream>
#include <limits>

class ICP2D {
public:
    ICP2D(const Eigen::MatrixXd& source_points, const Eigen::MatrixXd& target_points)
        : source_points(source_points), target_points(target_points) {}

    void runICP(int max_iterations = 100, double tolerance = 1e-6);

    Eigen::Matrix2d getRotationMatrix() const;
    Eigen::Vector2d getTranslationVector() const;

private:
    Eigen::MatrixXd source_points;
    Eigen::MatrixXd target_points;
    Eigen::Matrix2d R;
    Eigen::Vector2d t;

    Eigen::MatrixXd findCorrespondences(const Eigen::MatrixXd& source, const Eigen::MatrixXd& target);
    std::pair<Eigen::Matrix2d, Eigen::Vector2d> computeTransformation(const Eigen::MatrixXd& P, const Eigen::MatrixXd& Q);
    double computeError(const Eigen::MatrixXd& source, const Eigen::MatrixXd& target);
};