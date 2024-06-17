#include <cmath>
#include "point_localiser.hpp"
#include "plane_maker.cpp"

Eigen::MatrixXd ICP2D::findCorrespondences(const Eigen::MatrixXd& source, const Eigen::MatrixXd& target) {
    Eigen::MatrixXd correspondences(2, source.cols());
    for (int i = 0; i < source.cols(); ++i) {
        double min_dist = std::numeric_limits<double>::max();
        Eigen::Vector2d closest_point;
        for (int j = 0; j < target.cols(); ++j) {
            double dist = (source.col(i) - target.col(j)).squaredNorm();
            if (dist < min_dist) {
                min_dist = dist;
                closest_point = target.col(j);
            }
        }
        correspondences.col(i) = closest_point;
    }
    return correspondences;
}

std::pair<Eigen::Matrix2d, Eigen::Vector2d> ICP2D::computeTransformation(const Eigen::MatrixXd& P, const Eigen::MatrixXd& Q) {
    Eigen::Vector2d P_centroid = P.rowwise().mean();
    Eigen::Vector2d Q_centroid = Q.rowwise().mean();

    Eigen::MatrixXd P_prime = P.colwise() - P_centroid;
    Eigen::MatrixXd Q_prime = Q.colwise() - Q_centroid;

    Eigen::Matrix2d H = P_prime * Q_prime.transpose();

    Eigen::JacobiSVD svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix2d U = svd.matrixU();
    Eigen::Matrix2d V = svd.matrixV();

    Eigen::Matrix2d R = U * V.transpose();
    if (R.determinant() < 0) {
        V.col(1) *= -1;
        R = U * V.transpose();
    }
    Eigen::Vector2d t = Q_centroid - R * P_centroid;

    return {R, t};
}

double ICP2D::computeError(const Eigen::MatrixXd& P, const Eigen::MatrixXd& Q) {
    double error = 0.0;
    for (int i = 0; i < P.cols(); ++i) {
        error += (P.col(i) - Q.col(i)).squaredNorm();
    }
    return error;
}

std::tuple<Eigen::MatrixXd, Eigen::Matrix2d, Eigen::Vector2d> ICP2D::runICP(int max_iterations, double tolerance) {
    double previousError = std::numeric_limits<double>::max();
    Eigen::MatrixXd transformed_source = source_points;
    Eigen::Matrix2d R = Eigen::Matrix2d::Identity();
    Eigen::Vector2d t = Eigen::Vector2d::Zero();

    for (int i = 0; i < max_iterations; ++i) {
        Eigen::MatrixXd target_prime = findCorrespondences(transformed_source, target_points);
        auto [R_new, t_new] = computeTransformation(transformed_source, target_prime);
        transformed_source = (R_new * transformed_source).colwise() + t_new;

        std::cout << R_new << std::endl;
        R = R_new * R;
        t = R_new * t + t_new;
        std::cout << R << std::endl;
        std::cout << "_____________" <<std::endl;
        double currentError = computeError(transformed_source, target_prime);
        if (std::abs(previousError - currentError) < tolerance) {
            break;
        }
        previousError = currentError;
    }

    return {transformed_source, R, t};
}