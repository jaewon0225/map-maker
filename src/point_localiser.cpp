#include <cmath>
#include <vector>
#include <algorithm>

#include "point_localiser.hpp"
#include "plane_maker.cpp"
#include "helpers.hpp"


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

void ICP2D::extractFeatures() {
    int pointcloud_size = source_points.cols();
    std::vector<std::pair<int, float>> region_curvatures;
    float point_weight = -2 * num_regions;

    for (size_t i  = 0; i < num_regions; ++i) {
        // Calculate start and end point for current region
        int start = ((pointcloud_size - 1) * i) / num_regions;
        int end = ((region_size) * (num_regions - i - 1) + (pointcloud_size - 1) *(i + 1)) / num_regions -1;

        for (size_t j = start; j <= end; ++j) {
            float x_diff = point_weight * at(source_points, 0, j);
            float y_diff = point_weight * at(source_points, 1, j);

            for (int k = 1; k <= region_size; ++k) {
                x_diff += at(source_points, 0, j + k) + at(source_points, 0, j - k);
                y_diff += at(source_points, 1, j + k) + at(source_points, 1, j - k);
            }
            region_curvatures.emplace_back(j, x_diff * x_diff + y_diff * y_diff);
        }
        std::sort(region_curvatures.begin(), region_curvatures.end(), [](auto a, auto b) {
            return a.second > b.second;
        });
        int append_count = 0;
        for (auto& curvature : region_curvatures) {
            if (append_count > max_edge_per_region + 1 || curvature.second <= plane_threshold) {
                break;
            }
            if (curvature.second > plane_threshold) {
                edge_points.push_back(curvature.first);
                append_count += 1;
            }
        }
    }
}

void ICP2D::extractUnreliablePoints() {
    int pointcloud_size = source_points.cols();
    picked_points.assign(pointcloud_size, 0);
    for (size_t i = 0; i < pointcloud_size; ++i) {
        const Eigen::Vector2d prev_point = atCol(source_points, i - 1);
        const Eigen::Vector2d point = atCol(source_points, i);
        const Eigen::Vector2d next_point = atCol(source_points, i + 1);

        double next_dist = pointPointDistanceSquared(prev_point, point);

        if (next_dist > 0.1) {
            double depth1 = originPointDistance(point);
            double depth2 = originPointDistance(next_point);
            
            if (depth1 > depth2) {
                double weighted_distance = weightedPointDistance(next_point, point, depth2 / depth1) / depth2;
                if (weighted_distance < 0.1) {
                    if (i - region_size >= 0) {
                        std::fill_n(&picked_points[i - region_size], region_size + 1, 1);
                    }
                    else {
                        std::fill_n(&picked_points[i - region_size + pointcloud_size], region_size - i, 1);
                        std::fill_n(&picked_points[0], i + 1, 1);
                    }
                    continue;
                }
            }
            else {
                double weighted_distance = weightedPointDistance(point, next_point, depth1 / depth2) / depth1;
                if (weighted_distance < 0.1) {
                    if (i + 1 >= pointcloud_size) {
                        std::fill_n(&picked_points[0], region_size + 1, 1);
                    }
                    else if (i + 1 + region_size >= pointcloud_size) {
                        std::fill_n(&picked_points[i + 1], i + region_size + 1 - pointcloud_size, 1);
                        std::fill_n(&picked_points[0], pointcloud_size - i - 1, 1);
                    }
                    else {
                        std::fill_n(&picked_points[i + 1], region_size + 1, 1);
                    }
                }
            } 

            float prev_dist = pointPointDistanceSquared(point, prev_point);
            float dis = pow(originPointDistance(point),2);

            if (next_dist > 0.0002 * dis && prev_dist > 0.0002 * dis) {
                picked_points[i] = 1;
            }
        }
    }
}

// TODO: Plane downsizer