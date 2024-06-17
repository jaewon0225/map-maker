#include "point_localiser.cpp"

int main() {
    Eigen::MatrixXd source(2, 3);
    source << 0, 1, 0,
              0, 0, 1;

    Eigen::MatrixXd target(2, 3);
    target << 0, 1, 0,
              0, 0, 1;

    ICP2D icp(source, target);
    auto [points, R, t] = icp.runICP();

    

    std::cout << "Rotation Matrix:\n" << R << std::endl;
    std::cout << "Translation Vector:\n" << t.transpose() << std::endl;

    return 0;
}