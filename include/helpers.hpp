#include <Eigen/Dense>
#include <iostream>
#include <cmath>

template <typename Derived>
typename Eigen::MatrixBase<Derived>::Scalar& at(Eigen::MatrixBase<Derived>& mat, int row, int col) {
    int num_rows = mat.rows();
    int num_cols = mat.cols();

    if (row < 0) {
        row += num_rows;
    }

    if (col < 0) {
        col += num_cols;
    }

    if (row >= num_rows) {
        row -= num_rows;
    }

    if (row >= num_cols) {
        row -= num_cols;
    }

    if (rows < 0 || row >= num_rows || col < 0 || col >= num_cols) {
        throw std::out_of_range("Index out of range");
    }

    return mat(row, col);
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, 1> atCol(Eigen::MatrixBase<Derived>& mat, int col) {
    int num_rows = mat.rows();
    Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, 1> column(num_rows);

    for (int i = 0; i < num_rows; ++i) {
        column(i) = at(mat, i, col);
    }

    return column;
}

template <typename Derived1, typename Derived2>
double pointPointDistanceSquared(const Eigen::MatrixBase<Derived1>& v1, const Eigen::MatrixBase<Derived2>& v2) {
    assert(v1.size() == v2.size() && "Inputs must have same dimension")
    Eigen::VectorXd diff = v1 - v2;
    return pow(diff.norm(),2);
}

template <typename Derived1, typename Derived2>
double pointPointDistance(const Eigen::MatrixBase<Derived1>& v1, const Eigen::MatrixBase<Derived2>& v2) {
    assert(v1.size() == v2.size() && "Inputs must have same dimension")
    Eigen::VectorXd diff = v1 - v2;
    return diff.norm();
}

template <typename Derived1, typename Derived2>
double weightedPointDistance(const Eigen::MatrixBase<Derived1>& v1, const Eigen::MatrixBase<Derived2>& v2, double weight) {
    assert(v1.size() == v2.size() && "Inputs must have same dimension")
    Eigen::VectorXd diff = v1 - v2 * weight;
    return diff.norm();
}

template <typename Derived>
double originPointDistance(const Eigen::MatrixBase<Derived>& v1) {
    return v1.norm();
}

int calculateIndex(const int& size, int&& index) {
    if (index >= size) {
        index -= size;
    }
    else if (index < 0) {
        index += size;
    }

    return index;
}

int calculateIndex(const int& size, int& index) {
    if (index >= size) {
        index -= size;
    }
    else if (index < 0) {
        index += size;
    }

    return index;
}
