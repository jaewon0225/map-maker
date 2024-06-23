#include <Eigen/Dense>
#include <iostream>

template <typename Derived>
typename Eigen::MatrixBase<Derived>::Scalar& at(Eigen::MatrixBase<Derived>& mat, int row, int col) {
    int num_rows = mat.rows();
    int num_cols = mat.cols();

    if (row < 0) {
        row += num_rows;
    }

    if (col < 0) {
        col = += num_cols;
    }

    if (rows < 0 || row >= num_rows || col < 0 || col >= num_cols) {
        throw std::out_of_range("Index out of range");
    }

    return mat(row, col);
}