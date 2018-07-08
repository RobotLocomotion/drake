#pragma once
#include <Eigen/Core>

namespace drake {
namespace solvers {

/**
  * Finds all integer solutions x to the linear inequalities
  * <pre>
  *                    Ax <= b,
  *                    x <= upper_bound,
  *                    x >= lower_bound.
  * </pre>
  * @param A An (m x n) integer matrix.
  * @param b An (m x 1) integer vector.
  * @param upper_bound A (n x 1) integer vector.
  * @param lower_bound A (n x 1) integer vector.
  * @return A (p x n) matrix whose rows are the solutions.
*/
Eigen::Matrix<int, -1, -1, Eigen::RowMajor> EnumerateIntegerSolutions(
    const Eigen::Ref<const Eigen::MatrixXi>& A,
    const Eigen::Ref<const Eigen::VectorXi>& b,
    const Eigen::Ref<const Eigen::VectorXi>& lower_bound,
    const Eigen::Ref<const Eigen::VectorXi>& upper_bound);
}  // namespace solvers
}  // namespace drake
