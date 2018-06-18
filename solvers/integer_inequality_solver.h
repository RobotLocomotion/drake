#pragma once
#include <Eigen/Core>

namespace drake {
namespace solvers {

/**
  * Find all solutions x to the linear inequalities
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
Eigen::MatrixXi EnumerateIntegerSolutions(const Eigen::MatrixXi & A,
                                const Eigen::MatrixXi & b,
                                const Eigen::VectorXi & lower_bound,
                                const Eigen::VectorXi & upper_bound);
}  // namespace solvers
}  // namespace drake
