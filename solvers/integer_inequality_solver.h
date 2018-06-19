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
Eigen::MatrixXi EnumerateIntegerSolutions(const Eigen::MatrixXi & A,
                                const Eigen::MatrixXi & b,
                                const Eigen::VectorXi & lower_bound,
                                const Eigen::VectorXi & upper_bound);

/**
  * Finds all integer solutions x to the linear inequalities
  * <pre>
  *                    Ax <= b, 
  *                    x <= upper_bound,
  *                    x >= lower_bound.
  * </pre>
  * @param A An (m x n) double matrix.
  * @param b An (m x 1) double vector.
  * @param upper_bound A (n x 1) integer vector. 
  * @param lower_bound A (n x 1) integer vector.
  * @return A (p x n) matrix whose rows are the solutions.
*/
Eigen::MatrixXi EnumerateIntegerSolutions(const Eigen::MatrixXd & A,
                                const Eigen::MatrixXd & b,
                                const Eigen::VectorXi & lower_bound,
                                const Eigen::VectorXi & upper_bound);
}  // namespace solvers
}  // namespace drake
