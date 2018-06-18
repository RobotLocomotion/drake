#pragma once
#include <Eigen/Core>

namespace drake{
namespace solvers {

/* Returns a (p x n) matrix whose rows are the integer 
 * vectors x that satisfy the linear equalities
                      Ax <= b, 
                      x <= upper_bound,
                      x >= lower_bound.
 * Inputs are an (m x n) integer matrix A, an (m x 1) integer vector b,
 * and two (n x 1) integer vectors upper_bound and lower_bound.
*/
Eigen::MatrixXi EnumerateIntegerSolutions(const Eigen::MatrixXi & A,
                                const Eigen::MatrixXi & b,
                                const Eigen::VectorXi & lower_bound,
                                const Eigen::VectorXi & upper_bound);
}
}
