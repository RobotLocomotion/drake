#pragma once

#include <utility>

#include <Eigen/Core>

namespace drake {
namespace math {
/**
 * Rewrite a quadratic form xᵀQx + bᵀx + c to
 * (Rx+d)ᵀ(Rx+d)
 * where
 * RᵀR = Q
 * Rᵀd = b / 2
 * Notice that this decomposition is not unique. For example, with any
 * permutation matrix P, we can define
 * R₁ = P*R
 * d₁ = P*d
 * Then (R₁*x+d₁)ᵀ(R₁*x+d₁) gives the same quadratic form.
 * @param Q The square matrix.
 * @param b The vector containing the linear coefficients.
 * @param c The constatnt term.
 * @param tol We will determine if this quadratic form is always non-negative,
 * by checking the Eigen values of the matrix
 * [Q    b/2]
 * [bᵀ/2   c]
 * are all greater than -tol. @default is 0.
 * @retval (R, d). R and d has the same number of rows. We do not guarantee that
 * R has the same number of rows as Q.
 * @pre 1. The quadratic form is always non-negative, namely the matrix
 *         <pre>
 *         [Q    b/2]
 *         [bᵀ/2   c]
 *         </pre>
 *         is positive semidefinite.
 *      2. `Q` and `b` are of the correct size.
 *      3. `tol` is non-negative.
 * @throw a runtime_error if the precondition is not
 * satisfied.
 */
std::pair<Eigen::MatrixXd, Eigen::MatrixXd>
DecomposePositiveQuadraticForm(const Eigen::Ref<const Eigen::MatrixXd>& Q,
                               const Eigen::Ref<const Eigen::VectorXd>& b,
                               double c, double tol = 0);
}  // namespace math
}  // namespace drake
