#pragma once

#include <utility>

#include <Eigen/Core>

namespace drake {
namespace math {
/**
 * For a symmetric positive semidefinite matrix Y, decompose it into XᵀX, where
 * the number of rows in X equals to the rank of Y.
 * Notice that this decomposition is not unique. For any orthonormal matrix U,
 * s.t UᵀU = identity, X_prime = UX also satisfies X_primeᵀX_prime = Y. Here
 * we only return one valid decomposition.
 * @param Y A symmetric positive semidefinite matrix.
 * @param zero_tol We will need to check if some value (for example, the
 * absolute value of Y's eigenvalues) is smaller than zero_tol. If it is, then
 * we deem that value as 0.
 * @retval X. The matrix X satisfies XᵀX = Y and X.rows() = rank(Y).
 * @pre 1. Y is positive semidefinite.
 *      2. zero_tol is non-negative.
 *      @throw std::runtime_error when the pre-conditions are not satisfied.
 */
Eigen::MatrixXd DecomposePSDmatrixIntoXtransposeTimesX(
    const Eigen::Ref<const Eigen::MatrixXd>& Y, double zero_tol);

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
 * @retval (R, d). R and d have the same number of rows. R.cols() == x.rows().
 * The matrix X = [R d] has the same number of rows as the rank of
 * <pre>
 *    [Q    b/2]
 *    [bᵀ/2   c]
 * </pre>
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
