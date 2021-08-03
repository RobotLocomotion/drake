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
 * @throws std::exception when the pre-conditions are not satisfied.
 * @note We only use the lower triangular part of Y.
 */
Eigen::MatrixXd DecomposePSDmatrixIntoXtransposeTimesX(
    const Eigen::Ref<const Eigen::MatrixXd>& Y, double zero_tol);

/**
 * Rewrite a quadratic form xᵀQx + bᵀx + c to
 * (Rx+d)ᵀ(Rx+d)
 * where
 * <pre>
 * RᵀR = Q
 * Rᵀd = b / 2
 * dᵀd = c
 * </pre>
 *
 * This decomposition requires the matrix
 * <pre>
 * ⌈Q     b/2⌉
 * ⌊bᵀ/2    c⌋
 * </pre>
 * to be positive semidefinite.
 *
 * We return R and d with the minimal number of rows, namely the rows of R
 * and d equal to the rank of the matrix
 * <pre>
 * ⌈Q     b/2⌉
 * ⌊bᵀ/2    c⌋
 * </pre>
 *
 * Notice that R might have more rows than Q, For example, the quadratic
 * expression x² + 2x + 5 =(x+1)² + 2², it can be decomposed as
 * <pre>
 * ⎛⌈1⌉ * x + ⌈1⌉⎞ᵀ * ⎛⌈1⌉ * x + ⌈1⌉⎞
 * ⎝⌊0⌋       ⌊2⌋⎠    ⎝⌊0⌋       ⌊2⌋⎠
 * </pre>
 * Here R has 2 rows while Q only has 1 row.
 *
 * On the other hand the quadratic expression x² + 2x + 1 can be decomposed
 * as (x+1) * (x+1), where R has 1 row, same as Q.
 *
 * Also notice that this decomposition is not unique. For example, with any
 * permutation matrix P, we can define
 * <pre>
 * R₁ = P*R
 * d₁ = P*d
 * </pre>
 * Then (R₁*x+d₁)ᵀ(R₁*x+d₁) gives the same quadratic form.
 * @param Q The square matrix.
 * @param b The vector containing the linear coefficients.
 * @param c The constant term.
 * @param tol We will determine if this quadratic form is always non-negative,
 * by checking the Eigen values of the matrix
 * [Q    b/2]
 * [bᵀ/2   c]
 * are all greater than -tol. @default is 0.
 * @retval (R, d). R and d have the same number of rows. R.cols() == x.rows().
 * R.rows() equals to the rank of the matrix
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
 * @throws std::exception if the precondition is not satisfied.
 */
std::pair<Eigen::MatrixXd, Eigen::MatrixXd>
DecomposePositiveQuadraticForm(const Eigen::Ref<const Eigen::MatrixXd>& Q,
                               const Eigen::Ref<const Eigen::VectorXd>& b,
                               double c, double tol = 0);

/** Given two quadratic forms, x'Sx > 0 and x'Px, (with P symmetric and full
 * rank), finds a change of variables x = Ty, which simultaneously diagonalizes
 * both forms (as inspired by "balanced truncation" in model-order reduction
 * [1]).  In this note, we use abs(M) to indicate the elementwise absolute
 * value.
 *
 * Adapting from [1], we observe that there is a family of coordinate systems
 * that can simultaneously diagonalize T'ST and T'PT.  Using D to denote a
 * diagonal matrix, we call the result S-normal if T'ST = I and abs(T'PT) = D⁻²,
 * call it P-normal if T'ST = D² and abs(T'PT) = I, and call it "balanced" if
 * T'ST = D and abs(T'PT) = D⁻¹. Note that if P > 0, then T'PT = D⁻¹.
 *
 * We find x=Ty such that T'ST = D and abs(T'PT) = D⁻¹, where D is diagonal. The
 * recipe is:
 * - Factorize S = LL', and choose R=L⁻¹.
 * - Take svd(RPR') = UΣV', and note that U=V for positive definite matrices,
 *     and V is U up to a sign flip of the singular vectors for all symmetric
 *     matrices.
 * - Choose T = R'U Σ^{-1/4}, where the matrix exponent can be taken elementwise
 *   because Σ is diagonal. This gives T'ST = Σ^{-1/2} (by using U'U=I), and
 *   abs(T'PT) = Σ^{1/2}.  If P > 0, then T'PT = Σ^{1/2}.
 *
 * Note that the numerical "balancing" can address the absolute scaling of the
 * quadratic forms, but not the relative scaling.  To understand this, consider
 * the scalar case: we have two quadratic functions, sx² and px², with s>0, p>0.
 * We'd like to choose x=Ty so that sT²y² and pT²y² are "balanced" (we'd like
 * them both to be close to y²).  We'll choose T=p^{-1/4}s^{-1/4}, which gives
 * sx² = sqrt(s/p)y², and px² = sqrt(p/s)y². For instance if s=1e8 and p=1e8,
 * then t=1e-4 and st^2 = pt^2 = 1.  But if s=10, p=1e7, then t=0.01, and st^2 =
 * 1e-3, pt^2 = 1e3.
 *
 * In the matrix case, the absolute scaling is important -- it ensures that the
 * two quadratic forms have the same matrix condition number and makes them as
 * close as possible to 1. Besides absolute scaling, in the matrix case the
 * balancing transform diagonalizes both quadratic forms.
 *
 * [1] B. Moore, “Principal component analysis in linear systems:
 * Controllability, observability, and model reduction,” IEEE Trans. Automat.
 * Contr., vol. 26, no. 1, pp. 17–32, Feb. 1981.
 */
Eigen::MatrixXd BalanceQuadraticForms(
    const Eigen::Ref<const Eigen::MatrixXd>& S,
    const Eigen::Ref<const Eigen::MatrixXd>& P);

}  // namespace math
}  // namespace drake
