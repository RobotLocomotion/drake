#pragma once

#include <Eigen/Dense>
#include <Eigen/QR>

#include "drake/common/eigen_types.h"

namespace drake {
namespace math {

// TODO(FischerGundlach) Reserve memory and pass it to recusrive function
// calls.

/**
 * @param A A user defined real values matrix.
 * @param Q A user defined real symmetric matrix. Even if Q is non-symmetric,
 * only the upper triangular part will be used. Thus symmetry is implied for all
 * Q.
 * Computes a unique solution X to the continuouse-time Lyapunov equation:
 *
 * @verbatim
 * A'X + XA + Q = 0
 * @endverbatim
 *
 * where @p A is real and square, and @ Q is real, symmetric and of
 * equal size as @ A. If @p A or @p Q are not square or of same size,
 * @throws std::runtime_error("A and Q must be square and of equal size!")
 *
 * As @p Q is required to be symmetric, only the upper triangular part of @p Q
 * is used. Therefore, no tests on symmetry of @p Q are performed.
 *
 * Limitations: Given the Eigenvalues of @p A as λ₁, ..., λ₁, there exists
 * a unique solution if and only if λᵢ + λ̅ ⱼ ≠ 0 ∀ i,j, where λ̅ ⱼ is
 * the complex conjugate of λⱼ. If the solution is not unique, @throws
 * std::runtime_error("Solution is not unique!"). There are no further
 * limitations on the eigenvalues of A.
 * Further, if all lambda_i are negative, and if @p Q is
 * semi-positive definite, then X is also semi-positive definite [1].
 * Therefore, if one searchs for a Lyapunov function V(z) = z'Xz for the stable
 * linear system z_dot = Az, then the solution of the Lyapunov Equation A'X + XA
 * + Q = 0 only returns a valid Lyapunov function if Q is semi-positive
 * definite.
 *
 * The implementation is based on SLICOT routine SB03MD [2]. Note the
 * transformation Q = -C. The complexity of this routine is O(n³).
 * If @p A is larger than 2-by-2, then a Schur factorization is performed. If
 * the factorization failed, a @throw std::runtime_error("Schur
 * factorization failed.")/ is thrown.
 *
 * [1]  Barraud, A.Y., "A numerical algorithm to solve A XA - X = Q," IEEE®
 * Trans. Auto. Contr., AC-22, pp. 883-885, 1977.
 * [2] http://slicot.org/objects/software/shared/doc/SB03MD.html
 *
 */

Eigen::MatrixXd RealContinuousLyapunovEquation(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::MatrixXd>& Q);

namespace internal {

// Subroutines which help special cases. These cases are also called within
// SolveReducedRealContinuousLyapunovFunction.

Vector1d Solve1By1RealContinuousLyapunovEquation(
    const Eigen::Ref<const Vector1d>& A, const Eigen::Ref<const Vector1d>& Q);

Eigen::Matrix2d Solve2By2RealContinuousLyapunovEquation(
    const Eigen::Ref<const Eigen::Matrix2d>& A,
    const Eigen::Ref<const Eigen::Matrix2d>& Q);

// If the problem is larger than in size 2-by-2, than it is reduced into a form
// which can be recursively solved by smaller problems.

Eigen::MatrixXd SolveReducedRealContinuousLyapunovEquation(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::MatrixXd>& Q);

}  // namespace internal
}  // namespace math
}  // namespace drake
