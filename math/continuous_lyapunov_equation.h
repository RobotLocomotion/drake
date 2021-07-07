#pragma once

#include <Eigen/Dense>
#include <Eigen/QR>

#include "drake/common/eigen_types.h"

namespace drake {
namespace math {

// TODO(FischerGundlach) Reserve memory and pass it to recursive function
// calls.

/**
 * @param A A user defined real square matrix.
 * @param Q A user defined real symmetric matrix.
 *
 * @pre Q is a symmetric matrix.
 *
 * Computes a unique solution X to the continuous Lyapunov equation: `AᵀX + XA +
 * Q = 0`, where A is real and square, and Q is real, symmetric and of equal
 * size as A.
 * @throws std::exception if A or Q are not square matrices or do not
 * have the same size.
 *
 * Limitations: Given the Eigenvalues of A as λ₁, ..., λₙ, there exists
 * a unique solution if and only if λᵢ + λ̅ⱼ ≠ 0 ∀ i,j, where λ̅ⱼ is
 * the complex conjugate of λⱼ.
 * @throws std::exception if the solution is not unique.
 *
 * There are no further limitations on the eigenvalues of A.
 * Further, if all λᵢ have negative real parts, and if Q is positive
 * semi-definite, then X is also positive semi-definite [1]. Therefore, if one
 * searches for a Lyapunov function V(z) = zᵀXz for the stable linear system
 * ż = Az, then the solution of the Lyapunov Equation `AᵀX + XA + Q = 0` only
 * returns a valid Lyapunov function if Q is positive semi-definite.
 *
 * The implementation is based on SLICOT routine SB03MD [2]. Note the
 * transformation Q = -C. The complexity of this routine is O(n³).
 * If A is larger than 2-by-2, then a Schur factorization is performed.
 * @throws std::exception if Schur factorization failed.
 *
 * A tolerance of ε is used to check if a double variable is equal to zero,
 * where the default value for ε is 1e-10. It has been used to check (1) if λᵢ +
 * λ̅ⱼ = 0, ∀ i,j; (2) if A is a 1-by-1 zero matrix; (3) if A's trace or
 * determinant is 0 when A is a 2-by-2 matrix.
 *
 * [1] Bartels, R.H. and G.W. Stewart, "Solution of the Matrix Equation AX + XB
 * = C," Comm. of the ACM, Vol. 15, No. 9, 1972.
 *
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

// If the problem size is larger than 2-by-2, then it is reduced into a form
// which can be recursively solved by smaller problems.

Eigen::MatrixXd SolveReducedRealContinuousLyapunovEquation(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::MatrixXd>& Q);

}  // namespace internal
}  // namespace math
}  // namespace drake
