#pragma once

#include <Eigen/Dense>
#include <Eigen/QR>

#include "drake/common/eigen_types.h"

namespace drake {
namespace math {

// TODO(FischerGundlach) Add complex case.

/**
 * Computes a unique solution to the continuouse-time Lyapunov equation:
 *
 * @verbatim
 * A'X + XA + Q = 0
 * @endverbatim
 *
 * where @param A is real and square, and @param Q is real, symmetric and of
 * equal size as @param A.
 *
 * Limitations: Given the Eigenvalues of @param A as lambda_1, ..., lambda_n,
 * then the solution is unique if and only if lambda_i +
 * complexConjugate(lambda_j) != 0 for all i, j.
 * Further, if all lambda_i are negative, and if @param Q is
 * semi-positive definite, then X is also semi-positive definite [1].
 *
 * The implementation is based on SLICOT routine SB03MD [2]. Note the
 * tranformation
 * Q = -C.
 *
 * [1] Hammarling, S.J., "Numerical solution of the stable, non-negative
 * definite Lyapunov equation," IMA J. Num. Anal., Vol. 2, pp. 303–325, 1982.
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
// which can be recursivly solved by smaller problems.

Eigen::MatrixXd SolveReducedRealContinuousLyapunovEquation(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::MatrixXd>& Q);

}  // namespace internal
}  // namespace math
}  // namespace drake
