#pragma once

#include <Eigen/Dense>
#include <Eigen/QR>

#include "drake/common/eigen_types.h"

namespace drake {
namespace math {

// TODO(FischerGundlach) Add complex case.

/**
 * Computes a unique solution to the discrete-time Lyapunov equation:
 *
 * @verbatim
 * A'XA -X + Q = 0
 * @endverbatim
 *
 * where @param A is real and square, and @param Q is real, symmetric and of
 * equal size as @param A.
 *
 * Limitations: Given the Eigenvalues of @param A as lambda_1, ..., lambda_n,
 * then the solution is unique if and only if lambda_i * lambda_j != 0 for all
 * i, j. [1]
 * Further, if all lambda_i are negative, and if @param Q is
 * semi-positive definite, then X is also semi-positive definite [2].
 *
 * The implementation is based on SLICOT routine SB03MD [2]. Note the
 * tranformation Q = -C.
 *
 * [1]  Barraud, A.Y., "A numerical algorithm to solve A XA - X = Q," IEEE®
 * Trans. Auto. Contr., AC-22, pp. 883-885, 1977.
 * [2] http://slicot.org/objects/software/shared/doc/SB03MD.html
 *
 */

Eigen::MatrixXd RealDiscreteLyapunovEquation(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::MatrixXd>& Q);

namespace internal {

// Subroutines which help special cases.

Vector1d Solve1By1RealDiscreteLyapunovEquation(
    const Eigen::Ref<const Vector1d>& A, const Eigen::Ref<const Vector1d>& Q);

Eigen::Matrix2d Solve2By2RealDiscreteLyapunovEquation(
    const Eigen::Ref<const Eigen::Matrix2d>& A,
    const Eigen::Ref<const Eigen::Matrix2d>& Q);

Eigen::MatrixXd SolveReducedRealDiscreteLyapunovEquation(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::MatrixXd>& Q);

}  // namespace internal
}  // namespace math
}  // namespace drake
