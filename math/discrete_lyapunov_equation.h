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
 * Computes the unique solution X to the discrete Lyapunov equation: `AᵀXA - X +
 * Q = 0`, where A is real and square, and Q is real, symmetric and of equal
 * size as A.
 * @throws std::runtime_error if A or Q are not square matrices or do not
 * have the same size.
 *
 * Limitations: Given the Eigenvalues of A as λ₁, ..., λₙ, there exists
 * a unique solution if and only if λᵢ * λⱼ ≠ 1 ∀ i,j and λᵢ ≠ ±1, ∀ i [1].
 * @throws std::runtime_error if the solution is not unique.[3]
 *
 * There are no further limitations on the eigenvalues of A.
 * Further, if |λᵢ|<1, ∀ i, and if Q is
 * positive semi-definite, then X is also positive semi-definite [2].
 * Therefore, if one searches for a Lyapunov function V(z) = zᵀXz for the stable
 * linear system zₙ₊₁ = Azₙ, then the solution of the Lyapunov Equation `AᵀXA -
 * X + Q = 0` only returns a valid Lyapunov function if Q is positive
 * semi-definite.
 *
 * The implementation is based on SLICOT routine SB03MD [2]. Note the
 * transformation Q = -C. The complexity of this routine is O(n³).
 * If A is larger than 2-by-2, then a Schur factorization is performed.
 * @throws std::runtime_error if Schur factorization fails.
 *
 * A tolerance of ε is used to check if a double variable is equal to zero,
 * where the default value for ε is 1e-10. It has been used to check (1) if λᵢ =
 * ±1 ∀ i; (2) if λᵢ * λⱼ = 1, i ≠ j.
 *
 * [1]  Barraud, A.Y., "A numerical algorithm to solve AᵀXA - X = Q," IEEE®
 * Trans. Auto. Contr., AC-22, pp. 883-885, 1977.
 *
 * [2] http://slicot.org/objects/software/shared/doc/SB03MD.html
 *
 * [3] https://www.mathworks.com/help/control/ref/dlyap.html
 *
 */

Eigen::MatrixXd RealDiscreteLyapunovEquation(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::MatrixXd>& Q);

namespace internal {

// Subroutines which help special cases. These cases are also called within
// SolveReducedRealContinuousLyapunovFunction.

Vector1d Solve1By1RealDiscreteLyapunovEquation(
    const Eigen::Ref<const Vector1d>& A, const Eigen::Ref<const Vector1d>& Q);

Eigen::Matrix2d Solve2By2RealDiscreteLyapunovEquation(
    const Eigen::Ref<const Eigen::Matrix2d>& A,
    const Eigen::Ref<const Eigen::Matrix2d>& Q);

// If the problem is larger than in size 2-by-2, than it is reduced into a form
// which can be recursively solved by smaller problems.

Eigen::MatrixXd SolveReducedRealDiscreteLyapunovEquation(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::MatrixXd>& Q);

}  // namespace internal
}  // namespace math
}  // namespace drake
