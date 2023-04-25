#pragma once

#include <Eigen/Dense>

namespace drake {
namespace math {

/// Computes the unique stabilizing solution S to the continuous-time algebraic
/// Riccati equation:
///
/// @f[
/// S A + A' S - S B R^{-1} B' S + Q = 0
/// @f]
///
/// @throws std::exception if the Hamiltanoian matrix
/// <pre>
/// ⌈A   BR⁻¹Bᵀ⌉
/// ⌊Q      −Aᵀ⌋
/// </pre>
/// is not invertible.
/// @throws std::exception if R is not positive definite.
/// @note the pair (A, B) should be stabilizable, and (Q, A) should be
/// detectable. For more information, please refer to page 526-527 of Linear
/// Systems by Thomas Kailath.
///
/// Based on the Matrix Sign Function method outlined in this paper:
/// http://www.engr.iupui.edu/~skoskie/ECE684/Riccati_algorithms.pdf
///
Eigen::MatrixXd ContinuousAlgebraicRiccatiEquation(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::MatrixXd>& B,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R);

/// This is functionally the same as
/// ContinuousAlgebraicRiccatiEquation(A, B, Q, R).
/// The Cholesky decomposition of R is passed in instead of R.
Eigen::MatrixXd ContinuousAlgebraicRiccatiEquation(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::MatrixXd>& B,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::LLT<Eigen::MatrixXd>& R_cholesky);

}  // namespace math
}  // namespace drake
