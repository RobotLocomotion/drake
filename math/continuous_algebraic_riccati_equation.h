#pragma once

#include <Eigen/Dense>

namespace drake {
namespace math {

/// Computes the unique stabilizing solution X to the continuous-time algebraic
/// Riccati equation:
///
/// @verbatim
///  S'A + A'S - S B inv(R) B' S + Q = 0
/// @endverbatim
///
/// @throws std::runtime_error if R is not positive definite.
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
