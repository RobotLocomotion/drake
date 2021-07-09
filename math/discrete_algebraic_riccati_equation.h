#pragma once

#include <cmath>
#include <cstdlib>

#include <Eigen/Dense>

namespace drake {
namespace math {

/// Computes the unique stabilizing solution X to the discrete-time algebraic
/// Riccati equation:
///
/// @f[
/// AᵀXA - X - AᵀXB(BᵀXB + R)^{-1}BᵀXA + Q = 0
/// @f]
///
/// @throws std::exception if Q is not positive semi-definite.
/// @throws std::exception if R is not positive definite.
///
/// Based on the Schur Vector approach outlined in this paper:
/// "On the Numerical Solution of the Discrete-Time Algebraic Riccati Equation"
/// by Thrasyvoulos Pappas, Alan J. Laub, and Nils R. Sandell
///
Eigen::MatrixXd DiscreteAlgebraicRiccatiEquation(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::MatrixXd>& B,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R);

/// DiscreteAlgebraicRiccatiEquation function
/// computes the unique stabilizing solution X to the discrete-time algebraic
/// Riccati equation:
/// @f[
/// AᵀXA - X - (AᵀXB + N)(BᵀXB + R)^{-1}(BᵀXA + Nᵀ) + Q = 0
/// @f]
///
/// This is equivalent to solving the original DARE:
/// @f[
/// A_2ᵀXA_2 - X - A_2ᵀXB(BᵀXB + R)^{-1}BᵀXA_2 + Q_2 = 0
/// @f]
///
/// where A_2 and Q_2 are a change of variables:
/// @f[
/// A_2 = A - BR^{-1}Nᵀ and Q_2 = Q - NR^{-1}Nᵀ
/// @f]
///
/// This overload of the DARE is useful for finding the control law that
/// minimizes the following cost function subject to dx/dt = Ax + Bu.
/// @f[
/// J = \int\limits_0^\infty
/// \begin{bmatrix}
///   x \\
///   u
/// \end{bmatrix}^\mathsf{T}
/// \begin{bmatrix}
///   Q & N \\
///   N^\mathsf{T} & R
/// \end{bmatrix}
/// \begin{bmatrix}
///   x \\
///   u
/// \end{bmatrix} dt
/// @f]
///
/// This is a more general form of the following. The linear-quadratic regulator
/// is the feedback control law that minimizes the following cost function
/// subject to dx/dt = Ax + Bu:
/// @f[
/// J = \int\limits_0^\infty (xᵀQx + uᵀRu) dt
/// @f]
///
/// This can be refactored as:
/// @f[
/// J = \int\limits_0^\infty
/// \begin{bmatrix}
///   x \\
///   u
/// \end{bmatrix}^\mathsf{T}
/// \begin{bmatrix}
///   Q & 0 \\
///   0 & R
/// \end{bmatrix}
/// \begin{bmatrix}
///   x \\
///   u
/// \end{bmatrix} dt
/// @f]
///
/// @throws std::runtime_error if Q - NR^{-1}Nᵀ is not positive semi-definite.
/// @throws std::runtime_error if R is not positive definite.
///
Eigen::MatrixXd DiscreteAlgebraicRiccatiEquation(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::MatrixXd>& B,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R,
    const Eigen::Ref<const Eigen::MatrixXd>& N);
}  // namespace math
}  // namespace drake

