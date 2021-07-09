#pragma once

#include <cmath>
#include <cstdlib>

#include <Eigen/Dense>

namespace drake {
namespace math {

/// Computes the unique stabilizing solution X to the discrete-time algebraic
/// Riccati equation:
///
/// AᵀXA − X − AᵀXB(BᵀXB + R)⁻¹BᵀXA + Q = 0
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

/// Computes the unique stabilizing solution X to the discrete-time algebraic
/// Riccati equation:
///
/// AᵀXA − X − (AᵀXB + N)(BᵀXB + R)⁻¹(BᵀXA + Nᵀ) + Q = 0
///
/// This is equivalent to solving the original DARE:
///
/// A₂ᵀXA₂ − X − A₂ᵀXB(BᵀXB + R)⁻¹BᵀXA₂ + Q₂ = 0
///
/// where A₂ and Q₂ are a change of variables:
///
/// A₂ = A − BR⁻¹Nᵀ and Q₂ = Q − NR⁻¹Nᵀ
///
/// This overload of the DARE is useful for finding the control law that
/// minimizes the following cost function subject to dx/dt = Ax + Bu.
///
/// @verbatim
///     ∞ [x]ᵀ[Q  N][x]
/// J = ∫ [u] [Nᵀ R][u] dt
///     0
/// @endverbatim
///
/// This is a more general form of the following. The linear-quadratic regulator
/// is the feedback control law that minimizes the following cost function
/// subject to dx/dt = Ax + Bu:
///
/// @verbatim
///     ∞
/// J = ∫ (xᵀQx + uᵀRu) dt
///     0
/// @endverbatim
///
/// This can be refactored as:
///
/// @verbatim
///     ∞ [x]ᵀ[Q 0][x]
/// J = ∫ [u] [0 R][u] dt
///     0
/// @endverbatim
///
/// @throws std::runtime_error if Q − NR⁻¹Nᵀ is not positive semi-definite.
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

