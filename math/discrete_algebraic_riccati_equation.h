#pragma once

#include <Eigen/Core>

namespace drake {
namespace math {

/**
Computes the unique stabilizing solution X to the discrete-time algebraic
Riccati equation:

AᵀXA − X − AᵀXB(BᵀXB + R)⁻¹BᵀXA + Q = 0

@throws std::exception if Q is not symmetric positive semidefinite.
@throws std::exception if R is not symmetric positive definite.
@throws std::exception if (A, B) isn't a stabilizable pair.
@throws std::exception if (A, C) isn't a detectable pair where Q = CᵀC.
*/
Eigen::MatrixXd DiscreteAlgebraicRiccatiEquation(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::MatrixXd>& B,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R);

/**
Computes the unique stabilizing solution X to the discrete-time algebraic
Riccati equation:

AᵀXA − X − (AᵀXB + N)(BᵀXB + R)⁻¹(BᵀXA + Nᵀ) + Q = 0

This is equivalent to solving the original DARE:

A₂ᵀXA₂ − X − A₂ᵀXB(BᵀXB + R)⁻¹BᵀXA₂ + Q₂ = 0

where A₂ and Q₂ are a change of variables:

A₂ = A − BR⁻¹Nᵀ and Q₂ = Q − NR⁻¹Nᵀ

This overload of the DARE is useful for finding the control law uₖ that
minimizes the following cost function subject to xₖ₊₁ = Axₖ + Buₖ.

@verbatim
    ∞ [xₖ]ᵀ[Q  N][xₖ]
J = Σ [uₖ] [Nᵀ R][uₖ] ΔT
   k=0
@endverbatim

This is a more general form of the following. The linear-quadratic regulator
is the feedback control law uₖ that minimizes the following cost function
subject to xₖ₊₁ = Axₖ + Buₖ:

@verbatim
    ∞
J = Σ (xₖᵀQxₖ + uₖᵀRuₖ) ΔT
   k=0
@endverbatim

This can be refactored as:

@verbatim
    ∞ [xₖ]ᵀ[Q 0][xₖ]
J = Σ [uₖ] [0 R][uₖ] ΔT
   k=0
@endverbatim

@throws std::exception if Q₂ is not symmetric positive semidefinite.
@throws std::exception if R is not symmetric positive definite.
@throws std::exception if (A₂, B) isn't a stabilizable pair.
@throws std::exception if (A₂, C) isn't a detectable pair where Q₂ = CᵀC.
*/
Eigen::MatrixXd DiscreteAlgebraicRiccatiEquation(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::MatrixXd>& B,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R,
    const Eigen::Ref<const Eigen::MatrixXd>& N);

}  // namespace math
}  // namespace drake
