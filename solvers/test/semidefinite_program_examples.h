#pragma once

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
namespace test {
/// Test a trivial semidefinite problem.
/// min S(0, 0) + S(1, 1)
/// s.t S(1, 0) = 1
///     S is p.s.d
/// The analytical solution is
/// S = [1 1]
///     [1 1]
void TestTrivialSDP(const MathematicalProgramSolverInterface& solver);

/// Solve a semidefinite programming problem.
/// Find the common Lyapunov function for linear systems
/// xdot = Ai*x
/// The condition is
/// min 0
/// s.t P is positive definite
///     - (Ai'*P + P*Ai) is positive definite
void FindCommonLyapunov(const MathematicalProgramSolverInterface& solver);


/// Given some ellipsoids ℰᵢ : xᵀ*Qi*x + 2 * biᵀ * x <= 1, i = 1, 2, ..., n,
/// find an ellipsoid
/// xᵀ*P*x + 2*cᵀ*x <= 1 as an outer approximation for the union of ellipsoids
/// ℰᵢ.
/// Using s-lemma, the ellipsoid xᵀ*P*x + 2*cᵀ*x <= 1 contains the ellipsoid
/// ℰᵢ : xᵀ*Qi*x + 2*biᵀ*x <= 1, if and only if there exists a scalar si >= 0
/// and (1 - xᵀ*P*x - cᵀ * x) - si*(1 - xᵀ*Qi*x - biᵀ * x) >= 0 ∀x.
/// Namely the matrix
/// [ si*Qi - P    si*bi - c]
/// [si*biᵀ - cᵀ      1 - si]
/// is positive semidefinite.
/// In order to find a tight outer approximation, we choose to minimize the
/// trace of P.
/// The optimiation problem becomes
/// min_{P, c, si} -trace(P)
/// s.t [ si*Qi - P    si*bi - c] is p.s.d
///     [si*biᵀ - cᵀ      1 - si]
///     P is p.s.d
void FindOuterEllipsoid(const MathematicalProgramSolverInterface& solver);

/// Solve an eigen value problem through a semidefinite programming.
/// Minimize the maximum eigen value of a matrix that depends affinely on a
/// variable x
/// min  z
/// s.t z * Identity - x1 * F1 - ... - xn * Fn is p.s.d
///     A * x <= b
///     C * x = d
void SolveEigenvalueProblem(const MathematicalProgramSolverInterface& solver);
}  // namespace test
}  // namespace solvers
}  // namespace drake
