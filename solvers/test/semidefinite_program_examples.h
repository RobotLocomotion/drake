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
void TestTrivialSDP(const MathematicalProgramSolverInterface& solver,
                    double tol);

// Solve a semidefinite programming problem.
// Find the common Lyapunov function for linear systems
// xdot = Ai*x
// The condition is
// min 0
// s.t P is positive definite
//     - (Ai'*P + P*Ai) is positive definite
void FindCommonLyapunov(const MathematicalProgramSolverInterface& solver,
                        double tol);

/*
 * Given some ellipsoids ℰᵢ : xᵀQᵢx + 2 bᵢᵀx ≤ 1, i = 1, 2, ..., n, find an
 * ellipsoid xᵀPx + 2cᵀx ≤ 1 as an outer approximation for the union of
 * ellipsoids ℰᵢ.
 *
 * Using s-lemma, the ellipsoid xᵀPx + 2cᵀx ≤ 1 contains the ellipsoid ℰᵢ,
 * if and only if there exists a scalar sᵢ ≥ 0 such that
 *
 * (1 - xᵀPx - cᵀx) - sᵢ(1 - xᵀQᵢx - bᵢᵀx) ≥ 0 ∀x.
 *
 * This is equivalent to requiring that the matrix
 *
 * ⎡sᵢQᵢ - P   sᵢbᵢ - c⎤
 * ⎣sᵢbᵢᵀ - cᵀ   1 - sᵢ⎦
 *
 * is positive semidefinite.
 *
 * In order to find a tight outer approximation, we choose to maximize the
 * trace of P. The optimization problem becomes
 *
 * min_{P, c, si} -trace(P)
 * s.t ⎡sᵢQᵢ - P   sᵢbᵢ - c⎤ is p.s.d
 *     ⎣sᵢbᵢᵀ - cᵀ   1 - sᵢ⎦
 * P is p.s.d
 */
void FindOuterEllipsoid(const MathematicalProgramSolverInterface& solver,
                        double tol);

// Solve an eigen value problem through a semidefinite programming.
// Minimize the maximum eigen value of a matrix that depends affinely on a
// variable x
// min  z
// s.t z * Identity - x1 * F1 - ... - xn * Fn is p.s.d
//     A * x <= b
//     C * x = d
void SolveEigenvalueProblem(const MathematicalProgramSolverInterface& solver,
                            double tol);
}  // namespace test
}  // namespace solvers
}  // namespace drake
