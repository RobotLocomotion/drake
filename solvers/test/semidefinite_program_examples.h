#pragma once

#include <optional>

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solver_interface.h"

namespace drake {
namespace solvers {
namespace test {
/// Test a trivial semidefinite problem.
/// min S(0, 0) + S(1, 1) + S(2, 2)
/// s.t S(1, 0) = 1
///     S is p.s.d
/// The analytical solution is
/// S = [1 1 0]
///     [1 1 0]
///     [0 0 0]
void TestTrivialSDP(const SolverInterface& solver, double tol);

// Solve a semidefinite programming problem.
// Find the common Lyapunov function for linear systems
// xdot = Ai*x
// The condition is
// min 0
// s.t P is positive definite
//     - (Ai'*P + P*Ai) is positive definite
void FindCommonLyapunov(const SolverInterface& solver,
                        const std::optional<SolverOptions>& solver_options,
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
void FindOuterEllipsoid(const SolverInterface& solver,
                        const std::optional<SolverOptions>& solver_options,
                        double tol);

// Solve an eigen value problem through a semidefinite programming.
// Minimize the maximum eigen value of a matrix that depends affinely on a
// variable x
// min  z
// s.t z * Identity - x1 * F1 - ... - xn * Fn is p.s.d
//     A * x <= b
//     C * x = d
void SolveEigenvalueProblem(const SolverInterface& solver,
                            const std::optional<SolverOptions>& solver_options,
                            double tol, bool check_dual);

/// Solve an SDP with a second order cone constraint. This example is taken from
/// https://docs.mosek.com/11.1/capi/tutorial-sdo-shared.html
void SolveSDPwithSecondOrderConeExample1(const SolverInterface& solver,
                                         double tol);

/** Solve an SDP with second order cone constraints. Notice that the variables
 * appear in the second order cone constraints appear also in the positive
 * semidefinite constraint.
 * min X(0, 0) + X(1, 1) + x(0)
 * s.t X(0, 0) + 2 * X(1, 1) + X(2, 2) + 3 * x(0) = 3
 *     X(0, 0) >= sqrt((X(1, 1) + x(0))² + (X(1, 1) + X(2, 2))²)
 *     X(1, 0) + X(2, 1) = 1
 *     X is psd, x(0) >= 0
 */
void SolveSDPwithSecondOrderConeExample2(const SolverInterface& solver,
                                         double tol);

/** Solve an SDP with two PSD constraint, where each PSD constraint has
 * duplicate entries and the two PSD matrix share a common variables.
 * min 2 * x0 + x2 + x5
 * s.t [x0 x1 x3] is psd
 *     [x1 x0 x4]
 *     [x3 x4 x5]
 *
 *     [x0 x2 x3] is psd
 *     [x2 x0 x4]
 *     [x3 x4 x5]
 *     x1 == 1
 * The optimal solution will be x = (1, 1, -1, 0, 0, 0).
 */
void SolveSDPwithOverlappingVariables(const SolverInterface& solver,
                                      double tol);

/** Solve an SDP with quadratic cost and two PSD constraints, where each PSD
 * constraint has duplicate entries and the two PSD matrix share a common
 * variables.
 * min x0² + 2*x0 + x2 + x5
 * s.t ⎡x0 x1 x3⎤ is psd
 *     |x1 x0 x4|
 *     ⎣x3 x4 x5⎦
 *
 *     ⎡x0 x2 x3⎤ is psd
 *     |x2 x0 x4|
 *     ⎣x3 x4 x5⎦
 *
 *     x1 == 1
 *
 * The optimal solution will be x = (1, 1, -1, 0, 0, 0).
 */
void SolveSDPwithQuadraticCosts(const SolverInterface& solver, double tol);

/**
 * Test a simple SDP with only PSD constraint and bounding box constraint.
 * min x1 + x5
 * s.t ⎡x0 x1 x3⎤ is psd
 *     |x1 x2 x4|
 *     ⎣x3 x4 x5⎦
 *     x0 <= 4
 *     x2 <= 1
 */
void TestSDPDualSolution1(const SolverInterface& solver, double tol,
                          double complementarity_tol = 5E-7);

// A trivial semidefinite program
// min S(0, 0)
// s.t S is psd
// where S is a 1x1 psd matrix
// The analytical solution is S = [0]
void TestTrivial1x1SDP(const SolverInterface& solver, double primal_tol,
                       bool check_dual = true, double dual_tol = 1E-5);

// A trivial semidefinite program
// min S(0, 0) + S(1, 1)
// s.t S(1, 0) = 1
//     S is psd.
// The analytical solution is
// S = [1 1]
//     [1 1]
void TestTrivial2x2SDP(const SolverInterface& solver, double primal_tol,
                       bool check_dual = true, double dual_tol = 1E-5);

// A semidefinite program with both a 1x1 psd matrix and a 3x3 psd matrix
// min x[1] + x[3] + x[4]
// s.t [x(1)] is psd
//     [x(0) x(1) x(3)]  is psd
//     [x(1) x(2) x(4)]
//     [x(3) x(4) x(5)]
//     x[0] = 9
//     x[2] = 1
//     x[5] = 4
void Test1x1with3x3SDP(const SolverInterface& solver, double primal_tol,
                       bool check_dual = true, double dual_tol = 1E-5);

// A semidefinite program with both a 2x2 psd matrix and a 3x3 psd matrix
// min x[1] + x[2] + x[4]
// s.t [x[0] x[1]] is psd
//     [x[1] x[2]]
//
//     [x[0] x[1] x[2]]
//     [x[1] x[3] x[4]] is psd
//     [x[2] x[4] x[2]]
//     x[0] = 1
//     x[3] = 1
// Notice that the psd matrices share variables.
void Test2x2with3x3SDP(const SolverInterface& solver, double primal_tol,
                       bool check_dual = true, double dual_tol = 1E-5);

// Test LMI constraints on 1x1 matrices (scalar).
// min x(0) + x(1)
// s.t [1] + x(0) * [2] + x(1) * [-3] is psd
//     [-3] + x(0) * [-1] + x(1) * [-3] is psd
//     [4] + x(0) * [1] + x(1) * [2] is psd
void TestTrivial1x1LMI(const SolverInterface& solver, double primal_tol,
                       bool check_dual, double dual_tol);

// Test LMI constraints on 2x2 matrices.
// min x(0)
// s.t [1, 1] + [2, 2] * x(0) is psd.
//     [1, -2]  [2, 8]
void Test2x2LMI(const SolverInterface& solver, double primal_tol,
                bool check_dual, double dual_tol);

// min trace(X)
// s.t X[0, 1] >= 1
//     X is psd
//     X is hankel.
// Since X is hankel, some of its entries contain repeated variables. This test
// is reported from github issue
// https://github.com/RobotLocomotion/drake/issues/22158
void TestHankel(const SolverInterface& solver, double primal_tol,
                bool check_dual, double dual_tol);
}  // namespace test
}  // namespace solvers
}  // namespace drake
