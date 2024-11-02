/**
 * Includes Semidefinite programs, but some of the PSD constraints are on 1x1 or
 * 2x2 matrices, which can be formulated as a bounding box constraint on the
 * scalar, or the second order cone constraints.
 */
#pragma once

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solver_interface.h"

namespace drake {
namespace solvers {
namespace test {
// A trivial semidefinite program
// min S(0, 0)
// s.t S is psd
// where S is a 1x1 psd matrix
// The analytical solution is S = [0]
void TestTrivial1x1SDP(const SolverInterface& solver, double tol,
                       bool check_dual = true);

// A trivial semidefinite program
// min S(0, 0) + S(1, 1)
// s.t S(1, 0) = 1
//     S is psd.
// The analytical solution is
// S = [1 1]
//     [1 1]
void TestTrivial2x2SDP(const SolverInterface& solver, double tol,
                       bool check_dual = true);

// A semidefinite program with both 1x1 psd matrix and 3x3 psd matrix
// min x[1] + x[3] + x[4]
// s.t [x(1)] is psd
//     [x(0) x(1) x(3)]  is psd
//     [x(1) x(2) x(4)]
//     [x(3) x(4) x(5)]
//     x[0] = 9
//     x[2] = 1
//     x[5] = 4
void Test1x1with3x3SDP(const SolverInterface& solver, double tol,
                       bool check_dual = true);

// A semidefinite program with both 2x2 psd matrix and 3x3 psd matrix
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
void Test2x2with3x3SDP(const SolverInterface& solver, double tol,
                       bool check_dual = true);
}  // namespace test
}  // namespace solvers
}  // namespace drake
