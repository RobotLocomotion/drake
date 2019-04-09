#pragma once

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solver_interface.h"

namespace drake {
namespace solvers {
namespace test {
/**
 * Solves a trivial problem with exponential cone constraint
 * min y*exp(z/y)
 * s.t y + z = 1
 *     y > 0
 */
void ExponentialConeTrivialExample(const SolverInterface& solver, double tol);

/**
 * For a random variable x (assuming that the sample space of x is  {0, 1, 2,
 * 3}), with a given probability p(x), find the other probability q(x), such
 * that the KL divergence KL(p(x) | q(x)) is minimized. The optimal probability
 * is q(x) = p(x).
 */
void MinimizeKLDivergence(const SolverInterface& solver, double tol);

}  // namespace test
}  // namespace solvers
}  // namespace drake
