#pragma once

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solver_interface.h"

namespace drake {
namespace solvers {
namespace test {
/**
 * For a random variable x (assuming that the sample space of x is  {0, 1, 2,
 * 3}), with a given probability p(x), find the other probability q(x), such
 * that the KL divergence KL(q(x) | p(x)) and KL(p(x) | q(x)) is minimized. In
 * both cases, the optimal probability is q(x) = p(x).
 */
void MinimizeKLDivergence(const SolverInterface& solver, double tol);

}  // namespace test
}  // namespace solvers
}  // namespace drake
