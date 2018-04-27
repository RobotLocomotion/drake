#include "drake/solvers/unrevised_lemke_solver.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <utility>
#include <vector>

#include <Eigen/LU>
#include <Eigen/SparseCore>
#include <Eigen/SparseLU>

#include "drake/common/autodiff.h"
#include "drake/common/drake_assert.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/text_logging.h"

using drake::log;

namespace drake {
namespace solvers {

template <>
SolutionResult
    UnrevisedLemkeSolver<Eigen::AutoDiffScalar<drake::Vector1d>>::Solve(
    MathematicalProgram&) const {
  DRAKE_ABORT_MSG("UnrevisedLemkeSolver cannot yet be used in a "
                  "MathematicalProgram while templatized as an AutoDiff");
  return SolutionResult::kUnknownError;
}

template <>
SolutionResult
    UnrevisedLemkeSolver<Eigen::AutoDiffScalar<Eigen::VectorXd>>::Solve(
    MathematicalProgram&) const {
  DRAKE_ABORT_MSG("UnrevisedLemkeSolver cannot yet be used in a "
                  "MathematicalProgram while templatized as an AutoDiff");
  return SolutionResult::kUnknownError;
}

template <typename T>
// NOLINTNEXTLINE(*)  Don't lint old, non-style-compliant code below.
SolutionResult UnrevisedLemkeSolver<T>::Solve(MathematicalProgram& prog) const {
  // This solver imposes restrictions that its problem:
  //
  // (1) Contains only linear complementarity constraints,
  // (2) Has no element of any decision variable appear in more than one
  //     constraint, and
  // (3) Has every element of every decision variable in a constraint.
  //
  // Restriction 1 could reasonably be relaxed by reformulating other
  // constraint types that can be expressed as LCPs (eg, convex QLPs),
  // although this would also entail adding an output stage to convert
  // the LCP results back to the desired form.  See eg. @RussTedrake on
  // how to convert a linear equality constraint of n elements to an
  // LCP of 2n elements.
  //
  // There is no obvious way to relax restriction 2.
  //
  // Restriction 3 could reasonably be relaxed to simply let unbound
  // variables sit at 0.

  DRAKE_ASSERT(prog.generic_constraints().empty());
  DRAKE_ASSERT(prog.generic_costs().empty());
  DRAKE_ASSERT(prog.GetAllLinearConstraints().empty());
  DRAKE_ASSERT(prog.bounding_box_constraints().empty());

  const auto& bindings = prog.linear_complementarity_constraints();

  // Assert that the available LCPs cover the program and no two LCPs cover
  // the same variable.
  for (int i = 0; i < static_cast<int>(prog.num_vars()); ++i) {
    int coverings = 0;
    for (const auto& binding : bindings) {
      if (binding.ContainsVariable(prog.decision_variable(i))) {
        coverings++;
      }
    }
    DRAKE_ASSERT(coverings == 1);
  }

  // Solve each individual LCP, writing the result back to the decision
  // variables through the binding and returning true iff all LCPs are
  // feasible.
  //
  // If any is infeasible, returns false and does not alter the decision
  // variables.
  //

  // We don't actually indicate different results.
  SolverResult solver_result(UnrevisedLemkeSolver::id());

  // Create a dummy variable for the number of pivots used.
  int num_pivots = 0;

  Eigen::VectorXd x_sol(prog.num_vars());
  for (const auto& binding : bindings) {
    Eigen::VectorXd constraint_solution(binding.GetNumElements());
    const std::shared_ptr<LinearComplementarityConstraint> constraint =
        binding.evaluator();
    bool solved = SolveLcpLemke(
        constraint->M(), constraint->q(), &constraint_solution, &num_pivots);
    if (!solved) {
      prog.SetSolverResult(solver_result);
      return SolutionResult::kUnknownError;
    }
    for (int i = 0; i < binding.evaluator()->num_vars(); ++i) {
      const int variable_index =
          prog.FindDecisionVariableIndex(binding.variables()(i));
      x_sol(variable_index) = constraint_solution(i);
    }
    solver_result.set_optimal_cost(0.0);
  }
  solver_result.set_decision_variable_values(x_sol);
  prog.SetSolverResult(solver_result);
  return SolutionResult::kSolutionFound;
}

template <typename T>
bool UnrevisedLemkeSolver<T>::SolveLcpLemke(const MatrixX<T>&,
                                     const VectorX<T>& q, VectorX<T>* z,
                                     int* num_pivots,
                                     const T&) const {
  DRAKE_DEMAND(num_pivots);

  // Note: when the solver returns `false`, z is required to be set to zero.
  z->setZero(q.size());
  return false;
}

template <typename T>
SolverId UnrevisedLemkeSolver<T>::solver_id() const {
  return UnrevisedLemkeSolver::id();
}

template <class T>
SolverId UnrevisedLemkeSolver<T>::id() {
  static const never_destroyed<SolverId> singleton{"Unrevised Lemke"};
  return singleton.access();
}

// Instantiate templates.
template class UnrevisedLemkeSolver<double>;
template class
    solvers::UnrevisedLemkeSolver<Eigen::AutoDiffScalar<drake::Vector1d>>;
template class
    solvers::UnrevisedLemkeSolver<Eigen::AutoDiffScalar<Eigen::VectorXd>>;

}  // namespace solvers
}  // namespace drake
