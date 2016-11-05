#include "drake/solvers/equality_constrained_qp_solver.h"

#include <cstring>
#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

bool EqualityConstrainedQPSolver::available() const { return true; }

SolutionResult EqualityConstrainedQPSolver::Solve(
    MathematicalProgram& prog) const {
  // Given a QP with equality constraints, we can use the KKT conditions
  // for the optimal solution to write a linear system that produces
  // our answer.
  // (see https://www.math.uh.edu/~rohop/fall_06/Chapter3.pdf)
  // Does not explicitly handle undertermined cases.

  // TODO(gizatt@mit.edu): This way of detecting whether our
  // constraints are satisfied will scale poorly as we add
  // more constraint and cost types, and more solvers, as
  // a change to any of these accessors or addition of new
  // cost or constraint types will require editing this check
  // in every solver.
  DRAKE_ASSERT(prog.generic_constraints().empty());
  DRAKE_ASSERT(prog.generic_costs().empty());
  DRAKE_ASSERT(prog.linear_constraints().empty());
  DRAKE_ASSERT(prog.bounding_box_constraints().empty());
  DRAKE_ASSERT(prog.linear_complementarity_constraints().empty());

  size_t num_constraints = 0;
  for (auto const& binding : prog.linear_equality_constraints()) {
    num_constraints += binding.constraint()->A().rows();
  }

  // The expanded problem introduces a lagragian multiplier for each
  // linear equality constraint.
  size_t num_full_vars = prog.num_vars() + num_constraints;
  Eigen::MatrixXd A_full = Eigen::MatrixXd::Zero(num_full_vars, num_full_vars);
  Eigen::VectorXd b_full = Eigen::VectorXd::Zero(num_full_vars);

  // Assemble the A and b matrices -- first by summing over
  // quadratic costs ...
  for (auto const& binding : prog.quadratic_costs()) {
    size_t index = 0;
    const auto& Q = binding.constraint()->Q();
    const auto& b = binding.constraint()->b();
    for (const DecisionVariableMatrix& v : binding.variable_list()) {
      int num_v_variables = v.NumberOfVariables();
      for (int i = 0; i < num_v_variables; ++i) {
        for (int j = 0; j < num_v_variables; ++j) {
          A_full(v.index(i), v.index(j)) += Q(index + i, index + j);
        }
        b_full(v.index(i)) -= b(index + i);
      }
      index += num_v_variables;
    }
  }

  // ... and then the lagrangian multiplier penalty entries:
  size_t constraint_index = prog.num_vars();
  for (auto const& binding : prog.linear_equality_constraints()) {
    auto const& c = binding.constraint();
    size_t n = c->A().rows();
    size_t var_index = 0;
    for (const DecisionVariableMatrix& v : binding.variable_list()) {
      int num_v_variables = v.NumberOfVariables();
      for (int i = 0; i < num_v_variables; ++i) {
        A_full.block(constraint_index, v.index(i), n, 1) =
            c->A().col(var_index + i);
        A_full.block(v.index(i), constraint_index, 1, n) =
            (c->A().col(var_index + i)).transpose();
      }
      var_index += num_v_variables;
    }
    b_full.segment(constraint_index, n) =
        c->lower_bound().segment(0, n);  // = c->upper_bound() since it's
                                         //  an equality constraint
    constraint_index += n;
  }

  // Compute the least-squares solution.
  Eigen::VectorXd sol =
      A_full.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b_full);
  prog.SetDecisionVariableValues(sol.segment(0, prog.num_vars()));

  prog.SetSolverResult(SolverName(), 0);
  return SolutionResult::kSolutionFound;
}

}  // namespace solvers
}  // namespace drake
