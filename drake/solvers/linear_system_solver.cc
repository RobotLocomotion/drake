#include "drake/solvers/linear_system_solver.h"

#include <cstring>
#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"

namespace drake {
namespace solvers {

bool LinearSystemSolver::available() const { return true; }

SolutionResult LinearSystemSolver::Solve(MathematicalProgram& prog) const {
  size_t num_constraints = 0;
  for (auto const& binding : prog.linear_equality_constraints()) {
    num_constraints += binding.constraint()->A().rows();
  }

  DRAKE_ASSERT(prog.generic_constraints().empty());
  DRAKE_ASSERT(prog.generic_costs().empty());
  DRAKE_ASSERT(prog.quadratic_costs().empty());
  DRAKE_ASSERT(prog.linear_constraints().empty());
  DRAKE_ASSERT(prog.bounding_box_constraints().empty());
  DRAKE_ASSERT(prog.linear_complementarity_constraints().empty());

  Eigen::MatrixXd Aeq = Eigen::MatrixXd::Zero(num_constraints, prog.num_vars());
  // TODO(naveenoid) : use a sparse matrix here?
  Eigen::VectorXd beq(num_constraints);

  size_t constraint_index = 0;
  for (auto const& binding : prog.linear_equality_constraints()) {
    auto const& c = binding.constraint();
    size_t n = c->A().rows();
    for (int i = 0; i < static_cast<int>(binding.GetNumElements()); ++i) {
      size_t variable_index =
          prog.FindDecisionVariableIndex(binding.variables()(i));
      Aeq.block(constraint_index, variable_index, n, 1) = c->A().col(i);
    }
    beq.segment(constraint_index, n) =
        c->lower_bound();  // = c->upper_bound() since it's an equality
    // constraint
    constraint_index += n;
  }

  // least-squares solution
  prog.SetDecisionVariableValues(
      Aeq.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(beq));

  prog.SetSolverResult(solver_type(), 0);
  return SolutionResult::kSolutionFound;
}

}  // namespace solvers
}  // namespace drake
