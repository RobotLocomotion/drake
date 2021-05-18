#include "drake/solvers/linear_system_solver.h"

#include <cstring>
#include <initializer_list>
#include <limits>
#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/text_logging.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

LinearSystemSolver::LinearSystemSolver()
    : SolverBase(&id, &is_available, &is_enabled, &ProgramAttributesSatisfied,
                 &UnsatisfiedProgramAttributes) {}

LinearSystemSolver::~LinearSystemSolver() = default;

bool LinearSystemSolver::is_available() { return true; }

bool LinearSystemSolver::is_enabled() { return true; }

void LinearSystemSolver::DoSolve(
    const MathematicalProgram& prog,
    const Eigen::VectorXd& initial_guess,
    const SolverOptions& merged_options,
    MathematicalProgramResult* result) const {
  if (!prog.GetVariableScaling().empty()) {
    static const logging::Warn log_once(
      "LinearSystemSolver doesn't support the feature of variable scaling.");
  }

  // The initial guess doesn't help us, and we don't offer any tuning options.
  unused(initial_guess, merged_options);
  size_t num_constraints = 0;
  for (auto const& binding : prog.linear_equality_constraints()) {
    num_constraints += binding.evaluator()->A().rows();
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
    auto const& c = binding.evaluator();
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
  const Eigen::VectorXd least_square_sol =
      Aeq.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(beq);

  result->set_x_val(least_square_sol);
  if (beq.isApprox(Aeq * least_square_sol)) {
    result->set_optimal_cost(0.);
    result->set_solution_result(SolutionResult::kSolutionFound);
  } else {
    result->set_optimal_cost(MathematicalProgram::kGlobalInfeasibleCost);
    result->set_solution_result(SolutionResult::kInfeasibleConstraints);
  }
}

SolverId LinearSystemSolver::id() {
  static const never_destroyed<SolverId> singleton{"Linear system"};
  return singleton.access();
}

namespace {
// If the program is compatible with this solver, returns true and clears the
// explanation.  Otherwise, returns false and sets the explanation.  In either
// case, the explanation can be nullptr in which case it is ignored.
bool CheckAttributes(
      const MathematicalProgram& prog,
      std::string* explanation) {
  static const never_destroyed<ProgramAttributes> solver_capabilities(
      std::initializer_list<ProgramAttribute>{
          ProgramAttribute::kLinearEqualityConstraint});
  const ProgramAttributes& required_capabilities = prog.required_capabilities();
  const bool capabilities_match = AreRequiredAttributesSupported(
      required_capabilities, solver_capabilities.access(), explanation);
  if (!capabilities_match || required_capabilities.empty()) {
    if (explanation) {
      if (required_capabilities.empty()) {
        *explanation =
          " a LinearEqualityConstraint is required but has not beed declared";
      }
      *explanation = fmt::format(
          "LinearSystemSolver is unable to solve because {}.", *explanation);
    }
    return false;
  }
  if (explanation) {
    explanation->clear();
  }
  return true;
}
}  // namespace

bool LinearSystemSolver::ProgramAttributesSatisfied(
    const MathematicalProgram& prog) {
  return CheckAttributes(prog, nullptr);
}

std::string LinearSystemSolver::UnsatisfiedProgramAttributes(
    const MathematicalProgram& prog) {
  std::string explanation;
  CheckAttributes(prog, &explanation);
  return explanation;
}

}  // namespace solvers
}  // namespace drake
