#include "drake/solvers/projected_gradient_descent_solver.h"

#include <drake/solvers/choose_best_solver.h>

#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace solvers {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::VectorXi;

namespace {
constexpr const char kConvergenceTolOptionName[] = "ConvergenceTol";
constexpr const char kFeasibilityTolOptionName[] = "FeasibilityTol";
constexpr const char kMaxIterationsOptionName[] = "MaxIterations";
constexpr const char kBacktrackingCOptionName[] = "BacktrackingC";
constexpr const char kBacktrackingTauOptionName[] = "BacktrackingTau";
constexpr const char kBacktrackingAlpha0OptionName[] = "BacktrackingAlpha0";

struct KnownOptions {
  double convergence_tol{1e-12};
  double feasibility_tol{1e-6};
  int max_iterations{1000};
  double backtracking_c{0.5};
  double backtracking_tau{0.5};
  double backtracking_alpha_0{0.1};
};

void Serialize(internal::SpecificOptions* archive,
               // NOLINTNEXTLINE(runtime/references) to match Serialize concept.
               KnownOptions& options) {
  archive->Visit(
      MakeNameValue(kConvergenceTolOptionName, &options.convergence_tol));
  archive->Visit(
      MakeNameValue(kFeasibilityTolOptionName, &options.feasibility_tol));
  archive->Visit(
      MakeNameValue(kMaxIterationsOptionName, &options.max_iterations));
  archive->Visit(
      MakeNameValue(kBacktrackingCOptionName, &options.backtracking_c));
  archive->Visit(
      MakeNameValue(kBacktrackingTauOptionName, &options.backtracking_tau));
  archive->Visit(MakeNameValue(kBacktrackingAlpha0OptionName,
                               &options.backtracking_alpha_0));
}

KnownOptions ParseOptions(internal::SpecificOptions* options) {
  KnownOptions result;
  options->CopyToSerializableStruct(&result);
  if (result.convergence_tol <= 0) {
    throw std::invalid_argument(
        "ConvergenceTol should be a non-negative number.");
  }
  if (result.feasibility_tol <= 0) {
    throw std::invalid_argument(
        "FeasibilityTol should be a non-negative number.");
  }
  if (result.max_iterations < 1) {
    throw std::invalid_argument("MaxIterations must be at least one.");
  }
  if (result.backtracking_c >= 1.0 || result.backtracking_c <= 0.0) {
    throw std::invalid_argument("BacktrackingC must be between 0 and 1.");
  }
  if (result.backtracking_tau >= 1.0 || result.backtracking_tau <= 0.0) {
    throw std::invalid_argument("BacktrackingTau must be between 0 and 1.");
  }
  return result;
}

void ConvertQuadraticErrorCost(const VectorXd& x_desired, const MatrixXd& Q,
                               VectorXd* b, double* c) {
  *b = -Q * x_desired;
  *c = x_desired.dot(Q * x_desired);
}

}  // namespace

ProjectedGradientDescentSolver::ProjectedGradientDescentSolver()
    : SolverBase(id(), &is_available, &is_enabled, &ProgramAttributesSatisfied),
      custom_gradient_function_(std::nullopt),
      custom_projection_function_(std::nullopt),
      projection_solver_interface_(nullptr) {}
ProjectedGradientDescentSolver::~ProjectedGradientDescentSolver() = default;

void ProjectedGradientDescentSolver::DoSolve2(
    const MathematicalProgram& prog, const VectorXd& initial_guess,
    internal::SpecificOptions* options,
    MathematicalProgramResult* result) const {
  const KnownOptions parsed_options = ParseOptions(options);

  VectorXd x_current = initial_guess;
  for (int i = 0; i < x_current.size(); ++i) {
    if (std::isnan(x_current[i])) {
      x_current[i] = 0.0;
    }
  }

  // First, we have to build some machinery to quickly find which variables in
  // prog.decision_variables() each Binding<Cost> and Binding<Constraint> need.
  std::unordered_map<symbolic::Variable::Id, int> variable_to_index;
  for (int i = 0; i < prog.decision_variables().size(); ++i) {
    variable_to_index.emplace(prog.decision_variables()[i].get_id(), i);
  }

  std::vector<Binding<Cost>> costs = prog.GetAllCosts();
  std::vector<VectorXi> cost_variable_indices(costs.size());
  for (int i = 0; i < ssize(costs); ++i) {
    const auto& binding_variables = costs[i].variables();
    cost_variable_indices[i] = VectorXi::Zero(binding_variables.size());
    for (int j = 0; j < binding_variables.size(); ++j) {
      auto iterator = variable_to_index.find(binding_variables[j].get_id());
      DRAKE_DEMAND(iterator != variable_to_index.end());
      cost_variable_indices[i][j] = iterator->second;
    }
  }

  std::vector<Binding<Constraint>> constraints = prog.GetAllConstraints();
  std::vector<VectorXi> constraint_variable_indices(constraints.size());
  for (int i = 0; i < ssize(constraints); ++i) {
    const auto& binding_variables = constraints[i].variables();
    constraint_variable_indices[i] = VectorXi::Zero(binding_variables.size());
    for (int j = 0; j < binding_variables.size(); ++j) {
      auto iterator = variable_to_index.find(binding_variables[j].get_id());
      DRAKE_DEMAND(iterator != variable_to_index.end());
      constraint_variable_indices[i][j] = iterator->second;
    }
  }

  // Next, we construct an auxiliary MathematicalProgram, for use in the
  // projection step.
  std::unique_ptr<MathematicalProgram> projection_prog_ptr = prog.Clone();
  for (const auto& cost : costs) {
    projection_prog_ptr->RemoveCost(cost);
  }
  MatrixXd projection_cost_Q = MatrixXd::Identity(
      projection_prog_ptr->num_vars(), projection_prog_ptr->num_vars());
  VectorXd projection_cost_b;
  double projection_cost_c;
  ConvertQuadraticErrorCost(x_current, projection_cost_Q, &projection_cost_b,
                            &projection_cost_c);
  auto projection_cost = projection_prog_ptr->AddQuadraticCost(
      2 * projection_cost_Q, projection_cost_b, projection_cost_c,
      projection_prog_ptr->decision_variables());

  // Here, we create two lambdas, which we can evaluate to obtain the descent
  // direction and projection.
  std::function<double(const VectorXd&)> cost_function =
      [&](const VectorXd& x) {
        double cost = 0.0;
        for (int i = 0; i < ssize(costs); ++i) {
          VectorXd x_relevant(cost_variable_indices[i].size());
          for (int j = 0; j < cost_variable_indices[i].size(); ++j) {
            x_relevant[j] = x[cost_variable_indices[i][j]];
          }
          VectorXd y_output;
          costs[i].evaluator()->Eval(x_relevant, &y_output);
          DRAKE_THROW_UNLESS(y_output.size() == 1);
          cost += y_output[0];
        }
        return cost;
      };
  std::function<VectorXd(const VectorXd&)> gradient_function =
      custom_gradient_function_.value_or([&](const VectorXd& x) {
        // TODO(cohnt): Switch to using
        // MathematicalProgram::EvalBindingsAtInitialGuess?
        AutoDiffVecXd x_ad = math::InitializeAutoDiff(x);
        VectorXd gradient = VectorXd::Zero(x.size());
        for (int i = 0; i < ssize(costs); ++i) {
          AutoDiffVecXd x_relevant(cost_variable_indices[i].size());
          for (int j = 0; j < cost_variable_indices[i].size(); ++j) {
            x_relevant[j] = x_ad[cost_variable_indices[i][j]];
          }
          AutoDiffVecXd y_relevant;
          costs[i].evaluator()->Eval(x_relevant, &y_relevant);
          VectorXd gradient_part =
              math::ExtractGradient(y_relevant).row(0).transpose();
          for (int j = 0; j < cost_variable_indices[i].size(); ++j) {
            gradient[cost_variable_indices[i][j]] += gradient_part[j];
          }
        }
        return gradient;
      });

  std::unique_ptr<solvers::SolverInterface> projection_solver_interface;
  if (!projection_solver_interface_) {
    const SolverId solver_id = ChooseBestSolver(prog);
    projection_solver_interface = MakeSolver(solver_id);
  }
  std::function<VectorXd(const VectorXd&)> projection_function =
      custom_projection_function_.value_or([&](const VectorXd& x) {
        // Update the quadratic error cost.
        ConvertQuadraticErrorCost(x, projection_cost_Q, &projection_cost_b,
                                  &projection_cost_c);
        projection_cost.evaluator()->UpdateCoefficients(
            projection_cost_Q, projection_cost_b, projection_cost_c);

        // Solve the program.
        MathematicalProgramResult projection_result;
        if (projection_solver_interface_) {
          projection_solver_interface_->Solve(
              *projection_prog_ptr, x /* initial_guess */,
              std::nullopt /* solver_options */, &projection_result);
        } else {
          DRAKE_DEMAND(projection_solver_interface != nullptr);
          projection_solver_interface->Solve(
              *projection_prog_ptr, x /* initial_guess */,
              std::nullopt /* solver_options */, &projection_result);
        }

        return projection_result.get_x_val();
      });

  const double convergence_tol_squared =
      parsed_options.convergence_tol * parsed_options.convergence_tol;

  // Finally, we perform the projected gradient descent loop.
  // TODO(cohnt): Should we project to feasibility before starting the first
  // iteration?
  bool converged = false;
  bool failed = false;
  for (int iteration_count = 0; iteration_count < parsed_options.max_iterations;
       ++iteration_count) {
    // Gradient step.
    VectorXd descent_direction = -gradient_function(x_current);

    // Line search (using the Armijo condition).
    const double c = parsed_options.backtracking_c;
    const double tau = parsed_options.backtracking_tau;
    double alpha = parsed_options.backtracking_alpha_0;

    double m = descent_direction.dot(descent_direction);
    double t = -c * m;
    double old_cost = cost_function(x_current);
    while (true) {
      double new_cost = cost_function(x_current + alpha * descent_direction);
      if (old_cost - new_cost >= alpha * t) {
        break;
      } else {
        alpha *= tau;
      }
    }

    // Now project back to feasibility.
    VectorXd projected_value =
        projection_function(x_current + alpha * descent_direction);
    VectorXd step = projected_value - x_current;
    x_current = projected_value;

    // Check that the projection step succeeded.
    if (!projection_prog_ptr->CheckSatisfied(
            projection_prog_ptr->GetAllConstraints(), x_current,
            parsed_options.feasibility_tol)) {
      failed = true;
      break;
    }

    if (step.dot(step) < convergence_tol_squared) {
      converged = true;
      break;
    }
  }

  // Populate results.
  result->set_x_val(x_current);
  result->set_optimal_cost(cost_function(x_current));
  if (converged) {
    result->set_solution_result(SolutionResult::kSolutionFound);
  } else if (failed) {
    result->set_solution_result(SolutionResult::kInfeasibleConstraints);
  } else {
    result->set_solution_result(SolutionResult::kIterationLimit);
  }
}

std::string ProjectedGradientDescentSolver::ConvergenceTolOptionName() {
  return kConvergenceTolOptionName;
}

std::string ProjectedGradientDescentSolver::FeasibilityTolOptionName() {
  return kFeasibilityTolOptionName;
}

std::string ProjectedGradientDescentSolver::MaxIterationsOptionName() {
  return kMaxIterationsOptionName;
}

std::string ProjectedGradientDescentSolver::BacktrackingCOptionName() {
  return kBacktrackingCOptionName;
}

std::string ProjectedGradientDescentSolver::BacktrackingTauOptionName() {
  return kBacktrackingTauOptionName;
}

std::string ProjectedGradientDescentSolver::BacktrackingAlpha0OptionName() {
  return kBacktrackingAlpha0OptionName;
}

SolverId ProjectedGradientDescentSolver::id() {
  static const never_destroyed<SolverId> singleton{"PGD"};
  return singleton.access();
}

bool ProjectedGradientDescentSolver::is_available() {
  return true;
}

bool ProjectedGradientDescentSolver::is_enabled() {
  return true;
}

bool ProjectedGradientDescentSolver::ProgramAttributesSatisfied(
    const MathematicalProgram& prog) {
  static const never_destroyed<ProgramAttributes> solver_capabilities(
      std::initializer_list<ProgramAttribute>{
          ProgramAttribute::kGenericConstraint,
          ProgramAttribute::kLinearEqualityConstraint,
          ProgramAttribute::kLinearConstraint,
          ProgramAttribute::kQuadraticConstraint,
          ProgramAttribute::kLorentzConeConstraint,
          ProgramAttribute::kRotatedLorentzConeConstraint,
          ProgramAttribute::kGenericCost, ProgramAttribute::kLinearCost,
          ProgramAttribute::kL2NormCost, ProgramAttribute::kQuadraticCost,
          ProgramAttribute::kCallback});
  return AreRequiredAttributesSupported(prog.required_capabilities(),
                                        solver_capabilities.access());
}

}  // namespace solvers
}  // namespace drake
