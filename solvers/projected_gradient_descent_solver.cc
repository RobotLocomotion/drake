#include "drake/solvers/projected_gradient_descent_solver.h"

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
constexpr const char kCustomGradientFunctionOptionsName[] =
    "CustomGradientFunction";
constexpr const char kCustomProjectionFunctionOptionsName[] =
    "CustomProjectionFunction";
constexpr const char kProjectionSolverInterfaceOptionName[] =
    "ProjectionSolverInterface";
constexpr const char kConvergenceTolOptionName[] = "ConvergenceTol";
constexpr const char kFeasibilityTolOptionName[] = "FeasibilityTol";
constexpr const char kMaxIterationsOptionName[] = "MaxIterations";

struct KnownOptions {
  std::optional<std::function<VectorXd(const VectorXd&)>>
      custom_gradient_function{std::nullopt};
  std::optional<std::function<VectorXd(const VectorXd&)>>
      custom_projection_function{std::nullopt};
  SolverInterface* projection_solver_interface{nullptr};
  double convergence_tol{1e-6};
  double feasibility_tol{1e-6};
  int max_iterations{100};
};

void Serialize(internal::SpecificOptions* archive,
               // NOLINTNEXTLINE(runtime/references) to match Serialize concept.
               KnownOptions& options) {
  archive->Visit(MakeNameValue(kConvergenceTolOptionName,  // BR
                               &options.convergence_tol));
  archive->Visit(MakeNameValue(kMaxIterationsOptionName,  // BR
                               &options.max_iterations));
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
  return result;
}

void ConvertQuadraticErrorCost(const VectorXd& x_desired, const MatrixXd& Q,
                               VectorXd* b, double* c) {
  *b = -2 * Q * x_desired;
  *c = x_desired.dot(Q * x_desired);
}

}  // namespace

ProjectedGradientDescentSolver::ProjectedGradientDescentSolver()
    : SolverBase(id(), &is_available, &is_enabled,
                 &ProgramAttributesSatisfied) {}
ProjectedGradientDescentSolver::~ProjectedGradientDescentSolver() = default;

void ProjectedGradientDescentSolver::DoSolve2(
    const MathematicalProgram& prog, const VectorXd& initial_guess,
    internal::SpecificOptions* options,
    MathematicalProgramResult* result) const {
  const KnownOptions parsed_options = ParseOptions(options);

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
  ConvertQuadraticErrorCost(initial_guess, projection_cost_Q,
                            &projection_cost_b, &projection_cost_c);
  auto projection_cost = projection_prog_ptr->AddQuadraticCost(
      2 * projection_cost_Q, projection_cost_b, projection_cost_c,
      projection_prog_ptr->decision_variables());

  // Here, we create two lambdas, which we can evaluate to obtain the descent
  // direction and projection.
  std::function<double(const VectorXd&)> cost_function =
      [&](const VectorXd& x) {
        double cost = 0.0;
        for (int i = 0; i < ssize(costs); ++i) {
          VectorXd x_relevant = cost_variable_indices[i].unaryExpr(x);
          VectorXd y_output;
          costs[i].evaluator()->Eval(x_relevant, &y_output);
          DRAKE_ASSERT(y_output.size() == 1);
          cost += y_output[0];
        }
        return cost;
      };
  std::function<VectorXd(const VectorXd&)> gradient_function =
      parsed_options.custom_gradient_function.value_or([&](const VectorXd& x) {
        // TODO(cohnt): Switch to using
        // MathematicalProgram::EvalBindingsAtInitialGuess?
        AutoDiffVecXd x_ad = math::InitializeAutoDiff(x);
        AutoDiffVecXd y_ad = math::InitializeAutoDiff(VectorXd::Zero(x.size()));
        for (int i = 0; i < ssize(costs); ++i) {
          AutoDiffVecXd x_relevant = cost_variable_indices[i].unaryExpr(x_ad);
          AutoDiffVecXd y_relevant;
          costs[i].evaluator()->Eval(x_relevant, &y_relevant);
          for (int j = 0; j < cost_variable_indices[i].size(); ++j) {
            y_ad[cost_variable_indices[i][j]] += y_relevant[j];
          }
        }
        return math::ExtractGradient(y_ad);
      });
  std::function<VectorXd(const VectorXd&)> projection_function =
      parsed_options.custom_projection_function.value_or(
          [&](const VectorXd& x) {
            // Update the quadratic error cost.
            ConvertQuadraticErrorCost(x, projection_cost_Q, &projection_cost_b,
                                      &projection_cost_c);
            projection_cost.evaluator()->UpdateCoefficients(
                projection_cost_Q, projection_cost_b, projection_cost_c);

            // Solve the program.
            MathematicalProgramResult projection_result;
            if (parsed_options.projection_solver_interface) {
              parsed_options.projection_solver_interface->Solve(
                  *projection_prog_ptr, x /* initial_guess */,
                  std::nullopt /* solver_options */, &projection_result);
            } else {
              projection_result = Solve(*projection_prog_ptr);
            }

            return projection_result.get_x_val();
          });

  const double convergence_tol_squared =
      parsed_options.convergence_tol * parsed_options.convergence_tol;

  // Project the initial guess to feasibility.
  VectorXd x = projection_function(initial_guess);

  // Finally, we perform the projected gradient descent loop.
  bool converged = false;
  bool failed = false;
  for (int iteration_count = 0; iteration_count < parsed_options.max_iterations;
       ++iteration_count) {
    // Gradient step.
    VectorXd descent_direction = gradient_function(x);

    // Line search (using the Armijo condition).
    const double tau = 0.9;
    const double c = 0.5;
    double alpha = 1.0;

    double m = descent_direction.dot(descent_direction);
    double t = -c * m;
    while (true) {
      double new_cost = cost_function(x + alpha * descent_direction);
      if (new_cost >= alpha * t) {
        break;
      } else {
        alpha *= tau;
      }
    }

    // Now project back to feasibility.
    VectorXd step = projection_function(x + alpha * descent_direction) - x;
    x += step;

    // Check that the projection step succeeded.
    if (projection_prog_ptr->CheckSatisfied(
            projection_prog_ptr->GetAllConstraints(), x,
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
  if (converged) {
    result->set_solution_result(SolutionResult::kSolutionFound);
  } else if (failed) {
    result->set_solution_result(SolutionResult::kInfeasibleConstraints);
  } else {
    result->set_solution_result(SolutionResult::kIterationLimit);
  }
}

std::string ProjectedGradientDescentSolver::CustomGradientFunctionOptionName() {
  return kCustomGradientFunctionOptionsName;
}

std::string
ProjectedGradientDescentSolver::CustomProjectionFunctionOptionName() {
  return kCustomProjectionFunctionOptionsName;
}

std::string
ProjectedGradientDescentSolver::ProjectionSolverInterfaceOptionName() {
  return kProjectionSolverInterfaceOptionName;
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

SolverId ProjectedGradientDescentSolver::id() {
  static const never_destroyed<SolverId> singleton{"ProjectedGradientDescent"};
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
