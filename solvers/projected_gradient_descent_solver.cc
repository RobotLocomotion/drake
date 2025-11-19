#include "drake/solvers/projected_gradient_descent_solver.h"

#include <memory>
#include <vector>

#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace solvers {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::VectorXi;

namespace {
constexpr const char kConvergenceTolOptionName[] = "ConvergenceTol";
constexpr const char kMaxIterationsOptionName[] = "MaxIterations";
constexpr const char kBacktrackingCOptionName[] = "BacktrackingC";
constexpr const char kBacktrackingTauOptionName[] = "BacktrackingTau";
constexpr const char kBacktrackingAlpha0OptionName[] = "BacktrackingAlpha0";

struct KnownOptions {
  double convergence_tol{
      ProjectedGradientDescentSolver::kDefaultConvergenceTol};
  int max_iterations{ProjectedGradientDescentSolver::kDefaultMaxIterations};
  double backtracking_c{ProjectedGradientDescentSolver::kDefaultBacktrackingC};
  double backtracking_tau{
      ProjectedGradientDescentSolver::kDefaultBacktrackingTau};
  double backtracking_alpha_0{
      ProjectedGradientDescentSolver::kDefaultBacktrackingAlpha0};
};

void Serialize(internal::SpecificOptions* archive,
               // NOLINTNEXTLINE(runtime/references) to match Serialize concept.
               KnownOptions& options) {
  archive->Visit(
      MakeNameValue(kConvergenceTolOptionName, &options.convergence_tol));
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
  if (result.max_iterations < 1) {
    throw std::invalid_argument("MaxIterations must be at least one.");
  }
  if (result.backtracking_c >= 1.0 || result.backtracking_c <= 0.0) {
    throw std::invalid_argument("BacktrackingC must be between 0 and 1.");
  }
  if (result.backtracking_tau >= 1.0 || result.backtracking_tau <= 0.0) {
    throw std::invalid_argument("BacktrackingTau must be between 0 and 1.");
  }
  if (result.backtracking_alpha_0 <= 0) {
    throw std::invalid_argument(
        "BacktrackingAlpha0 should be a non-negative number.");
  }
  return result;
}

// Given a (square, symmetric) matrix Q, compute b and c such that
// AddQuadraticErrorCost(Q, x_desired, x) is equivalent to
// AddQuadraticCost(2*Q, b, c, x).
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

  // First, we replace any nan values in the initial guess with zero.
  VectorXd x_current = initial_guess;
  for (int i = 0; i < x_current.size(); ++i) {
    if (std::isnan(x_current[i])) {
      x_current[i] = 0.0;
    }
  }

  std::vector<Binding<Cost>> costs = prog.GetAllCosts();

  // Next, we construct an auxiliary MathematicalProgram, for use in the
  // projection step. We clone prog, remove all costs, and add a quadratic
  // cost penalizing distance to the initial guess. (We then later update
  // the coefficients of this cost at each iteration.)
  std::unique_ptr<MathematicalProgram> projection_prog_ptr = prog.Clone();
  for (const auto& cost : costs) {
    projection_prog_ptr->RemoveCost(cost);
  }
  // We parse the quadratic error cost into a standard-form quadratic cost to
  // match the use later in the program.
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
          VectorXd y = prog.EvalBinding(costs[i], x);
          DRAKE_ASSERT(y.size() == 1);
          cost += y[0];
        }
        return cost;
      };
  std::function<VectorXd(const VectorXd&)> gradient_function =
      custom_gradient_function_.value_or([&](const VectorXd& x) {
        AutoDiffVecXd x_ad = math::InitializeAutoDiff(x);
        VectorXd gradient = VectorXd::Zero(x.size());
        for (int i = 0; i < ssize(costs); ++i) {
          AutoDiffVecXd y_ad = prog.EvalBinding(costs[i], x_ad);
          Eigen::RowVectorXd gradient_part = math::ExtractGradient(y_ad);
          DRAKE_ASSERT(gradient_part.size() == gradient.size());
          gradient += gradient_part.transpose();
        }
        return gradient;
      });

  // TODO(cohnt): Allow user to specify an initial guess strategy, for cases
  // where the feasible set is nonconvex. (For example, we may want to use the
  // previous feasible iterate, or some other strategy.)
  std::unique_ptr<solvers::SolverInterface> projection_solver_interface;
  if (!projection_solver_interface_) {
    const SolverId solver_id = ChooseBestSolver(*projection_prog_ptr);
    projection_solver_interface = MakeSolver(solver_id);
  }
  std::function<bool(const VectorXd&, VectorXd*)> projection_function =
      custom_projection_function_.value_or([&](const VectorXd& x, VectorXd* y) {
        // Update the quadratic error cost.
        ConvertQuadraticErrorCost(x, projection_cost_Q, &projection_cost_b,
                                  &projection_cost_c);
        projection_cost.evaluator()->UpdateCoefficients(
            projection_cost_Q, projection_cost_b, projection_cost_c);

        // Solve the program. We use the target value x as the initial guess.
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

        DRAKE_DEMAND(y != nullptr);
        *y = projection_result.get_x_val();
        return projection_result.is_success();
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

    // Line search (using the Armijo condition). If the feasible set is convex
    // and the objective function has a Lipschitz gradient, then a step
    // satisfying the Armijo condition guarantees a decrease in the objective
    // function, even after the projection step. This is true for any choice of
    // C, alpha_0, and tau -- regardless of the Lipschitz constant.
    const double c = parsed_options.backtracking_c;
    const double tau = parsed_options.backtracking_tau;
    double alpha = parsed_options.backtracking_alpha_0;
    // TODO(cohnt): To handle cases where the objective function isn't Lipschitz
    // or the feasible set isn't convex, we should allow the user to impose more
    // general convergence criteria and/or line search strategies.

    double m = descent_direction.dot(descent_direction);
    double t = c * m;
    double old_cost = cost_function(x_current);
    // TODO(cohnt): Allow user to set the maximum number of line search steps.
    for (int line_search_steps = 0;
         line_search_steps < kDefaultMaxLineSearchSteps; ++line_search_steps) {
      double new_cost = cost_function(x_current + alpha * descent_direction);
      if (old_cost - new_cost >= alpha * t) {
        break;
      } else {
        alpha *= tau;
      }
    }

    // Now project back to feasibility.
    VectorXd projected_value;
    bool projection_succeeded = projection_function(
        x_current + alpha * descent_direction, &projected_value);
    VectorXd step = projected_value - x_current;
    x_current = projected_value;

    // Check that the projection step succeeded.
    if (!projection_succeeded) {
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
