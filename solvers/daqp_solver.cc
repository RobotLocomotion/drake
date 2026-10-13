#include "drake/solvers/daqp_solver.h"

#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <api.h>

#include "drake/common/name_value.h"
#include "drake/common/text_logging.h"
#include "drake/common/unused.h"
#include "drake/solvers/aggregate_costs_constraints.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"
#include "drake/solvers/specific_options.h"

// DAQPSettings lives in the global namespace, so Serialize() must live there
// too, for argument-dependent lookup. The soft-constraint settings (rho_soft,
// w_soft) and the branch-and-bound settings (rel_subopt, abs_subopt) are not
// listed, because this wrapper uses neither feature.
static void Serialize(
    drake::solvers::internal::SpecificOptions* archive,
    // NOLINTNEXTLINE(runtime/references) to match Serialize concept.
    DAQPSettings& settings) {
  using drake::MakeNameValue;
  archive->Visit(MakeNameValue("primal_tol", &settings.primal_tol));
  archive->Visit(MakeNameValue("dual_tol", &settings.dual_tol));
  archive->Visit(MakeNameValue("zero_tol", &settings.zero_tol));
  archive->Visit(MakeNameValue("pivot_tol", &settings.pivot_tol));
  archive->Visit(MakeNameValue("progress_tol", &settings.progress_tol));
  archive->Visit(MakeNameValue("cycle_tol", &settings.cycle_tol));
  archive->Visit(MakeNameValue("iter_limit", &settings.iter_limit));
  archive->Visit(MakeNameValue("fval_bound", &settings.fval_bound));
  archive->Visit(MakeNameValue("eps_prox", &settings.eps_prox));
  archive->Visit(MakeNameValue("eta_prox", &settings.eta_prox));
  archive->Visit(MakeNameValue("sing_tol", &settings.sing_tol));
  archive->Visit(MakeNameValue("refactor_tol", &settings.refactor_tol));
  archive->Visit(MakeNameValue("time_limit", &settings.time_limit));
  archive->Visit(MakeNameValue("eq_reduction", &settings.eq_reduction));
}

namespace drake {
namespace solvers {
namespace {

using RowMajorMatrix =
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

// DAQP uses finite sentinels for absent bounds.
double DaqpBound(double value) {
  if (std::isinf(value)) {
    return std::signbit(value) ? -DAQP_INF : DAQP_INF;
  }
  return value;
}

// Equality rows are marked active and immutable, so that DAQP never removes
// them from the working set.
int DaqpSense(double lower, double upper) {
  return std::isfinite(lower) && lower == upper ? DAQP_ACTIVE | DAQP_IMMUTABLE
                                                : 0;
}

// Appends the rows of `bindings` to the general constraints, starting at row
// `*row` of `A` (which must already be zero), and advances `*row`.
template <typename C>
void AppendLinearConstraints(const MathematicalProgram& prog,
                             const std::vector<Binding<C>>& bindings,
                             RowMajorMatrix* A, std::vector<double>* lower,
                             std::vector<double>* upper,
                             std::vector<int>* sense, int* row) {
  for (const auto& binding : bindings) {
    const auto& evaluator = binding.evaluator();
    const std::vector<int> indices =
        prog.FindDecisionVariableIndices(binding.variables());
    const Eigen::SparseMatrix<double>& local_A = evaluator->get_sparse_A();
    for (int col = 0; col < local_A.outerSize(); ++col) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(local_A, col); it;
           ++it) {
        (*A)(*row + it.row(), indices[col]) += it.value();
      }
    }
    for (int i = 0; i < evaluator->num_constraints(); ++i) {
      const double lb = evaluator->lower_bound()(i);
      const double ub = evaluator->upper_bound()(i);
      lower->push_back(DaqpBound(lb));
      upper->push_back(DaqpBound(ub));
      sense->push_back(DaqpSense(lb, ub));
    }
    *row += evaluator->num_constraints();
  }
}

template <typename C>
void SetDualSolutions(const std::vector<Binding<C>>& bindings,
                      const Eigen::VectorXd& multipliers, int* row,
                      MathematicalProgramResult* result) {
  for (const auto& binding : bindings) {
    const int count = binding.evaluator()->num_constraints();
    // DAQP's multiplier is positive at an upper bound; Drake's dual solution
    // convention has the opposite sign.
    result->set_dual_solution(binding, -multipliers.segment(*row, count));
    *row += count;
  }
}

// DAQP has one bound pair per variable, aggregated over all bounding box
// constraints. The dual of each active bound is assigned to the first binding
// whose bound equals the aggregated (tightest) bound; all other bindings get
// zero.
void SetBoundingBoxDualSolutions(const MathematicalProgram& prog,
                                 const std::vector<double>& lower,
                                 const std::vector<double>& upper,
                                 const Eigen::VectorXd& multipliers,
                                 MathematicalProgramResult* result) {
  const auto& bindings = prog.bounding_box_constraints();
  using Owner = std::pair<int, int>;  // Binding index, then row within binding.
  std::vector<Owner> lower_owner(prog.num_vars(), {-1, -1});
  std::vector<Owner> upper_owner(prog.num_vars(), {-1, -1});
  std::vector<Eigen::VectorXd> duals;
  duals.reserve(bindings.size());
  for (int i = 0; i < ssize(bindings); ++i) {
    const auto& binding = bindings[i];
    const int num_rows = binding.evaluator()->num_constraints();
    duals.push_back(Eigen::VectorXd::Zero(num_rows));
    for (int row = 0; row < num_rows; ++row) {
      const int index =
          prog.FindDecisionVariableIndex(binding.variables()(row));
      if (lower_owner[index].first < 0 &&
          binding.evaluator()->lower_bound()(row) == lower[index]) {
        lower_owner[index] = {i, row};
      }
      if (upper_owner[index].first < 0 &&
          binding.evaluator()->upper_bound()(row) == upper[index]) {
        upper_owner[index] = {i, row};
      }
    }
  }
  for (int index = 0; index < prog.num_vars(); ++index) {
    const double multiplier = multipliers(index);
    const Owner& owner =
        multiplier > 0 ? upper_owner[index] : lower_owner[index];
    if (owner.first >= 0) {
      duals[owner.first](owner.second) = -multiplier;
    }
  }
  for (int i = 0; i < ssize(bindings); ++i) {
    result->set_dual_solution(bindings[i], duals[i]);
  }
}

SolutionResult ConvertExitFlag(int exitflag) {
  switch (exitflag) {
    case DAQP_EXIT_OPTIMAL:
    case DAQP_EXIT_SOFT_OPTIMAL:
      return SolutionResult::kSolutionFound;
    case DAQP_EXIT_INFEASIBLE:
      return SolutionResult::kInfeasibleConstraints;
    case DAQP_EXIT_UNBOUNDED:
      return SolutionResult::kUnbounded;
    case DAQP_EXIT_ITERLIMIT:
    case DAQP_EXIT_TIMELIMIT:
      return SolutionResult::kIterationLimit;
    case DAQP_EXIT_NONCONVEX:
    case DAQP_EXIT_UNSUPPORTED:
      return SolutionResult::kInvalidInput;
    default:
      // For example, DAQP_EXIT_CYCLE or DAQP_EXIT_OVERDETERMINED_INITIAL.
      return SolutionResult::kSolverSpecificError;
  }
}

}  // namespace

bool DaqpSolver::is_available() {
  return true;
}

void DaqpSolver::DoSolve2(const MathematicalProgram& prog,
                          const Eigen::VectorXd& initial_guess,
                          internal::SpecificOptions* options,
                          MathematicalProgramResult* result) const {
  // DAQP's one-shot API has no way to use a primal initial guess.
  unused(initial_guess);
  if (!prog.GetVariableScaling().empty()) {
    static const logging::Warn log_once(
        "DaqpSolver doesn't support the feature of variable scaling.");
  }

  const int n = prog.num_vars();
  std::vector<Eigen::Triplet<double>> H_triplets;
  std::vector<double> f(n, 0.0);
  double constant = 0.0;
  internal::ParseQuadraticCosts(prog, &H_triplets, &f, &constant);
  internal::ParseLinearCosts(prog, &f, &constant);

  // DAQP expects a dense, row-major Hessian. ParseQuadraticCosts() returns
  // only its upper triangle.
  RowMajorMatrix H = RowMajorMatrix::Zero(n, n);
  for (const auto& triplet : H_triplets) {
    H(triplet.row(), triplet.col()) += triplet.value();
    if (triplet.row() != triplet.col()) {
      H(triplet.col(), triplet.row()) += triplet.value();
    }
  }

  // The first n constraints are "simple bounds" on the variables (DAQP's ms),
  // followed by the general (dense, row-major) constraints A.
  std::vector<double> lower(n, -std::numeric_limits<double>::infinity());
  std::vector<double> upper(n, std::numeric_limits<double>::infinity());
  AggregateBoundingBoxConstraints(prog, &lower, &upper);
  std::vector<int> sense(n);
  for (int i = 0; i < n; ++i) {
    sense[i] = DaqpSense(lower[i], upper[i]);
    lower[i] = DaqpBound(lower[i]);
    upper[i] = DaqpBound(upper[i]);
  }
  int num_rows = 0;
  for (const auto& binding : prog.linear_constraints()) {
    num_rows += binding.evaluator()->num_constraints();
  }
  for (const auto& binding : prog.linear_equality_constraints()) {
    num_rows += binding.evaluator()->num_constraints();
  }
  RowMajorMatrix A = RowMajorMatrix::Zero(num_rows, n);
  lower.reserve(n + num_rows);
  upper.reserve(n + num_rows);
  sense.reserve(n + num_rows);
  int row = 0;
  AppendLinearConstraints(prog, prog.linear_constraints(), &A, &lower, &upper,
                          &sense, &row);
  AppendLinearConstraints(prog, prog.linear_equality_constraints(), &A, &lower,
                          &upper, &sense, &row);

  DAQPSettings settings;
  daqp_default_settings(&settings);
  options->CopyToSerializableStruct(&settings);

  DAQPProblem qp{};
  qp.n = n;
  qp.m = n + num_rows;
  qp.ms = n;
  qp.H = H.data();
  qp.f = f.data();
  qp.A = num_rows > 0 ? A.data() : nullptr;
  qp.blower = lower.data();
  qp.bupper = upper.data();
  qp.sense = sense.data();

  Eigen::VectorXd x(n);
  Eigen::VectorXd multipliers(qp.m);
  DAQPResult daqp_result{};
  daqp_result.x = x.data();
  daqp_result.lam = multipliers.data();
  daqp_quadprog(&daqp_result, &qp, &settings);

  auto& details = result->SetSolverDetailsType<DaqpSolverDetails>();
  details.exitflag = daqp_result.exitflag;
  details.iterations = daqp_result.iter;
  details.setup_time = daqp_result.setup_time;
  details.solve_time = daqp_result.solve_time;
  details.multipliers.resize(0);

  const SolutionResult solution_result = ConvertExitFlag(daqp_result.exitflag);
  result->set_solution_result(solution_result);
  switch (solution_result) {
    case SolutionResult::kSolutionFound: {
      details.multipliers = multipliers;
      result->set_x_val(x);
      result->set_optimal_cost(daqp_result.fval + constant);
      int dual_row = n;
      SetDualSolutions(prog.linear_constraints(), multipliers, &dual_row,
                       result);
      SetDualSolutions(prog.linear_equality_constraints(), multipliers,
                       &dual_row, result);
      SetBoundingBoxDualSolutions(prog, lower, upper, multipliers, result);
      break;
    }
    case SolutionResult::kInfeasibleConstraints:
      result->set_optimal_cost(MathematicalProgram::kGlobalInfeasibleCost);
      break;
    case SolutionResult::kUnbounded:
      result->set_optimal_cost(MathematicalProgram::kUnboundedCost);
      break;
    default:
      break;
  }
}

}  // namespace solvers
}  // namespace drake
