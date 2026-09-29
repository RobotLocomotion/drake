#include "drake/solvers/daqp_solver.h"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include "api.h"
#include <Eigen/Core>
#include <Eigen/SparseCore>

#include "drake/common/name_value.h"
#include "drake/common/never_destroyed.h"
#include "drake/solvers/aggregate_costs_constraints.h"
#include "drake/solvers/mathematical_program_result.h"
#include "drake/solvers/specific_options.h"

// DAQPSettings lives in the global namespace; Serialize must live there too.
static void Serialize(drake::solvers::internal::SpecificOptions* archive,
                      DAQPSettings& settings) {
  using drake::MakeNameValue;
  archive->Visit(MakeNameValue("primal_tol", &settings.primal_tol));
  archive->Visit(MakeNameValue("dual_tol", &settings.dual_tol));
  archive->Visit(MakeNameValue("zero_tol", &settings.zero_tol));
  archive->Visit(MakeNameValue("pivot_tol", &settings.pivot_tol));
  archive->Visit(MakeNameValue("progress_tol", &settings.progress_tol));
  archive->Visit(MakeNameValue("cycle_tol", &settings.cycle_tol));
  archive->Visit(MakeNameValue("iter_limit", &settings.iter_limit));
  archive->Visit(MakeNameValue("eps_prox", &settings.eps_prox));
  archive->Visit(MakeNameValue("eta_prox", &settings.eta_prox));
  archive->Visit(MakeNameValue("eq_reduction", &settings.eq_reduction));
  archive->Visit(MakeNameValue("time_limit", &settings.time_limit));
}

namespace drake {
namespace solvers {
namespace {

// DAQP uses finite sentinels for absent bounds.
double DaqpBound(double value) {
  if (std::isinf(value)) {
    return std::signbit(value) ? -DAQP_INF : DAQP_INF;
  }
  return value;
}

template <typename C>
void AppendLinearConstraints(
    const MathematicalProgram& prog, const std::vector<Binding<C>>& bindings,
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>* A,
    std::vector<double>* lower, std::vector<double>* upper,
    std::vector<int>* sense) {
  for (const auto& binding : bindings) {
    const auto& evaluator = binding.evaluator();
    const auto indices = prog.FindDecisionVariableIndices(binding.variables());
    const auto& local_A = evaluator->get_sparse_A();
    const int first_row = lower->size() - prog.num_vars();
    A->middleRows(first_row, evaluator->num_constraints()).setZero();
    for (int col = 0; col < local_A.outerSize(); ++col) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(local_A, col); it;
           ++it) {
        (*A)(first_row + it.row(), indices.at(col)) += it.value();
      }
    }
    for (int row = 0; row < evaluator->num_constraints(); ++row) {
      const double lb = evaluator->lower_bound()(row);
      const double ub = evaluator->upper_bound()(row);
      lower->push_back(DaqpBound(lb));
      upper->push_back(DaqpBound(ub));
      sense->push_back(
          std::isfinite(lb) && lb == ub ? DAQP_ACTIVE | DAQP_IMMUTABLE : 0);
    }
  }
}

template <typename C>
void SetDualSolutions(const std::vector<Binding<C>>& bindings,
                      const Eigen::VectorXd& multipliers, int* row,
                      MathematicalProgramResult* result) {
  for (const auto& binding : bindings) {
    const int count = binding.evaluator()->num_constraints();
    // DAQP's multiplier is positive at an upper bound; Drake's shadow price
    // convention has the opposite sign.
    result->set_dual_solution(binding, -multipliers.segment(*row, count));
    *row += count;
  }
}

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
  for (int i = 0; i < static_cast<int>(bindings.size()); ++i) {
    const auto& binding = bindings[i];
    duals.push_back(
        Eigen::VectorXd::Zero(binding.evaluator()->num_constraints()));
    for (int row = 0; row < binding.evaluator()->num_constraints(); ++row) {
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
    const Owner& owner = multiplier < 0   ? lower_owner[index]
                         : multiplier > 0 ? upper_owner[index]
                                          : lower_owner[index];
    if (owner.first >= 0) {
      duals[owner.first](owner.second) = -multiplier;
    }
  }
  for (int i = 0; i < static_cast<int>(bindings.size()); ++i) {
    result->set_dual_solution(bindings[i], duals[i]);
  }
}

}  // namespace

DaqpSolver::DaqpSolver()
    : SolverBase(id(), &is_available, &is_enabled, &ProgramAttributesSatisfied,
                 &UnsatisfiedProgramAttributes) {}

DaqpSolver::~DaqpSolver() = default;

SolverId DaqpSolver::id() {
  static const never_destroyed<SolverId> singleton{"DAQP"};
  return singleton.access();
}

bool DaqpSolver::is_available() {
  return true;
}
bool DaqpSolver::is_enabled() {
  return true;
}

namespace {
bool CheckAttributes(const MathematicalProgram& prog,
                     std::string* explanation) {
  static const never_destroyed<ProgramAttributes> capabilities(
      std::initializer_list<ProgramAttribute>{
          ProgramAttribute::kLinearCost, ProgramAttribute::kQuadraticCost,
          ProgramAttribute::kLinearConstraint,
          ProgramAttribute::kLinearEqualityConstraint});
  if (!internal::CheckConvexSolverAttributes(prog, capabilities.access(),
                                             "DaqpSolver", explanation)) {
    return false;
  }
  if (prog.quadratic_costs().empty()) {
    if (explanation) {
      *explanation = "DaqpSolver requires a quadratic cost";
    }
    return false;
  }
  return true;
}
}  // namespace

bool DaqpSolver::ProgramAttributesSatisfied(const MathematicalProgram& prog) {
  return CheckAttributes(prog, nullptr);
}

std::string DaqpSolver::UnsatisfiedProgramAttributes(
    const MathematicalProgram& prog) {
  std::string explanation;
  CheckAttributes(prog, &explanation);
  return explanation;
}

void DaqpSolver::DoSolve2(const MathematicalProgram& prog,
                          const Eigen::VectorXd& initial_guess,
                          internal::SpecificOptions* options,
                          MathematicalProgramResult* result) const {
  // The one-shot DAQP API does not reuse state from previous solves.
  (void)initial_guess;
  if (!prog.GetVariableScaling().empty()) {
    throw std::runtime_error("DaqpSolver does not support variable scaling");
  }
  const int n = prog.num_vars();
  std::vector<Eigen::Triplet<double>> h_triplets;
  std::vector<double> f(n, 0.0);
  double constant = 0.0;
  internal::ParseQuadraticCosts(prog, &h_triplets, &f, &constant);
  internal::ParseLinearCosts(prog, &f, &constant);

  // DAQP expects a dense, row-major Hessian and constraint matrix.
  using RowMajorMatrix =
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
  RowMajorMatrix H = RowMajorMatrix::Zero(n, n);
  for (const auto& term : h_triplets) {
    H(term.row(), term.col()) += term.value();
    if (term.row() != term.col()) {
      H(term.col(), term.row()) += term.value();
    }
  }

  int num_rows = 0;
  for (const auto& binding : prog.linear_constraints()) {
    num_rows += binding.evaluator()->num_constraints();
  }
  for (const auto& binding : prog.linear_equality_constraints()) {
    num_rows += binding.evaluator()->num_constraints();
  }
  RowMajorMatrix A(num_rows, n);
  std::vector<double> lower(n, -std::numeric_limits<double>::infinity());
  std::vector<double> upper(n, std::numeric_limits<double>::infinity());
  AggregateBoundingBoxConstraints(prog, &lower, &upper);
  std::vector<int> sense(n, 0);
  for (int i = 0; i < n; ++i) {
    const double lb = lower[i];
    const double ub = upper[i];
    sense[i] = std::isfinite(lb) && lb == ub ? DAQP_ACTIVE | DAQP_IMMUTABLE : 0;
    lower[i] = DaqpBound(lb);
    upper[i] = DaqpBound(ub);
  }
  lower.reserve(n + num_rows);
  upper.reserve(n + num_rows);
  sense.reserve(n + num_rows);
  AppendLinearConstraints(prog, prog.linear_constraints(), &A, &lower, &upper,
                          &sense);
  AppendLinearConstraints(prog, prog.linear_equality_constraints(), &A, &lower,
                          &upper, &sense);
  DAQPSettings settings;
  daqp_default_settings(&settings);
  options->CopyToSerializableStruct(&settings);
  DAQPProblem qp{};
  qp.n = n;
  qp.m = n + num_rows;
  qp.ms = n;
  qp.H = H.data();
  qp.f = f.data();
  qp.A = A.size() ? A.data() : nullptr;
  qp.blower = lower.empty() ? nullptr : lower.data();
  qp.bupper = upper.empty() ? nullptr : upper.data();
  qp.sense = sense.empty() ? nullptr : sense.data();

  Eigen::VectorXd x(n);
  Eigen::VectorXd multipliers(qp.m);
  DAQPResult daqp_result{};
  daqp_result.x = x.data();
  daqp_result.lam = qp.m ? multipliers.data() : nullptr;
  daqp_quadprog(&daqp_result, &qp, &settings);

  auto& details = result->SetSolverDetailsType<DaqpSolverDetails>();
  details.exitflag = daqp_result.exitflag;
  details.iterations = daqp_result.iter;
  details.setup_time = daqp_result.setup_time;
  details.solve_time = daqp_result.solve_time;
  if (daqp_result.exitflag == DAQP_EXIT_OPTIMAL) {
    details.multipliers = multipliers;
    result->set_x_val(x);
    result->set_optimal_cost(daqp_result.fval + constant);
    int row = n;
    SetDualSolutions(prog.linear_constraints(), multipliers, &row, result);
    SetDualSolutions(prog.linear_equality_constraints(), multipliers, &row,
                     result);
    SetBoundingBoxDualSolutions(prog, lower, upper, multipliers, result);
    result->set_solution_result(SolutionResult::kSolutionFound);
  } else if (daqp_result.exitflag == DAQP_EXIT_INFEASIBLE) {
    result->set_optimal_cost(MathematicalProgram::kGlobalInfeasibleCost);
    result->set_solution_result(SolutionResult::kInfeasibleConstraints);
  } else if (daqp_result.exitflag == DAQP_EXIT_UNBOUNDED) {
    result->set_optimal_cost(MathematicalProgram::kUnboundedCost);
    result->set_solution_result(SolutionResult::kUnbounded);
  } else if (daqp_result.exitflag == DAQP_EXIT_ITERLIMIT) {
    result->set_solution_result(SolutionResult::kIterationLimit);
  } else {
    result->set_solution_result(SolutionResult::kSolverSpecificError);
  }
}

}  // namespace solvers
}  // namespace drake
