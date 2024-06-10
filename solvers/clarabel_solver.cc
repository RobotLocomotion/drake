#include "drake/solvers/clarabel_solver.h"

#include <iostream>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Clarabel>
#include <Eigen/Eigen>

#include "drake/common/fmt_eigen.h"
#include "drake/common/name_value.h"
#include "drake/common/ssize.h"
#include "drake/common/text_logging.h"
#include "drake/solvers/aggregate_costs_constraints.h"
#include "drake/solvers/scs_clarabel_common.h"
#include "drake/tools/workspace/clarabel_cpp_internal/serialize.h"

namespace drake {
namespace solvers {
namespace {

// The solver status is defined in
// https://oxfordcontrol.github.io/ClarabelDocs/stable/api_jl/#Clarabel.SolverStatus
std::string SolverStatusToString(const clarabel::SolverStatus status) {
  switch (status) {
    case clarabel::SolverStatus::Unsolved:
      return "Unsolved";
    case clarabel::SolverStatus::Solved:
      return "Solved";
    case clarabel::SolverStatus::PrimalInfeasible:
      return "PrimalInfeasible";
    case clarabel::SolverStatus::DualInfeasible:
      return "DualInfeasible";
    case clarabel::SolverStatus::AlmostSolved:
      return "AlmostSolved";
    case clarabel::SolverStatus::AlmostPrimalInfeasible:
      return "AlmostPrimalInfeasible";
    case clarabel::SolverStatus::AlmostDualInfeasible:
      return "AlmostDualInfeasible";
    case clarabel::SolverStatus::MaxIterations:
      return "MaxIterations";
    case clarabel::SolverStatus::MaxTime:
      return "MaxTime";
    case clarabel::SolverStatus::NumericalError:
      return "NumericalError";
    case clarabel::SolverStatus::InsufficientProgress:
      return "InsufficientProgress";
  }
  DRAKE_UNREACHABLE();
}

void SetSolverDetails(
    const clarabel::DefaultSolution<double>& clarabel_solution,
    ClarabelSolverDetails* solver_details) {
  solver_details->iterations = clarabel_solution.iterations;
  solver_details->solve_time = clarabel_solution.solve_time;
  solver_details->status = SolverStatusToString(clarabel_solution.status);
}

class SettingsConverter {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SettingsConverter);

  explicit SettingsConverter(const SolverOptions& solver_options) {
    // Propagate Drake's common options into `settings_`.
    settings_.verbose = solver_options.get_print_to_console();
    // TODO(jwnimmer-tri) Handle get_print_file_name().

    // Copy the Clarabel-specific `solver_options` to pending maps.
    pending_options_double_ =
        solver_options.GetOptionsDouble(ClarabelSolver::id());
    pending_options_int_ = solver_options.GetOptionsInt(ClarabelSolver::id());
    pending_options_str_ = solver_options.GetOptionsStr(ClarabelSolver::id());

    // Move options from `pending_..._` to `settings_`.
    Serialize(this, settings_);

    // Identify any unsupported names (i.e., any leftovers in `pending_..._`).
    std::vector<std::string> unknown_names;
    for (const auto& [name, _] : pending_options_double_) {
      unknown_names.push_back(name);
    }
    for (const auto& [name, _] : pending_options_int_) {
      unknown_names.push_back(name);
    }
    for (const auto& [name, _] : pending_options_str_) {
      unknown_names.push_back(name);
    }
    if (unknown_names.size() > 0) {
      throw std::logic_error(fmt::format(
          "ClarabelSolver: unrecognized solver options {}. Please check "
          "https://oxfordcontrol.github.io/ClarabelDocs/stable/api_settings/ "
          "for the supported solver options.",
          fmt::join(unknown_names, ", ")));
    }
  }

  const clarabel::DefaultSettings<double>& settings() const {
    return settings_;
  }

  void Visit(const NameValue<double>& x) {
    this->SetFromDoubleMap(x.name(), x.value());
  }
  void Visit(const NameValue<bool>& x) {
    auto it = pending_options_int_.find(x.name());
    if (it != pending_options_int_.end()) {
      const int option_value = it->second;
      DRAKE_THROW_UNLESS(option_value == 0 || option_value == 1);
    }
    this->SetFromIntMap(x.name(), x.value());
  }
  void Visit(const NameValue<uint32_t>& x) {
    auto it = pending_options_int_.find(x.name());
    if (it != pending_options_int_.end()) {
      const int option_value = it->second;
      DRAKE_THROW_UNLESS(option_value >= 0);
    }
    this->SetFromIntMap(x.name(), x.value());
  }
  void Visit(const NameValue<clarabel::ClarabelDirectSolveMethods>& x) {
    DRAKE_THROW_UNLESS(x.name() == std::string{"direct_solve_method"});
    // TODO(jwnimmer-tri) Add support for this option.
    // For now it is unsupported and will throw (as an unknown name, below).
  }
  void Visit(const NameValue<clarabel::ClarabelCliqueMergeMethods>& x) {
    DRAKE_THROW_UNLESS(x.name() ==
                       std::string{"chordal_decomposition_merge_method"});
    // TODO(jwnimmer-tri) Add support for this option.
    // For now it is unsupported and will throw (as an unknown name, below).
  }

 private:
  void SetFromDoubleMap(const char* name, double* clarabel_value) {
    auto it = pending_options_double_.find(name);
    if (it != pending_options_double_.end()) {
      *clarabel_value = it->second;
      pending_options_double_.erase(it);
    }
  }
  template <typename T>
  void SetFromIntMap(const char* name, T* clarabel_value) {
    auto it = pending_options_int_.find(name);
    if (it != pending_options_int_.end()) {
      *clarabel_value = it->second;
      pending_options_int_.erase(it);
    }
  }

  std::unordered_map<std::string, double> pending_options_double_;
  std::unordered_map<std::string, int> pending_options_int_;
  std::unordered_map<std::string, std::string> pending_options_str_;

  clarabel::DefaultSettings<double> settings_ =
      clarabel::DefaultSettingsBuilder<double>::default_settings().build();
};

// See internal::ParseBoundingBoxConstraints for the meaning of
// bbcon_dual_indices.
void SetBoundingBoxDualSolution(
    const MathematicalProgram& prog,
    const Eigen::Ref<const Eigen::VectorXd>& dual,
    const std::vector<std::vector<std::pair<int, int>>>& bbcon_dual_indices,
    MathematicalProgramResult* result) {
  for (int i = 0; i < ssize(prog.bounding_box_constraints()); ++i) {
    Eigen::VectorXd bbcon_dual = Eigen::VectorXd::Zero(
        prog.bounding_box_constraints()[i].variables().rows());
    for (int j = 0; j < prog.bounding_box_constraints()[i].variables().rows();
         ++j) {
      if (prog.bounding_box_constraints()[i].evaluator()->lower_bound()(j) ==
          prog.bounding_box_constraints()[i].evaluator()->upper_bound()(j)) {
        // This is an equality constraint.
        // Notice that we have a negative sign in front of dual.
        // This is because in Clarabel, for a problem with the linear equality
        // constraint
        // min cᵀx
        // s.t A*x=b
        // Clarabel formulates the dual problem as
        // max -bᵀy
        // s.t Aᵀy = -c
        // Note that there is a negation sign before b and c in the Clarabel
        // dual problem, which is different from the standard formulation (no
        // negation sign). Hence the dual variable for the linear equality
        // constraint is the negation of the shadow price.
        bbcon_dual[j] = -dual(bbcon_dual_indices[i][j].second);
      } else {
        if (bbcon_dual_indices[i][j].first != -1) {
          // lower bound is not infinity.
          // The shadow price for the lower bound is positive. The Clarabel dual
          // for the positive cone is also positive, so we add the Clarabel
          // dual.
          bbcon_dual[j] += dual(bbcon_dual_indices[i][j].first);
        }
        if (bbcon_dual_indices[i][j].second != -1) {
          // upper bound is not infinity.
          // The shadow price for the upper bound is negative. The Clarabel dual
          // for the positive cone is positive, so we subtract the Clarabel
          // dual.
          bbcon_dual[j] -= dual(bbcon_dual_indices[i][j].second);
        }
      }
    }
    result->set_dual_solution(prog.bounding_box_constraints()[i], bbcon_dual);
  }
}

}  // namespace

bool ClarabelSolver::is_available() {
  return true;
}

void ClarabelSolver::DoSolve(const MathematicalProgram& prog,
                             const Eigen::VectorXd& initial_guess,
                             const SolverOptions& merged_options,
                             MathematicalProgramResult* result) const {
  if (!prog.GetVariableScaling().empty()) {
    static const logging::Warn log_once(
        "ClarabelSolver doesn't support the feature of variable scaling.");
  }

  // Clarabel doesn't have an API to provide the initial guess yet.
  unused(initial_guess);

  // Clarabel solves the problem in this form
  // min 0.5xᵀPx + qᵀx
  // s.t A x + s = b
  //     s in K
  // where K is a Cartesian product of some primitive cones.
  // The cones include
  // Zero cone {x | x = 0 }
  // Positive orthant {x | x ≥ 0 }
  // Second-order cone {(t, x) | |x|₂ ≤ t }
  // Positive semidefinite cone { X | min(eig(X)) ≥ 0, X = Xᵀ }
  // Exponential cone {x,y,z): y>0, y*exp(x/y) <= z}
  // Power cone {(x, y, z): pow(x, α)*pow(y, 1-α) >= |z|, x>=0, y>=0} with α in
  // (0, 1)
  // Clarabel doesn't impose an ordering of specifying these cone constraints
  // (which is different from SCS). Notice that due to the special problem form
  // supported by Clarabel, we need to convert our generic constraints to
  // Clarabel form. For example, a linear inequality constraint
  //   A x ≤ b
  // will be converted as
  //   A x + s = b
  //   s in positive orthant cone.
  // by introducing the slack variable s.

  // num_x is the number of variables in A*x+s=b. It can contain more variables
  // than in prog. For some constraint/cost, we need to append slack variables
  // to convert the constraint/cost to the Clarabel form.
  int num_x = prog.num_vars();

  // Since we add an array of quadratic cost, each cost has its own
  // associated variables. We use Eigen sparse matrix to aggregate the quadratic
  // cost Hessian matrices. Clarabel only takes the upper triangular entries of
  // the symmetric Hessian.
  std::vector<Eigen::Triplet<double>> P_upper_triplets;

  std::vector<clarabel::SupportedConeT<double>> cones;

  // `q` is the coefficient in the linear cost qᵀx
  std::vector<double> q(num_x, 0.0);

  // Our costs (LinearCost, QuadraticCost, etc) also allows a constant term, we
  // add these constant terms to `cost_constant`.
  double cost_constant{0};

  internal::ParseLinearCosts(prog, &q, &cost_constant);

  internal::ParseQuadraticCosts(prog, &P_upper_triplets, &q, &cost_constant);

  internal::ConvexConstraintAggregationInfo info;
  internal::ConvexConstraintAggregationOptions aggregation_options;
  int expected_A_row_count = 0;

  // TODO(Alexandre.Amice) Handle this special case more cleanly.
  std::vector<int> l2norm_costs_second_order_cone_length;
  std::vector<int> l2norm_costs_lorentz_cone_y_start_indices;
  std::vector<int> l2norm_costs_t_slack_indices;
  internal::ParseL2NormCosts(prog, &num_x, &(info.A_triplets), &(info.b_std),
                             &(info.A_row_count),
                             &l2norm_costs_second_order_cone_length,
                             &l2norm_costs_lorentz_cone_y_start_indices, &q,
                             &l2norm_costs_t_slack_indices);
  for (const int soc_length : l2norm_costs_second_order_cone_length) {
    expected_A_row_count += soc_length;
    cones.push_back(clarabel::SecondOrderConeT<double>(soc_length));
  }

  Eigen::SparseMatrix<double> P(num_x, num_x);
  P.setFromTriplets(P_upper_triplets.begin(), P_upper_triplets.end());
  Eigen::Map<Eigen::VectorXd> q_vec{q.data(), ssize(q)};

  // Now parse the constraints.
  internal::DoAggregateConvexConstraints(prog, aggregation_options, &info);

  Eigen::SparseMatrix<double> A(info.A_row_count, num_x);
  A.setFromTriplets(info.A_triplets.begin(), info.A_triplets.end());
  const Eigen::Map<Eigen::VectorXd> b_vec{info.b_std.data(), ssize(info.b_std)};

  cones.push_back(
      clarabel::ZeroConeT<double>(info.num_linear_equality_constraint_rows));
  cones.push_back(clarabel::NonnegativeConeT<double>(
      info.num_linear_constraint_rows +
      info.num_bounding_box_inequality_constraint_rows));
  expected_A_row_count += info.num_linear_equality_constraint_rows;
  expected_A_row_count += info.num_linear_constraint_rows;
  expected_A_row_count += info.num_bounding_box_inequality_constraint_rows;
  for (const int soc_length : info.second_order_cone_lengths) {
    expected_A_row_count += soc_length;
    cones.push_back(clarabel::SecondOrderConeT<double>(soc_length));
  }
  for (const int length : info.psd_cone_lengths) {
    expected_A_row_count += (length * (length + 1)) / 2;
    cones.push_back(clarabel::PSDTriangleConeT<double>(length));
  }
  for (int i = 0; i < ssize(prog.exponential_cone_constraints()); ++i) {
    expected_A_row_count += 3;
    cones.push_back(clarabel::ExponentialConeT<double>());
  }
  DRAKE_DEMAND(expected_A_row_count == A.rows());

  const SettingsConverter settings_converter(merged_options);
  clarabel::DefaultSettings<double> settings = settings_converter.settings();

  clarabel::DefaultSolver<double> solver(P, q_vec, A, b_vec, cones, settings);

  solver.solve();
  clarabel::DefaultSolution<double> solution = solver.solution();

  // Now set the solution.
  ClarabelSolverDetails& solver_details =
      result->SetSolverDetailsType<ClarabelSolverDetails>();
  SetSolverDetails(solution, &solver_details);

  SolutionResult solution_result{SolutionResult::kSolutionResultNotSet};

  result->set_x_val(
      Eigen::Map<Eigen::VectorXd>(solution.x.data(), prog.num_vars()));

  //  std::cout << fmt::format("Clarabel x =\n{}",
  //  fmt_eigen(solution.x.transpose())) << std::endl; std::cout <<
  //  fmt::format("Clarabel z =\n{}", fmt_eigen(solution.z.transpose())) <<
  //  std::endl;
  SetBoundingBoxDualSolution(prog, solution.z,
                             info.bounding_box_constraint_dual_indices, result);
  internal::SetDualSolution(
      prog, solution.z, info.linear_constraint_dual_indices,
      info.linear_eq_dual_variable_start_indices,
      info.lorentz_cone_dual_variable_start_indices,
      info.rotated_lorentz_cone_dual_variable_start_indices, result);
  if (solution.status == clarabel::SolverStatus::Solved ||
      solution.status == clarabel::SolverStatus::AlmostSolved) {
    solution_result = SolutionResult::kSolutionFound;
    result->set_optimal_cost(solution.obj_val + cost_constant);
  } else if (solution.status == clarabel::SolverStatus::PrimalInfeasible ||
             solution.status ==
                 clarabel::SolverStatus::AlmostPrimalInfeasible) {
    solution_result = SolutionResult::kInfeasibleConstraints;
    result->set_optimal_cost(MathematicalProgram::kGlobalInfeasibleCost);
  } else if (solution.status == clarabel::SolverStatus::DualInfeasible ||
             solution.status == clarabel::SolverStatus::AlmostDualInfeasible) {
    solution_result = SolutionResult::kDualInfeasible;
    result->set_optimal_cost(MathematicalProgram::kUnboundedCost);
  } else if (solution.status == clarabel::SolverStatus::MaxIterations) {
    solution_result = SolutionResult::kIterationLimit;
    result->set_optimal_cost(solution.obj_val + cost_constant);
  } else {
    drake::log()->info("Clarabel returns {}",
                       SolverStatusToString(solution.status));
    solution_result = SolutionResult::kSolverSpecificError;
  }
  result->set_solution_result(solution_result);
}

}  // namespace solvers
}  // namespace drake
