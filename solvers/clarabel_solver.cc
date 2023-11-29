#include "drake/solvers/clarabel_solver.h"

#include <unordered_map>
#include <utility>
#include <vector>

#include <Clarabel>
#include <Eigen/Eigen>

#include "drake/common/ssize.h"
#include "drake/common/text_logging.h"
#include "drake/solvers/aggregate_costs_constraints.h"

namespace drake {
namespace solvers {
namespace {
// Parse all the bounding box constraints in `prog` to Clarabel form A*x+s=b, s
// in zero cone or positive cone.
// @param[in/out] A_triplets Append non-zero (row, col, val) triplet in matrix
// A.
// @param[in/out] b append entries to b.
// @param[in/out] A_row_count The number of rows in A before/after calling this
// function.
// @param[in/out] cones The type of cones on s before/after calling this
// function.
// @param[out] bbcon_dual_indices bbcon_dual_indices[i][j] are the indices of
// the dual variable for the j'th row of prog.bounding_box_constraints()[i]. We
// use -1 to indicate that it is impossible for this constraint to be active
// (for example, another BoundingBoxConstraint imposes a tighter bound on the
// same variable).
//
// Unlike SCS, Clarabel allows specifying the cone type of each individual s
// variable (in SCS, the s variables for the same type of cone have to be
// grouped together as a continuous fragment). Hence in Clarabel we can check
// the type of each individual bounding box constraint, and differentiate them
// based on whether it is an equality (s in zero-cone) or an inequality (s in
// positive orthant cone) constraint.
void ParseBoundingBoxConstraints(
    const MathematicalProgram& prog,
    std::vector<Eigen::Triplet<double>>* A_triplets, std::vector<double>* b,
    int* A_row_count, std::vector<clarabel::SupportedConeT<double>>* cones,
    std::vector<std::vector<std::pair<int, int>>>* bbcon_dual_indices) {
  const std::unordered_map<symbolic::Variable, Bound> variable_bounds =
      AggregateBoundingBoxConstraints(prog.bounding_box_constraints());

  // For each variable with lb <= x <= ub, we check the following
  // 1. If lb == ub (and both are finite), then we add the constraint x + s = ub
  // with s in the zero cone.
  // 2. Otherwise, if ub is finite, then we add the constraint x + s = ub with s
  // in the positive orthant cone. If lb is finite, then we add the constraint
  // -x + s = -lb with s in the positive orthant cone.

  // Set the dual variable indices.
  bbcon_dual_indices->reserve(ssize(prog.bounding_box_constraints()));
  for (int i = 0; i < ssize(prog.bounding_box_constraints()); ++i) {
    bbcon_dual_indices->emplace_back(
        prog.bounding_box_constraints()[i].variables().rows(),
        std::pair<int, int>(-1, -1));
    for (int j = 0; j < prog.bounding_box_constraints()[i].variables().rows();
         ++j) {
      const int var_index = prog.FindDecisionVariableIndex(
          prog.bounding_box_constraints()[i].variables()(j));
      const Bound& var_bound =
          variable_bounds.at(prog.bounding_box_constraints()[i].variables()(j));
      // We use the lower bound of this constraint in the optimization
      // program.
      const bool use_lb =
          var_bound.lower ==
              prog.bounding_box_constraints()[i].evaluator()->lower_bound()(
                  j) &&
          std::isfinite(var_bound.lower);
      const bool use_ub =
          var_bound.upper ==
              prog.bounding_box_constraints()[i].evaluator()->upper_bound()(
                  j) &&
          std::isfinite(var_bound.upper);
      if (use_lb && use_ub && var_bound.lower == var_bound.upper) {
        // This is an equality constraint x = ub.
        // Add the constraint x + s = ub and s in the zero cone.
        A_triplets->emplace_back(*A_row_count, var_index, 1);
        b->push_back(var_bound.upper);
        (*bbcon_dual_indices)[i][j].first = *A_row_count;
        (*bbcon_dual_indices)[i][j].second = *A_row_count;
        cones->push_back(clarabel::ZeroConeT<double>(1));
        ++(*A_row_count);
      } else {
        if (use_ub) {
          // Add the constraint x + s = ub and s in the nonnegative orthant
          // cone.
          A_triplets->emplace_back(*A_row_count, var_index, 1);
          b->push_back(var_bound.upper);
          (*bbcon_dual_indices)[i][j].second = *A_row_count;
          cones->push_back(clarabel::NonnegativeConeT<double>(1));
          ++(*A_row_count);
        }
        if (use_lb) {
          // Add the constraint -x + s = -lb and s in the nonnegative orthant
          // cone.
          A_triplets->emplace_back(*A_row_count, var_index, -1);
          b->push_back(-var_bound.lower);
          (*bbcon_dual_indices)[i][j].first = *A_row_count;
          cones->push_back(clarabel::NonnegativeConeT<double>(1));
          ++(*A_row_count);
        }
      }
    }
  }
}

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

template <typename T, typename U>
void SetClarabelSetting(std::unordered_map<std::string, U>* solver_options,
                        const std::string& name, T* clarabel_setting) {
  auto it = solver_options->find(name);
  if (it != solver_options->end()) {
    *clarabel_setting = it->second;
    // We will erase the recognized option from solver_options. In the end,
    // if solver_options has remaining options, we will throw a warning
    // about the un-recognized options.
    solver_options->erase(it);
  }
}

clarabel::DefaultSettings<double> SetClarabelSettings(
    const SolverOptions& solver_options) {
  clarabel::DefaultSettings<double> settings =
      clarabel::DefaultSettingsBuilder<double>::default_settings().build();
  // Clarabel defaults to print to console. But that would create too much noise
  // on the console. So we overwrite the default value with
  // solver_options.get_print_to_console().
  settings.verbose = solver_options.get_print_to_console();

  std::unordered_map<std::string, double> solver_options_double =
      solver_options.GetOptionsDouble(ClarabelSolver::id());

  std::unordered_map<std::string, int> solver_options_int =
      solver_options.GetOptionsInt(ClarabelSolver::id());
  // The options can be found in Clarabel upstream code DefaultSettings.h
  SetClarabelSetting(&solver_options_int, "max_iter", &(settings.max_iter));
  SetClarabelSetting(&solver_options_double, "time_limit",
                     &(settings.time_limit));
  SetClarabelSetting(&solver_options_int, "verbose", &(settings.verbose));
  SetClarabelSetting(&solver_options_double, "max_step_fraction",
                     &(settings.max_step_fraction));
  SetClarabelSetting(&solver_options_double, "tol_gap_abs",
                     &(settings.tol_gap_abs));
  SetClarabelSetting(&solver_options_double, "tol_gap_rel",
                     &(settings.tol_gap_rel));
  SetClarabelSetting(&solver_options_double, "tol_feas", &(settings.tol_feas));
  SetClarabelSetting(&solver_options_double, "tol_infeas_abs",
                     &(settings.tol_infeas_abs));
  SetClarabelSetting(&solver_options_double, "tol_infeas_rel",
                     &(settings.tol_infeas_rel));
  SetClarabelSetting(&solver_options_double, "tol_ktratio",
                     &(settings.tol_ktratio));
  SetClarabelSetting(&solver_options_double, "reduced_tol_gap_abs",
                     &(settings.reduced_tol_gap_abs));
  SetClarabelSetting(&solver_options_double, "reduced_tol_gap_rel",
                     &(settings.reduced_tol_gap_rel));
  SetClarabelSetting(&solver_options_double, "reduced_tol_feas",
                     &(settings.reduced_tol_feas));
  SetClarabelSetting(&solver_options_double, "reduced_tol_infeas_abs",
                     &(settings.reduced_tol_infeas_abs));
  SetClarabelSetting(&solver_options_double, "reduced_tol_infeas_rel",
                     &(settings.reduced_tol_infeas_rel));
  SetClarabelSetting(&solver_options_double, "reduced_tol_ktratio",
                     &(settings.reduced_tol_ktratio));
  SetClarabelSetting(&solver_options_int, "equilibrate_enable",
                     &(settings.equilibrate_enable));
  SetClarabelSetting(&solver_options_int, "equilibrate_max_iter",
                     &(settings.equilibrate_max_iter));
  SetClarabelSetting(&solver_options_double, "equilibrate_min_scaling",
                     &(settings.equilibrate_min_scaling));
  SetClarabelSetting(&solver_options_double, "equilibrate_max_scaling",
                     &(settings.equilibrate_max_scaling));
  SetClarabelSetting(&solver_options_double, "linesearch_backtrack_step",
                     &(settings.linesearch_backtrack_step));
  SetClarabelSetting(&solver_options_double, "min_switch_step_length",
                     &(settings.min_switch_step_length));
  SetClarabelSetting(&solver_options_double, "min_terminate_step_length",
                     &(settings.min_terminate_step_length));
  SetClarabelSetting(&solver_options_int, "direct_kkt_solver",
                     &(settings.direct_kkt_solver));
  SetClarabelSetting(&solver_options_int, "static_regularization_enable",
                     &(settings.static_regularization_enable));
  SetClarabelSetting(&solver_options_double, "static_regularization_constant",
                     &(settings.static_regularization_constant));
  SetClarabelSetting(&solver_options_double,
                     "static_regularization_proportional",
                     &(settings.static_regularization_proportional));
  SetClarabelSetting(&solver_options_int, "dynamic_regularization_enable",
                     &(settings.dynamic_regularization_enable));
  SetClarabelSetting(&solver_options_double, "dynamic_regularization_eps",
                     &(settings.dynamic_regularization_eps));
  SetClarabelSetting(&solver_options_double, "&dynamic_regularization_delta",
                     &(settings.dynamic_regularization_delta));
  SetClarabelSetting(&solver_options_int, "iterative_refinement_enable",
                     &(settings.iterative_refinement_enable));
  SetClarabelSetting(&solver_options_double, "iterative_refinement_reltol",
                     &(settings.iterative_refinement_reltol));
  SetClarabelSetting(&solver_options_double, "iterative_refinement_abstol",
                     &(settings.iterative_refinement_abstol));
  SetClarabelSetting(&solver_options_int, "iterative_refinement_max_iter",
                     &(settings.iterative_refinement_max_iter));
  SetClarabelSetting(&solver_options_double, "iterative_refinement_stop_ratio",
                     &(settings.iterative_refinement_stop_ratio));
  SetClarabelSetting(&solver_options_int, "presolve_enable",
                     &(settings.presolve_enable));
  if (!solver_options_int.empty()) {
    for (const auto& [option_name, option_value] : solver_options_int) {
      throw std::runtime_error(fmt::format(
          "ClarabelSolver: unrecognized solver options {}. Please check "
          "https://github.com/oxfordcontrol/Clarabel.cpp/blob/main/include/cpp/"
          "DefaultSettings.h for all supported solver options.",
          option_name));
    }
  }
  if (!solver_options_double.empty()) {
    for (const auto& [option_name, option_value] : solver_options_double) {
      throw std::runtime_error(fmt::format(
          "ClarabelSolver: unrecognized solver options {}. Please check "
          "https://github.com/oxfordcontrol/Clarabel.cpp/blob/main/include/cpp/"
          "DefaultSettings.h for all supported solver options.",
          option_name));
    }
  }
  return settings;
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

  // Clarabel takes an Eigen::Sparse matrix as A. We will build this sparse
  // matrix from the triplets recording its non-zero entries.
  std::vector<Eigen::Triplet<double>> A_triplets;
  // Since we add an array of quadratic cost, each cost has its own
  // associated variables. We use Eigen sparse matrix to aggregate the quadratic
  // cost Hessian matrices. Clarabel only takes the upper triangular entries of
  // the symmetric Hessian.
  std::vector<Eigen::Triplet<double>> P_upper_triplets;

  // A_row_count will increment, when we add each constraint.
  int A_row_count = 0;
  std::vector<double> b;

  std::vector<clarabel::SupportedConeT<double>> cones;

  // `q` is the coefficient in the linear cost qᵀx
  std::vector<double> q(num_x, 0.0);

  // Our costs (LinearCost, QuadraticCost, etc) also allows a constant term, we
  // add these constant terms to `cost_constant`.
  double cost_constant{0};

  internal::ParseLinearCosts(prog, &q, &cost_constant);

  internal::ParseQuadraticCosts(prog, &P_upper_triplets, &q, &cost_constant);

  // Parse linear equality constraint
  // linear_eq_y_start_indices[i] is the starting index of the dual
  // variable for the constraint prog.linear_equality_constraints()[i]. Namely
  // y[linear_eq_y_start_indices[i]:
  // linear_eq_y_start_indices[i] +
  // prog.linear_equality_constraints()[i].evaluator()->num_constraints()] are
  // the dual variables for the linear equality constraint
  // prog.linear_equality_constraint()(i), where y is the vector containing all
  // dual variables.
  std::vector<int> linear_eq_y_start_indices;
  int num_linear_equality_constraints_rows;
  internal::ParseLinearEqualityConstraints(
      prog, &A_triplets, &b, &A_row_count, &linear_eq_y_start_indices,
      &num_linear_equality_constraints_rows);
  cones.push_back(
      clarabel::ZeroConeT<double>(num_linear_equality_constraints_rows));

  // Parse bounding box constraints.
  // bbcon_dual_indices[i][j][0] (resp. bbcon_dual_indices[i][j][1]) is the dual
  // variable for the lower (resp. upper) bound of the j'th row in the bounding
  // box constraint prog.bounding_box_constraint()[i]
  // Since the dual variables are constrained to be positive, we use -1 to
  // indicate that this bound can never be active (for example, another
  // BoundingBoxConstraint imposes a tighter bound on the same variable).
  std::vector<std::vector<std::pair<int, int>>> bbcon_dual_indices;
  ParseBoundingBoxConstraints(prog, &A_triplets, &b, &A_row_count, &cones,
                              &bbcon_dual_indices);

  // Parse linear constraint
  // linear_constraint_dual_indices[i][j][0]/linear_constraint_dual_indices[i][j][1]
  // is the dual variable for the lower/upper bound of the j'th row in the
  // linear constraint prog.linear_constraint()[i], we use -1 to indicate that
  // the lower or upper bound is infinity.
  std::vector<std::vector<std::pair<int, int>>> linear_constraint_dual_indices;
  int num_linear_constraint_rows = 0;
  internal::ParseLinearConstraints(prog, &A_triplets, &b, &A_row_count,
                                   &linear_constraint_dual_indices,
                                   &num_linear_constraint_rows);
  cones.push_back(
      clarabel::NonnegativeConeT<double>(num_linear_constraint_rows));

  // Parse Lorentz cone and rotated Lorentz cone constraint
  std::vector<int> second_order_cone_length;
  // y[lorentz_cone_y_start_indices[i]:
  //   lorentz_cone_y_start_indices[i] + second_order_cone_length[i]]
  // are the dual variables for prog.lorentz_cone_constraints()[i].
  std::vector<int> lorentz_cone_y_start_indices;
  std::vector<int> rotated_lorentz_cone_y_start_indices;
  internal::ParseSecondOrderConeConstraints(
      prog, &A_triplets, &b, &A_row_count, &second_order_cone_length,
      &lorentz_cone_y_start_indices, &rotated_lorentz_cone_y_start_indices);
  for (const int soc_length : second_order_cone_length) {
    cones.push_back(clarabel::SecondOrderConeT<double>(soc_length));
  }

  std::vector<int> psd_cone_length;
  internal::ParsePositiveSemidefiniteConstraints(
      prog, /* upper triangular = */ true, &A_triplets, &b, &A_row_count,
      &psd_cone_length);
  for (const int length : psd_cone_length) {
    cones.push_back(clarabel::PSDTriangleConeT<double>(length));
  }

  internal::ParseExponentialConeConstraints(prog, &A_triplets, &b,
                                            &A_row_count);
  for (int i = 0; i < ssize(prog.exponential_cone_constraints()); ++i) {
    cones.push_back(clarabel::ExponentialConeT<double>());
  }

  Eigen::SparseMatrix<double> P(num_x, num_x);
  P.setFromTriplets(P_upper_triplets.begin(), P_upper_triplets.end());
  Eigen::Map<Eigen::VectorXd> q_vec{q.data(), ssize(q)};
  Eigen::SparseMatrix<double> A(A_row_count, num_x);
  A.setFromTriplets(A_triplets.begin(), A_triplets.end());
  const Eigen::Map<Eigen::VectorXd> b_vec{b.data(), ssize(b)};

  clarabel::DefaultSettings<double> settings =
      SetClarabelSettings(merged_options);

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
