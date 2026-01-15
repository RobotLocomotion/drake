#include "drake/solvers/clarabel_solver.h"

#include <fstream>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Eigen>
#include <clarabel.hpp>
#include <fmt/ranges.h>

#include "drake/common/fmt_eigen.h"
#include "drake/common/name_value.h"
#include "drake/common/text_logging.h"
#include "drake/math/matrix_util.h"
#include "drake/solvers/aggregate_costs_constraints.h"
#include "drake/solvers/scs_clarabel_common.h"
#include "drake/tools/workspace/clarabel_cpp_internal/serialize.h"

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
    case clarabel::SolverStatus::CallbackTerminated:
      return "CallbackTerminated";
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

// See ParseBoundingBoxConstraints for the meaning of bbcon_dual_indices.
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

void WriteClarabelReproduction(
    std::string filename, const Eigen::SparseMatrix<double>& P,
    const Eigen::Map<Eigen::VectorXd>& q_vec,
    const Eigen::SparseMatrix<double>& A,
    const Eigen::Map<Eigen::VectorXd>& b_vec,
    const std::vector<clarabel::SupportedConeT<double>>& cones) {
  std::ofstream out_file(filename);
  if (!out_file.is_open()) {
    log()->error(
        "Failed to open kStandaloneReproductionFileName {} for writing; no "
        "reproduction will be generated.");
    return;
  }

  out_file << fmt::format(
      R"""(
import clarabel
import numpy as np
from scipy import sparse

q = [{}]
b = [{}]
)""",
      fmt::join(q_vec.data(), q_vec.data() + q_vec.size(), ", "),
      fmt::join(b_vec.data(), b_vec.data() + b_vec.size(), ", "));
  out_file << math::GeneratePythonCsc(P, "P");
  out_file << math::GeneratePythonCsc(A, "A");
  out_file << "cones = [" << std::endl;

  for (const clarabel::SupportedConeT<double>& cone : cones) {
    switch (cone.tag) {
      case clarabel::SupportedConeT<double>::Tag::ZeroConeT:
        out_file << "  clarabel.ZeroConeT(" << cone.nvars() << "),"
                 << std::endl;
        break;
      case clarabel::SupportedConeT<double>::Tag::NonnegativeConeT:
        out_file << "  clarabel.NonnegativeConeT(" << cone.nvars() << "),"
                 << std::endl;
        break;
      case clarabel::SupportedConeT<double>::Tag::SecondOrderConeT:
        out_file << "  clarabel.SecondOrderConeT(" << cone.nvars() << "),"
                 << std::endl;
        break;
      case clarabel::SupportedConeT<double>::Tag::PSDTriangleConeT: {
        const clarabel::PSDTriangleConeT<double>* psd_cone =
            static_cast<const clarabel::PSDTriangleConeT<double>*>(&cone);
        out_file << "  clarabel.PSDTriangleConeT(" << psd_cone->dimension()
                 << ")," << std::endl;
        break;
      }
      case clarabel::SupportedConeT<double>::Tag::ExponentialConeT:
        out_file << "  clarabel.ExponentialConeT()," << std::endl;
        break;
      default:
        log()->error(
            "WriteClarabelReproduction: Found unsupported cone type; "
            "reproduction will likely be invalid.");
    }
  }

  // TODO(russt): write solver options.
  out_file << R"""(]

settings = clarabel.DefaultSettings()
solver = clarabel.DefaultSolver(P, q, A, b, cones, settings)
solver.solve()
)""";

  log()->info("Clarabel reproduction successfully written to {}.", filename);

  out_file.close();
}

}  // namespace

bool ClarabelSolver::is_available() {
  return true;
}

void ClarabelSolver::DoSolve2(const MathematicalProgram& prog,
                              const Eigen::VectorXd& initial_guess,
                              internal::SpecificOptions* options,
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

  std::vector<int> l2norm_costs_second_order_cone_length;
  std::vector<int> l2norm_costs_lorentz_cone_y_start_indices;
  std::vector<int> l2norm_costs_t_slack_indices;
  internal::ParseL2NormCosts(prog, &num_x, &A_triplets, &b, &A_row_count,
                             &l2norm_costs_second_order_cone_length,
                             &l2norm_costs_lorentz_cone_y_start_indices, &q,
                             &l2norm_costs_t_slack_indices);
  for (const int soc_length : l2norm_costs_second_order_cone_length) {
    cones.push_back(clarabel::SecondOrderConeT<double>(soc_length));
  }

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
  if (num_linear_equality_constraints_rows > 0) {
    cones.push_back(
        clarabel::ZeroConeT<double>(num_linear_equality_constraints_rows));
  }

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
  if (num_linear_constraint_rows > 0) {
    cones.push_back(
        clarabel::NonnegativeConeT<double>(num_linear_constraint_rows));
  }

  // Parse scalar PSD constraint as linear constraint.
  int scalar_psd_positive_cone_length{};
  std::vector<std::optional<int>> scalar_psd_dual_indices;
  std::vector<std::optional<int>> scalar_lmi_dual_indices;
  internal::ParseScalarPositiveSemidefiniteConstraints(
      prog, &A_triplets, &b, &A_row_count, &scalar_psd_positive_cone_length,
      &scalar_psd_dual_indices, &scalar_lmi_dual_indices);
  if (scalar_psd_positive_cone_length > 0) {
    cones.push_back(
        clarabel::NonnegativeConeT<double>(scalar_psd_positive_cone_length));
  }

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
  // Parse PSD/LMI constraints on 2x2 matrices as second order cone constraints.
  int num_second_order_cones_from_psd{};
  std::vector<std::optional<int>> twobytwo_psd_y_start_indices;
  std::vector<std::optional<int>> twobytwo_lmi_y_start_indices;
  internal::Parse2x2PositiveSemidefiniteConstraints(
      prog, &A_triplets, &b, &A_row_count, &num_second_order_cones_from_psd,
      &twobytwo_psd_y_start_indices, &twobytwo_lmi_y_start_indices);
  for (int i = 0; i < num_second_order_cones_from_psd; ++i) {
    cones.push_back(clarabel::SecondOrderConeT<double>(3));
  }

  std::vector<std::optional<int>> psd_cone_length;
  std::vector<std::optional<int>> lmi_cone_length;
  std::vector<std::optional<int>> psd_y_start_indices;
  std::vector<std::optional<int>> lmi_y_start_indices;
  internal::ParsePositiveSemidefiniteConstraints(
      prog, /* upper triangular = */ true, &A_triplets, &b, &A_row_count,
      &psd_cone_length, &lmi_cone_length, &psd_y_start_indices,
      &lmi_y_start_indices);
  for (const auto& length : psd_cone_length) {
    if (length.has_value()) {
      cones.push_back(clarabel::PSDTriangleConeT<double>(*length));
    }
  }
  for (const auto& length : lmi_cone_length) {
    if (length.has_value()) {
      cones.push_back(clarabel::PSDTriangleConeT<double>(*length));
    }
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

  options->Respell([&](const auto& common, auto* respelled) {
    respelled->emplace("verbose", common.print_to_console ? 1 : 0);
    // TODO(jwnimmer-tri) Handle common.print_file_name.
    if (!common.standalone_reproduction_file_name.empty()) {
      WriteClarabelReproduction(common.standalone_reproduction_file_name, P,
                                q_vec, A, b_vec, cones);
    }
    if (common.max_threads.has_value()) {
      respelled->emplace("max_threads", common.max_threads.value());
    }
  });
  clarabel::DefaultSettings<double> settings =
      clarabel::DefaultSettingsBuilder<double>::default_settings().build();
  options->CopyToSerializableStruct(&settings);

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

  SetBoundingBoxDualSolution(prog, solution.z, bbcon_dual_indices, result);
  internal::SetDualSolution(
      prog, solution.z, linear_constraint_dual_indices,
      linear_eq_y_start_indices, lorentz_cone_y_start_indices,
      rotated_lorentz_cone_y_start_indices, psd_y_start_indices,
      lmi_y_start_indices, scalar_psd_dual_indices, scalar_lmi_dual_indices,
      twobytwo_psd_y_start_indices, twobytwo_lmi_y_start_indices,
      /*upper_triangular_psd=*/true, result);
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
