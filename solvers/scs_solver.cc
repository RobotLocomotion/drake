#include "drake/solvers/scs_solver.h"

#include <fstream>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Sparse>
#include <fmt/format.h>
#include <fmt/ranges.h>

// clang-format off
#include <scs.h>
#include <cones.h>
#include <linalg.h>
#include <util.h>
// clang-format on

#include "drake/common/scope_exit.h"
#include "drake/common/text_logging.h"
#include "drake/math/eigen_sparse_triplet.h"
#include "drake/math/matrix_util.h"
#include "drake/math/quadratic_form.h"
#include "drake/solvers/aggregate_costs_constraints.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"
#include "drake/solvers/scs_clarabel_common.h"

// This function must appear in the global namespace -- the Serialize pattern
// uses ADL (argument-dependent lookup) and the namespace for the ScsSettings
// struct is the global namespace. (We can't even use an anonymous namespace!)
static void Serialize(
    drake::solvers::internal::SpecificOptions* archive,
    // NOLINTNEXTLINE(runtime/references) to match Serialize concept.
    ScsSettings& settings) {
  using drake::MakeNameValue;
  archive->Visit(MakeNameValue("normalize", &settings.normalize));
  archive->Visit(MakeNameValue("scale", &settings.scale));
  archive->Visit(MakeNameValue("adaptive_scale", &settings.adaptive_scale));
  archive->Visit(MakeNameValue("rho_x", &settings.rho_x));
  archive->Visit(MakeNameValue("max_iters", &settings.max_iters));
  archive->Visit(MakeNameValue("eps_abs", &settings.eps_abs));
  archive->Visit(MakeNameValue("eps_rel", &settings.eps_rel));
  archive->Visit(MakeNameValue("eps_infeas", &settings.eps_infeas));
  archive->Visit(MakeNameValue("alpha", &settings.alpha));
  archive->Visit(MakeNameValue("time_limit_secs", &settings.time_limit_secs));
  archive->Visit(MakeNameValue("verbose", &settings.verbose));
  archive->Visit(MakeNameValue("warm_start", &settings.warm_start));
  archive->Visit(MakeNameValue("acceleration_lookback",  // BR
                               &settings.acceleration_lookback));
  archive->Visit(MakeNameValue("acceleration_interval",  // BR
                               &settings.acceleration_interval));
  // TODO(jwnimmer-tri) Handle write_data_filename.
  // TODO(jwnimmer-tri) Handle log_csv_filename.
}

namespace drake {
namespace solvers {
namespace {

void ParseQuadraticCostWithRotatedLorentzCone(
    const MathematicalProgram& prog, std::vector<double>* c,
    std::vector<Eigen::Triplet<double>>* A_triplets, std::vector<double>* b,
    int* A_row_count, std::vector<int>* second_order_cone_length, int* num_x) {
  // A QuadraticCost encodes cost of the form
  //   0.5 zᵀQz + pᵀz + r
  // We introduce a new slack variable y as the upper bound of the cost, with
  // the rotated Lorentz cone constraint
  // 2(y - r - pᵀz) ≥ zᵀQz.
  // We only need to minimize y then.
  for (const auto& cost : prog.quadratic_costs()) {
    // We will convert the expression 2(y - r - pᵀz) ≥ zᵀQz, to the constraint
    // that the vector A_cone * x + b_cone is in the rotated Lorentz cone, where
    // x = [z; y], and A_cone * x + b_cone is
    // [y - r - pᵀz]
    // [          2]
    // [        C*z]
    // where C satisfies Cᵀ*C = Q
    const VectorXDecisionVariable& z = cost.variables();
    const int y_index = z.rows();
    // Ai_triplets are the non-zero entries in the matrix A_cone.
    std::vector<Eigen::Triplet<double>> Ai_triplets;
    Ai_triplets.emplace_back(0, y_index, 1);
    for (int i = 0; i < z.rows(); ++i) {
      Ai_triplets.emplace_back(0, i, -cost.evaluator()->b()(i));
    }
    // Decompose Q to Cᵀ*C
    const Eigen::MatrixXd& Q = cost.evaluator()->Q();
    const Eigen::MatrixXd C =
        math::DecomposePSDmatrixIntoXtransposeTimesX(Q, 1E-10);
    for (int i = 0; i < C.rows(); ++i) {
      for (int j = 0; j < C.cols(); ++j) {
        if (C(i, j) != 0) {
          Ai_triplets.emplace_back(2 + i, j, C(i, j));
        }
      }
    }
    // append the variable y to the end of x
    (*num_x)++;
    std::vector<int> Ai_var_indices = prog.FindDecisionVariableIndices(z);
    Ai_var_indices.push_back(*num_x - 1);
    // Set b_cone
    Eigen::VectorXd b_cone = Eigen::VectorXd::Zero(2 + C.rows());
    b_cone(0) = -cost.evaluator()->c();
    b_cone(1) = 2;
    // Add the rotated Lorentz cone constraint
    internal::ParseRotatedLorentzConeConstraint(
        Ai_triplets, b_cone, Ai_var_indices, A_triplets, b, A_row_count,
        second_order_cone_length, std::nullopt);
    // Add the cost y.
    c->push_back(1);
  }
}

void ParseBoundingBoxConstraint(
    const MathematicalProgram& prog,
    std::vector<Eigen::Triplet<double>>* A_triplets, std::vector<double>* b,
    int* A_row_count, ScsCone* cone,
    std::vector<std::vector<std::pair<int, int>>>* bbcon_dual_indices) {
  // A bounding box constraint lb ≤ x ≤ ub is converted to the SCS form as
  // x + s1 = ub, -x + s2 = -lb, s1, s2 in the positive cone.
  // TODO(hongkai.dai) : handle the special case l = u, such that we can convert
  // it to x + s = l, s in zero cone.
  int num_bounding_box_constraint_rows = 0;
  bbcon_dual_indices->reserve(prog.bounding_box_constraints().size());
  for (const auto& bounding_box_constraint : prog.bounding_box_constraints()) {
    int num_scs_new_constraint = 0;
    const VectorXDecisionVariable& xi = bounding_box_constraint.variables();
    const int num_xi_rows = xi.rows();
    A_triplets->reserve(A_triplets->size() + 2 * num_xi_rows);
    b->reserve(b->size() + 2 * num_xi_rows);
    bbcon_dual_indices->emplace_back(num_xi_rows);
    for (int i = 0; i < num_xi_rows; ++i) {
      std::pair<int, int> dual_index = {-1, -1};
      if (!std::isinf(bounding_box_constraint.evaluator()->upper_bound()(i))) {
        // if ub != ∞, then add the constraint x + s1 = ub, s1 in the positive
        // cone.
        A_triplets->emplace_back(num_scs_new_constraint + *A_row_count,
                                 prog.FindDecisionVariableIndex(xi(i)), 1);
        b->push_back(bounding_box_constraint.evaluator()->upper_bound()(i));
        dual_index.second = num_scs_new_constraint + *A_row_count;
        ++num_scs_new_constraint;
      }
      if (!std::isinf(bounding_box_constraint.evaluator()->lower_bound()(i))) {
        // if lb != -∞, then add the constraint -x + s2 = -lb, s2 in the
        // positive cone.
        A_triplets->emplace_back(num_scs_new_constraint + *A_row_count,
                                 prog.FindDecisionVariableIndex(xi(i)), -1);
        b->push_back(-bounding_box_constraint.evaluator()->lower_bound()(i));
        dual_index.first = num_scs_new_constraint + *A_row_count;
        ++num_scs_new_constraint;
      }
      bbcon_dual_indices->back()[i] = dual_index;
    }
    *A_row_count += num_scs_new_constraint;
    num_bounding_box_constraint_rows += num_scs_new_constraint;
  }
  cone->l += num_bounding_box_constraint_rows;
}

std::string Scs_return_info(scs_int scs_status) {
  switch (scs_status) {
    case SCS_INFEASIBLE_INACCURATE:
      return "SCS infeasible inaccurate";
    case SCS_UNBOUNDED_INACCURATE:
      return "SCS unbounded inaccurate";
    case SCS_SIGINT:
      return "SCS sigint";
    case SCS_FAILED:
      return "SCS failed";
    case SCS_INDETERMINATE:
      return "SCS indeterminate";
    case SCS_INFEASIBLE:
      return "SCS primal infeasible, dual unbounded";
    case SCS_UNBOUNDED:
      return "SCS primal unbounded, dual infeasible";
    case SCS_UNFINISHED:
      return "SCS unfinished";
    case SCS_SOLVED:
      return "SCS solved";
    case SCS_SOLVED_INACCURATE:
      return "SCS solved inaccurate";
    default:
      throw std::runtime_error("Unknown scs status.");
  }
}

void SetScsProblemData(
    int A_row_count, int num_vars, const Eigen::SparseMatrix<double>& A,
    const std::vector<double>& b,
    const std::vector<Eigen::Triplet<double>>& P_upper_triplets,
    const std::vector<double>& c, ScsData* scs_problem_data) {
  scs_problem_data->m = A_row_count;
  scs_problem_data->n = num_vars;

  scs_problem_data->A = static_cast<ScsMatrix*>(malloc(sizeof(ScsMatrix)));
  // This scs_calloc doesn't need to accompany a ScopeExit since
  // scs_problem_data->A->x will be cleaned up recursively by freeing up
  // scs_problem_data in scs_free_data()
  scs_problem_data->A->x =
      static_cast<scs_float*>(scs_calloc(A.nonZeros(), sizeof(scs_float)));

  // This scs_calloc doesn't need to accompany a ScopeExit since
  // scs_problem_data->A->i will be cleaned up recursively by freeing up
  // scs_problem_data in scs_free_data()
  scs_problem_data->A->i =
      static_cast<scs_int*>(scs_calloc(A.nonZeros(), sizeof(scs_int)));

  // This scs_calloc doesn't need to accompany a ScopeExit since
  // scs_problem_data->A->p will be cleaned up recursively by freeing up
  // scs_problem_data in scs_free_data()
  scs_problem_data->A->p = static_cast<scs_int*>(
      scs_calloc(scs_problem_data->n + 1, sizeof(scs_int)));

  // TODO(hongkai.dai): should I use memcpy for the assignment in the for loop?
  for (int i = 0; i < A.nonZeros(); ++i) {
    scs_problem_data->A->x[i] = *(A.valuePtr() + i);
    scs_problem_data->A->i[i] = *(A.innerIndexPtr() + i);
  }
  for (int i = 0; i < scs_problem_data->n + 1; ++i) {
    scs_problem_data->A->p[i] = *(A.outerIndexPtr() + i);
  }
  scs_problem_data->A->m = scs_problem_data->m;
  scs_problem_data->A->n = scs_problem_data->n;

  // This scs_calloc doesn't need to accompany a ScopeExit since
  // scs_problem_data->b will be cleaned up recursively by freeing up
  // scs_problem_data in scs_free_data()
  scs_problem_data->b =
      static_cast<scs_float*>(scs_calloc(b.size(), sizeof(scs_float)));

  for (int i = 0; i < static_cast<int>(b.size()); ++i) {
    scs_problem_data->b[i] = b[i];
  }

  if (P_upper_triplets.empty()) {
    scs_problem_data->P = SCS_NULL;
  } else {
    Eigen::SparseMatrix<double> P_upper(num_vars, num_vars);
    P_upper.setFromTriplets(P_upper_triplets.begin(), P_upper_triplets.end());
    scs_problem_data->P = static_cast<ScsMatrix*>(malloc(sizeof(ScsMatrix)));
    // This scs_calloc doesn't need to accompany a ScopeExit since
    // scs_problem_data->P->x will be cleaned up recursively by freeing up
    // scs_problem_data in scs_free_data()
    scs_problem_data->P->x = static_cast<scs_float*>(
        scs_calloc(P_upper.nonZeros(), sizeof(scs_float)));

    // This scs_calloc doesn't need to accompany a ScopeExit since
    // scs_problem_data->P->i will be cleaned up recursively by freeing up
    // scs_problem_data in scs_free_data()
    scs_problem_data->P->i =
        static_cast<scs_int*>(scs_calloc(P_upper.nonZeros(), sizeof(scs_int)));

    // This scs_calloc doesn't need to accompany a ScopeExit since
    // scs_problem_data->P->p will be cleaned up recursively by freeing up
    // scs_problem_data in scs_free_data()
    scs_problem_data->P->p = static_cast<scs_int*>(
        scs_calloc(scs_problem_data->n + 1, sizeof(scs_int)));
    for (int i = 0; i < P_upper.nonZeros(); ++i) {
      scs_problem_data->P->x[i] = *(P_upper.valuePtr() + i);
      scs_problem_data->P->i[i] = *(P_upper.innerIndexPtr() + i);
    }
    for (int i = 0; i < scs_problem_data->n + 1; ++i) {
      scs_problem_data->P->p[i] = *(P_upper.outerIndexPtr() + i);
    }
    scs_problem_data->P->m = scs_problem_data->n;
    scs_problem_data->P->n = scs_problem_data->n;
  }

  // This scs_calloc doesn't need to accompany a ScopeExit since
  // scs_problem_data->c will be cleaned up recursively by freeing up
  // scs_problem_data in scs_free_data()
  scs_problem_data->c =
      static_cast<scs_float*>(scs_calloc(num_vars, sizeof(scs_float)));
  for (int i = 0; i < num_vars; ++i) {
    scs_problem_data->c[i] = c[i];
  }
}

void SetBoundingBoxDualSolution(
    const MathematicalProgram& prog, const Eigen::Ref<const Eigen::VectorXd>& y,
    const std::vector<std::vector<std::pair<int, int>>>& bbcon_dual_indices,
    MathematicalProgramResult* result) {
  for (int i = 0; i < static_cast<int>(prog.bounding_box_constraints().size());
       ++i) {
    Eigen::VectorXd bbcon_dual = Eigen::VectorXd::Zero(
        prog.bounding_box_constraints()[i].variables().rows());
    for (int j = 0; j < prog.bounding_box_constraints()[i].variables().rows();
         ++j) {
      if (bbcon_dual_indices[i][j].first != -1) {
        // lower bound is not infinity.
        // The shadow price for the lower bound is positive. SCS dual for the
        // positive cone is also positive, so we add the SCS dual.
        bbcon_dual[j] += y(bbcon_dual_indices[i][j].first);
      }
      if (bbcon_dual_indices[i][j].second != -1) {
        // upper bound is not infinity.
        // The shadow price for the upper bound is negative. SCS dual for the
        // positive cone is positive, so we subtract the SCS dual.
        bbcon_dual[j] -= y(bbcon_dual_indices[i][j].second);
      }
    }
    result->set_dual_solution(prog.bounding_box_constraints()[i], bbcon_dual);
  }
}

void WriteScsReproduction(std::string filename,
                          const Eigen::SparseMatrix<double>& P,
                          const Eigen::Map<Eigen::VectorXd>& c_vec,
                          const Eigen::SparseMatrix<double>& A,
                          const Eigen::Map<Eigen::VectorXd>& b_vec,
                          const ScsCone& cone) {
  std::ofstream out_file(filename);
  if (!out_file.is_open()) {
    log()->error(
        "Failed to open kStandaloneReproductionFileName {} for writing; no "
        "reproduction will be generated.");
    return;
  }
  out_file << fmt::format(
      R"""(
import scs
import numpy as np
from scipy import sparse

c = np.array([{}], dtype=np.float64)
b = np.array([{}], dtype=np.float64)
)""",
      fmt::join(c_vec.data(), c_vec.data() + c_vec.size(), ", "),
      fmt::join(b_vec.data(), b_vec.data() + b_vec.size(), ", "));

  out_file << math::GeneratePythonCsc(P, "P");
  out_file << math::GeneratePythonCsc(A, "A");

  out_file << "data=dict(P=P, A=A, b=b, c=c)\n";

  out_file << fmt::format(
      R"""(cone=dict(z={}, l={}, bu=[{}], bl=[{}], q=[{}], s=[{}], ep={}, ed={}, p=[{}]))""",
      cone.z, cone.l, fmt::join(cone.bu, cone.bu + cone.bsize, ","),
      fmt::join(cone.bl, cone.bl + cone.bsize, ","),
      fmt::join(cone.q, cone.q + cone.qsize, ","),
      fmt::join(cone.s, cone.s + cone.ssize, ","), cone.ep, cone.ed,
      fmt::join(cone.p, cone.p + cone.psize, ","));

  // TODO(hongkai.dai): write solver options.
  out_file << R"""(

solver = scs.SCS(data, cone)
sol = solver.solve()
)""";
  log()->info("SCS reproduction successfully written to {}.", filename);

  out_file.close();
}
}  // namespace

bool ScsSolver::is_available() {
  return true;
}

void ScsSolver::DoSolve2(const MathematicalProgram& prog,
                         const Eigen::VectorXd& initial_guess,
                         internal::SpecificOptions* options,
                         MathematicalProgramResult* result) const {
  if (!prog.GetVariableScaling().empty()) {
    static const logging::Warn log_once(
        "ScsSolver doesn't support the feature of variable scaling.");
  }

  // TODO(hongkai.dai): allow warm starting SCS with initial guess on
  // primal/dual variables and primal residues.
  unused(initial_guess);
  // The initial guess for SCS is unused.
  // SCS solves the problem in this form
  // min 0.5xᵀPx + cᵀx
  // s.t A x + s = b
  //     s in K
  // where K is a Cartesian product of some primitive cones.
  // The cones have to be in this order
  // Zero cone {x | x = 0 }
  // Positive orthant {x | x ≥ 0 }
  // Second-order cone {(t, x) | |x|₂ ≤ t }
  // Positive semidefinite cone { X | min(eig(X)) ≥ 0, X = Xᵀ }
  // There are more types that are supported by SCS, after the Positive
  // semidefinite cone. Please refer to https://github.com/cvxgrp/scs for more
  // details on the cone types.
  // Notice that due to the special problem form supported by SCS, we need to
  // convert our generic constraints to SCS form. For example, a linear
  // inequality constraint
  //   A x ≤ b
  // will be converted as
  //   A x + s = b
  //   s in positive orthant cone.
  // by introducing the slack variable s.

  // num_x is the number of variables in `x`. It can contain more variables
  // than in prog. For some constraint/cost, we need to append slack variables
  // to convert the constraint/cost to the SCS form. For example, SCS does not
  // support un-constrained QP. So when we have an un-constrained QP, we need to
  // convert the quadratic cost
  //   min 0.5zᵀQz + bᵀz + d
  // to the form
  //   min y
  //   s.t 2(y - bᵀz - d) ≥ zᵀQz     (1)
  // now the cost is a linear function of y, with a rotated Lorentz cone
  // constraint(1). So we need to append the slack variable `y` to the variables
  // in `prog`.
  int num_x = prog.num_vars();

  // We need to construct a sparse matrix in Column Compressed Storage (CCS)
  // format. On the other hand, we add the constraint row by row, instead of
  // column by column, as preferred by CCS. As a result, we use Eigen sparse
  // matrix to construct a sparse matrix in column order first, and then
  // compress it to get CCS.
  std::vector<Eigen::Triplet<double>> A_triplets;

  // We need to construct a sparse matrix in Column Compressed Storage (CCS)
  // format. Since we add an array of quadratic cost, each cost has its own
  // associated variables, we use Eigen sparse matrix to aggregate the quadratic
  // cost Hessian matrices. SCS only takes the upper triangular entries of the
  // symmetric Hessian.
  std::vector<Eigen::Triplet<double>> P_upper_triplets;

  // cone stores all the cones K in the problem.
  ScsCone* cone = static_cast<ScsCone*>(scs_calloc(1, sizeof(ScsCone)));
  ScsData* scs_problem_data =
      static_cast<ScsData*>(scs_calloc(1, sizeof(ScsData)));
  ScsSettings* scs_stgs =
      static_cast<ScsSettings*>(scs_calloc(1, sizeof(ScsSettings)));
  // This guard will free cone, scs_problem_data, and scs_stgs (together with
  // their instantiated members) upon return from the DoSolve function.
  ScopeExit scs_free_guard([&cone, &scs_problem_data, &scs_stgs]() {
    SCS(free_cone)(cone);
    SCS(free_data)(scs_problem_data);
    scs_free(scs_stgs);
  });

  // A_row_count will increment, when we add each constraint.
  int A_row_count = 0;
  std::vector<double> b;

  // `c` is the coefficient in the linear cost cᵀx
  std::vector<double> c(num_x, 0.0);

  // Our cost (LinearCost, QuadraticCost, etc) also allows a constant term, we
  // add these constant terms to `cost_constant`.
  double cost_constant{0};

  // Parse linear cost
  internal::ParseLinearCosts(prog, &c, &cost_constant);

  // Parse linear equality constraint
  // linear_eq_y_start_indices[i] is the starting index of the dual
  // variable for the constraint prog.linear_equality_constraints()[i]. Namely
  // y[linear_eq_y_start_indices[i]:
  // linear_eq_y_start_indices[i] +
  // prog.linear_equality_constraints()[i].evaluator()->num_constraints] are the
  // dual variables for  the linear equality constraint
  // prog.linear_equality_constraint()(i), where y is the vector containing all
  // dual variables.
  std::vector<int> linear_eq_y_start_indices;
  int num_linear_equality_constraints_rows;
  internal::ParseLinearEqualityConstraints(
      prog, &A_triplets, &b, &A_row_count, &linear_eq_y_start_indices,
      &num_linear_equality_constraints_rows);
  cone->z += num_linear_equality_constraints_rows;

  // Parse bounding box constraint
  // bbcon_dual_indices[i][j][0]/bbcon_dual_indices[i][j][1] is the dual
  // variable for the lower/upper bound of the j'th row in the bounding box
  // constraint prog.bounding_box_constraint()[i], we use -1 to indicate that
  // the lower or upper bound is infinity.
  std::vector<std::vector<std::pair<int, int>>> bbcon_dual_indices;
  ParseBoundingBoxConstraint(prog, &A_triplets, &b, &A_row_count, cone,
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
  cone->l += num_linear_constraint_rows;

  // Parse scalar PSD constraints as linear constraints.
  // SCS requires ordering the cone with the positive orthant cone coming before
  // the positive semidefinite cones. So we call
  // ParseScalarPositiveSemidefiniteConstraints() next to
  // ParseLinearConstraints(), and finally calling
  // ParsePositiveSemidefiniteConstraints().
  int scalar_psd_positive_cone_length{};
  std::vector<std::optional<int>> scalar_psd_dual_indices;
  std::vector<std::optional<int>> scalar_lmi_dual_indices;
  internal::ParseScalarPositiveSemidefiniteConstraints(
      prog, &A_triplets, &b, &A_row_count, &scalar_psd_positive_cone_length,
      &scalar_psd_dual_indices, &scalar_lmi_dual_indices);
  if (scalar_psd_positive_cone_length > 0) {
    cone->l += scalar_psd_positive_cone_length;
  }

  // Parse Lorentz cone and rotated Lorentz cone constraint
  std::vector<int> second_order_cone_length;
  // y[lorentz_cone_y_start_indices[i]:
  //   lorentz_cone_y_start_indices[i] + second_order_cone_length[i]]
  // are the dual variables for prog.lorentz_cone_constraints()[i].
  std::vector<int> lorentz_cone_y_start_indices;
  // y[rotated_lorentz_cone_y_start_indices[i]:
  // rotated_lorentz_cone_y_start_indices[i] +
  // prog.rotate_lorentz_cone()[i].evaluator().A().rows] are the y variables for
  // prog.rotated_lorentz_cone_constraints()[i]. Note that SCS doesn't have a
  // rotated Lorentz cone constraint, instead we convert the rotated Lorentz
  // cone constraint to the Lorentz cone constraint through a linear
  // transformation. Hence we need to apply the transpose of that linear
  // transformation on the y variable to get the dual variable in the dual cone
  // of rotated Lorentz cone.
  std::vector<int> rotated_lorentz_cone_y_start_indices;
  internal::ParseSecondOrderConeConstraints(
      prog, &A_triplets, &b, &A_row_count, &second_order_cone_length,
      &lorentz_cone_y_start_indices, &rotated_lorentz_cone_y_start_indices);

  // Add PSD or LMI constraint on 2x2 matrices. This must be called with the
  // other second order cone constraint parsing code before
  // ParsePositiveSemidefiniteConstraints() as the 2x2 PSD/LMI constraints are
  // formulated as second order cones.
  int num_second_order_cones_from_psd{};
  std::vector<std::optional<int>> twobytwo_psd_dual_start_indices;
  std::vector<std::optional<int>> twobytwo_lmi_dual_start_indices;
  internal::Parse2x2PositiveSemidefiniteConstraints(
      prog, &A_triplets, &b, &A_row_count, &num_second_order_cones_from_psd,
      &twobytwo_psd_dual_start_indices, &twobytwo_lmi_dual_start_indices);
  for (int i = 0; i < num_second_order_cones_from_psd; ++i) {
    second_order_cone_length.push_back(3);
  }

  // Add L2NormCost. L2NormCost should be parsed together with the other second
  // order cone constraints, since we introduce new second order cone
  // constraints to formulate the L2 norm cost.
  std::vector<int> l2norm_costs_lorentz_cone_y_start_indices;
  std::vector<int> l2norm_costs_t_slack_indices;
  internal::ParseL2NormCosts(prog, &num_x, &A_triplets, &b, &A_row_count,
                             &second_order_cone_length,
                             &l2norm_costs_lorentz_cone_y_start_indices, &c,
                             &l2norm_costs_t_slack_indices);

  // Parse quadratic cost. This MUST be called after parsing the second order
  // cone constraint, as we might convert quadratic cost to second order cone
  // constraint.
  if (A_triplets.empty() && prog.positive_semidefinite_constraints().empty() &&
      prog.linear_matrix_inequality_constraints().empty() &&
      prog.exponential_cone_constraints().empty()) {
    // A_triplets.empty() = true means that up to now (after looping through
    // box, linear and second-order cone constraints) no constraints have been
    // added to SCS. Combining this with the fact that the
    // positive semidefinite, linear matrix inequality and exponential cone
    // constraints are all empty, this means that `prog` is un-constrained. If
    // the program is un-constrained but with a quadratic cost, since SCS
    // doesn't handle un-constrained QP, we convert this un-constrained QP to a
    // program with linear cost and rotated Lorentz cone constraint.
    ParseQuadraticCostWithRotatedLorentzCone(prog, &c, &A_triplets, &b,
                                             &A_row_count,
                                             &second_order_cone_length, &num_x);
  } else {
    internal::ParseQuadraticCosts(prog, &P_upper_triplets, &c, &cost_constant);
  }

  // Set the lorentz cone length in the SCS cone.
  cone->qsize = second_order_cone_length.size();
  cone->q = static_cast<scs_int*>(scs_calloc(cone->qsize, sizeof(scs_int)));
  for (int i = 0; i < cone->qsize; ++i) {
    cone->q[i] = second_order_cone_length[i];
  }

  // Parse PositiveSemidefiniteConstraint and LinearMatrixInequalityConstraint.
  std::vector<std::optional<int>> psd_cone_length;
  std::vector<std::optional<int>> lmi_cone_length;
  std::vector<std::optional<int>> psd_y_start_indices;
  std::vector<std::optional<int>> lmi_y_start_indices;
  internal::ParsePositiveSemidefiniteConstraints(
      prog, /* upper_triangular = */ false, &A_triplets, &b, &A_row_count,
      &psd_cone_length, &lmi_cone_length, &psd_y_start_indices,
      &lmi_y_start_indices);
  // Set the psd cone length in the SCS cone.
  // Note that when psd_cone_length[i] = std::nullopt or lmi_cone_length[i] =
  // std::nullopt, the constraint will not be parsed as PSD cone in SCS. So we
  // should filter out these entries with value std::nullopt.
  cone->ssize = 0;
  for (const auto& length : psd_cone_length) {
    if (length.has_value()) {
      ++(cone->ssize);
    }
  }
  for (const auto& length : lmi_cone_length) {
    if (length.has_value()) {
      cone->ssize++;
    }
  }
  // This scs_calloc doesn't need to accompany a ScopeExit since cone->s will be
  // cleaned up recursively by freeing up cone in scs_free_data()
  cone->s = static_cast<scs_int*>(scs_calloc(cone->ssize, sizeof(scs_int)));
  int scs_psd_cone_count = 0;
  for (int i = 0; i < ssize(psd_cone_length); ++i) {
    if (psd_cone_length[i].has_value()) {
      cone->s[scs_psd_cone_count++] = *(psd_cone_length[i]);
    }
  }
  for (int i = 0; i < ssize(lmi_cone_length); ++i) {
    if (lmi_cone_length[i].has_value()) {
      cone->s[scs_psd_cone_count++] = *(lmi_cone_length[i]);
    }
  }

  // Parse ExponentialConeConstraint.
  internal::ParseExponentialConeConstraints(prog, &A_triplets, &b,
                                            &A_row_count);
  cone->ep = static_cast<int>(prog.exponential_cone_constraints().size());

  Eigen::SparseMatrix<double> A(A_row_count, num_x);
  A.setFromTriplets(A_triplets.begin(), A_triplets.end());
  A.makeCompressed();

  // Set the parameters to default values.
  scs_set_default_settings(scs_stgs);
  // Customize the defaults for Drake:
  // - SCS 3.0 uses 1E-4 as the default value, see
  //   https://www.cvxgrp.org/scs/api/settings.html?highlight=eps_abs). This
  //   tolerance is too loose. We set the default tolerance to 1E-5 for better
  //   accuracy.
  scs_stgs->eps_abs = 1E-5;
  // - SCS 3.0 uses 1E-4 as the default value, see
  //   https://www.cvxgrp.org/scs/api/settings.html?highlight=eps_rel). This
  //   tolerance is too loose. We set the default tolerance to 1E-5 for better
  //   accuracy.
  scs_stgs->eps_rel = 1E-5;
  // Apply the user's additional custom options (if any).
  options->Respell([&](const auto& common, auto* respelled) {
    respelled->emplace("verbose", common.print_to_console ? 1 : 0);
    // TODO(jwnimmer-tri) Handle common.print_file_name.
    if (!common.standalone_reproduction_file_name.empty()) {
      Eigen::SparseMatrix<double> P(num_x, num_x);
      P.setFromTriplets(P_upper_triplets.begin(), P_upper_triplets.end());
      WriteScsReproduction(common.standalone_reproduction_file_name, P,
                           Eigen::Map<Eigen::VectorXd>(c.data(), num_x), A,
                           Eigen::Map<Eigen::VectorXd>(b.data(), b.size()),
                           *cone);
    }
    // SCS does not support setting the number of threads so we ignore the
    // kMaxThreads option.
  });
  options->CopyToSerializableStruct(scs_stgs);

  SetScsProblemData(A_row_count, num_x, A, b, P_upper_triplets, c,
                    scs_problem_data);

  ScsInfo scs_info{0};

  ScsSolution* scs_sol =
      static_cast<ScsSolution*>(scs_calloc(1, sizeof(ScsSolution)));
  ScopeExit sol_guard([&scs_sol]() {
    SCS(free_sol)(scs_sol);
  });

  ScsSolverDetails& solver_details =
      result->SetSolverDetailsType<ScsSolverDetails>();
  solver_details.scs_status =
      scs(scs_problem_data, cone, scs_stgs, scs_sol, &scs_info);

  solver_details.iter = scs_info.iter;
  solver_details.primal_objective = scs_info.pobj;
  solver_details.dual_objective = scs_info.dobj;
  solver_details.primal_residue = scs_info.res_pri;
  solver_details.residue_infeasibility = scs_info.res_infeas;
  solver_details.residue_unbounded_a = scs_info.res_unbdd_a;
  solver_details.residue_unbounded_p = scs_info.res_unbdd_p;
  solver_details.duality_gap = scs_info.gap;
  solver_details.scs_setup_time = scs_info.setup_time;
  solver_details.scs_solve_time = scs_info.solve_time;

  SolutionResult solution_result{SolutionResult::kSolverSpecificError};
  solver_details.y.resize(A_row_count);
  solver_details.s.resize(A_row_count);
  for (int i = 0; i < A_row_count; ++i) {
    solver_details.y(i) = scs_sol->y[i];
    solver_details.s(i) = scs_sol->s[i];
  }
  // Always set the primal and dual solution.
  result->set_x_val(
      (Eigen::Map<VectorX<scs_float>>(scs_sol->x, prog.num_vars()))
          .cast<double>());
  SetBoundingBoxDualSolution(prog, solver_details.y, bbcon_dual_indices,
                             result);
  internal::SetDualSolution(
      prog, solver_details.y, linear_constraint_dual_indices,
      linear_eq_y_start_indices, lorentz_cone_y_start_indices,
      rotated_lorentz_cone_y_start_indices, psd_y_start_indices,
      lmi_y_start_indices, scalar_psd_dual_indices, scalar_lmi_dual_indices,
      twobytwo_psd_dual_start_indices, twobytwo_lmi_dual_start_indices,
      /*upper_triangular_psd=*/false, result);
  // Set the solution_result enum and the optimal cost based on SCS status.
  if (solver_details.scs_status == SCS_SOLVED ||
      solver_details.scs_status == SCS_SOLVED_INACCURATE) {
    solution_result = SolutionResult::kSolutionFound;
    result->set_optimal_cost(scs_info.pobj + cost_constant);
  } else if (solver_details.scs_status == SCS_UNBOUNDED ||
             solver_details.scs_status == SCS_UNBOUNDED_INACCURATE) {
    solution_result = SolutionResult::kUnbounded;
    result->set_optimal_cost(MathematicalProgram::kUnboundedCost);
  } else if (solver_details.scs_status == SCS_INFEASIBLE ||
             solver_details.scs_status == SCS_INFEASIBLE_INACCURATE) {
    solution_result = SolutionResult::kInfeasibleConstraints;
    result->set_optimal_cost(MathematicalProgram::kGlobalInfeasibleCost);
  }
  if (solver_details.scs_status != SCS_SOLVED) {
    drake::log()->info("SCS returns code {}, with message \"{}\".\n",
                       solver_details.scs_status,
                       Scs_return_info(solver_details.scs_status));
  }

  result->set_solution_result(solution_result);
}

}  // namespace solvers
}  // namespace drake
