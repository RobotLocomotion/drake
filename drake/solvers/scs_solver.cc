#include "drake/solvers/scs_solver.h"

#include <Eigen/Sparse>

// clang-format off
// scs.h should be included before linsys/amatrix.h, since amatrix.h uses types
// scs_float, scs_int, etc, defined in scs.h
#include "scs.h"
#include "linsys/amatrix.h"
// clang-format on

#include "drake/common/text_logging.h"
#include "drake/math/eigen_sparse_triplet.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
namespace {
void ParseLinearCost(const MathematicalProgram& prog, Eigen::VectorXd* c,
                     double* constant) {
  for (const auto& linear_cost : prog.linear_costs()) {
    // Each linear cost is in the form of aᵀx + b
    const auto& a = linear_cost.constraint()->a();
    const VectorXDecisionVariable& x = linear_cost.variables();
    for (int i = 0; i < a.rows(); ++i) {
      (*c)(prog.FindDecisionVariableIndex(x(i))) += a(i);
    }
    (*constant) += linear_cost.constraint()->b();
  }
}

void ParseLinearConstraint(const MathematicalProgram& prog,
                           std::vector<Eigen::Triplet<double>>* A_triplets,
                           std::vector<double>* b, int* A_row_count,
                           SCS_CONE* cone) {
  // The linear constraint lb ≤ aᵀx ≤ ub is converted to
  //  aᵀx + s1 = ub,
  // -aᵀx + s2 = lb
  // s1, s2 in the positive cone.
  // The special cases are when ub = ∞ or lb = -∞.
  // When ub = ∞, then we only add the constraint
  // -aᵀx + s = lb, s in the positive cone.
  // When lb = -∞, then we only add the constraint
  // aᵀx + s = ub, s in the positive cone.
  int num_linear_constraint_rows = 0;
  for (const auto& linear_constraint : prog.linear_constraints()) {
    const Eigen::VectorXd& ub = linear_constraint.constraint()->upper_bound();
    const Eigen::VectorXd& lb = linear_constraint.constraint()->lower_bound();
    const VectorXDecisionVariable& x = linear_constraint.variables();
    const Eigen::MatrixXd& Ai = linear_constraint.constraint()->A();
    for (int i = 0; i < static_cast<int>(
        linear_constraint.constraint()->num_constraints());
         ++i) {
      const bool is_ub_finite{!std::isinf(ub(i))};
      const bool is_lb_finite{!std::isinf(lb(i))};
      if (is_ub_finite || is_lb_finite) {
        // If lb != -∞, then the constraint -aᵀx + s = lb will be added to the
        // matrix A, in the row lower_bound_row_index.
        const int lower_bound_row_index =
            *A_row_count + num_linear_constraint_rows;
        // If ub != ∞, then the constraint aᵀx + s = ub will be added to the
        // matrix A, in the row upper_bound_row_index.
        const int upper_bound_row_index =
            *A_row_count + num_linear_constraint_rows + (is_lb_finite ? 1 : 0);
        for (int j = 0; j < x.rows(); ++j) {
          if (Ai(i, j) != 0) {
            const int xj_index = prog.FindDecisionVariableIndex(x(j));
            if (is_ub_finite) {
              A_triplets->emplace_back(upper_bound_row_index, xj_index,
                                       Ai(i, j));
            }
            if (is_lb_finite) {
              A_triplets->emplace_back(lower_bound_row_index, xj_index,
                                       -Ai(i, j));
            }
          }
        }
        if (is_lb_finite) {
          b->push_back(-lb(i));
          ++num_linear_constraint_rows;
        }
        if (is_ub_finite) {
          b->push_back(ub(i));
          ++num_linear_constraint_rows;
        }
      }
    }
  }
  *A_row_count += num_linear_constraint_rows;
  cone->l += num_linear_constraint_rows;
}

void ParseLinearEqualityConstraint(
    const MathematicalProgram& prog,
    std::vector<Eigen::Triplet<double>>* A_triplets, std::vector<double>* b,
    int* A_row_count, SCS_CONE* cone) {
  int num_linear_equality_constraints_rows = 0;
  // The linear equality constraint A x = b is converted to
  // A x + s = b. s in zero cone.
  for (const auto& linear_equality_constraint :
      prog.linear_equality_constraints()) {
    const Eigen::SparseMatrix<double> Ai =
        linear_equality_constraint.constraint()->GetSparseMatrix();
    const std::vector<Eigen::Triplet<double>> Ai_triplets =
        math::SparseMatrixToTriplets(Ai);
    A_triplets->reserve(A_triplets->size() + Ai_triplets.size());
    const solvers::VectorXDecisionVariable& x =
        linear_equality_constraint.variables();
    // x_indices[i] is the index of x(i)
    std::vector<int> x_indices(x.rows());
    for (int i = 0; i < x.rows(); ++i) {
      x_indices[i] = prog.FindDecisionVariableIndex(x(i));
    }
    for (const auto& Ai_triplet : Ai_triplets) {
      A_triplets->emplace_back(Ai_triplet.row() + *A_row_count,
                               x_indices[Ai_triplet.col()], Ai_triplet.value());
    }
    const int num_Ai_rows =
        linear_equality_constraint.constraint()->num_constraints();
    b->reserve(b->size() + num_Ai_rows);
    for (int i = 0; i < num_Ai_rows; ++i) {
      b->push_back(linear_equality_constraint.constraint()->lower_bound()(i));
    }
    *A_row_count += num_Ai_rows;
    num_linear_equality_constraints_rows += num_Ai_rows;
  }
  cone->f += num_linear_equality_constraints_rows;
}

void ParseBoundingBoxConstraint(const MathematicalProgram& prog,
                                std::vector<Eigen::Triplet<double>>* A_triplets,
                                std::vector<double>* b, int* A_row_count,
                                SCS_CONE* cone) {
  // A bounding box constraint lb ≤ x ≤ ub is converted to the SCS form as
  // x + s1 = ub, -x + s2 = -lb, s1, s2 in the positive cone.
  // TODO(hongkai.dai) : handle the special case l = u, such that we can convert
  // it to x + s = l, s in zero cone.
  int num_bounding_box_constraint_rows = 0;
  for (const auto& bounding_box_constraint : prog.bounding_box_constraints()) {
    int num_scs_new_constraint = 0;
    const VectorXDecisionVariable& xi = bounding_box_constraint.variables();
    const int num_xi_rows = xi.rows();
    A_triplets->reserve(A_triplets->size() + 2 * num_xi_rows);
    b->reserve(b->size() + 2 * num_xi_rows);
    for (int i = 0; i < num_xi_rows; ++i) {
      if (!std::isinf(bounding_box_constraint.constraint()->upper_bound()(i))) {
        // if ub != ∞, then add the constraint x + s1 = ub, s1 in the positive
        // cone.
        A_triplets->emplace_back(num_scs_new_constraint + *A_row_count,
                                 prog.FindDecisionVariableIndex(xi(i)), 1);
        b->push_back(bounding_box_constraint.constraint()->upper_bound()(i));
        ++num_scs_new_constraint;
      }
      if (!std::isinf(bounding_box_constraint.constraint()->lower_bound()(i))) {
        // if lb != -∞, then add the constraint -x + s2 = -lb, s2 in the positive
        // cone.
        A_triplets->emplace_back(num_scs_new_constraint + *A_row_count,
                                 prog.FindDecisionVariableIndex(xi(i)), -1);
        b->push_back(-bounding_box_constraint.constraint()->lower_bound()(i));
        ++num_scs_new_constraint;
      }
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

void ExtractSolution(MathematicalProgram* prog,
                     const SCS_SOL_VARS& scs_sol_vars) {
  // For zero, positive or second-order cones, the primal variable x is the
  // same variable as in `prog`. For semidefinite cones, the variable x is a
  // scaled version of the lower-triangular part of the variable in `prog`, with
  // a scaling factor of √2.
  prog->SetDecisionVariableValues(
      Eigen::Map<Eigen::VectorXd>(scs_sol_vars.x, prog->num_vars()));
}

void SetScsProblemSettingsToDefault(SCS_SETTINGS* settings) {
  // These macro's are defined in SCS. For their actual value, please refer to
  // https://github.com/cvxgrp/scs/blob/master/include/constants.h
  settings->alpha = ALPHA;
  settings->cg_rate = CG_RATE;
  settings->eps = EPS;
  settings->max_iters = MAX_ITERS;
  settings->normalize = NORMALIZE;
  settings->rho_x = RHO_X;
  settings->scale = SCALE;
  settings->verbose = VERBOSE;
  settings->warm_start = WARM_START;
}

void SetScsProblemData(int A_row_count, int num_vars,
                       const Eigen::SparseMatrix<double>& A,
                       const std::vector<double>& b, const Eigen::VectorXd& c,
                       SCS_PROBLEM_DATA* scs_problem_data) {
  scs_problem_data->m = A_row_count;
  scs_problem_data->n = num_vars;

  scs_problem_data->A = static_cast<AMatrix*>(malloc(sizeof(AMatrix)));
  scs_problem_data->A->x =
    static_cast<scs_float*>(scs_calloc(A.nonZeros(), sizeof(scs_float)));

  scs_problem_data->A->i =
    static_cast<scs_int*>(scs_calloc(A.nonZeros(), sizeof(scs_int)));

  scs_problem_data->A->p =
    static_cast<scs_int*>(scs_calloc(scs_problem_data->n + 1, sizeof(scs_int)));

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

  scs_problem_data->b =
      static_cast<scs_float*>(scs_calloc(b.size(), sizeof(scs_float)));

  for (int i = 0; i < static_cast<int>(b.size()); ++i) {
    scs_problem_data->b[i] = b[i];
  }
  scs_problem_data->c =
      static_cast<scs_float*>(scs_calloc(num_vars, sizeof(scs_float)));
  for (int i = 0; i < num_vars; ++i) {
    scs_problem_data->c[i] = c(i);
  }

  // Set the parameters to default values.
  scs_problem_data->stgs =
      static_cast<SCS_SETTINGS*>(scs_malloc(sizeof(SCS_SETTINGS)));
  SetScsProblemSettingsToDefault(scs_problem_data->stgs);
}
} // namespace

bool ScsSolver::available() const { return true; }

SolutionResult ScsSolver::Solve(MathematicalProgram& prog) const {
  // SCS solves the problem in this form
  // min  cᵀx
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
  const int num_vars = prog.num_vars();

  // We need to construct a sparse matrix in Column Compressed Storage (CCS)
  // format. On the other hand, we add the constraint row by row, instead of
  // column by column, as preferred by CCS. As a result, we use Eigen sparse
  // matrix to construct a sparse matrix in column order first, and then
  // compress it to get CCS.
  std::vector<Eigen::Triplet<double>> A_triplets;

  // cone stores all the cones K in the problem.
  SCS_CONE* cone = static_cast<SCS_CONE*>(scs_calloc(1, sizeof(SCS_CONE)));

  // A_row_count will increment, when we add each constraint.
  int A_row_count = 0;
  std::vector<double> b;

  Eigen::VectorXd c = Eigen::VectorXd::Zero(num_vars);
  double cost_constant{0};

  // Parse linear cost
  ParseLinearCost(prog, &c, &cost_constant);

  // Parse linear equality constraint
  ParseLinearEqualityConstraint(prog, &A_triplets, &b, &A_row_count, cone);

  // Parse bounding box constraint
  ParseBoundingBoxConstraint(prog, &A_triplets, &b, &A_row_count, cone);

  // Parse linear constraint
  ParseLinearConstraint(prog, &A_triplets, &b, &A_row_count, cone);

  Eigen::SparseMatrix<double> A(A_row_count, num_vars);
  A.setFromTriplets(A_triplets.begin(), A_triplets.end());
  A.makeCompressed();

  SCS_PROBLEM_DATA* scs_problem_data =
      static_cast<SCS_PROBLEM_DATA*>(scs_calloc(1, sizeof(SCS_PROBLEM_DATA)));
  SetScsProblemData(A_row_count, num_vars, A, b, c, scs_problem_data);
  scs_problem_data->stgs->verbose = verbose_ ? VERBOSE : 0;

  SCS_INFO scs_info{0};
  SCS_WORK* scs_work = scs_init(scs_problem_data, cone, &scs_info);

  SCS_SOL_VARS* scs_sol =
      static_cast<SCS_SOL_VARS*>(scs_calloc(1, sizeof(SCS_SOL_VARS)));

  scs_int scs_status =
      scs_solve(scs_work, scs_problem_data, cone, scs_sol, &scs_info);

  SolutionResult sol_result{SolutionResult::kUnknownError};
  if (scs_status == SCS_SOLVED || scs_status == SCS_SOLVED_INACCURATE) {
    sol_result = SolutionResult::kSolutionFound;
    ExtractSolution(&prog, *scs_sol);
    prog.SetOptimalCost(scs_info.pobj + cost_constant);
  } else if (scs_status == SCS_UNBOUNDED ||
             scs_status == SCS_UNBOUNDED_INACCURATE) {
    sol_result = SolutionResult::kUnbounded;
  } else if (scs_status == SCS_INFEASIBLE ||
             scs_status == SCS_INFEASIBLE_INACCURATE) {
    sol_result = SolutionResult::kInfeasibleConstraints;
  }
  if (scs_status != SCS_SOLVED) {
    drake::log()->info("SCS returns code {}, with message \"{}\".\n",
                       scs_status, Scs_return_info(scs_status));
  }

  // Free allocated memory
  scs_finish(scs_work);
  freeData(scs_problem_data, cone);
  freeSol(scs_sol);
  return sol_result;
}
}  // namespace solvers
}  // namespace drake
