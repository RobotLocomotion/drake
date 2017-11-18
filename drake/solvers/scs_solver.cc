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
std::vector<int> FindAllDecisionVariableIndices(const MathematicalProgram& prog, const Eigen::Ref<const VectorXDecisionVariable>& x) {
  std::vector<int> x_indices(x.rows());
  for (int i = 0; i < x.rows(); ++i) {
    x_indices[i] = prog.FindDecisionVariableIndex(x(i));
  }
  return x_indices;
}

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

/*void ParseQuadraticCost(const MathematicalProgram& prog, Eigen::VectorXd* c, double* constant, std::vector,Eigen::Triplet<double>>* A_triplets, std::vector<double>* b, int* A_row_count, SCS_CONE* cone, int* num_x) {

}*/

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
    const std::vector<int> x_indices = FindAllDecisionVariableIndices(prog, x);
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

void ParseSecondOrderConeConstraints(const MathematicalProgram& prog, std::vector<Eigen::Triplet<double>>* A_triplets,
                                     std::vector<double>* b, int* A_row_count,
                                     SCS_CONE* cone) {
  std::vector<int> lorentz_cone_length;
  // Our LorentzConeConstraint encodes that Ax + b is in Lorentz cone. We
  // convert this to SCS form, as -Ax + s = b, s in Lorentz cone.
  for (const auto& lorentz_cone_constraint : prog.lorentz_cone_constraints()) {
    // x_indices[i] is the index of x(i)
    const VectorXDecisionVariable& x = lorentz_cone_constraint.variables();
    const std::vector<int> x_indices = FindAllDecisionVariableIndices(prog, x);
    const Eigen::SparseMatrix<double> Ai = lorentz_cone_constraint.constraint()->A().sparseView();
    const std::vector<Eigen::Triplet<double>> Ai_triplets = math::SparseMatrixToTriplets(Ai);
    for (const auto& Ai_triplet : Ai_triplets) {
      A_triplets->emplace_back(Ai_triplet.row() + *A_row_count, x_indices[Ai_triplet.col()], -Ai_triplet.value());
    }
    const int num_Ai_rows = lorentz_cone_constraint.constraint()->A().rows();
    for (int i = 0; i < num_Ai_rows; ++i) {
      b->push_back(lorentz_cone_constraint.constraint()->b()(i));
    }
    lorentz_cone_length.push_back(num_Ai_rows);
    *A_row_count += num_Ai_rows;
  }

  // Our RotatedLorentzConeConstraint encodes that Ax + b is in the rotated
  // Lorentz cone, namely
  //  (a₀ᵀx + b₀) (a₁ᵀx + b₁) ≥ (a₂ᵀx + b₂)² + ... + (aₙ₋₁ᵀx + bₙ₋₁)²
  //  (a₀ᵀx + b₀) ≥ 0
  //  (a₁ᵀx + b₁) ≥ 0
  // , where aᵢᵀ is the i'th row of A, bᵢ is the i'th row of b. Equivalently the
  // vector
  // [ 0.5(a₀ + a₁)ᵀx + 0.5(b₀ + b₁) ]
  // [ 0.5(a₀ - a₁)ᵀx + 0.5(b₀ - b₁) ]
  // [           a₂ᵀx +           b₂ ]
  //             ...
  // [         aₙ₋₁ᵀx +         bₙ₋₁ ]
  // is in the Lorentz cone. We convert this to the SCS form, that
  //  Cx + s = d
  //  s in Lorentz cone,
  // where C = [ -0.5(a₀ + a₁)ᵀ ]   d = [ 0.5(b₀ + b₁) ]
  //           [ -0.5(a₀ - a₁)ᵀ ]       [ 0.5(b₀ - b₁) ]
  //           [           -a₂ᵀ ]       [           b₂ ]
  //                 ...                      ...
  //           [         -aₙ₋₁ᵀ ]       [          bₙ₋₁]
  for (const auto& rotated_lorentz_cone : prog.rotated_lorentz_cone_constraints()) {
    const VectorXDecisionVariable& x = rotated_lorentz_cone.variables();
    const std::vector<int> x_indices = FindAllDecisionVariableIndices(prog, x);
    const Eigen::SparseMatrix<double> Ai = rotated_lorentz_cone.constraint()->A().sparseView();
    const std::vector<Eigen::Triplet<double>> Ai_triplets = math::SparseMatrixToTriplets(Ai);
    for (const auto& Ai_triplet : Ai_triplets) {
      const int x_index = x_indices[Ai_triplet.col()];
      if (Ai_triplet.row() == 0) {
        A_triplets->emplace_back(*A_row_count, x_index, -0.5 * Ai_triplet.value());
        A_triplets->emplace_back(*A_row_count + 1, x_index, -0.5 * Ai_triplet.value());
      } else if (Ai_triplet.row() == 1) {
        A_triplets->emplace_back(*A_row_count, x_index, -0.5 * Ai_triplet.value());
        A_triplets->emplace_back(*A_row_count + 1, x_index, 0.5 * Ai_triplet.value());
      } else {
        A_triplets->emplace_back(*A_row_count + Ai_triplet.row(), x_index, -Ai_triplet.value());
      }
    }
    const Eigen::VectorXd& bi = rotated_lorentz_cone.constraint()->b();
    b->push_back(0.5 * (bi(0) + bi(1)));
    b->push_back(0.5 * (bi(0) - bi(1)));
    for (int i = 2; i < bi.rows(); ++i) {
      b->push_back(bi(i));
    }
    *A_row_count += bi.rows();
    lorentz_cone_length.push_back(bi.rows());
  }
  cone->qsize = lorentz_cone_length.size();
  cone->q = static_cast<scs_int*>(scs_calloc(cone->qsize, sizeof(scs_int)));
  for (int i = 0; i < cone->qsize; ++i) {
    cone->q[i] = lorentz_cone_length[i];
  }
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

  // num_x is the number of variables in `x`. It can contain more variables
  // than in prog. For some constraint/cost, we need to append slack variables
  // to convert the constraint/cost to the SCS form. For example, SCS does not
  // support quadratic cost. So we need to convert the quadratic cost
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

  // cone stores all the cones K in the problem.
  SCS_CONE* cone = static_cast<SCS_CONE*>(scs_calloc(1, sizeof(SCS_CONE)));

  // A_row_count will increment, when we add each constraint.
  int A_row_count = 0;
  std::vector<double> b;

  // `c` is the coefficient in the linear cost cᵀx
  Eigen::VectorXd c = Eigen::VectorXd::Zero(num_x);
  // Our cost (LinearCost, QuadraticCost, etc) also allows a constant term, we
  // add these constant terms to `cost_constant`.
  double cost_constant{0};

  // Parse linear cost
  ParseLinearCost(prog, &c, &cost_constant);

  // Parse linear equality constraint
  ParseLinearEqualityConstraint(prog, &A_triplets, &b, &A_row_count, cone);

  // Parse bounding box constraint
  ParseBoundingBoxConstraint(prog, &A_triplets, &b, &A_row_count, cone);

  // Parse linear constraint
  ParseLinearConstraint(prog, &A_triplets, &b, &A_row_count, cone);

  // Parse Lorentz cone and rotated Lorentz cone constraint
  ParseSecondOrderConeConstraints(prog, &A_triplets, &b, &A_row_count, cone);

  Eigen::SparseMatrix<double> A(A_row_count, num_x);
  A.setFromTriplets(A_triplets.begin(), A_triplets.end());
  A.makeCompressed();

  SCS_PROBLEM_DATA* scs_problem_data =
      static_cast<SCS_PROBLEM_DATA*>(scs_calloc(1, sizeof(SCS_PROBLEM_DATA)));
  SetScsProblemData(A_row_count, num_x, A, b, c, scs_problem_data);
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

  prog.SetSolverId(id());
  return sol_result;
}
}  // namespace solvers
}  // namespace drake
