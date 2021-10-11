#include "drake/solvers/scs_solver.h"

#include <Eigen/Sparse>
#include <fmt/format.h>

// clang-format off
#include <scs.h>
#include <cones.h>
#include <linalg.h>
#include <util.h>
// clang-format on

#include "drake/common/scope_exit.h"
#include "drake/common/text_logging.h"
#include "drake/math/eigen_sparse_triplet.h"
#include "drake/math/quadratic_form.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
namespace {
void ParseLinearCost(const MathematicalProgram& prog, std::vector<double>* c,
                     double* constant) {
  for (const auto& linear_cost : prog.linear_costs()) {
    // Each linear cost is in the form of aᵀx + b
    const auto& a = linear_cost.evaluator()->a();
    const VectorXDecisionVariable& x = linear_cost.variables();
    for (int i = 0; i < a.rows(); ++i) {
      (*c)[prog.FindDecisionVariableIndex(x(i))] += a(i);
    }
    (*constant) += linear_cost.evaluator()->b();
  }
}

// Add a rotated Lorentz cone constraint that A_cone * x + b_cone is in the
// rotated Lorentz cone.
// @param A_cone_triplets The triplets of non-zero entries in A_cone.
// @param b_cone A_cone * x + b_cone is in the rotated Lorentz cone.
// @param x_indices The index of the variable in SCS.
// @param A_triplets[in/out] The non-zero entry triplet in A before and after
// adding the Lorentz cone.
// @param b[in/out] The right-hand side vector b before and after adding the
// Lorentz cone.
// @param A_row_count[in/out] The number of rows in A before and after adding
// the Lorentz cone.
// @param lorentz_cone_length[in/out] The length of each Lorentz cone before
// and after adding the Lorentz cone constraint.
void ParseRotatedLorentzConeConstraint(
    const std::vector<Eigen::Triplet<double>>& A_cone_triplets,
    const Eigen::Ref<const Eigen::VectorXd>& b_cone,
    const std::vector<int>& x_indices,
    std::vector<Eigen::Triplet<double>>* A_triplets, std::vector<double>* b,
    int* A_row_count, std::vector<int>* lorentz_cone_length) {
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
  for (const auto& Ai_triplet : A_cone_triplets) {
    const int x_index = x_indices[Ai_triplet.col()];
    if (Ai_triplet.row() == 0) {
      A_triplets->emplace_back(*A_row_count, x_index,
                               -0.5 * Ai_triplet.value());
      A_triplets->emplace_back(*A_row_count + 1, x_index,
                               -0.5 * Ai_triplet.value());
    } else if (Ai_triplet.row() == 1) {
      A_triplets->emplace_back(*A_row_count, x_index,
                               -0.5 * Ai_triplet.value());
      A_triplets->emplace_back(*A_row_count + 1, x_index,
                               0.5 * Ai_triplet.value());
    } else {
      A_triplets->emplace_back(*A_row_count + Ai_triplet.row(), x_index,
                               -Ai_triplet.value());
    }
  }
  b->push_back(0.5 * (b_cone(0) + b_cone(1)));
  b->push_back(0.5 * (b_cone(0) - b_cone(1)));
  for (int i = 2; i < b_cone.rows(); ++i) {
    b->push_back(b_cone(i));
  }
  *A_row_count += b_cone.rows();
  lorentz_cone_length->push_back(b_cone.rows());
}

void ParseQuadraticCost(const MathematicalProgram& prog, std::vector<double>* c,
                        std::vector<Eigen::Triplet<double>>* A_triplets,
                        std::vector<double>* b, int* A_row_count,
                        std::vector<int>* lorentz_cone_length, int* num_x) {
  // A QuadraticCost encodes cost of the form
  //   0.5 zᵀQz + pᵀz + r
  // Since SCS only supports linear cost, we introduce a new slack variable y
  // as the upper bound of the cost, with the rotated Lorentz cone constraint
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
    ParseRotatedLorentzConeConstraint(Ai_triplets, b_cone, Ai_var_indices,
                                      A_triplets, b, A_row_count,
                                      lorentz_cone_length);
    // Add the cost y.
    c->push_back(1);
  }
}

void ParseLinearConstraint(const MathematicalProgram& prog,
                           std::vector<Eigen::Triplet<double>>* A_triplets,
                           std::vector<double>* b, int* A_row_count,
                           ScsCone* cone) {
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
    const Eigen::VectorXd& ub = linear_constraint.evaluator()->upper_bound();
    const Eigen::VectorXd& lb = linear_constraint.evaluator()->lower_bound();
    const VectorXDecisionVariable& x = linear_constraint.variables();
    const Eigen::MatrixXd& Ai = linear_constraint.evaluator()->A();
    for (int i = 0; i < linear_constraint.evaluator()->num_constraints();
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
    int* A_row_count, ScsCone* cone) {
  int num_linear_equality_constraints_rows = 0;
  // The linear equality constraint A x = b is converted to
  // A x + s = b. s in zero cone.
  for (const auto& linear_equality_constraint :
       prog.linear_equality_constraints()) {
    const Eigen::SparseMatrix<double> Ai =
        linear_equality_constraint.evaluator()->GetSparseMatrix();
    const std::vector<Eigen::Triplet<double>> Ai_triplets =
        math::SparseMatrixToTriplets(Ai);
    A_triplets->reserve(A_triplets->size() + Ai_triplets.size());
    const solvers::VectorXDecisionVariable& x =
        linear_equality_constraint.variables();
    // x_indices[i] is the index of x(i)
    const std::vector<int> x_indices = prog.FindDecisionVariableIndices(x);
    for (const auto& Ai_triplet : Ai_triplets) {
      A_triplets->emplace_back(Ai_triplet.row() + *A_row_count,
                               x_indices[Ai_triplet.col()], Ai_triplet.value());
    }
    const int num_Ai_rows =
        linear_equality_constraint.evaluator()->num_constraints();
    b->reserve(b->size() + num_Ai_rows);
    for (int i = 0; i < num_Ai_rows; ++i) {
      b->push_back(linear_equality_constraint.evaluator()->lower_bound()(i));
    }
    *A_row_count += num_Ai_rows;
    num_linear_equality_constraints_rows += num_Ai_rows;
  }
  cone->z += num_linear_equality_constraints_rows;
}

void ParseBoundingBoxConstraint(const MathematicalProgram& prog,
                                std::vector<Eigen::Triplet<double>>* A_triplets,
                                std::vector<double>* b, int* A_row_count,
                                ScsCone* cone) {
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
      if (!std::isinf(bounding_box_constraint.evaluator()->upper_bound()(i))) {
        // if ub != ∞, then add the constraint x + s1 = ub, s1 in the positive
        // cone.
        A_triplets->emplace_back(num_scs_new_constraint + *A_row_count,
                                 prog.FindDecisionVariableIndex(xi(i)), 1);
        b->push_back(bounding_box_constraint.evaluator()->upper_bound()(i));
        ++num_scs_new_constraint;
      }
      if (!std::isinf(bounding_box_constraint.evaluator()->lower_bound()(i))) {
        // if lb != -∞, then add the constraint -x + s2 = -lb, s2 in the
        // positive cone.
        A_triplets->emplace_back(num_scs_new_constraint + *A_row_count,
                                 prog.FindDecisionVariableIndex(xi(i)), -1);
        b->push_back(-bounding_box_constraint.evaluator()->lower_bound()(i));
        ++num_scs_new_constraint;
      }
    }
    *A_row_count += num_scs_new_constraint;
    num_bounding_box_constraint_rows += num_scs_new_constraint;
  }
  cone->l += num_bounding_box_constraint_rows;
}

void ParseSecondOrderConeConstraints(
    const MathematicalProgram& prog,
    std::vector<Eigen::Triplet<double>>* A_triplets, std::vector<double>* b,
    int* A_row_count, std::vector<int>* lorentz_cone_length) {
  // Our LorentzConeConstraint encodes that Ax + b is in Lorentz cone. We
  // convert this to SCS form, as -Ax + s = b, s in Lorentz cone.
  for (const auto& lorentz_cone_constraint : prog.lorentz_cone_constraints()) {
    // x_indices[i] is the index of x(i)
    const VectorXDecisionVariable& x = lorentz_cone_constraint.variables();
    const std::vector<int> x_indices = prog.FindDecisionVariableIndices(x);
    const Eigen::SparseMatrix<double> Ai =
        lorentz_cone_constraint.evaluator()->A();
    const std::vector<Eigen::Triplet<double>> Ai_triplets =
        math::SparseMatrixToTriplets(Ai);
    for (const auto& Ai_triplet : Ai_triplets) {
      A_triplets->emplace_back(Ai_triplet.row() + *A_row_count,
                               x_indices[Ai_triplet.col()],
                               -Ai_triplet.value());
    }
    const int num_Ai_rows = lorentz_cone_constraint.evaluator()->A().rows();
    for (int i = 0; i < num_Ai_rows; ++i) {
      b->push_back(lorentz_cone_constraint.evaluator()->b()(i));
    }
    lorentz_cone_length->push_back(num_Ai_rows);
    *A_row_count += num_Ai_rows;
  }

  for (const auto& rotated_lorentz_cone :
       prog.rotated_lorentz_cone_constraints()) {
    const VectorXDecisionVariable& x = rotated_lorentz_cone.variables();
    const std::vector<int> x_indices = prog.FindDecisionVariableIndices(x);
    const Eigen::SparseMatrix<double> Ai =
        rotated_lorentz_cone.evaluator()->A();
    const Eigen::VectorXd& bi = rotated_lorentz_cone.evaluator()->b();
    const std::vector<Eigen::Triplet<double>> Ai_triplets =
        math::SparseMatrixToTriplets(Ai);
    ParseRotatedLorentzConeConstraint(Ai_triplets, bi, x_indices, A_triplets, b,
                                      A_row_count, lorentz_cone_length);
  }
}

// This function parses both PositiveSemidefinite and
// LinearMatrixInequalityConstraint.
void ParsePositiveSemidefiniteConstraint(
    const MathematicalProgram& prog,
    std::vector<Eigen::Triplet<double>>* A_triplets, std::vector<double>* b,
    int* A_row_count, ScsCone* cone) {
  std::vector<int> psd_cone_length;
  const double sqrt2 = std::sqrt(2);
  for (const auto& psd_constraint : prog.positive_semidefinite_constraints()) {
    // PositiveSemidefiniteConstraint encodes the matrix X being psd.
    // We convert it to SCS form
    // A * x + s = 0
    // s in positive semidefinite cone.
    // where A is a diagonal matrix, with its diagonal entries being the stacked
    // column vector of the lower triangular part of matrix
    // ⎡ -1 -√2 -√2 ... -√2⎤
    // ⎢-√2  -1 -√2 ... -√2⎥
    // ⎢-√2 -√2  -1 ... -√2⎥
    // ⎢    ...            ⎥
    // ⎣-√2 -√2 -√2 ...  -1⎦
    // The √2 scaling factor in the off-diagonal entries are required by SCS,
    // as it uses only the lower triangular part of the symmetric matrix, as
    // explained in https://github.com/cvxgrp/scs
    // x is the stacked column vector of the lower triangular part of the
    // symmetric matrix X.
    const int X_rows = psd_constraint.evaluator()->matrix_rows();
    int x_index_count = 0;
    const VectorXDecisionVariable& flat_X = psd_constraint.variables();
    DRAKE_DEMAND(flat_X.rows() == X_rows * X_rows);
    b->reserve(b->size() + X_rows * (X_rows + 1) / 2);
    for (int j = 0; j < X_rows; ++j) {
      A_triplets->emplace_back(
          *A_row_count + x_index_count,
          prog.FindDecisionVariableIndex(flat_X(j * X_rows + j)), -1);
      b->push_back(0);
      ++x_index_count;
      for (int i = j + 1; i < X_rows; ++i) {
        A_triplets->emplace_back(
            *A_row_count + x_index_count,
            prog.FindDecisionVariableIndex(flat_X(j * X_rows + i)), -sqrt2);
        b->push_back(0);
        ++x_index_count;
      }
    }
    (*A_row_count) += X_rows * (X_rows + 1) / 2;
    psd_cone_length.push_back(X_rows);
  }
  for (const auto& lmi_constraint :
       prog.linear_matrix_inequality_constraints()) {
    // LinearMatrixInequalityConstraint encodes
    // F₀ + x₁*F₁ + x₂*F₂ + ... + xₙFₙ is p.s.d
    // We convert this to SCS form as
    // A_cone * x + s = b_cone
    // s in SCS positive semidefinite cone.
    // where
    //              ⎡  F₁(0, 0)   F₂(0, 0) ...   Fₙ(0, 0)⎤
    //              ⎢√2F₁(1, 0) √2F₂(1, 0) ... √2Fₙ(1, 0)⎥
    //   A_cone = - ⎢√2F₁(2, 0) √2F₂(2, 0) ... √2Fₙ(2, 0)⎥,
    //              ⎢            ...                     ⎥
    //              ⎣  F₁(m, m)   F₂(m, m) ...   Fₙ(m, m)⎦
    //
    //              ⎡  F₀(0, 0)⎤
    //              ⎢√2F₀(1, 0)⎥
    //   b_cone =   ⎢√2F₀(2, 0)⎥,
    //              ⎢   ...    ⎥
    //              ⎣  F₀(m, m)⎦
    // and
    //   x = [x₁; x₂; ... ; xₙ].
    // As explained above, the off-diagonal rows are scaled by √2. Please refer
    // to https://github.com/cvxgrp/scs about the scaling factor √2.
    const std::vector<Eigen::MatrixXd>& F = lmi_constraint.evaluator()->F();
    const VectorXDecisionVariable& x = lmi_constraint.variables();
    const int F_rows = lmi_constraint.evaluator()->matrix_rows();
    const std::vector<int> x_indices = prog.FindDecisionVariableIndices(x);
    int A_cone_row_count = 0;
    b->reserve(b->size() + F_rows * (F_rows + 1) / 2);
    for (int j = 0; j < F_rows; ++j) {
      for (int i = j; i < F_rows; ++i) {
        const double scale_factor = i == j ? 1 : sqrt2;
        for (int k = 1; k < static_cast<int>(F.size()); ++k) {
          A_triplets->emplace_back(*A_row_count + A_cone_row_count,
                                   x_indices[k - 1],
                                   -scale_factor * F[k](i, j));
        }
        b->push_back(scale_factor * F[0](i, j));
        ++A_cone_row_count;
      }
    }
    *A_row_count += F_rows * (F_rows + 1) / 2;
    psd_cone_length.push_back(F_rows);
  }
  cone->ssize = psd_cone_length.size();
  // This scs_calloc doesn't need to accompany a ScopeExit since cone->s will be
  // cleaned up recursively by freeing up cone in scs_free_data()
  cone->s = static_cast<scs_int*>(scs_calloc(cone->ssize, sizeof(scs_int)));
  for (int i = 0; i < cone->ssize; ++i) {
    cone->s[i] = psd_cone_length[i];
  }
}

void ParseExponentialConeConstraint(
    const MathematicalProgram& prog,
    std::vector<Eigen::Triplet<double>>* A_triplets, std::vector<double>* b,
    int* A_row_count, ScsCone* cone) {
  cone->ep = static_cast<int>(prog.exponential_cone_constraints().size());
  for (const auto& binding : prog.exponential_cone_constraints()) {
    // drake::solvers::ExponentialConstraint enforces that z = A * x + b is in
    // the exponential cone (z₀ ≥ z₁*exp(z₂ / z₁)). This is different from the
    // exponential cone used in SCS. In SCS, a vector s is in the exponential
    // cone, if s₂≥ s₁*exp(s₀ / s₁). To transform drake's Exponential cone to
    // SCS's exponential cone, we use
    // -[A.row(2); A.row(1); A.row(0)] * x + s = [b(2); b(1); b(0)], and s is
    // in SCS's exponential cone, where A = binding.evaluator()->A(), b =
    // binding.evaluator()->b().
    const int num_bound_variables = binding.variables().rows();
    for (int i = 0; i < num_bound_variables; ++i) {
      A_triplets->reserve(A_triplets->size() +
                          binding.evaluator()->A().nonZeros());
      const int decision_variable_index =
          prog.FindDecisionVariableIndex(binding.variables()(i));
      for (Eigen::SparseMatrix<double>::InnerIterator it(
               binding.evaluator()->A(), i);
           it; ++it) {
        // 2 - it.row() is used for reverse the row order, as mentioned in the
        // function documentation above.
        A_triplets->emplace_back(*A_row_count + 2 - it.row(),
                                 decision_variable_index, -it.value());
      }
    }
    // The exponential cone constraint is on a 3 x 1 vector, hence for each
    // ExponentialConeConstraint, we append 3 rows to A and b.
    b->reserve(b->size() + 3);
    for (int i = 0; i < 3; ++i) {
      b->push_back(binding.evaluator()->b()(2 - i));
    }
    *A_row_count += 3;
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

void SetScsProblemData(int A_row_count, int num_vars,
                       const Eigen::SparseMatrix<double>& A,
                       const std::vector<double>& b,
                       const std::vector<double>& c,
                       ScsData* scs_problem_data) {
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
  // This scs_calloc doesn't need to accompany a ScopeExit since
  // scs_problem_data->c will be cleaned up recursively by freeing up
  // scs_problem_data in scs_free_data()
  scs_problem_data->c =
      static_cast<scs_float*>(scs_calloc(num_vars, sizeof(scs_float)));
  for (int i = 0; i < num_vars; ++i) {
    scs_problem_data->c[i] = c[i];
  }
}
}  // namespace

bool ScsSolver::is_available() { return true; }

namespace {
// This should be invoked only once on each unique instance of ScsSettings.
// Namely, only call this function for once in DoSolve.
void SetScsSettings(
    std::unordered_map<std::string, int>* solver_options_int,
    const bool print_to_console,
    ScsSettings* scs_settings) {
  auto it = solver_options_int->find("normalize");
  if (it != solver_options_int->end()) {
    scs_settings->normalize = it->second;
    solver_options_int->erase(it);
  }
  it = solver_options_int->find("adaptive_scale;");
  if (it != solver_options_int->end()) {
    scs_settings->adaptive_scale = it->second;
    solver_options_int->erase(it);
  }
  it = solver_options_int->find("max_iters");
  if (it != solver_options_int->end()) {
    scs_settings->max_iters = it->second;
    solver_options_int->erase(it);
  }
  it = solver_options_int->find("verbose");
  if (it != solver_options_int->end()) {
    // The solver specific option has the highest priority.
    scs_settings->verbose = it->second;
    solver_options_int->erase(it);
  } else {
    // The common option has the second highest priority.
    scs_settings->verbose = print_to_console ? 1 : 0;
  }
  it = solver_options_int->find("warm_start");
  if (it != solver_options_int->end()) {
    scs_settings->warm_start = it->second;
    solver_options_int->erase(it);
  }
  it = solver_options_int->find("acceleration_lookback");
  if (it != solver_options_int->end()) {
    scs_settings->acceleration_lookback = it->second;
    solver_options_int->erase(it);
  }
  it = solver_options_int->find("acceleration_interval");
  if (it != solver_options_int->end()) {
    scs_settings->acceleration_interval = it->second;
    solver_options_int->erase(it);
  }
  if (!solver_options_int->empty()) {
    throw std::invalid_argument("Unsupported SCS solver options.");
  }
}

// This should be invoked only once on each unique instance of ScsSettings.
// Namely, only call this function for once in DoSolve.
void SetScsSettings(
    std::unordered_map<std::string, double>* solver_options_double,
    ScsSettings* scs_settings) {
  auto it = solver_options_double->find("scale");
  if (it != solver_options_double->end()) {
    scs_settings->scale = it->second;
    solver_options_double->erase(it);
  }
  it = solver_options_double->find("rho_x");
  if (it != solver_options_double->end()) {
    scs_settings->rho_x = it->second;
    solver_options_double->erase(it);
  }
  it = solver_options_double->find("eps_abs");
  if (it != solver_options_double->end()) {
    scs_settings->eps_abs = it->second;
    solver_options_double->erase(it);
  } else {
    // SCS 3.0 uses 1E-4 as the default value, see
    // https://www.cvxgrp.org/scs/api/settings.html?highlight=eps_abs). This
    // tolerance is too loose. We set the default tolerance to 1E-5 for better
    // accuracy.
    scs_settings->eps_abs = 1E-5;
  }
  it = solver_options_double->find("eps_rel");
  if (it != solver_options_double->end()) {
    scs_settings->eps_rel = it->second;
    solver_options_double->erase(it);
  } else {
    // SCS 3.0 uses 1E-4 as the default value, see
    // https://www.cvxgrp.org/scs/api/settings.html?highlight=eps_rel). This
    // tolerance is too loose. We set the default tolerance to 1E-5 for better
    // accuracy.
    scs_settings->eps_rel = 1E-5;
  }
  it = solver_options_double->find("eps_infeas");
  if (it != solver_options_double->end()) {
    scs_settings->eps_infeas = it->second;
    solver_options_double->erase(it);
  }
  it = solver_options_double->find("alpha");
  if (it != solver_options_double->end()) {
    scs_settings->alpha = it->second;
    solver_options_double->erase(it);
  }
  it = solver_options_double->find("time_limit_secs");
  if (it != solver_options_double->end()) {
    scs_settings->time_limit_secs = it->second;
    solver_options_double->erase(it);
  }
  if (!solver_options_double->empty()) {
    throw std::invalid_argument("Unsupported SCS solver options.");
  }
}

}  // namespace

void ScsSolver::DoSolve(
    const MathematicalProgram& prog,
    const Eigen::VectorXd& initial_guess,
    const SolverOptions& merged_options,
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
  ScsCone* cone = static_cast<ScsCone*>(scs_calloc(1, sizeof(ScsCone)));
  ScsData* scs_problem_data =
      static_cast<ScsData*>(scs_calloc(1, sizeof(ScsData)));
  ScsSettings* scs_stgs =
      static_cast<ScsSettings*>(scs_calloc(1, sizeof(ScsSettings)));
  // This guard will free cone, scs_problem_data, and scs_stgs (together with
  // their instantiated members) upon return from the DoSolve function.
  ScopeExit scs_free_guard([&cone, &scs_problem_data, &scs_stgs]() {
    scs_free_data(scs_problem_data, cone, scs_stgs);
  });

  // Set the parameters to default values.
  scs_set_default_settings(scs_stgs);

  // A_row_count will increment, when we add each constraint.
  int A_row_count = 0;
  std::vector<double> b;

  // `c` is the coefficient in the linear cost cᵀx
  std::vector<double> c(num_x, 0.0);

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
  std::vector<int> lorentz_cone_length;
  ParseSecondOrderConeConstraints(prog, &A_triplets, &b, &A_row_count,
                                  &lorentz_cone_length);

  // Parse quadratic cost. This MUST be called after parsing the second order
  // cone constraint, as we convert quadratic cost to second order cone
  // constraint.
  ParseQuadraticCost(prog, &c, &A_triplets, &b, &A_row_count,
                     &lorentz_cone_length, &num_x);

  // Set the lorentz cone length in the SCS cone.
  cone->qsize = lorentz_cone_length.size();
  cone->q = static_cast<scs_int*>(scs_calloc(cone->qsize, sizeof(scs_int)));
  for (int i = 0; i < cone->qsize; ++i) {
    cone->q[i] = lorentz_cone_length[i];
  }

  // Parse PositiveSemidefiniteConstraint and LinearMatrixInequalityConstraint.
  ParsePositiveSemidefiniteConstraint(prog, &A_triplets, &b, &A_row_count,
                                      cone);

  // Parse ExponentialConeConstraint.
  ParseExponentialConeConstraint(prog, &A_triplets, &b, &A_row_count, cone);

  Eigen::SparseMatrix<double> A(A_row_count, num_x);
  A.setFromTriplets(A_triplets.begin(), A_triplets.end());
  A.makeCompressed();

  SetScsProblemData(A_row_count, num_x, A, b, c, scs_problem_data);
  std::unordered_map<std::string, int> input_solver_options_int =
      merged_options.GetOptionsInt(id());
  std::unordered_map<std::string, double> input_solver_options_double =
      merged_options.GetOptionsDouble(id());
  SetScsSettings(&input_solver_options_int,
                 merged_options.get_print_to_console(), scs_stgs);
  SetScsSettings(&input_solver_options_double, scs_stgs);

  ScsInfo scs_info{0};

  ScsSolution* scs_sol =
      static_cast<ScsSolution*>(scs_calloc(1, sizeof(ScsSolution)));
  ScopeExit sol_guard([&scs_sol]() { scs_free_sol(scs_sol); });

  ScsSolverDetails& solver_details =
      result->SetSolverDetailsType<ScsSolverDetails>();
  solver_details.scs_status = scs(
      scs_problem_data, cone, scs_stgs, scs_sol, &scs_info);

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

  SolutionResult solution_result{SolutionResult::kUnknownError};
  solver_details.y.resize(A_row_count);
  solver_details.s.resize(A_row_count);
  for (int i = 0; i < A_row_count; ++i) {
    solver_details.y(i) = scs_sol->y[i];
    solver_details.s(i) = scs_sol->s[i];
  }
  if (solver_details.scs_status == SCS_SOLVED ||
      solver_details.scs_status == SCS_SOLVED_INACCURATE) {
    solution_result = SolutionResult::kSolutionFound;
    result->set_x_val(
        (Eigen::Map<VectorX<scs_float>>(scs_sol->x, prog.num_vars()))
            .cast<double>());
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
