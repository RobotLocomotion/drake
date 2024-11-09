#include "drake/solvers/scs_clarabel_common.h"

#include "drake/common/ssize.h"

namespace drake {
namespace solvers {
namespace internal {
namespace {
// Return the index of S(i, j) in the flattened vector consisting of upper
// triangular part os matrix S (using column major).
int IndexInUpperTrianglePart(int i, int j) {
  return (1 + j) * j / 2 + i;
}

// SCS and Clarabel takes the upper (or lower) triangular part of a symmetric
// matrix, scale the off-diagonal term by sqrt(2), and then return the flattened
// scaled triangular part as the dual. This function scales the off-diagonal
// terms back. Also if upper_triangular_psd is true, we need to re-order `dual`
// so that it stores the flattened lower triangular part (as
// MathematicalProgram::GetDualSolution() returns the lower triangular part).
void ScalePsdConeDualVariable(int matrix_rows, bool upper_triangular_psd,
                              Eigen::VectorXd* dual) {
  DRAKE_ASSERT(dual->rows() == matrix_rows * (matrix_rows + 1) / 2);
  const double sqrt2 = std::sqrt(2);
  int count = 0;
  for (int j = 0; j < matrix_rows; ++j) {
    for (int i = (upper_triangular_psd ? 0 : j);
         i < (upper_triangular_psd ? j + 1 : matrix_rows); ++i) {
      if (i != j) {
        (*dual)(count) /= sqrt2;
      }
      count++;
    }
  }
  if (upper_triangular_psd) {
    // dual is the flattend upper triangular part. We need to re-order it to
    // store the flatted lower triangular part.
    Eigen::VectorXd dual_lower_triangular(dual->rows());
    int lower_triangular_count = 0;
    for (int j = 0; j < matrix_rows; ++j) {
      for (int i = j; i < matrix_rows; ++i) {
        // For a symmetric matrix S (the dual psd matrix), S(i, j) in the lower
        // triangular part (with i >= j) is the same as S(j, i) in the upper
        // triangular part, hence we compute the index of S(j, i) in the upper
        // triangular flat vector.
        dual_lower_triangular(lower_triangular_count++) =
            (*dual)(IndexInUpperTrianglePart(j, i));
      }
    }
    // Copy dual_lower_triangular to dual
    for (int i = 0; i < dual->rows(); ++i) {
      (*dual)(i) = dual_lower_triangular(i);
    }
  }
}
}  // namespace
void SetDualSolution(
    const MathematicalProgram& prog,
    const Eigen::Ref<const Eigen::VectorXd>& dual,
    const std::vector<std::vector<std::pair<int, int>>>&
        linear_constraint_dual_indices,
    const std::vector<int>& linear_eq_y_start_indices,
    const std::vector<int>& lorentz_cone_y_start_indices,
    const std::vector<int>& rotated_lorentz_cone_y_start_indices,
    const std::vector<int>& psd_y_start_indices,
    const std::vector<int>& lmi_y_start_indices, bool upper_triangular_psd,
    MathematicalProgramResult* result) {
  for (int i = 0; i < ssize(prog.linear_constraints()); ++i) {
    Eigen::VectorXd lin_con_dual = Eigen::VectorXd::Zero(
        prog.linear_constraints()[i].evaluator()->num_constraints());
    for (int j = 0; j < lin_con_dual.rows(); ++j) {
      if (linear_constraint_dual_indices[i][j].first != -1) {
        // lower bound is not infinity.
        // The shadow price for the lower bound is positive. SCS and Clarabel
        // dual for the positive cone is also positive, so we add the
        // SCS/Clarabel dual.
        lin_con_dual[j] += dual(linear_constraint_dual_indices[i][j].first);
      }
      if (linear_constraint_dual_indices[i][j].second != -1) {
        // upper bound is not infinity.
        // The shadow price for the upper bound is negative. SCS and Clarabel
        // dual for the positive cone is positive, so we subtract the
        // SCS/Clarabel dual.
        lin_con_dual[j] -= dual(linear_constraint_dual_indices[i][j].second);
      }
    }
    result->set_dual_solution(prog.linear_constraints()[i], lin_con_dual);
  }
  for (int i = 0;
       i < static_cast<int>(prog.linear_equality_constraints().size()); ++i) {
    // Notice that we have a negative sign in front of y.
    // This is because in SCS/Clarabel, for a problem with the linear equality
    // constraint
    // min cᵀx
    // s.t A*x=b
    // SCS/Clarabel formulates the dual problem as
    // max -bᵀy
    // s.t Aᵀy = -c
    // Note that there is a negation sign before b and c in SCS/Clarabel dual
    // problem, which is different from the standard formulation (no negation
    // sign). Hence the dual variable y for the linear equality constraint is
    // the negation of the shadow price.
    result->set_dual_solution(
        prog.linear_equality_constraints()[i],
        -dual.segment(linear_eq_y_start_indices[i],
                      prog.linear_equality_constraints()[i]
                          .evaluator()
                          ->num_constraints()));
  }
  for (int i = 0; i < static_cast<int>(prog.lorentz_cone_constraints().size());
       ++i) {
    result->set_dual_solution(
        prog.lorentz_cone_constraints()[i],
        dual.segment(
            lorentz_cone_y_start_indices[i],
            prog.lorentz_cone_constraints()[i].evaluator()->A().rows()));
  }
  for (int i = 0;
       i < static_cast<int>(prog.rotated_lorentz_cone_constraints().size());
       ++i) {
    // SCS/Clarabel doesn't provide rotated Lorentz cone constraint, it only
    // supports Lorentz cone constraint. Hence if Drake says "x in rotated
    // Lorentz cone constraint", we then take a linear transformation xbar = C*x
    // such that xbar is in the Lorentz cone constraint. By duality we know that
    // if y is the dual variable for the Lorentz cone constraint, then Cᵀy is
    // the dual variable for the rotated Lorentz cone constraint. The linear
    // transformation is C = [0.5  0.5 0 0 ... 0]
    //     [0.5 -0.5 0 0 ... 0]
    //     [  0    0 1 0 ... 0]
    //     [  0    0 0 1 ... 0]
    //            ...
    //     [  0    0 0 0 ... 1]
    // Namely if x is in the rotated Lorentz cone, then
    // [(x₀+x₁)/2, (x₀ − x₁)/2, x₂, ..., xₙ₋₁] is in the Lorentz cone.
    // Note that C = Cᵀ
    const auto& lorentz_cone_dual = dual.segment(
        rotated_lorentz_cone_y_start_indices[i],
        prog.rotated_lorentz_cone_constraints()[i].evaluator()->A().rows());
    Eigen::VectorXd rotated_lorentz_cone_dual = lorentz_cone_dual;
    rotated_lorentz_cone_dual(0) =
        (lorentz_cone_dual(0) + lorentz_cone_dual(1)) / 2;
    rotated_lorentz_cone_dual(1) =
        (lorentz_cone_dual(0) - lorentz_cone_dual(1)) / 2;
    result->set_dual_solution(prog.rotated_lorentz_cone_constraints()[i],
                              rotated_lorentz_cone_dual);
  }
  for (int i = 0; i < ssize(prog.positive_semidefinite_constraints()); ++i) {
    const int matrix_rows =
        prog.positive_semidefinite_constraints()[i].evaluator()->matrix_rows();
    if (psd_y_start_indices[i] != -1) {
      Eigen::VectorXd psd_dual = dual.segment(
          psd_y_start_indices[i], matrix_rows * (matrix_rows + 1) / 2);
      ScalePsdConeDualVariable(matrix_rows, upper_triangular_psd, &psd_dual);
      result->set_dual_solution(prog.positive_semidefinite_constraints()[i],
                                psd_dual);
    }
  }
  for (int i = 0; i < ssize(prog.linear_matrix_inequality_constraints()); ++i) {
    const int matrix_rows = prog.linear_matrix_inequality_constraints()[i]
                                .evaluator()
                                ->matrix_rows();
    if (lmi_y_start_indices[i] != -1) {
      Eigen::VectorXd lmi_dual = dual.segment(
          lmi_y_start_indices[i], matrix_rows * (matrix_rows + 1) / 2);
      ScalePsdConeDualVariable(matrix_rows, upper_triangular_psd, &lmi_dual);
      result->set_dual_solution(prog.linear_matrix_inequality_constraints()[i],
                                lmi_dual);
    }
  }
}

}  // namespace internal
}  // namespace solvers
}  // namespace drake
