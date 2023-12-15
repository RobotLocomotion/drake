#include "drake/solvers/scs_clarabel_common.h"

#include "drake/common/ssize.h"

namespace drake {
namespace solvers {
namespace internal {
void SetDualSolution(
    const MathematicalProgram& prog,
    const Eigen::Ref<const Eigen::VectorXd>& dual,
    const std::vector<std::vector<std::pair<int, int>>>&
        linear_constraint_dual_indices,
    const std::vector<int>& linear_eq_y_start_indices,
    const std::vector<int>& lorentz_cone_y_start_indices,
    const std::vector<int>& rotated_lorentz_cone_y_start_indices,
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
}

}  // namespace internal
}  // namespace solvers
}  // namespace drake
