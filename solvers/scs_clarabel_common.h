// The solvers Clarabel and Scs share a lot of similar APIs. This file contains
// some common functions for ClarabelSolver and ScsSolver
#pragma once

#include <optional>
#include <utility>
#include <vector>

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"

namespace drake {
namespace solvers {
namespace internal {
// @param dual The pointer to the start of the dual solution.
// @param dual_length The length of the dual solution (as a continuous chunk of
// memory).
// @param linear_constraint_dual_indices
// linear_constraint_dual_indices[i][j][0]/linear_constraint_dual_indices[i][j][1]
// is the index of the dual variable for the lower/upper bound of the j'th row
// in the linear constraint prog.linear_constraint()[i]. We use an index of -1
// to indicate that the lower or upper bound is infinity and therefore there is
// no associated dual variable.
// @param linear_eq_y_start_indices linear_eq_y_start_indices[i] is the starting
// index of the dual variable for the constraint
// prog.linear_equality_constraints()[i]. Namely y[linear_eq_y_start_indices[i]:
// linear_eq_y_start_indices[i] +
// prog.linear_equality_constraints()[i].evaluator()->num_constraints()] are
// the dual variables for the linear equality constraint
// prog.linear_equality_constraint()(i), where y is the vector containing all
// dual variables.
// @param lorentz_cone_y_start_indices lorentz_cone_y_start_indices[i] is the
// starting index of the dual variable for the constraint
// prog.lorentz_cone_constraints()[i]. y[lorentz_cone_y_start_indices[i]:
//   lorentz_cone_y_start_indices[i] + second_order_cone_length[i]]
// are the dual variables for prog.lorentz_cone_constraints()[i].
// @param rotated_lorentz_cone_y_start_indices
// rotated_lorentz_cone_y_start_indices[i] is the starting index of the dual
// variable for the constraint prog.rotated_lorentz_cone_constraints()[i].
// y[rotated_lorentz_cone_y_start_indices[i]:
//   rotated_lorentz_cone_y_start_indices[i] + second_order_cone_length[i]]
// are the dual variables for prog.rotated_lorentz_cone_constraints()[i].
// @param psd_y_start_indices y[psd_y_start_indices[i]: psd_y_start_indices[i] +
// (matrix_rows+1)*matrix_rows/2] are the dual variables for
// prog.positive_semidefinite_constraints()[i]
// @param lmi_y_start_indices y[lmi_y_start_indices[i]: lmi_y_start_indices[i] +
// (matrix_rows+1)*matrix_rows/2] are the dual variables for
// prog.linear_matrix_inequality_constraints()[i]
// @param upper_triangular_psd Use upper-triangular (for Clarabel) or
// lower-triangular (for SCS) part of the symmetric matrix in the dual variable.
void SetDualSolution(
    const MathematicalProgram& prog,
    const Eigen::Ref<const Eigen::VectorXd>& dual,
    const std::vector<std::vector<std::pair<int, int>>>&
        linear_constraint_dual_indices,
    const std::vector<int>& linear_eq_y_start_indices,
    const std::vector<int>& lorentz_cone_y_start_indices,
    const std::vector<int>& rotated_lorentz_cone_y_start_indices,
    const std::vector<std::optional<int>>& psd_y_start_indices,
    const std::vector<std::optional<int>>& lmi_y_start_indices,
    const std::vector<std::optional<int>>& scalar_psd_y_indices,
    const std::vector<std::optional<int>>& scalar_lmi_y_indices,
    const std::vector<std::optional<int>>& twobytwo_psd_y_start_indices,
    const std::vector<std::optional<int>>& twobytwo_lmi_y_start_indices,
    bool upper_triangular_psd, MathematicalProgramResult* result);
}  // namespace internal
}  // namespace solvers
}  // namespace drake
