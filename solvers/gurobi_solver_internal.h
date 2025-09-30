#pragma once

// For external users, please do not include this header file. It only exists so
// that we can expose the internals to gurobi_solver_internal_test.

#include <limits>
#include <vector>

// NOLINTNEXTLINE(build/include) False positive due to weird include style.
#include "gurobi_c.h"

#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
namespace internal {
// Adds a constraint of one of the following forms :
// lb ≤ A*x ≤ ub
// or
// A*x == lb
//
// @param is_equality True if the imposed constraint is
// A*x == lb, false otherwise.
// @param[in, out] num_gurobi_linear_constraints The number of linear
// constraints stored in the gurobi model.
// @return error as an integer. The full set of error values are
// described here :
// https://docs.gurobi.com/projects/optimizer/en/12.0/reference/numericcodes/errors.html
// This function assumes `vars` doesn't contain duplicate variables.
int AddLinearConstraintNoDuplication(
    const MathematicalProgram& prog, GRBmodel* model,
    const Eigen::SparseMatrix<double>& A, const Eigen::VectorXd& lb,
    const Eigen::VectorXd& ub, const VectorXDecisionVariable& vars,
    bool is_equality, int* num_gurobi_linear_constraints);

// Add the linear constraint lb <= A * vars <= ub to Gurobi model.
int AddLinearConstraint(const MathematicalProgram& prog, GRBmodel* model,
                        const Eigen::SparseMatrix<double>& A,
                        const Eigen::VectorXd& lb, const Eigen::VectorXd& ub,
                        const VectorXDecisionVariable& vars, bool is_equality,
                        int* num_gurobi_linear_constraints);

/*
 * Add (rotated) Lorentz cone constraints, that z = A*x+b is in the (rotated)
 * Lorentz cone.
 * A vector z is in the Lorentz cone, if
 * z(0) >= sqrt(z(1)^2 + ... + z(N-1)^2)
 * A vector z is in the rotated Lorentz cone, if
 * z(0)*z(1) >= z(2)^2 + ... + z(N-1)^2
 * z(0) >= 0, z(1) >= 0
 * @tparam C  A constraint type, either LorentzConeConstraint or
 * RotatedLorentzConeConstraint.
 * @param second_order_cone_constraints  A vector of Binding objects, containing
 * either Lorentz cone constraints, or rotated Lorentz cone constraints.
 * @param second_order_cone_new_variable_indices. The indices of variable z in
 * the Gurobi model.
 * @param model The Gurobi model.
 * @param[in, out] num_gurobi_linear_constraints The number of linear
 * constraints stored in the gurobi model.
 */
template <typename C>
int AddSecondOrderConeConstraints(
    const MathematicalProgram& prog,
    const std::vector<Binding<C>>& second_order_cone_constraints,
    const std::vector<std::vector<int>>& second_order_cone_new_variable_indices,
    GRBmodel* model, int* num_gurobi_linear_constraints);

// For Lorentz and rotated Lorentz cone constraints
// Ax + b in (rotated) Lorentz cone, we will introduce new variables z as
// z = Ax+b
// z in (rotated) Lorentz cone.
// So add the new variable z before constructing the Gurobi model, as
// recommended by the Gurobi manual, to add all decision variables at once
// when constructing the problem.
// @param second_order_cone_variable_indices
// second_order_cone_variable_indices[i]
// contains the indices of the newly added variable z for the i'th second order
// cone in @p second_order_cones[i].
// @p tparam C Either LorentzConeConstraint or RotatedLorentzConeConstraint.
// TODO(hongkai.dai): rewrite this function not templated on Binding, when
// Binding class is moved out from MathematicalProgram as a public class.
// @param second_order_cones A vector of bindings, containing either Lorentz
// cone constraint, or rotated Lorentz cone constraint.
// @param is_new_variable is_new_variable[i] is true if the i'th variable in
// Gurobi model is not included in MathematicalProgram.
// @param num_gurobi_vars Number of variables in Gurobi model.
// @param second_order_cone_variable_indices
// second_order_cone_variable_indices[i]
// contains the indices of variable z stored in Gurobi model, in \p
// second_order_cones[i].
// @param gurobi_var_type. The type of the Gurobi variables.
// @param xlow The lower bound of the Gurobi variables.
// @param xupp The upper bound of the Gurobi variables.
template <typename C>
void AddSecondOrderConeVariables(
    const std::vector<Binding<C>>& second_order_cones,
    std::vector<bool>* is_new_variable, int* num_gurobi_vars,
    std::vector<std::vector<int>>* second_order_cone_variable_indices,
    std::vector<char>* gurobi_var_type, std::vector<double>* xlow,
    std::vector<double>* xupp);

// For an L2 norm cost min |Cx+d|₂, we consider introducing a Lorentz cone
// constraint as
// z in Lorentz cone.
// z[1:] = C*x + d
// min z[0]
// So we will need to add the slack variable z.
// @param[in/out] is_new_variable is_new_variable[i] is true if the i'th
// variable in Gurobi model is not included in MathematicalProgram.
// @param[in/out] num_gurobi_vars Number of variables in Gurobi model.
// @param[out] lorentz_cone_variable_indices lorentz_cone_variable_indices[i] is
// the indices of the variable z for l2_norm_costs[i] in the Gurobi model.
// @param[in/out] gurobi_var_type gurobi_var_type[i] is the type of the i'th
// variable in the Gurobi model.
// @param[in/out] xlow The lower bound of the Gurobi variables.
// @param[in/out] xupp The upper bound of the Gurobi variables.
void AddL2NormCostVariables(
    const std::vector<Binding<L2NormCost>>& l2_norm_costs,
    std::vector<bool>* is_new_variable, int* num_gurobi_vars,
    std::vector<std::vector<int>>* lorentz_cone_variable_indices,
    std::vector<char>* gurobi_var_type, std::vector<double>* xlow,
    std::vector<double>* xupp);

// Adds L2 norm cost |Cx+d|₂ to Gurobi.
// We introduce a Lorentz cone constraint as
// z in Lorentz cone.
// z[1:] = C*x + d
// min z[0]
// @param[in] prog All prog.l2norm_costs() will be added to Gurobi model.
// @param[in] l2norm_costs_lorentz_cone_variable_indices
// l2norm_costs_lorentz_cone_variable_indices[i] are the indices of the slack
// variable z for prog.l2norm_costs()[i] in Gurobi model.
// @param[in/out] model The Gurobi model.
// @param[in/out] num_gurobi_linear_constraints The number of linear constraints
// in Gurobi before/after calling this function.
int AddL2NormCosts(const MathematicalProgram& prog,
                   const std::vector<std::vector<int>>&
                       l2norm_costs_lorentz_cone_variable_indices,
                   GRBmodel* model, int* num_gurobi_linear_constraints);
}  // namespace internal
}  // namespace solvers
}  // namespace drake
