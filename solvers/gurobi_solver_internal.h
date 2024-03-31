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
// https://www.gurobi.com/documentation/10.0/refman/error_codes.html
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
    GRBmodel* model, int* num_gurobi_linear_constraints) {
  static_assert(
      std::is_same_v<C, LorentzConeConstraint> ||
          std::is_same_v<C, RotatedLorentzConeConstraint>,
      "Expects either LorentzConeConstraint or RotatedLorentzConeConstraint");
  bool is_rotated_cone = std::is_same_v<C, RotatedLorentzConeConstraint>;

  DRAKE_ASSERT(second_order_cone_constraints.size() ==
               second_order_cone_new_variable_indices.size());
  int second_order_cone_count = 0;
  int num_gurobi_vars;
  int error = GRBgetintattr(model, "NumVars", &num_gurobi_vars);
  DRAKE_ASSERT(!error);
  for (const auto& binding : second_order_cone_constraints) {
    const auto& A = binding.evaluator()->A();
    const auto& b = binding.evaluator()->b();

    int num_x = A.cols();
    int num_z = A.rows();

    // Add the constraint z - A*x = b
    // xz_indices records the indices of [x; z] in Gurobi.
    std::vector<int> xz_indices(num_x + num_z, 0);

    for (int i = 0; i < num_x; ++i) {
      xz_indices[i] = prog.FindDecisionVariableIndex(binding.variables()(i));
    }
    for (int i = 0; i < num_z; ++i) {
      xz_indices[num_x + i] =
          second_order_cone_new_variable_indices[second_order_cone_count][i];
    }
    // z - A*x will be written as M * [x; z], where M = [-A I].
    // Gurobi expects M in compressed sparse row format, so we will first find
    // out the non-zero entries in each row of M.
    std::vector<Eigen::Triplet<double>> M_triplets;
    M_triplets.reserve(A.nonZeros() + num_z);
    for (int i = 0; i < A.outerSize(); ++i) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(A, i); it; ++it) {
        M_triplets.emplace_back(it.row(), xz_indices[it.col()], -it.value());
      }
    }
    for (int i = 0; i < num_z; ++i) {
      M_triplets.emplace_back(i, xz_indices[num_x + i], 1);
    }

    Eigen::SparseMatrix<double, Eigen::RowMajor> M(num_z, num_gurobi_vars);
    // Eigen::SparseMatrix::setFromTriplets will automatically group the sum of
    // the values in M_triplets that correspond to the same entry in the sparse
    // matrix.
    M.setFromTriplets(M_triplets.begin(), M_triplets.end());

    std::vector<char> sense(num_z, GRB_EQUAL);

    error = GRBaddconstrs(model, num_z, M.nonZeros(), M.outerIndexPtr(),
                          M.innerIndexPtr(), M.valuePtr(), sense.data(),
                          const_cast<double*>(b.data()), nullptr);
    DRAKE_ASSERT(!error);
    *num_gurobi_linear_constraints += num_z;

    // Gurobi uses a matrix Q to differentiate Lorentz cone and rotated Lorentz
    // cone constraint.
    // https://www.gurobi.com/documentation/10.0/refman/c_addqconstr.html
    // For Lorentz cone constraint,
    // Q = [-1 0 0 ... 0]
    //     [ 0 1 0 ... 0]
    //     [ 0 0 1 ... 0]
    //          ...
    //     [ 0 0 0 ... 1]
    // namely Q = diag([-1; 1; 1; ...; 1], so
    // z' * Q * z = z(1)^2 + ... + z(n-1)^2 - z(0)^2.
    // For rotated Lorentz cone constraint
    // Q = [0 -1 0 0 ... 0]
    //     [0  0 0 0 ... 0]
    //     [0  0 1 0 ... 0]
    //     [0  0 0 1 ... 0]
    //           ...
    //     [0  0 0 0 ... 1]
    // so z' * Q * z = z(2)^2 + ... + z(n-1)^2 - z(0) * z(1).
    // We will store Q in a sparse format.
    // qrow stores the row    indices of the non-zero entries of Q.
    // qcol stores the column indices of the non-zero entries of Q.
    // qval stores the value          of the non-zero entries of Q.
    size_t num_Q_nonzero = is_rotated_cone ? num_z - 1 : num_z;
    std::vector<int> qrow(num_Q_nonzero);
    std::vector<int> qcol(num_Q_nonzero);
    std::vector<double> qval(num_Q_nonzero);
    for (int i = 0; i < num_z - 2; ++i) {
      int zi_index =
          second_order_cone_new_variable_indices[second_order_cone_count]
                                                [i + 2];
      qrow[i] = zi_index;
      qcol[i] = zi_index;
      qval[i] = 1.0;
    }
    int z0_index =
        second_order_cone_new_variable_indices[second_order_cone_count][0];
    int z1_index =
        second_order_cone_new_variable_indices[second_order_cone_count][1];
    if (is_rotated_cone) {
      qrow[num_z - 2] = z0_index;
      qcol[num_z - 2] = z1_index;
      qval[num_z - 2] = -1;
    } else {
      qrow[num_z - 2] = z0_index;
      qcol[num_z - 2] = z0_index;
      qval[num_z - 2] = -1;
      qrow[num_z - 1] = z1_index;
      qcol[num_z - 1] = z1_index;
      qval[num_z - 1] = 1;
    }
    error =
        GRBaddqconstr(model, 0, nullptr, nullptr, num_Q_nonzero, qrow.data(),
                      qcol.data(), qval.data(), GRB_LESS_EQUAL, 0.0, NULL);
    if (error) {
      return error;
    }
    ++second_order_cone_count;
  }
  return 0;
}

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
    std::vector<double>* xupp) {
  static_assert(
      std::is_same_v<C, LorentzConeConstraint> ||
          std::is_same_v<C, RotatedLorentzConeConstraint>,
      "Expects LorentzConeConstraint and RotatedLorentzConeConstraint.");
  bool is_rotated_cone = std::is_same_v<C, RotatedLorentzConeConstraint>;

  int num_new_second_order_cone_var = 0;
  second_order_cone_variable_indices->resize(second_order_cones.size());

  // The newly added variable z for the Lorentz cone constraint is appended
  // to the existing variables. So increment the variable indices
  // accordingly.
  int lorentz_cone_count = 0;
  for (const auto& binding : second_order_cones) {
    int num_new_lorentz_cone_var_i = binding.evaluator()->A().rows();
    (*second_order_cone_variable_indices)[lorentz_cone_count].resize(
        num_new_lorentz_cone_var_i);
    for (int i = 0; i < num_new_lorentz_cone_var_i; ++i) {
      (*second_order_cone_variable_indices)[lorentz_cone_count][i] =
          *num_gurobi_vars + num_new_second_order_cone_var + i;
    }
    num_new_second_order_cone_var += num_new_lorentz_cone_var_i;
    ++lorentz_cone_count;
  }
  *num_gurobi_vars += num_new_second_order_cone_var;
  is_new_variable->resize(*num_gurobi_vars, true);

  // Newly added variable z is continuous variable.
  gurobi_var_type->resize(*num_gurobi_vars, GRB_CONTINUOUS);

  // For Lorentz cone constraint, z(0) >= 0.
  // For rotated Lorentz cone constraint, z(0) >= 0, z(1) >= 0.
  xlow->resize(*num_gurobi_vars, -std::numeric_limits<double>::infinity());
  xupp->resize(*num_gurobi_vars, std::numeric_limits<double>::infinity());
  for (int i = 0; i < static_cast<int>(second_order_cones.size()); ++i) {
    xlow->at((*second_order_cone_variable_indices)[i][0]) = 0;
    if (is_rotated_cone) {
      xlow->at((*second_order_cone_variable_indices)[i][1]) = 0;
    }
  }
}

}  // namespace internal
}  // namespace solvers
}  // namespace drake
