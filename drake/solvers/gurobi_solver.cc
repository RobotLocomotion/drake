#include "drake/solvers/gurobi_solver.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SparseCore>

// NOLINTNEXTLINE(build/include) False positive due to weird include style.
#include "gurobi_c++.h"

#include "drake/common/drake_assert.h"
#include "drake/math/eigen_sparse_triplet.h"

namespace drake {
namespace solvers {
namespace {
/// Check is the number of variables in the Gurobi model is as expected. This
/// operation can be EXPENSIVE, since it requires calling GRBupdatemodel
/// (Gurobi typically adopts lazy update, that it does not update the model
/// until calling optimize function).
bool IsNumberOfVariablesAsExpected(GRBmodel* model, int num_vars_expected) {
  int error = GRBupdatemodel(model);
  if (error) return false;
  int num_vars;
  error = GRBgetintattr(model, "NumVars", &num_vars);
  if (error) return false;
  return (num_vars == num_vars_expected);
}

/**
 * Adds a constraint of one of the following forms :
 * Ax>=b, Ax<=b, or Ax==b,
 * where the character variable @p constraint_sense specifies the type.
 * x in this case is the full dimensional variable being optimised.
 *
 * @param[in] constraint_sense a character variable specifying the
 * sense on the constraint. The Gurobi macros maybe used to specify the
 * constraint sense.
 *  i.e.
 *  GRB_LESS_EQUAL    : '<'
 *  GRB_GREATER_EQUAL : '>'
 *  GRB_EQUAL         : '='
 *
 * @return error as an integer. The full set of error values are
 * described here :
 * http://www.gurobi.com/documentation/6.5/refman/error_codes.html#sec:ErrorCodes
 *
 * TODO(hongkai.dai): Use a sparse matrix A.
 */
template <typename DerivedA, typename DerivedB>
int AddLinearConstraint(GRBmodel* model, const Eigen::MatrixBase<DerivedA>& A,
                        const Eigen::MatrixBase<DerivedB>& b,
                        const std::vector<int>& variable_indices,
                        char constraint_sense, double sparseness_threshold) {
  for (int i = 0; i < A.rows(); i++) {
    int non_zeros_index = 0;
    std::vector<int> constraint_index(A.cols(), 0);
    std::vector<double> constraint_value(A.cols(), 0.0);

    for (int j = 0; j < A.cols(); j++) {
      if (std::abs(A(i, j)) > sparseness_threshold) {
        constraint_value[non_zeros_index] = A(i, j);
        constraint_index[non_zeros_index++] = variable_indices[j];
      }
    }
    int error =
        GRBaddconstr(model, non_zeros_index, &constraint_index[0],
                     &constraint_value[0], constraint_sense, b(i), nullptr);
    if (error) return error;
  }
  // If loop completes, no errors exist so the value '0' must be returned.
  return 0;
}

/*
 * Add (rotated) Lorentz cone constraints, that A*x+b is in the (rotated)
 * Lorentz cone.
 * A vector z is in the Lorentz cone, if
 * z(0) >= sqrt(z(1)^2 + ... + z(N-1)^2)
 * A vector z is in the rotated Lorentz cone, if
 * z(0)*z(1) >= z(2)^2 + ... + z(N-1)^2
 * z(0) >= 0, z(1) >= 0
 */
template <typename Binding>
int AddSecondOrderConeConstraints(
    const std::vector<Binding>& second_order_cone_constraints,
    bool is_rotated_cone, double sparseness_threshold, GRBmodel* model,
    std::vector<bool>* is_new_variable) {
  for (const auto& binding : second_order_cone_constraints) {
    int num_constraint_variable = static_cast<int>(binding.GetNumElements());
    std::vector<int> variable_indices;
    variable_indices.reserve(static_cast<size_t>(num_constraint_variable));
    const auto& variable_list = binding.variable_list();
    for (const DecisionVariableMatrixX& var : variable_list.variables()) {
      DRAKE_ASSERT(var.cols() == 1);
      for (int i = 0; i < static_cast<int>(var.rows()); ++i) {
        variable_indices.push_back(static_cast<int>(var(i, 0).index()));
      }
    }
    int num_x = static_cast<int>(variable_indices.size());

    const auto& A = binding.constraint()->A();
    const auto& b = binding.constraint()->b();
    int num_total_variables = is_new_variable->size();
    // First add new decision variables z, with the constraints z - A*x = b
    int num_z = A.rows();
    std::vector<char> new_variable_types(num_z, GRB_CONTINUOUS);
    std::vector<double> new_variable_lb(
        num_z, -std::numeric_limits<double>::infinity());
    new_variable_lb[0] = 0.0;
    if (is_rotated_cone) {
      new_variable_lb[1] = 0.0;
    }
    int error = GRBaddvars(model, A.rows(), 0, nullptr, nullptr, nullptr,
                           nullptr, new_variable_lb.data(), nullptr,
                           new_variable_types.data(), nullptr);
    if (error) {
      return error;
    }
    // Append the newly added variable indices to variable_indices.
    variable_indices.reserve(variable_indices.size() + num_z);
    is_new_variable->reserve(is_new_variable->size() + num_z);
    for (int i = 0; i < num_z; ++i) {
      variable_indices.push_back(num_total_variables + i);
      is_new_variable->push_back(true);
    }
    DRAKE_ASSERT(IsNumberOfVariablesAsExpected(model, is_new_variable->size()));
    // TODO(hongkai.dai): Use a sparse A_lorentz matrix.
    Eigen::MatrixXd A_lorentz(num_z, num_x + num_z);
    A_lorentz << -A, Eigen::MatrixXd::Identity(A.rows(), A.rows());
    error = AddLinearConstraint(model, A_lorentz, b, variable_indices,
                                GRB_EQUAL, sparseness_threshold);

    // Gurobi uses a matrix Q to differentiate Lorentz cone and rotated Lorentz
    // cone constraint.
    // https://www.gurobi.com/documentation/7.0/refman/c_grbaddqconstr.html
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
    size_t num_Q_nonzero =
        is_rotated_cone ? num_z - 1 : num_z;
    std::vector<int> qrow(num_Q_nonzero);
    std::vector<int> qcol(num_Q_nonzero);
    std::vector<double> qval(num_Q_nonzero);
    for (int i = 0; i < num_z - 2; ++i) {
      qrow[i] = variable_indices[num_x + i + 2];
      qcol[i] = variable_indices[num_x + i + 2];
      qval[i] = 1.0;
    }
    if (is_rotated_cone) {
      qrow[num_z - 2] = variable_indices[num_x];
      qcol[num_z - 2] = variable_indices[num_x + 1];
      qval[num_z - 2] = -1;
    } else {
      qrow[num_z - 2] = variable_indices[num_x];
      qcol[num_z - 2] = variable_indices[num_x];
      qval[num_z - 2] = -1;
      qrow[num_z - 1] = variable_indices[num_x + 1];
      qcol[num_z - 1] = variable_indices[num_x + 1];
      qval[num_z - 1] = 1;
    }
    error =
        GRBaddqconstr(model, 0, nullptr, nullptr, num_Q_nonzero, qrow.data(),
                      qcol.data(), qval.data(), GRB_LESS_EQUAL, 0.0, NULL);
    if (error) {
      return error;
    }
  }
  return 0;
}

/*
 * Add quadratic or linear costs to the optimization problem.
 */
int AddCosts(GRBmodel* model, const MathematicalProgram& prog,
             double sparseness_threshold) {
  // Aggregates the quadratic costs and linear costs in the form
  // 0.5 * x' * Q_all * x + linear_term' * x.
  using std::abs;
  // record the non-zero entries in the cost 0.5*x'*Q*x + b'*x.
  std::vector<Eigen::Triplet<double>> Q_nonzero_coefs;
  std::vector<Eigen::Triplet<double>> b_nonzero_coefs;
  for (const auto& binding : prog.quadratic_costs()) {
    const auto& constraint = binding.constraint();
    const int constraint_variable_dimension = binding.GetNumElements();
    const Eigen::MatrixXd& Q = constraint->Q();
    const Eigen::VectorXd& b = constraint->b();

    DRAKE_ASSERT(Q.rows() == constraint_variable_dimension);

    // constraint_variable_index[i] is the index of the i'th decision variable
    // binding.GetFlattendSolution(i).
    std::vector<int> constraint_variable_index(constraint_variable_dimension);
    int constraint_variable_count = 0;
    for (const DecisionVariableMatrixX& var :
         binding.variable_list().variables()) {
      DRAKE_ASSERT(var.cols() == 1);
      for (int i = 0; i < static_cast<int>(var.rows()); ++i) {
        constraint_variable_index[constraint_variable_count] =
            prog.FindDecisionVariableIndex(var(i, 0));
        constraint_variable_count++;
      }
    }
    for (int i = 0; i < Q.rows(); i++) {
      const double Qii = 0.5 * Q(i, i);
      if (abs(Qii) > sparseness_threshold) {
        Q_nonzero_coefs.push_back(Eigen::Triplet<double>(
            constraint_variable_index[i], constraint_variable_index[i], Qii));
      }
      for (int j = i + 1; j < Q.cols(); j++) {
        const double Qij = 0.5 * (Q(i, j) + Q(j, i));
        if (abs(Qij) > sparseness_threshold) {
          Q_nonzero_coefs.push_back(Eigen::Triplet<double>(
              constraint_variable_index[i], constraint_variable_index[j], Qij));
        }
      }
    }

    for (int i = 0; i < b.size(); i++) {
      if (abs(b(i)) > sparseness_threshold) {
        b_nonzero_coefs.push_back(
            Eigen::Triplet<double>(constraint_variable_index[i], 0, b(i)));
      }
    }
  }

  // Add linear cost in prog.linear_costs() to the aggregated cost.
  for (const auto& binding : prog.linear_costs()) {
    const auto& constraint = binding.constraint();
    Eigen::RowVectorXd c = constraint->A();
    int constraint_variable_count = 0;
    for (const DecisionVariableMatrixX& var :
         binding.variable_list().variables()) {
      DRAKE_ASSERT(var.cols() == 1);
      for (int i = 0; i < static_cast<int>(var.rows()); ++i) {
        b_nonzero_coefs.push_back(
            Eigen::Triplet<double>(prog.FindDecisionVariableIndex(var(i, 0)), 0,
                                   c(constraint_variable_count)));
        constraint_variable_count++;
      }
    }
  }

  Eigen::SparseMatrix<double> Q_all(prog.num_vars(), prog.num_vars());
  Eigen::SparseMatrix<double> linear_terms(prog.num_vars(), 1);
  Q_all.setFromTriplets(Q_nonzero_coefs.begin(), Q_nonzero_coefs.end());
  linear_terms.setFromTriplets(b_nonzero_coefs.begin(), b_nonzero_coefs.end());

  std::vector<Eigen::Index> Q_all_row;
  std::vector<Eigen::Index> Q_all_col;
  std::vector<double> Q_all_val;
  drake::math::SparseMatrixToRowColumnValueVectors(Q_all, Q_all_row, Q_all_col,
                                                   Q_all_val);

  std::vector<int> Q_all_row_indices_int(Q_all_row.size());
  std::vector<int> Q_all_col_indices_int(Q_all_col.size());
  for (int i = 0; i < static_cast<int>(Q_all_row_indices_int.size()); i++) {
    Q_all_row_indices_int[i] = static_cast<int>(Q_all_row[i]);
    Q_all_col_indices_int[i] = static_cast<int>(Q_all_col[i]);
  }

  std::vector<Eigen::Index> linear_row;
  std::vector<Eigen::Index> linear_col;
  std::vector<double> linear_val;
  drake::math::SparseMatrixToRowColumnValueVectors(linear_terms, linear_row,
                                                   linear_col, linear_val);

  std::vector<int> linear_row_indices_int(linear_row.size());
  for (int i = 0; i < static_cast<int>(linear_row_indices_int.size()); i++) {
    linear_row_indices_int[i] = static_cast<int>(linear_row[i]);
  }

  const int QPtermsError = GRBaddqpterms(
      model, static_cast<int>(Q_all_row.size()), Q_all_row_indices_int.data(),
      Q_all_col_indices_int.data(), Q_all_val.data());
  if (QPtermsError) {
    return QPtermsError;
  }
  for (int i = 0; i < static_cast<int>(linear_row.size()); i++) {
    const int LinearTermError = GRBsetdblattrarray(
        model, "Obj", linear_row_indices_int[i], 1, linear_val.data() + i);
    if (LinearTermError) {
      return LinearTermError;
    }
  }
  // If loop completes, no errors exist so the value '0' must be returned.
  return 0;
}

// Add both LinearConstraints and LinearEqualityConstraints to gurobi
// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
int ProcessLinearConstraints(GRBmodel* model, MathematicalProgram& prog,
                             double sparseness_threshold) {
  // TODO(naveenoid) : needs test coverage.
  for (const auto& binding : prog.linear_equality_constraints()) {
    const auto& constraint = binding.constraint();
    int var_dim = binding.GetNumElements();
    // variable_indices[i] is the index of the i'th variable.
    std::vector<int> variable_indices;
    variable_indices.reserve(var_dim);
    for (const DecisionVariableMatrixX& var :
         binding.variable_list().variables()) {
      DRAKE_ASSERT(var.cols() == 1);
      for (int i = 0; i < static_cast<int>(var.rows()); ++i) {
        variable_indices.push_back(prog.FindDecisionVariableIndex(var(i, 0)));
      }
    }
    const int error =
        AddLinearConstraint(model, constraint->A(), constraint->lower_bound(),
                            variable_indices, GRB_EQUAL, sparseness_threshold);
    if (error) {
      return error;
    }
  }

  for (const auto& binding : prog.linear_constraints()) {
    const auto& constraint = binding.constraint();
    int var_dim = binding.GetNumElements();
    // variable_indices[i] is the index of the i'th variable
    std::vector<int> variable_indices;
    variable_indices.reserve(var_dim);
    for (const DecisionVariableMatrixX& var :
         binding.variable_list().variables()) {
      DRAKE_ASSERT(var.cols() == 1);
      for (int i = 0; i < static_cast<int>(var.rows()); ++i) {
        variable_indices.push_back(prog.FindDecisionVariableIndex(var(i, 0)));
      }
    }
    const Eigen::MatrixXd& A = constraint->A();
    const Eigen::VectorXd& lb = constraint->lower_bound();
    const Eigen::VectorXd& ub = constraint->upper_bound();

    // Go through the matrix A row by row to determine whether to add it as
    // a less than or greater than constraint, or both.
    for (int i = 0; i < static_cast<int>(A.rows()); i++) {
      // In each row, we find out the non-zero entries in A.row(i), such that
      // the linear constraint is
      // lb(i) <= sum_j linear_coeff_row_i[j] * x[j] <= ub(i)
      // where x is the aggregated decision variable for the entire optimization
      // problem.
      std::vector<int> variable_indices_row_i;
      std::vector<double> linear_coeff_row_i;
      variable_indices_row_i.reserve(var_dim);
      linear_coeff_row_i.reserve(var_dim);
      for (int j = 0; j < var_dim; j++) {
        if (std::abs(A(i, j)) > sparseness_threshold) {
          variable_indices_row_i.push_back(variable_indices[j]);
          linear_coeff_row_i.push_back(A(i, j));
        }
      }
      if (!std::isinf(lb(i))) {
        int error = GRBaddconstr(
            model, variable_indices_row_i.size(), variable_indices_row_i.data(),
            linear_coeff_row_i.data(), GRB_GREATER_EQUAL, lb(i), nullptr);
        if (error) {
          return error;
        }
      }
      if (!std::isinf(ub(i))) {
        int error = GRBaddconstr(
            model, variable_indices_row_i.size(), variable_indices_row_i.data(),
            linear_coeff_row_i.data(), GRB_LESS_EQUAL, ub(i), nullptr);
        if (error) {
          return error;
        }
      }
    }
  }
  // If loop completes, no errors exist so the value '0' must be returned.
  return 0;
}
}  // close namespace

bool GurobiSolver::available() const { return true; }

SolutionResult GurobiSolver::Solve(MathematicalProgram& prog) const {
  // We only process quadratic costs and linear / bounding box
  // constraints.

  GRBenv* env = nullptr;
  GRBloadenv(&env, nullptr);
  // Corresponds to no console or file logging.
  GRBsetintparam(env, GRB_INT_PAR_OUTPUTFLAG, 0);

  DRAKE_ASSERT(prog.generic_costs().empty());
  DRAKE_ASSERT(prog.generic_constraints().empty());

  const int num_prog_vars = prog.num_vars();

  // Potentially Gurobi can add variables on top of the variables in
  // MathematicalProgram prog.
  // is_new_variable[i] is true if the i'th variable in Gurobi environment is
  // not stored in MathematicalProgram, but added by the GurobiSolver.
  // For example, for Lorentz cone and rotated Lorentz cone constraint,to impose
  // that A*x+b lies in the (rotated) Lorentz cone, we add decision variable z
  // to Gurobi, defined as z = A*x + b.
  // The size of is_new_variable should increase if we add new decision
  // variables to Gurobi model.
  // The invariant is
  // EXPECT_TRUE(IsNumberOfVariablesAsExpected(model, is_new_variables.size()))
  std::vector<bool> is_new_variable(num_prog_vars, false);

  // Bound constraints.
  std::vector<double> xlow(num_prog_vars,
                           -std::numeric_limits<double>::infinity());
  std::vector<double> xupp(num_prog_vars,
                           std::numeric_limits<double>::infinity());

  const std::vector<MathematicalProgram::VarType>& var_type =
      prog.DecisionVariableTypes();

  std::vector<char> gurobi_var_type(num_prog_vars);
  for (int i = 0; i < num_prog_vars; ++i) {
    switch (var_type[i]) {
      case MathematicalProgram::VarType::CONTINUOUS:
        gurobi_var_type[i] = GRB_CONTINUOUS;
        break;
      case MathematicalProgram::VarType::BINARY:
        gurobi_var_type[i] = GRB_BINARY;
        break;
      case MathematicalProgram::VarType::INTEGER:
        gurobi_var_type[i] = GRB_INTEGER;
    }
  }

  for (const auto& binding : prog.bounding_box_constraints()) {
    const auto& constraint = binding.constraint();
    const Eigen::VectorXd& lower_bound = constraint->lower_bound();
    const Eigen::VectorXd& upper_bound = constraint->upper_bound();
    int var_idx = 0;
    for (const DecisionVariableMatrixX& var :
         binding.variable_list().variables()) {
      DRAKE_ASSERT(var.cols() == 1);
      for (int k = 0; k < var.rows(); ++k) {
        const int idx = prog.FindDecisionVariableIndex(var(k, 0));
        xlow[idx] = std::max(lower_bound(var_idx), xlow[idx]);
        xupp[idx] = std::min(upper_bound(var_idx), xupp[idx]);
        var_idx++;
      }
    }
  }

  GRBmodel* model = nullptr;
  GRBnewmodel(env, &model, "gurobi_model", num_prog_vars, nullptr, &xlow[0],
              &xupp[0], gurobi_var_type.data(), nullptr);

  int error = 0;
  // TODO(naveenoid) : This needs access externally.
  double sparseness_threshold = 1e-14;
  error = AddCosts(model, prog, sparseness_threshold);

  if (!error) {
    error = ProcessLinearConstraints(model, prog, sparseness_threshold);
  }

  // Add Lorentz cone constraints.
  if (!error) {
    error = AddSecondOrderConeConstraints(prog.lorentz_cone_constraints(),
                                          false, sparseness_threshold, model,
                                          &is_new_variable);
  }

  // Add rotated Lorentz cone constraints.
  if (!error) {
    error = AddSecondOrderConeConstraints(
        prog.rotated_lorentz_cone_constraints(), true, sparseness_threshold,
        model, &is_new_variable);
  }

  DRAKE_ASSERT(IsNumberOfVariablesAsExpected(model, is_new_variable.size()));

  if (!error) {
    for (const auto it : prog.GetSolverOptionsDouble("GUROBI")) {
      error = GRBsetdblparam(env, it.first.c_str(), it.second);
      if (error) {
        continue;
      }
    }
  }
  if (!error) {
    for (const auto it : prog.GetSolverOptionsInt("GUROBI")) {
      error = GRBsetintparam(env, it.first.c_str(), it.second);
      if (error) {
        continue;
      }
    }
  }

  if (!error) {
    error = GRBoptimize(model);
  }

  SolutionResult result = SolutionResult::kUnknownError;

  // If any error exists so far, its either from invalid input or
  // from unknown errors.
  // TODO(naveenoid) : Properly handle gurobi specific error.
  // message.
  if (error) {
    // TODO(naveenoid) : log error message using GRBgeterrormsg(env).
    result = SolutionResult::kInvalidInput;
  } else {
    int optimstatus = 0;
    GRBgetintattr(model, GRB_INT_ATTR_STATUS, &optimstatus);

    if (optimstatus != GRB_OPTIMAL && optimstatus != GRB_SUBOPTIMAL) {
      if (optimstatus == GRB_INF_OR_UNBD) {
        result = SolutionResult::kInfeasibleConstraints;
      }
    } else {
      result = SolutionResult::kSolutionFound;
      int num_total_variables = is_new_variable.size();
      // solver_sol_vector includes the potentially newly added variables, i.e.,
      // variables not in MathematicalProgram prog, but added to Gurobi by
      // GurobiSolver.
      // prog_sol_vector only includes the original variables in
      // MathematicalProgram prog.
      std::vector<double> solver_sol_vector(num_total_variables);
      GRBgetdblattrarray(model, GRB_DBL_ATTR_X, 0, num_total_variables,
                         solver_sol_vector.data());
      Eigen::VectorXd prog_sol_vector(num_prog_vars);
      int prog_var_count = 0;
      for (int i = 0; i < num_total_variables; ++i) {
        if (!is_new_variable[i]) {
          prog_sol_vector(prog_var_count) = solver_sol_vector[i];
          ++prog_var_count;
        }
      }
      prog.SetDecisionVariableValues(prog_sol_vector);
    }
  }

  prog.SetSolverResult(SolverName(), error);

  GRBfreemodel(model);
  GRBfreeenv(env);

  return result;
}

}  // namespace solvers
}  // namespace drake
