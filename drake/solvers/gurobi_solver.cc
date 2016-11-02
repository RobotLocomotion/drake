#include "drake/solvers/gurobi_solver.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

// NOLINTNEXTLINE(build/include) False positive due to weird include style.
#include "gurobi_c++.h"

#include "drake/common/drake_assert.h"
#include "drake/math/eigen_sparse_triplet.h"

namespace drake {
namespace solvers {
namespace {

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
 * Add Lorentz cone constraints.
 * A Lorentz cone constraint is the convex conic constraint
 * x(0) >= sqrt(x(1)^2 + ... + x(N-1)^2)
 */
int AddLorentzConeConstraints(GRBmodel* model,
                              const MathematicalProgram& prog) {
  for (const auto& binding : prog.lorentz_cone_constraints()) {
    int num_constraint_variable = static_cast<int>(binding.GetNumElements());
    std::vector<int> variable_indices;
    variable_indices.reserve(static_cast<size_t>(num_constraint_variable));
    auto variable_list = binding.variable_list();
    for (const DecisionVariableView& var : variable_list) {
      for (int i = 0; i < static_cast<int>(var.size()); i++) {
        variable_indices.push_back(static_cast<int>(var.index()) + i);
      }
    }
    // We will build a matrix Q = diag([-1;1;1;...;1;], and we will use
    // qrow to store the row    indices of the non-zero entries of Q.
    // qcol to store the column indices of the non-zero entries of Q.
    // qval to store the value          of the non-zero entries of Q.
    std::vector<int> qrow;
    std::vector<int> qcol;
    std::vector<double> qval;
    qrow.reserve(static_cast<size_t>(num_constraint_variable));
    qcol.reserve(static_cast<size_t>(num_constraint_variable));
    qval.reserve(static_cast<size_t>(num_constraint_variable));
    qrow.push_back(variable_indices[0]);
    qcol.push_back(variable_indices[0]);
    qval.push_back(-1.0);
    for (int i = 1; i < num_constraint_variable; i++) {
      qrow.push_back(variable_indices[i]);
      qcol.push_back(variable_indices[i]);
      qval.push_back(1.0);
    }
    int error =
        GRBsetdblattrelement(model, GRB_DBL_ATTR_LB, variable_indices[0], 0.0);
    if (error) {
      return error;
    }
    error = GRBaddqconstr(model, 0, nullptr, nullptr, num_constraint_variable,
                          qrow.data(), qcol.data(), qval.data(), GRB_LESS_EQUAL,
                          0.0, NULL);
    if (error) {
      return error;
    }
  }
  return 0;
}

/*
 * Add the rotated lorentz cone constraint.
 * x(0) * x(1) >= x(2)^2 + .. x(N-1)^2
 * x(0) >= 0, x(1) >= 0
 */
int AddRotatedLorentzConeConstraint(GRBmodel* model,
                                    const MathematicalProgram& prog) {
  for (const auto& binding : prog.rotated_lorentz_cone_constraints()) {
    int num_constraint_variable = static_cast<int>(binding.GetNumElements());
    std::vector<int> variable_indices;
    variable_indices.reserve(static_cast<size_t>(num_constraint_variable));
    auto variable_list = binding.variable_list();
    for (const DecisionVariableView& var : variable_list) {
      for (int i = 0; i < static_cast<int>(var.size()); i++) {
        variable_indices.push_back(static_cast<int>(var.index()) + i);
      }
    }
    // Build a matrix Q = [0 -1 0 0 ... 0]
    //                    [0  0 0 0 ... 0]
    //                    [0  0 1 0 ... 0]
    //                    [0  0 0 1 ... 0]
    //                        ...
    //                    [0  0 0 0 ... 1]
    // qrow to store the row    indices of the non-zero entries of Q.
    // qcol to store the column indices of the non-zero entries of Q.
    // qval to store the value          of the non-zero entries of Q.
    std::vector<int> qrow;
    std::vector<int> qcol;
    std::vector<double> qval;
    qrow.reserve(static_cast<size_t>(num_constraint_variable) - 1);
    qcol.reserve(static_cast<size_t>(num_constraint_variable) - 1);
    qval.reserve(static_cast<size_t>(num_constraint_variable) - 1);
    qrow.push_back(variable_indices[0]);
    qcol.push_back(variable_indices[1]);
    qval.push_back(-1.0);
    for (int i = 2; i < num_constraint_variable; i++) {
      qrow.push_back(variable_indices[i]);
      qcol.push_back(variable_indices[i]);
      qval.push_back(1.0);
    }
    // x[0] >= 0, x[1] >= 0
    int error;
    for (int i = 0; i < 2; i++) {
      error = GRBsetdblattrelement(model, GRB_DBL_ATTR_LB, variable_indices[i],
                                   0.0);
      if (error) {
        return error;
      }
    }
    error = GRBaddqconstr(model, 0, nullptr, nullptr,
                          num_constraint_variable - 1, qrow.data(), qcol.data(),
                          qval.data(), GRB_LESS_EQUAL, 0.0, NULL);
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
    // binding.VariableListToVectorXd(i).
    std::vector<int> constraint_variable_index(constraint_variable_dimension);
    int constraint_variable_count = 0;
    for (const DecisionVariableView& var : binding.variable_list()) {
      for (int i = 0; i < static_cast<int>(var.size()); i++) {
        constraint_variable_index[constraint_variable_count] = var.index() + i;
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
    for (const DecisionVariableView& var : binding.variable_list()) {
      for (int i = 0; i < static_cast<int>(var.size()); i++) {
        b_nonzero_coefs.push_back(Eigen::Triplet<double>(
            var.index() + i, 0, c(constraint_variable_count)));
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
    for (const DecisionVariableView& var : binding.variable_list()) {
      for (int i = 0; i < static_cast<int>(var.size()); i++) {
        variable_indices.push_back(var.index() + i);
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
    for (const DecisionVariableView& var : binding.variable_list()) {
      for (int i = 0; i < static_cast<int>(var.size()); i++) {
        variable_indices.push_back(var.index() + i);
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

  const int num_vars = prog.num_vars();

  // Bound constraints.
  std::vector<double> xlow(num_vars, -std::numeric_limits<double>::infinity());
  std::vector<double> xupp(num_vars, std::numeric_limits<double>::infinity());

  const std::vector<DecisionVariable::VarType>& var_type = prog.VariableTypes();

  std::vector<char> gurobi_var_type(num_vars);
  for (int i = 0; i < num_vars; ++i) {
    switch (var_type[i]) {
      case DecisionVariable::VarType::CONTINUOUS:
        gurobi_var_type[i] = GRB_CONTINUOUS;
        break;
      case DecisionVariable::VarType::BINARY:
        gurobi_var_type[i] = GRB_BINARY;
        break;
      case DecisionVariable::VarType::INTEGER:
        gurobi_var_type[i] = GRB_INTEGER;
    }
  }

  for (const auto& binding : prog.bounding_box_constraints()) {
    const auto& constraint = binding.constraint();
    const Eigen::VectorXd& lower_bound = constraint->lower_bound();
    const Eigen::VectorXd& upper_bound = constraint->upper_bound();
    int var_idx = 0;
    for (const DecisionVariableView& decision_variable_view :
         binding.variable_list()) {
      for (size_t k = 0; k < decision_variable_view.size(); k++) {
        const int idx = decision_variable_view.index() + k;
        xlow[idx] = std::max(lower_bound(var_idx), xlow[idx]);
        xupp[idx] = std::min(upper_bound(var_idx), xupp[idx]);
        var_idx++;
      }
    }
  }

  GRBmodel* model = nullptr;
  GRBnewmodel(env, &model, "gurobi_model", num_vars, nullptr, &xlow[0],
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
    error = AddLorentzConeConstraints(model, prog);
  }

  // Add rotated Lorentz cone constraints.
  if (!error) {
    error = AddRotatedLorentzConeConstraint(model, prog);
  }

  SolutionResult result = SolutionResult::kUnknownError;

  if (!error) {
    error = GRBoptimize(model);
  }

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
      Eigen::VectorXd sol_vector = Eigen::VectorXd::Zero(num_vars);
      GRBgetdblattrarray(model, GRB_DBL_ATTR_X, 0, num_vars, sol_vector.data());
      prog.SetDecisionVariableValues(sol_vector);
    }
  }

  prog.SetSolverResult("Gurobi", error);

  GRBfreemodel(model);
  GRBfreeenv(env);

  return result;
}

}  // namespace solvers
}  // namespace drake
