#include "drake/solvers/mosek_solver.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <list>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <mosek.h>

namespace drake {
namespace solvers {
namespace {
// Add LinearConstraints and LinearEqualityConstraints to the Mosek task.
template <typename C>
MSKrescodee AddLinearConstraintsFromBindings(
    MSKtask_t* task, const std::vector<Binding<C>>& constraint_list,
    bool is_equality_constraint, const MathematicalProgram& prog) {
  for (const auto& binding : constraint_list) {
    auto constraint = binding.constraint();
    const Eigen::MatrixXd& A = constraint->A();
    const Eigen::VectorXd& lb = constraint->lower_bound();
    const Eigen::VectorXd& ub = constraint->upper_bound();
    MSKint32t constraint_idx = 0;
    MSKrescodee rescode = MSK_getnumcon(*task, &constraint_idx);
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
    rescode = MSK_appendcons(*task, A.rows());
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
    // Loop through each row of the constraint, and determine the sense of
    // each constraint. The sense can be equality constraint, less than,
    // greater than, or bounded on both side.
    for (int i = 0; i < A.rows(); ++i) {
      if (is_equality_constraint) {
        rescode =
            MSK_putconbound(*task, constraint_idx + i, MSK_BK_FX, lb(i), ub(i));
        if (rescode != MSK_RES_OK) {
          return rescode;
        }
      } else {
        if (std::isinf(lb(i)) && std::isinf(ub(i))) {
          rescode = MSK_putconbound(*task, constraint_idx + i, MSK_BK_FR,
                                    -MSK_INFINITY, MSK_INFINITY);
        } else if (std::isinf(lb(i)) && !std::isinf(ub(i))) {
          rescode = MSK_putconbound(*task, constraint_idx + i, MSK_BK_UP,
                                    -MSK_INFINITY, ub(i));
        } else if (!std::isinf(lb(i)) && std::isinf(ub(i))) {
          rescode = MSK_putconbound(*task, constraint_idx + i, MSK_BK_LO, lb(i),
                                    MSK_INFINITY);
        } else {
          rescode = MSK_putconbound(*task, constraint_idx + i, MSK_BK_RA, lb(i),
                                    ub(i));
        }
        if (rescode != MSK_RES_OK) {
          return rescode;
        }
      }
      std::vector<MSKint32t> A_nonzero_col_idx;
      std::vector<double> A_nonzero_val;
      A_nonzero_col_idx.reserve(A.cols());
      A_nonzero_val.reserve(A.cols());

      for (int k = 0; k < static_cast<int>(binding.GetNumElements()); ++k) {
        if (std::abs(A(i, k)) > Eigen::NumTraits<double>::epsilon()) {
          A_nonzero_col_idx.push_back(
              prog.FindDecisionVariableIndex(binding.variables()(k)));
          A_nonzero_val.push_back(A(i, k));
        }
      }

      rescode = MSK_putarow(*task, constraint_idx + i, A_nonzero_val.size(),
                            A_nonzero_col_idx.data(), A_nonzero_val.data());
      if (rescode != MSK_RES_OK) {
        return rescode;
      }
    }
  }
  return MSK_RES_OK;
}

MSKrescodee AddLinearConstraints(const MathematicalProgram& prog,
                                 MSKtask_t* task) {
  MSKrescodee rescode = AddLinearConstraintsFromBindings(
      task, prog.linear_equality_constraints(), true, prog);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  rescode = AddLinearConstraintsFromBindings(task, prog.linear_constraints(),
                                             false, prog);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }

  return rescode;
}

MSKrescodee AddBoundingBoxConstraints(const MathematicalProgram& prog,
                                      MSKtask_t* task) {
  int num_vars = prog.num_vars();
  std::vector<double> x_lb(num_vars, -std::numeric_limits<double>::infinity());
  std::vector<double> x_ub(num_vars, std::numeric_limits<double>::infinity());
  for (const auto& binding : prog.bounding_box_constraints()) {
    const auto& constraint = binding.constraint();
    const Eigen::VectorXd& lower_bound = constraint->lower_bound();
    const Eigen::VectorXd& upper_bound = constraint->upper_bound();

    for (int i = 0; i < static_cast<int>(binding.GetNumElements()); ++i) {
      size_t x_idx = prog.FindDecisionVariableIndex(binding.variables()(i));
      x_lb[x_idx] = std::max(x_lb[x_idx], lower_bound[i]);
      x_ub[x_idx] = std::min(x_ub[x_idx], upper_bound[i]);
    }
  }

  MSKrescodee rescode = MSK_RES_OK;
  for (int i = 0; i < num_vars; i++) {
    if (std::isinf(x_lb[i]) && std::isinf(x_ub[i])) {
      rescode =
          MSK_putvarbound(*task, i, MSK_BK_FR, -MSK_INFINITY, MSK_INFINITY);
    } else if (std::isinf(x_lb[i]) && !std::isinf(x_ub[i])) {
      rescode = MSK_putvarbound(*task, i, MSK_BK_UP, -MSK_INFINITY, x_ub[i]);
    } else if (!std::isinf(x_lb[i]) && std::isinf(x_ub[i])) {
      rescode = MSK_putvarbound(*task, i, MSK_BK_LO, x_lb[i], MSK_INFINITY);
    } else {
      rescode = MSK_putvarbound(*task, i, MSK_BK_RA, x_lb[i], x_ub[i]);
    }
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
  }
  return rescode;
}

/*
 * This is the helper function to add two types of second order cone
 * constraints:
 * 1. A Lorentz cone constraint:
 *    z = A*x+b
 *    z0 >= sqrt(z1^2 + .. zN^2)
 * 2. A rotated Lorentz cone constraint:
 *    z = A*x+b
 *    z0*z1 >= z2^2 + .. + zN^2,
 *    z0 >= 0, z1 >=0
 * Mosek does not allow two cones to share variables. To overcome this,
 * we will add a new set of variable (z0, ..., zN)
 * @param is_new_variable  Refer to the documentation on is_new_variable in
 * MosekSolver::Solve() function
 */
template <typename C>
MSKrescodee AddSecondOrderConeConstraints(
    const MathematicalProgram& prog,
    const std::vector<Binding<C>>& second_order_cone_constraints,
    MSKtask_t* task, std::vector<bool>* is_new_variable) {
  static_assert(std::is_same<C, LorentzConeConstraint>::value ||
                    std::is_same<C, RotatedLorentzConeConstraint>::value,
                "Should be either Lorentz cone constraint or rotated Lorentz "
                "cone constraint");
  bool is_rotated_cone = std::is_same<C, RotatedLorentzConeConstraint>::value;
  MSKrescodee rescode = MSK_RES_OK;
  for (auto const& binding : second_order_cone_constraints) {
    std::vector<MSKint32t> cone_var_indices(binding.GetNumElements());

    for (int i = 0; i < static_cast<int>(binding.GetNumElements()); ++i) {
      cone_var_indices[i] =
          prog.FindDecisionVariableIndex(binding.variables()(i));
    }

    const auto& A = binding.constraint()->A();
    const auto& b = binding.constraint()->b();
    const int num_z = A.rows();
    MSKint32t num_total_vars = 0;
    rescode = MSK_getnumvar(*task, &num_total_vars);
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
    rescode = MSK_appendvars(*task, num_z);
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
    is_new_variable->resize(num_total_vars + num_z);
    std::vector<MSKint32t> new_var_indices(num_z);
    for (int i = 0; i < num_z; ++i) {
      is_new_variable->at(num_total_vars + i) = true;
      new_var_indices[i] = num_total_vars + i;
      rescode = MSK_putvarbound(*task, new_var_indices[i], MSK_BK_FR,
                                -MSK_INFINITY, MSK_INFINITY);
      if (rescode != MSK_RES_OK) {
        return rescode;
      }
    }
    MSKconetypee cone_type = is_rotated_cone ? MSK_CT_RQUAD : MSK_CT_QUAD;
    rescode =
        MSK_appendcone(*task, cone_type, 0.0, num_z, new_var_indices.data());
    if (rescode != MSK_RES_OK) {
      return rescode;
    }

    // Add the linear constraint
    // z = A*x+b
    // Unfortunately Mosek's definition of rotated Lorentz cone is different
    // from ours. The rotated Lorentz cone in Mosek is defined as
    // 2*z(0) * z(1) >= z(2)^2 + ... + z(n-1)^2
    // Our definition of rotated Lorentz cone is
    //   z(0) * z(1) >= z(2)^2 + ... + z(n-1)^2
    // So there is a factor of 2 for rotated Lorentz cone.
    // With this difference in rotated Lorentz cone, the first row of constraint
    // z = A * x + b needs special treatment.
    // If using Lorentz cone,
    // Add the linear constraint
    //   z0 = a0^T * x + b0;
    // If using rotated Lorentz cone, add the linear constraint
    // 2*z0 = a0^T * x + b0
    int num_lin_cons;
    rescode = MSK_getnumcon(*task, &num_lin_cons);
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
    rescode = MSK_appendcons(*task, num_z);
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
    std::vector<MSKint32t> var_indices(cone_var_indices);
    var_indices.push_back(new_var_indices[0]);
    double y0_factor = is_rotated_cone ? -2 : -1;
    Eigen::RowVectorXd val0(1 + cone_var_indices.size());
    val0.head(cone_var_indices.size()) = A.row(0);
    val0(cone_var_indices.size()) = y0_factor;
    rescode = MSK_putarow(*task, num_lin_cons, 1 + cone_var_indices.size(),
                          var_indices.data(), val0.data());
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
    rescode = MSK_putconbound(*task, num_lin_cons, MSK_BK_FX, -b(0), -b(0));
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
    for (int i = 1; i < num_z; ++i) {
      // In row i of the linear constraint z = A*x+b, the decision variables are
      // [x z(i)]. So compared to the previous row of the constraint, the only
      // changed decision variable is z(i). We can thus pop the last variable
      // (z(i-1)), and push back z(i).
      var_indices.pop_back();
      var_indices.push_back(new_var_indices[i]);
      Eigen::RowVectorXd val(1 + cone_var_indices.size());
      val.head(cone_var_indices.size()) = A.row(i);
      val(cone_var_indices.size()) = -1;
      rescode =
          MSK_putarow(*task, num_lin_cons + i, 1 + cone_var_indices.size(),
                      var_indices.data(), val.data());
      if (rescode != MSK_RES_OK) {
        return rescode;
      }
      rescode =
          MSK_putconbound(*task, num_lin_cons + i, MSK_BK_FX, -b(i), -b(i));
      if (rescode != MSK_RES_OK) {
        return rescode;
      }
    }
  }
  // Expect rescode == MSK_RES_OK.
  return rescode;
}

/*
 * To add positive semidefinite matrix, Mosek needs to create a "bar variable"
 * We will need to add linear constraints
 * X = X_bar
 * where X is some matrix expression, that we want to be positive semidefinite.
 */
MSKrescodee AddBarVariable(int rows, MSKtask_t* task) {
  MSKrescodee rescode = MSK_RES_OK;
  rescode = MSK_appendbarvars(*task, 1, &rows);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }

  int num_bar_var = 0;
  rescode = MSK_getnumbarvar(*task, &num_bar_var);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }

  int num_linear_constraint = 0;
  rescode = MSK_getnumcon(*task, &num_linear_constraint);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }

  rescode = MSK_appendcons(*task, (rows + 1) * rows / 2);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }

  int new_linear_constraint_count = 0;
  for (int j = 0; j < rows; ++j) {
    for (int i = j; i < rows; ++i) {
      int linear_constraint_index =
          num_linear_constraint + new_linear_constraint_count;
      double bar_A_ij_val = i == j ? -1.0 : -0.5;
      MSKint64t bar_A_matrix_idx;
      rescode = MSK_appendsparsesymmat(*task, rows, 1, &i, &j, &bar_A_ij_val,
                                       &bar_A_matrix_idx);
      if (rescode != MSK_RES_OK) {
        return rescode;
      }

      double bar_A_weights = 1.0;
      rescode = MSK_putbaraij(*task, linear_constraint_index, num_bar_var - 1,
                              1, &bar_A_matrix_idx, &bar_A_weights);
      if (rescode != MSK_RES_OK) {
        return rescode;
      }

      ++new_linear_constraint_count;
    }
  }
  return rescode;
}

MSKrescodee AddPositiveSemidefiniteConstraints(const MathematicalProgram& prog,
                                               MSKtask_t* task) {
  MSKrescodee rescode = MSK_RES_OK;
  for (const auto& binding : prog.positive_semidefinite_constraints()) {
    const auto& symmetric_matrix_variable = binding.variables();

    int num_linear_constraint = 0;
    rescode = MSK_getnumcon(*task, &num_linear_constraint);
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
    // Add S_bar as new variables. Mosek needs to create so called "bar
    // variable"
    // for matrix in positive semidefinite cones.
    int matrix_rows = binding.constraint()->matrix_rows();

    AddBarVariable(matrix_rows, task);

    // Add the constraint S = S_bar
    // This linear constraint is imposed as
    // S(i, j) - trace(bar_A_ij * S_bar) = 0
    // where bar_A_ij has the same dimension as S_bar
    // bar_A_ij(i, j) = 0.5, bar_A_ij(j, i) = 0.5 if i != j
    // bar_A_ij(i, j) = 1                         if i == j

    int new_linear_constraint_count = 0;
    // It is important to use the same for-loop order as in
    // AddBarVariable().
    for (int j = 0; j < matrix_rows; ++j) {
      for (int i = j; i < matrix_rows; ++i) {
        int linear_constraint_index =
            num_linear_constraint + new_linear_constraint_count;
        double symmetric_matrix_val = 1.0;
        MSKint32t symmetric_matrix_var_ij_index =
            prog.FindDecisionVariableIndex(
                symmetric_matrix_variable(j * matrix_rows + i));
        rescode =
            MSK_putarow(*task, linear_constraint_index, 1,
                        &symmetric_matrix_var_ij_index, &symmetric_matrix_val);
        if (rescode != MSK_RES_OK) {
          return rescode;
        }

        rescode = MSK_putconbound(*task, linear_constraint_index, MSK_BK_FX,
                                  0.0, 0.0);
        if (rescode != MSK_RES_OK) {
          return rescode;
        }

        ++new_linear_constraint_count;
      }
    }
  }
  return rescode;
}

MSKrescodee AddLinearMatrixInequalityConstraint(const MathematicalProgram& prog,
                                                MSKtask_t* task) {
  MSKrescodee rescode = MSK_RES_OK;
  for (const auto& binding : prog.linear_matrix_inequality_constraints()) {
    int num_linear_constraint = 0;
    rescode = MSK_getnumcon(*task, &num_linear_constraint);
    if (rescode != MSK_RES_OK) {
      return rescode;
    }

    int rows = binding.constraint()->matrix_rows();

    AddBarVariable(rows, task);

    int new_linear_constraint_count = 0;
    // It is important to use the same for-loop order as in
    // AddBarVariable().
    for (int j = 0; j < rows; ++j) {
      for (int i = j; i < rows; ++i) {
        int linear_constraint_index =
            num_linear_constraint + new_linear_constraint_count;

        const auto& F = binding.constraint()->F();
        auto F_it = F.begin();
        rescode = MSK_putconbound(*task, linear_constraint_index, MSK_BK_FX,
                                  -(*F_it)(i, j), -(*F_it)(i, j));
        if (rescode != MSK_RES_OK) {
          return rescode;
        }
        ++F_it;

        Eigen::SparseVector<double, Eigen::RowMajor> A_row(prog.num_vars());
        A_row.setZero();
        A_row.reserve(binding.GetNumElements());

        for (int k = 0; k < static_cast<int>(binding.GetNumElements()); ++k) {
          A_row.coeffRef(prog.FindDecisionVariableIndex(
              binding.variables()(k))) += (*F_it)(i, j);
          ++F_it;
        }

        DRAKE_ASSERT(F_it == F.end());
        rescode = MSK_putarow(*task, linear_constraint_index, A_row.nonZeros(),
                              A_row.innerIndexPtr(), A_row.valuePtr());
        if (rescode != MSK_RES_OK) {
          return rescode;
        }

        ++new_linear_constraint_count;
      }
    }
  }
  return rescode;
}

MSKrescodee AddCosts(const MathematicalProgram& prog, MSKtask_t* task) {
  // Add the cost in the form 0.5 * x' * Q_all * x + linear_terms' * x
  MSKrescodee rescode = MSK_RES_OK;
  int xDim = prog.num_vars();
  // Mosek takes the lower triangular part of Q_all. Q_lower_triplets include
  // the triplets (row_index, col_index, val) on the lower triangular part
  // of Q_all.
  std::vector<Eigen::Triplet<double>> Q_lower_triplets;
  std::vector<Eigen::Triplet<double>> linear_term_triplets;
  for (const auto& binding : prog.quadratic_costs()) {
    const auto& constraint = binding.constraint();
    // The quadratic cost is of form 0.5*x'*Q*x + b*x.
    const auto& Q = constraint->Q();
    const auto& b = constraint->b();
    std::vector<int> var_indices(Q.rows());

    for (int i = 0; i < static_cast<int>(binding.GetNumElements()); ++i) {
      var_indices[i] = prog.FindDecisionVariableIndex(binding.variables()(i));
    }

    for (int i = 0; i < Q.rows(); ++i) {
      int var_index_i = var_indices[i];
      for (int j = 0; j < i; ++j) {
        const double Qij = (Q(i, j) + Q(j, i)) / 2;
        if (std::abs(Qij) > Eigen::NumTraits<double>::epsilon()) {
          if (var_index_i > var_indices[j]) {
            Q_lower_triplets.push_back(
                Eigen::Triplet<double>(var_index_i, var_indices[j], Qij));
          } else {
            Q_lower_triplets.push_back(
                Eigen::Triplet<double>(var_indices[j], var_index_i, Qij));
          }
        }
      }
      if (std::abs(Q(i, i)) > Eigen::NumTraits<double>::epsilon()) {
        Q_lower_triplets.push_back(
            Eigen::Triplet<double>(var_index_i, var_index_i, Q(i, i)));
      }
      if (std::abs(b(i)) > Eigen::NumTraits<double>::epsilon()) {
        linear_term_triplets.push_back(
            Eigen::Triplet<double>(var_index_i, 0, b(i)));
      }
    }
  }
  for (const auto& binding : prog.linear_costs()) {
    const auto& c = binding.constraint()->A();
    for (int i = 0; i < static_cast<int>(binding.GetNumElements()); ++i) {
      if (std::abs(c(i)) > Eigen::NumTraits<double>::epsilon()) {
        linear_term_triplets.push_back(Eigen::Triplet<double>(
            prog.FindDecisionVariableIndex(binding.variables()(i)), 0, c(i)));
      }
    }
  }

  Eigen::SparseMatrix<double> Q_lower(xDim, xDim);
  Q_lower.setFromTriplets(Q_lower_triplets.begin(), Q_lower_triplets.end());
  int Q_nnz = Q_lower.nonZeros();
  std::vector<MSKint32t> qrow(Q_nnz);
  std::vector<MSKint32t> qcol(Q_nnz);
  std::vector<double> qval(Q_nnz);
  int Q_nnz_count = 0;
  for (int i = 0; i < Q_lower.outerSize(); ++i) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(Q_lower, i); it; ++it) {
      qrow[Q_nnz_count] = it.row();
      qcol[Q_nnz_count] = it.col();
      qval[Q_nnz_count] = it.value();
      Q_nnz_count++;
    }
  }
  rescode = MSK_putqobj(*task, Q_nnz, qrow.data(), qcol.data(), qval.data());
  if (rescode != MSK_RES_OK) {
    return rescode;
  }

  Eigen::SparseMatrix<double, Eigen::ColMajor> linear_terms(xDim, 1);
  linear_terms.setFromTriplets(linear_term_triplets.begin(),
                               linear_term_triplets.end());
  for (Eigen::SparseMatrix<double, Eigen::ColMajor>::InnerIterator it(
           linear_terms, 0);
       it; ++it) {
    rescode = MSK_putcj(*task, static_cast<MSKint32t>(it.row()), it.value());
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
  }
  return rescode;
}

MSKrescodee SpecifyVariableType(const MathematicalProgram& prog,
                                MSKtask_t* task,
                                bool* with_integer_or_binary_variable) {
  MSKrescodee rescode = MSK_RES_OK;
  int num_vars = prog.num_vars();
  const std::vector<MathematicalProgram::VarType>& var_type =
      prog.DecisionVariableTypes();
  for (int i = 0; i < num_vars && rescode == MSK_RES_OK; ++i) {
    if (var_type[i] == MathematicalProgram::VarType::INTEGER) {
      rescode = MSK_putvartype(*task, i, MSK_VAR_TYPE_INT);
      if (rescode != MSK_RES_OK) {
        return rescode;
      }
      *with_integer_or_binary_variable = true;
    } else if (var_type[i] == MathematicalProgram::VarType::BINARY) {
      *with_integer_or_binary_variable = true;
      rescode = MSK_putvartype(*task, i, MSK_VAR_TYPE_INT);
      double xi_lb = NAN;
      double xi_ub = NAN;
      MSKboundkeye bound_key;
      if (rescode == MSK_RES_OK) {
        rescode = MSK_getvarbound(*task, i, &bound_key, &xi_lb, &xi_ub);
        if (rescode != MSK_RES_OK) {
          return rescode;
        }
      }
      if (rescode == MSK_RES_OK) {
        xi_lb = std::max(0.0, xi_lb);
        xi_ub = std::min(1.0, xi_ub);
        rescode = MSK_putvarbound(*task, i, MSK_BK_RA, xi_lb, xi_ub);
        if (rescode != MSK_RES_OK) {
          return rescode;
        }
      }
    }
  }
  return rescode;
}
}  // anonymous namespace

bool MosekSolver::available() const { return true; }

SolutionResult MosekSolver::Solve(MathematicalProgram& prog) const {
  const int num_vars = prog.num_vars();
  MSKenv_t env = nullptr;
  MSKtask_t task = nullptr;
  MSKrescodee rescode;

  // When solving optimization problem with Mosek, we sometimes need to add
  // new variables to Mosek, so that the solver can parse the constraint.
  // is_new_variable has the same length as the number of variables in Mosek
  // i.e. the invariant is  MSKint32t num_mosek_vars;
  //                        MSK_getnumvar(task, &num_mosek_vars);
  //                        assert(is_new_variable.length() ==  num_mosek_vars);
  // is_new_variable[i] is true if the variable is not a part of the variable
  // in MathematicalProgram prog, but added to Mosek solver.
  std::vector<bool> is_new_variable(num_vars, false);

  // Create the Mosek environment.
  rescode = MSK_makeenv(&env, nullptr);
  if (rescode == MSK_RES_OK) {
    // Create the optimization task.
    rescode = MSK_maketask(env, 0, num_vars, &task);
  }
  if (rescode == MSK_RES_OK) {
    rescode = MSK_appendvars(task, num_vars);
  }
  // Add costs
  if (rescode == MSK_RES_OK) {
    rescode = AddCosts(prog, &task);
  }
  // Add bounding box constraints on decision variables.
  if (rescode == MSK_RES_OK) {
    rescode = AddBoundingBoxConstraints(prog, &task);
  }
  // Specify binary variables.
  bool with_integer_or_binary_variable = false;
  if (rescode == MSK_RES_OK) {
    rescode =
        SpecifyVariableType(prog, &task, &with_integer_or_binary_variable);
  }
  // Add linear constraints.
  if (rescode == MSK_RES_OK) {
    rescode = AddLinearConstraints(prog, &task);
  }

  // Add Lorentz cone constraints.
  if (rescode == MSK_RES_OK) {
    rescode = AddSecondOrderConeConstraints(
        prog, prog.lorentz_cone_constraints(), &task, &is_new_variable);
  }

  // Add rotated Lorentz cone constraints.
  if (rescode == MSK_RES_OK) {
    rescode = AddSecondOrderConeConstraints(
        prog, prog.rotated_lorentz_cone_constraints(), &task, &is_new_variable);
  }

  // Add positive semidefinite constraints.
  if (rescode == MSK_RES_OK) {
    rescode = AddPositiveSemidefiniteConstraints(prog, &task);
  }

  // Add linear matrix inequality constraints.
  if (rescode == MSK_RES_OK) {
    rescode = AddLinearMatrixInequalityConstraint(prog, &task);
  }

  SolutionResult result = SolutionResult::kUnknownError;
  // Run optimizer.
  if (rescode == MSK_RES_OK) {
    // TODO(hongkai.dai@tri.global): add trmcode to the returned struct.
    MSKrescodee trmcode;  // termination code
    rescode = MSK_optimizetrm(task, &trmcode);
  }

  // Determines the solution type.
  // TODO(hongkai.dai@tri.global) : add the integer solution type. And test
  // the non-default optimizer.
  MSKsoltypee solution_type;
  if (with_integer_or_binary_variable) {
    solution_type = MSK_SOL_ITG;
  } else if (prog.quadratic_costs().empty() &&
             prog.lorentz_cone_constraints().empty() &&
             prog.rotated_lorentz_cone_constraints().empty() &&
             prog.positive_semidefinite_constraints().empty() &&
             prog.linear_matrix_inequality_constraints().empty()) {
    solution_type = MSK_SOL_BAS;
  } else {
    solution_type = MSK_SOL_ITR;
  }

  // TODO(hongkai.dai@tri.global) : Add MOSEK paramaters.
  // Mosek parameter are added by enum, not by string.
  if (rescode == MSK_RES_OK) {
    MSKsolstae solution_status;
    if (rescode == MSK_RES_OK) {
      rescode = MSK_getsolsta(task, solution_type, &solution_status);
    }
    if (rescode == MSK_RES_OK) {
      switch (solution_status) {
        case MSK_SOL_STA_OPTIMAL:
        case MSK_SOL_STA_NEAR_OPTIMAL:
        case MSK_SOL_STA_INTEGER_OPTIMAL:
        case MSK_SOL_STA_NEAR_INTEGER_OPTIMAL: {
          result = SolutionResult::kSolutionFound;
          MSKint32t num_mosek_vars;
          rescode = MSK_getnumvar(task, &num_mosek_vars);
          DRAKE_ASSERT(rescode == MSK_RES_OK);
          Eigen::VectorXd mosek_sol_vector(num_mosek_vars);
          rescode = MSK_getxx(task, solution_type, mosek_sol_vector.data());
          DRAKE_ASSERT(rescode == MSK_RES_OK);
          Eigen::VectorXd sol_vector(num_vars);
          int var_count = 0;
          for (int i = 0; i < num_mosek_vars; ++i) {
            if (!is_new_variable[i]) {
              sol_vector(var_count) = mosek_sol_vector(i);
              var_count++;
            }
          }
          if (rescode == MSK_RES_OK) {
            prog.SetDecisionVariableValues(sol_vector);
          }
          break;
        }
        case MSK_SOL_STA_DUAL_INFEAS_CER:
        case MSK_SOL_STA_PRIM_INFEAS_CER:
        case MSK_SOL_STA_NEAR_DUAL_FEAS:
        case MSK_SOL_STA_NEAR_PRIM_INFEAS_CER: {
          result = SolutionResult::kInfeasibleConstraints;
          break;
        }
        default: {
          result = SolutionResult::kUnknownError;
          break;
        }
      }
    }
  }

  prog.SetSolverResult(solver_type(), result);
  if (rescode != MSK_RES_OK) {
    result = SolutionResult::kUnknownError;
  }

  MSK_deletetask(&task);
  MSK_deleteenv(&env);
  return result;
}

}  // namespace solvers
}  // namespace drake
