#include "drake/solvers/mosek_solver.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <list>
#include <stdexcept>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <mosek.h>

#include "drake/common/scoped_singleton.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
namespace {
// For a positive semidefinite matrix variable X, Mosek treats it specially.
// Check https://docs.mosek.com/8.1/capi/tutorial-sdo-shared.html
// that Mosek doesn't allow us imposing a linear constraint on the entry of X
// directly; instead Mosek only allows to impose the linear constraint on X
// through matrix inner product. For example, if we want to impose a constraint
// X(0, 0) >= 0, Mosek doesn't allow the user to extract the (0, 0)'th entry of
// X, and then add a bound on X(0, 0). Instead we need to define a matrix E_00,
// such that E_00(0, 0) = 1, E_00(i, j) = 0, if i ≠ 0 or j ≠ 0, and impose the
// linear constraint <E_00, X> >= 0. Mosek internally stores the psd matrix
// variables, and we can access the matrix with a unique index.
struct MatrixVariableEntry {
  MSKint64t bar_matrix_index;
  MSKint32t row_index;
  MSKint32t col_index;
};

// This function is used to print information for each iteration to the console,
// it will show PRSTATUS, PFEAS, DFEAS, etc. For more information, check out
// https://docs.mosek.com/8.1/capi/solver-io.html. This printstr is copied
// directly from https://docs.mosek.com/8.1/capi/solver-io.html#stream-logging.
void MSKAPI printstr(void*, const char str[]) { printf("%s", str); }

// Add LinearConstraints and LinearEqualityConstraints to the Mosek task.
template <typename C>
MSKrescodee AddLinearConstraintsFromBindings(
    MSKtask_t* task, const std::vector<Binding<C>>& constraint_list,
    bool is_equality_constraint, const MathematicalProgram& prog) {
  for (const auto& binding : constraint_list) {
    auto constraint = binding.evaluator();
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

// Add the bounds on the decision variables in @p prog. Note that if a decision
// variable in positive definite matrix has a bound, we need to add new slack
// variable to Mosek, with the constraint that the slack variable equals to that
// decision variable (in Mosek matrix variable), and the bounds on this slack
// variable.
MSKrescodee AddBoundingBoxConstraints(
    const MathematicalProgram& prog,
    const std::unordered_map<int, MatrixVariableEntry>&
        map_decision_variable_index_to_mosek_matrix_variable,
    const std::unordered_map<int, int>&
        map_decision_variable_index_to_mosek_nonmatrix_variable,
    MSKtask_t* task, std::vector<bool>* is_new_variable) {
  int num_vars = prog.num_vars();
  std::vector<double> x_lb(num_vars, -std::numeric_limits<double>::infinity());
  std::vector<double> x_ub(num_vars, std::numeric_limits<double>::infinity());
  for (const auto& binding : prog.bounding_box_constraints()) {
    const auto& constraint = binding.evaluator();
    const Eigen::VectorXd& lower_bound = constraint->lower_bound();
    const Eigen::VectorXd& upper_bound = constraint->upper_bound();

    for (int i = 0; i < static_cast<int>(binding.GetNumElements()); ++i) {
      size_t x_idx = prog.FindDecisionVariableIndex(binding.variables()(i));
      x_lb[x_idx] = std::max(x_lb[x_idx], lower_bound[i]);
      x_ub[x_idx] = std::min(x_ub[x_idx], upper_bound[i]);
    }
  }

  auto add_variable_bound_in_mosek = [task](int mosek_var_index, double lower,
                                            double upper) {
    MSKrescodee rescode_bound{MSK_RES_OK};
    if (std::isinf(lower) && std::isinf(upper)) {
      rescode_bound = MSK_putvarbound(*task, mosek_var_index, MSK_BK_FR,
                                      -MSK_INFINITY, MSK_INFINITY);
    } else if (std::isinf(lower) && !std::isinf(upper)) {
      rescode_bound = MSK_putvarbound(*task, mosek_var_index, MSK_BK_UP,
                                      -MSK_INFINITY, upper);
    } else if (!std::isinf(lower) && std::isinf(upper)) {
      rescode_bound = MSK_putvarbound(*task, mosek_var_index, MSK_BK_LO, lower,
                                      MSK_INFINITY);
    }
    else_bound {
      rescode =
          MSK_putvarbound(*task, mosek_var_index, MSK_BK_RA, lower, upper);
    }
    return rescode_bound;
  };

  MSKrescodee rescode = MSK_RES_OK;
  std::vector<int> bounded_matrix_var_indices;
  bounded_matrix_var_indices.reserve(prog.num_vars());
  for (int i = 0; i < num_vars; i++) {
    auto it1 = map_decision_variable_index_to_mosek_nonmatrix_variable.find(i);
    if (it1 != map_decision_variable_index_to_mosek_nonmatrix_variable.end()) {
      // The variable is not a matrix variable in Mosek.
      const int mosek_var_index = it1->second;
      rescode = add_variable_bound_in_mosek(mosek_var_index, x_lb[i], x_ub[i]);
      if (rescode != MSK_RES_OK) {
        return rescode;
      }
    } else {
      const double lower = x_lb[i];
      const double upper = x_ub[i];
      if (!(std::isinf(lower) && std::isinf(upper))) {
        bounded_matrix_var_indices.push_back(i);
      }
    }
  }

  // The bounded variable is a matrix variable in Mosek.
  // 1. Add new non-matrix slack variable s to Mosek.
  // 2. Add the bound on s.
  // 3. Add the constraint s + <A̅ₘₙ, X̅> = 0, where <A̅ₘₙ, X̅> = -X̅(m, n)

  // step 1, add slack variable s.
  int mosek_var_count;
  rescode = MSK_getnumvar(task, &mosek_var_count);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  const int bounded_matrix_var_count =
      static_cast<int>(bounded_matrix_var_indices.size());
  rescode = MSK_appendvars(task, bounded_matrix_var_count);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  const std::vector<bool> bounded_matrix_var_is_new_variable(
      bounded_matrix_var_count, true);
  is_new_variable->insert(is_new_variable->end(),
                          bounded_matrix_var_is_new_variable.begin(),
                          bounded_matrix_var_is_new_variable.end());
  for (int i = 0; i < bounded_matrix_var_count; ++i) {
    // step 2, add bounds on s.
    rescode = add_variable_bound_in_mosek(mosek_var_count + i,
                                          x_lb[bounded_matrix_var_indices[i]],
                                          x_ub[bounded_matrix_var_indices[i]]);
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
  }
  // step 3, add linear constraint s = X̅(m, n)
  int mosek_linear_constraint_count;
  rescode = MSK_getnumcon(task, &mosek_linear_constraint_count);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  rescode = MSK_appendcons(task, bounded_matrix_var_count);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  // A is the matrix that multiplies s in the constraint s + <A̅ₘₙ, X̅> = 0, A * s
  // = s.
  std::vector<MSKint32t> A_row(bounded_matrix_var_count);
  std::vector<MSKint32t> A_col(bounded_matrix_var_count);
  std::vector<MSKrealt> A_val(bounded_matrix_var_count, 1.0);
  for (int i = 0; i < bounded_matrix_var_count; ++i) {
    A_row[i] = mosek_linear_constraint_count + i;
    A_col[i] = mosek_var_count + i;

    // Now construct A̅ₘₙ.
    const MatrixVariableEntry& matrix_variable_entry =
        map_decision_variable_index_to_mosek_matrix_variable
            [bounded_matrix_var_indices[i]];
    int matrix_rows;
    rescode = MSK_getdimbarvarj(task, matrix_variable_entry.bar_matrix_index,
                                &matrix_rows);
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
    const MSKint32t bar_A_mn_row_index = matrix_variable_entry.row_index;
    const MSKint32t bar_A_mn_col_index = matrix_variable_entry.col_index;
    const MSKrealt bar_A_mn_val =
        bar_A_mn_row_index == bar_A_mn_col_index ? -1.0 : -0.5;
    MSKint64t bar_A_mn_index;
    rescode = MSK_appendsparsesymmat(task, matrix_rows, 1, &bar_A_mn_row_index,
                                     &bar_A_mn_col_index, &bar_A_mn_val,
                                     &bar_A_mn_index);
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
    const MSKrealt weights{1.0};
    rescode =
        MSK_putbaraij(task, A_row[i], matrix_variable_entry.bar_matrix_index, 1,
                      &bar_A_mn_index, &weights);
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
  }
  rescode = MSK_putaijlist(task, bounded_matrix_var_count, A_row.data(),
                           A_col.data(), A_val.data());
  if (rescode != MSK_RES_OK) {
    return rescode;
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

    const auto& A = binding.evaluator()->A();
    const auto& b = binding.evaluator()->b();
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
    int matrix_rows = binding.evaluator()->matrix_rows();

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
  // 1. Create the matrix variable X_bar.
  // 2. Add the constraint
  //     x1 * F1(m, n) + ... + xk * Fk(m, n) + <E_mn, X_bar> = -F0(m, n)
  //    where E_mn is a symmetric matrix, the matrix inner product <E_mn, X_bar>
  //    = X_bar(m, n).
  MSKrescodee rescode = MSK_RES_OK;
  for (const auto& binding : prog.linear_matrix_inequality_constraints()) {
    int num_linear_constraint = 0;
    rescode = MSK_getnumcon(*task, &num_linear_constraint);
    if (rescode != MSK_RES_OK) {
      return rescode;
    }

    int rows = binding.evaluator()->matrix_rows();

    AddBarVariable(rows, task);

    int new_linear_constraint_count = 0;
    // It is important to use the same for-loop order as in
    // AddBarVariable().
    for (int j = 0; j < rows; ++j) {
      for (int i = j; i < rows; ++i) {
        int linear_constraint_index =
            num_linear_constraint + new_linear_constraint_count;

        const auto& F = binding.evaluator()->F();
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

MSKrescodee AggregateQuadraticCosts(
    const MathematicalProgram prog,
    const std::unordered_map<int, int>&
        map_decision_variable_index_to_mosek_nonmatrix_variable,
    std::vector<Eigen::Triplet<double>>* Q_lower_triplets,
    std::vector<Eigen::Triplet<double>>* linear_term_triplets,
    double* constant_cost) {
  MSKrescodee rescode = MSK_RES_OK;
  const int xDim =
      map_decision_variable_index_to_mosek_nonmatrix_variable.size();
  for (const auto& binding : prog.quadratic_costs()) {
    const auto& cost = binding.evaluator();
    // The quadratic cost is of form 0.5*x'*Q*x + b*x.
    const auto& Q = cost->Q();
    const auto& b = cost->b();
    constant_cost += cost->c();
    std::vector<int> var_indices(Q.rows());

    for (int i = 0; i < static_cast<int>(binding.GetNumElements()); ++i) {
      const int decision_var_index =
          prog.FindDecisionVariableIndex(binding.variables()(i));
      auto it = map_decision_variable_index_to_mosek_nonmatrix_variable.find(
          decision_var_index);
      if (it == map_decision_variable_index_to_mosek_nonmatrix_variable.end()) {
        throw std::runtime_error(
            "MosekSolver: we currently do not support an optimization variable "
            "with both positive semidefinite constraint and quadratic cost, "
            "with a variable in the psd matrix also having quadratic cost.");
      }

      var_indices[i] = it->second;
    }

    for (int i = 0; i < Q.rows(); ++i) {
      int var_index_i = var_indices[i];
      for (int j = 0; j < i; ++j) {
        const double Qij = (Q(i, j) + Q(j, i)) / 2;
        if (std::abs(Qij) > Eigen::NumTraits<double>::epsilon()) {
          if (var_index_i > var_indices[j]) {
            Q_lower_triplets->push_back(
                Eigen::Triplet<double>(var_index_i, var_indices[j], Qij));
          } else {
            Q_lower_triplets->push_back(
                Eigen::Triplet<double>(var_indices[j], var_index_i, Qij));
          }
        }
      }
      if (std::abs(Q(i, i)) > Eigen::NumTraits<double>::epsilon()) {
        Q_lower_triplets.push_back(
            Eigen::Triplet<double>(var_index_i, var_index_i, Q(i, i)));
      }
      if (std::abs(b(i)) > Eigen::NumTraits<double>::epsilon()) {
        linear_term_triplets->push_back(
            Eigen::Triplet<double>(var_index_i, 0, b(i)));
      }
    }
  }
  return rescode;
}

MSKrescodee AggregateLinearCosts(
    const MathematicalProgram& prog,
    const std::unordered_map<int, MatrixVariableEntry>&
        map_decision_variable_index_to_mosek_matrix_variable,
    const std::unordered_map<int, int>&
        map_decision_variable_index_to_mosek_nonmatrix_variable,
    std::vector<Eigen::Triplet<double>>* linear_term_triplets,
    double* constant_cost,
    std::vector<std::vector<Eigen::Triplet<double>>>* C_bar_lower_triplets) {
  MSKrescodee rescode = MSK_RES_OK;
  const int xDim =
      map_decision_variable_index_to_mosek_nonmatrix_variable.size();
  for (const auto& binding : prog.linear_costs()) {
    const auto& c = binding.evaluator()->a();
    *constant_cost += binding.evaluator()->b();
    for (int i = 0; i < static_cast<int>(binding.GetNumElements()); ++i) {
      if (std::abs(c(i)) > Eigen::NumTraits<double>::epsilon()) {
        const int decision_variable_index =
            prog.FindDecisionVariableIndex(binding.variables()(i));
        auto it1 = map_decision_variable_index_to_mosek_nonmatrix_variable.find(
            decision_variable_index);
        if (it1 !=
            map_decision_variable_index_to_mosek_nonmatrix_variable.end()) {
          linear_term_triplets->push_back(
              Eigen::Triplet<double>(it1->second, 0, c(i)));
        } else {
          auto it2 = map_decision_variable_index_to_mosek_matrix_variable.find(
              decision_variable_index);
          C_bar_lower_triplets[it2->second.bar_matrix_index]->emplace_back(
              it2->second.row_index, it2->second.col_index,
              it2->second.row_index == it2->second.col_index ? c(i) : c(i) / 2);
        }
      }
    }
  }
  return rescode;
}

// Given a vector of triplets (which might contain duplicated entries in the
// matrix), returns the vector of rows, columns and values.
void ConvertTripletsToVectors(
    const std::vector<Eigen::Triplet<double>>& triplets, int matrix_rows,
    int matrix_cols, std::vector<MSKint32t>* row_indices,
    std::vector<MSKint32t>* col_indices, std::vector<MSKrealt>* values) {
  // column major sparse matrix
  Eigen::SparseMatrix<double> A(matrix_rows, matrix_cols);
  A.setFromTriplets(triplets.begin(), triplets.end());
  const int num_nonzeros = A.nonZeros();
  DRAKE_ASSERT(row_indices && row_indices->empty());
  DRAKE_ASSERT(col_indices && col_indices->empty());
  DRAKE_ASSERT(values && values->empty());
  row_indices->reserve(num_nonzeros);
  col_indices->reserve(num_nonzeros);
  values->reserve(num_nonzeros);
  for (int i = 0; i < A.outerSize(); ++i) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(A, i); it; ++it) {
      row_indices->push_back(it.row());
      col_indices->push_back(it.col());
      values->push_back(it.value());
    }
  }
}

MSKrescodee AddCosts(
    const MathematicalProgram& prog,
    const std::unordered_map<int, MatrixVariableEntry>&
        map_decision_variable_index_to_mosek_matrix_variable,
    const std::unordered_map<int, int>&
        map_decision_variable_index_to_mosek_nonmatrix_variable,
    MSKtask_t* task) {
  // Add the cost in the form 0.5 * x' * Q_all * x + linear_terms' * x + ∑ᵢ <C̅ᵢ,
  // X̅ᵢ>, where X̅ᵢ is the i'th matrix variable stored inside Mosek.
  MSKrescodee rescode = MSK_RES_OK;
  const int xDim =
      map_decision_variable_index_to_mosek_nonmatrix_variable.size();
  // Mosek takes the lower triangular part of Q_all. Q_lower_triplets include
  // the triplets (row_index, col_index, val) on the lower triangular part
  // of Q_all.
  std::vector<Eigen::Triplet<double>> Q_lower_triplets;
  std::vector<Eigen::Triplet<double>> linear_term_triplets;
  double constant_cost = 0.;

  int num_bar_var = 0;
  rescode = MSK_getnumbarvar(*task, &num_bar_var);
  // C_bar_lower_triplets[i] stores the triplets for C̅ᵢ.
  std::vector<std::vector<Eigen::Triplet<double>>> C_bar_lower_triplets(
      num_bar_var);

  // Aggregate the quadratic costs.
  rescode = AggregateQuadraticCosts(
      prog, map_decision_variable_index_to_mosek_nonmatrix_variable,
      &Q_lower_triplets, &linear_term_triplets, &constant_cost);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  // Aggregate the linear costs.
  rescode = AggregateLinearCosts(
      prog, map_decision_variable_index_to_mosek_matrix_variable,
      map_decision_variable_index_to_mosek_nonmatrix_variable,
      &linear_term_triplets, &constant_cost, &C_bar_lower_triplets);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }

  // Add the quadratic cost.
  std::vector<MSKint32t> qrow, qcol;
  std::vector<double> qval;
  ConvertTripletsToVectors(Q_lower_triplets, xDim, xDim, &qrow, &qcol, &qval);
  const int Q_nnz = static_cast<int>(qrow.size());
  rescode = MSK_putqobj(*task, Q_nnz, qrow.data(), qcol.data(), qval.data());
  if (rescode != MSK_RES_OK) {
    return rescode;
  }

  // Add the linear cost.
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

  // Add the cost ∑ᵢ <C̅ᵢ, X̅ᵢ>
  for (int i = 0; i < num_bar_var; ++i) {
    if (C_bar_lower_triplets[i].size() > 0) {
      int matrix_rows{0};
      rescode = MSK_getdimbarvarj(*task, i, &matrix_rows);
      if (rescode != MSK_RES_OK) {
        return rescode;
      }
      std::vector<MSKint32t> Ci_bar_lower_rows, Ci_bar_lower_cols;
      std::vector<MSKrealt> Ci_bar_lower_values;
      ConvertTripletsToVectors(C_bar_lower_triplets[i], matrix_rows,
                               matrix_rows, &Ci_bar_lower_rows,
                               &Ci_bar_lower_cols, &Ci_bar_lower_values);
      MSKint64t Ci_bar_index{0};
      // Create the sparse matrix C̅ᵢ.
      rescode = MSK_appendsparsesymmat(
          task, matrix_rows, Ci_bar_lower_rows.size(), Ci_bar_lower_rows.data(),
          Ci_bar_lower_cols.data(), Ci_bar_lower_values.data(), &Ci_bar_index);
      if (rescode != MSK_RES_OK) {
        return rescode;
      }
      // Now add the cost <C̅ᵢ, X̅ᵢ>
      const MSKrealt weight{1.0};
      rescode = MSK_putbarcj(task, i, 1, &Ci_bar_index, weight);
    }
  }

  // Provide constant / fixed cost.
  MSK_putcfix(*task, constant_cost);

  return rescode;
}

MSKrescodee SpecifyVariableType(const MathematicalProgram& prog,
                                MSKtask_t* task,
                                bool* with_integer_or_binary_variable) {
  MSKrescodee rescode = MSK_RES_OK;
  int num_vars = prog.num_vars();
  for (int i = 0; i < num_vars && rescode == MSK_RES_OK; ++i) {
    switch (prog.decision_variable(i).get_type()) {
      case MathematicalProgram::VarType::INTEGER: {
        rescode = MSK_putvartype(*task, i, MSK_VAR_TYPE_INT);
        if (rescode != MSK_RES_OK) {
          return rescode;
        }
        *with_integer_or_binary_variable = true;
      } break;
      case MathematicalProgram::VarType::BINARY: {
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
        break;
      }
      case MathematicalProgram::VarType::CONTINUOUS: {
        // Do nothing.
        break;
      }
      case MathematicalProgram::VarType::BOOLEAN: {
        throw std::runtime_error(
            "Boolean variables should not be used with Mosek solver.");
      }
      case MathematicalProgram::VarType::RANDOM_UNIFORM:
      case MathematicalProgram::VarType::RANDOM_GAUSSIAN:
      case MathematicalProgram::VarType::RANDOM_EXPONENTIAL:
        throw std::runtime_error(
            "Random variables should not be used with Mosek solver.");
    }
  }
  return rescode;
}

// Mosek treats matrix variable (variables in the psd matrix) in a special
// manner, while MathematicalProgram doesn't. Hence we need to pick out the
// variables in prog.positive_semidefinite_constraint(), record how they will be
// stored in Mosek, and also how the remaining non-matrix variable will be
// stored in Mosek. Note that we only loop through
// PositiveSemidefiniteConstraint, not LinearMatrixInequalityConstraint. We
// should parse LinearMatrixInequalityConstraint __after__ calling this
// function.
void MapProgramDecisionVariableToMosekVariable(
    const MathematicalProgram& prog,
    std::unordered_map<int, MatrixVariableEntry>*
        map_decision_variable_index_to_mosek_matrix_variable,
    std::unordered_map<int, int>*
        map_decision_variable_index_to_mosek_nonmatrix_variable) {
  // Each PositiveSemidefiniteConstraint will add one matrix variable to Mosek.
  int psd_constraint_count = 0;
  MatrixVariableEntry matrix_variable_entry;
  for (const auto& psd_constraint : prog.positive_semidefinite_constraints()) {
    // The bounded variables of a psd constraint is the "flat" version of the
    // symmetrix matrix variables, stacked column by column. We only need to
    // store the lower triangular part of this symmetric matrix in Mosek.
    const int matrix_rows = psd_constraint.evaluator()->matrix_rows();
    matrix_variable_entry.bar_matrix_index = psd_constraint_count;
    for (int j = 0; j < matrix_rows; ++j) {
      for (int i = j; i < matrix_rows; ++i) {
        matrix_variable_entry.row_index = i;
        matrix_variable_entry.col_index = j;
        map_decision_variable_index_to_mosek_matrix_variable->emplace_hint(
            map_decision_variable_index_to_mosek_matrix_variable->end(),
            prog.FindDecisionVariableIndex(
                psd_constraint.variables()(j * matrix_rows + i)),
            matrix_variable_entry);
      }
    }
    psd_constraint_count++;
  }
  // All the non-matrix variables in @p prog is stored in another vector inside
  // Mosek.
  int nonmatrix_variable_count = 0;
  for (int i = 0; i < prog.num_vars(); ++i) {
    if (map_decision_variable_index_to_mosek_matrix_variable->count(i) == 0) {
      map_decision_variable_index_to_mosek_nonmatrix_variable->emplace(
          i, nonmatrix_variable_count++);
    }
  }
}
}  // anonymous namespace

/*
 * Implements RAII for a Mosek license / environment.
 */
class MosekSolver::License {
 public:
  License() {
    const char* moseklm_license_file = std::getenv("MOSEKLM_LICENSE_FILE");
    if (moseklm_license_file == nullptr) {
      throw std::runtime_error(
          "Could not locate MOSEK license file because MOSEKLM_LICENSE_FILE "
          "environment variable was not set.");
    }

    MSKrescodee rescode = MSK_makeenv(&mosek_env_, nullptr);
    if (rescode != MSK_RES_OK) {
      throw std::runtime_error("Could not create MOSEK environment.");
    }
    DRAKE_DEMAND(mosek_env_ != nullptr);

    const int num_tries = 3;
    rescode = MSK_RES_TRM_INTERNAL;
    for (int i = 0; i < num_tries && rescode != MSK_RES_OK; ++i) {
      // Acquire the license for the base MOSEK system so that we can
      // fail fast if the license file is missing or the server is
      // unavailable. Any additional features should be checked out
      // later by MSK_optimizetrm if needed (so there's still the
      // possibility of later failure at that stage if the desired
      // feature is unavailable or another error occurs).
      rescode = MSK_checkoutlicense(mosek_env_, MSK_FEATURE_PTS);
    }

    if (rescode != MSK_RES_OK) {
      throw std::runtime_error("Could not acquire MOSEK license.");
    }
  }

  ~License() {
    MSK_deleteenv(&mosek_env_);
    mosek_env_ = nullptr;  // Fail-fast if accidentally used after destruction.
  }

  MSKenv_t mosek_env() const { return mosek_env_; }

 private:
  MSKenv_t mosek_env_{nullptr};
};

std::shared_ptr<MosekSolver::License> MosekSolver::AcquireLicense() {
  // According to
  // http://docs.mosek.com/8.0/cxxfusion/solving-parallel.html sharing
  // an env used between threads is safe, but nothing mentions thread-safety
  // when allocating the environment. We can safeguard against this ambiguity
  // by using GetScopedSingleton for basic thread-safety when acquiring /
  // releasing the license.
  return GetScopedSingleton<MosekSolver::License>();
}

bool MosekSolver::is_available() { return true; }

void MosekSolver::DoSolve(const MathematicalProgram& prog,
                          const Eigen::VectorXd& initial_guess,
                          const SolverOptions& merged_options,
                          MathematicalProgramResult* result) const {
  // num_vars are the total number of decision variables in @p prog. It includes
  // both the matrix variables (for psd matrix variables) and non-matrix
  // variables.
  const int num_vars = prog.num_vars();
  MSKtask_t task = nullptr;
  MSKrescodee rescode;

  // When solving optimization problem with Mosek, we sometimes need to add
  // new variables to Mosek, so that the solver can parse the constraint.
  // is_new_variable has the same length as the number of non-matrix variables
  // in Mosek
  // i.e. the invariant is  MSKint32t num_mosek_vars;
  //                        MSK_getnumvar(task, &num_mosek_vars);
  //                        assert(is_new_variable.length() ==  num_mosek_vars);
  // is_new_variable[i] is true if the variable is not a part of the variable
  // in MathematicalProgram prog, but added to Mosek solver.
  std::vector<bool> is_new_variable(num_vars, false);

  if (!license_) {
    license_ = AcquireLicense();
  }
  MSKenv_t env = license_->mosek_env();

  // Mosek treats matrix variable (variables in a psd matrix) in a special
  // manner, but MathematicalProgram doesn't. Hence we need to pick out the
  // matrix and non-matrix variables in @p prog, and record how they will be
  // stored in Mosek.
  std::unordered_map<int, MatrixVariableEntry>
      map_decision_variable_index_to_mosek_matrix_variable;
  std::unordered_map<int, int>
      map_decision_variable_index_to_mosek_nonmatrix_variable;
  MapProgramDecisionVariableToMosekVariable(
      prog, &map_decision_variable_index_to_mosek_matrix_variable,
      &map_decision_variable_index_to_mosek_nonmatrix_variable);
  DRAKE_ASSERT(
      map_decision_variable_index_to_mosek_matrix_variable.size() +
          map_decision_variable_index_to_mosek_nonmatrix_variable.size() ==
      prog.num_vars());
  // The number of non-matrix variables in @p prog.
  const int num_nonmatrix_vars_in_prog =
      map_decision_variable_index_to_mosek_nonmatrix_variable.size();

  // Create the optimization task.
  rescode = MSK_maketask(env, 0, num_nonmatrix_vars_in_prog, &task);
  // Always check if rescode is MSK_RES_OK before we call any mosek functions.
  // If it is not MSK_RES_OK, then bypasses everything and exits.
  if (rescode == MSK_RES_OK) {
    rescode = MSK_appendvars(task, num_nonmatrix_vars_in_prog);
  }
  // Add costs
  if (rescode == MSK_RES_OK) {
    rescode = AddCosts(prog, &task);
  }
  // Add bounding box constraints on decision variables.
  if (rescode == MSK_RES_OK) {
    rescode = AddBoundingBoxConstraints(prog, &task, &is_new_variable);
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
  if (rescode == MSK_RES_OK && stream_logging_) {
    if (log_file_.empty()) {
      rescode =
          MSK_linkfunctotaskstream(task, MSK_STREAM_LOG, nullptr, printstr);
    } else {
      rescode =
          MSK_linkfiletotaskstream(task, MSK_STREAM_LOG, log_file_.c_str(), 0);
    }
  }

  if (rescode == MSK_RES_OK) {
    for (const auto& double_options : merged_options.GetOptionsDouble(id())) {
      if (rescode == MSK_RES_OK) {
        rescode = MSK_putnadouparam(task, double_options.first.c_str(),
                                    double_options.second);
      }
    }
    for (const auto& int_options : merged_options.GetOptionsInt(id())) {
      if (rescode == MSK_RES_OK) {
        rescode = MSK_putnaintparam(task, int_options.first.c_str(),
                                    int_options.second);
      }
    }
    for (const auto& str_options : merged_options.GetOptionsStr(id())) {
      if (rescode == MSK_RES_OK) {
        rescode = MSK_putnastrparam(task, str_options.first.c_str(),
                                    str_options.second.c_str());
      }
    }
  }

  // Mosek can accept the initial guess on its integer/binary variables, but
  // not on the continuous variables. So it allows some of the variables'
  // initial guess to be unset, while setting the others. If the initial guess
  // for any variable is finite, then we ask Mosek to set the initial guess.
  const bool has_any_finite_initial_guess =
      initial_guess.unaryExpr([](double g) { return std::isfinite(g); }).any();
  if (with_integer_or_binary_variable && has_any_finite_initial_guess) {
    // Set the initial guess for the integer/binary variables.
    DRAKE_ASSERT(initial_guess.size() == prog.num_vars());
    MSKint32t num_mosek_vars{0};
    if (rescode == MSK_RES_OK) {
      // num_mosek_vars is guaranteed to be no less than prog.num_vars(), as we
      // can add slack variables when we construct Mosek constraints. For
      // example, when we call AddSecondOrderConeConstraints().
      rescode = MSK_getnumvar(task, &num_mosek_vars);
      DRAKE_DEMAND(num_mosek_vars >= prog.num_vars());
    }
    if (rescode == MSK_RES_OK) {
      rescode = MSK_putintparam(task, MSK_IPAR_MIO_CONSTRUCT_SOL, MSK_ON);
    }
    int var_count = 0;
    for (int i = 0; i < num_mosek_vars; ++i) {
      if (!is_new_variable[i]) {
        const auto var_type = prog.decision_variable(var_count).get_type();
        if (var_type == MathematicalProgram::VarType::INTEGER ||
            var_type == MathematicalProgram::VarType::BINARY) {
          if (rescode == MSK_RES_OK) {
            const MSKrealt initial_guess_i = initial_guess(var_count);
            rescode =
                MSK_putxxslice(task, MSK_SOL_ITG, i, i + 1, &initial_guess_i);
          }
        }
        var_count++;
      }
    }
    DRAKE_DEMAND(var_count == prog.num_vars());
  }

  result->set_solution_result(SolutionResult::kUnknownError);
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

  // TODO(hongkai.dai@tri.global) : Add MOSEK parameters.
  // Mosek parameter are added by enum, not by string.
  MSKsolstae solution_status{MSK_SOL_STA_UNKNOWN};
  if (rescode == MSK_RES_OK) {
    if (rescode == MSK_RES_OK) {
      rescode = MSK_getsolsta(task, solution_type, &solution_status);
    }
    if (rescode == MSK_RES_OK) {
      switch (solution_status) {
        case MSK_SOL_STA_OPTIMAL:
        case MSK_SOL_STA_NEAR_OPTIMAL:
        case MSK_SOL_STA_INTEGER_OPTIMAL:
        case MSK_SOL_STA_NEAR_INTEGER_OPTIMAL:
        case MSK_SOL_STA_PRIM_FEAS: {
          result->set_solution_result(SolutionResult::kSolutionFound);
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
            result->set_x_val(sol_vector);
          }
          MSKrealt optimal_cost;
          rescode = MSK_getprimalobj(task, solution_type, &optimal_cost);
          DRAKE_ASSERT(rescode == MSK_RES_OK);
          if (rescode == MSK_RES_OK) {
            result->set_optimal_cost(optimal_cost);
          }
          break;
        }
        case MSK_SOL_STA_DUAL_INFEAS_CER:
        case MSK_SOL_STA_NEAR_DUAL_INFEAS_CER:
          result->set_solution_result(SolutionResult::kDualInfeasible);
          break;
        case MSK_SOL_STA_PRIM_INFEAS_CER:
        case MSK_SOL_STA_NEAR_PRIM_INFEAS_CER: {
          result->set_solution_result(SolutionResult::kInfeasibleConstraints);
          break;
        }
        default: {
          result->set_solution_result(SolutionResult::kUnknownError);
          break;
        }
      }
    }
  }

  MosekSolverDetails& solver_details =
      result->SetSolverDetailsType<MosekSolverDetails>();
  solver_details.rescode = rescode;
  solver_details.solution_status = solution_status;
  if (rescode == MSK_RES_OK) {
    rescode = MSK_getdouinf(task, MSK_DINF_OPTIMIZER_TIME,
                            &(solver_details.optimizer_time));
  }
  // rescode is not used after this. If in the future, the user wants to call
  // more MSK functions after this line, then he/she needs to check if rescode
  // is OK. But do not modify result->solution_result_ if rescode is not OK
  // after this line.
  unused(rescode);

  MSK_deletetask(&task);
}

}  // namespace solvers
}  // namespace drake
