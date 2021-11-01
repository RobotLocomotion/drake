#include "drake/solvers/mosek_solver.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <limits>
#include <list>
#include <optional>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <fmt/format.h>
#include <mosek.h>

#include "drake/common/never_destroyed.h"
#include "drake/common/scope_exit.h"
#include "drake/common/scoped_singleton.h"
#include "drake/common/text_logging.h"
#include "drake/math/quadratic_form.h"
#include "drake/solvers/aggregate_costs_constraints.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
namespace {
// Mosek treats psd matrix variables in a special manner.
// Check https://docs.mosek.com/9.2/capi/tutorial-sdo-shared.html for more
// details. To summarize, Mosek stores a positive semidefinite (psd) matrix
// variable as a "bar var" (as called in Mosek's API, for example
// https://docs.mosek.com/9.2/capi/tutorial-sdo-shared.html). Inside Mosek, it
// accesses each of the psd matrix variable with a unique ID. Moreover, the
// Mosek user cannot access the entries of the psd matrix variable individually;
// instead, the user can only access the matrix X̅ as a whole. To impose
// linear constraints on psd matrix variable X̅ entries, the user must specify a
// "coefficient matrix" A̅ that multiplies X̅ (using the matrix inner product) to
// yield the linear constraint
// lower ≤ <A̅, X̅> ≤ upper.
// For example, to impose the constraint X̅(0, 0) + X̅(1, 0) = 1, where X̅ is a
// 2 x 2 psd matrix, Mosek requires the user to write it as
// <A̅, X̅> = 1, where A̅ = ⌈1   0.5⌉
//                       ⌊0.5   0⌋
// On the other hand, drake::solvers::MathematicalProgram doesn't treat psd
// matrix variables in a special manner.
// MatrixVariableEntry stores the data needed to refer to a particular entry of
// a Mosek matrix variable.
class MatrixVariableEntry {
 public:
  typedef size_t Id;
  MatrixVariableEntry(MSKint64t bar_matrix_index, MSKint32t row_index,
                      MSKint32t col_index, int num_matrix_rows)
      : bar_matrix_index_{bar_matrix_index},
        row_index_{row_index},
        col_index_{col_index},
        num_matrix_rows_{num_matrix_rows},
        id_{get_next_id()} {
    // Mosek only stores the lower triangular part of the symmetric matrix.
    DRAKE_ASSERT(row_index_ >= col_index_);
  }
  MSKint64t bar_matrix_index() const { return bar_matrix_index_; }

  MSKint32t row_index() const { return row_index_; }

  MSKint32t col_index() const { return col_index_; }

  int num_matrix_rows() const { return num_matrix_rows_; }

  Id id() const { return id_; }

  // Returns the index of the entry in a vector formed by stacking the lower
  // triangular part of the symmetric matrix column by column.
  int IndexInLowerTrianglePart() const {
    return (2 * num_matrix_rows_ - col_index_ + 1) * col_index_ / 2 +
           row_index_ - col_index_;
  }

 private:
  static size_t get_next_id() {
    static never_destroyed<std::atomic<int>> next_id(0);
    return next_id.access()++;
  }
  MSKint64t bar_matrix_index_;
  MSKint32t row_index_;
  MSKint32t col_index_;
  int num_matrix_rows_;
  Id id_;
};

// Mosek stores dual variable in different categories, called slc, suc, slx, sux
// and snx. Refer to
// https://docs.mosek.com/9.2/capi/alphabetic-functionalities.html#mosek.task.getsolution
// for more information.
enum class DualVarType {
  kLinearConstraint,  ///< Corresponds to Mosek's slc and suc.
  kVariableBound,     ///< Corresponds to Mosek's slx and sux.
  kNonlinearConic,    ///< Corresponds to Mosek's snx.
};

struct ConstraintDualIndex {
  // Type of the dual variable.
  DualVarType type;
  // Index of the dual variable. We will use -1 to indicate that a constraint
  // can never be active.
  int index{};
};

using ConstraintDualIndices = std::vector<ConstraintDualIndex>;

// This function is used to print information for each iteration to the console,
// it will show PRSTATUS, PFEAS, DFEAS, etc. For more information, check out
// https://docs.mosek.com/9.2/capi/solver-io.html. This printstr is copied
// directly from https://docs.mosek.com/9.2/capi/solver-io.html#stream-logging.
void MSKAPI printstr(void*, const char str[]) { printf("%s", str); }

enum class LinearConstraintBoundType {
  kEquality,
  kInequality,
};

// If a matrix variable entry X̅(m, n) appears in a cost or a constraint
// (except the psd constraint), then we need a matrix Eₘₙ stored inside Mosek,
// such that <Eₘₙ, X̅> = X̅(m, n). In this function we add the symmetric matrix
// Eₘₙ into Mosek, and record the index of Eₘₙ in Mosek.
MSKrescodee AddMatrixVariableEntryCoefficientMatrixIfNonExistent(
    const MatrixVariableEntry& matrix_variable_entry,
    std::unordered_map<MatrixVariableEntry::Id, MSKint64t>*
        matrix_variable_entry_to_selection_matrix_id,
    MSKint64t* E_index, MSKtask_t* task) {
  MSKrescodee rescode{MSK_RES_OK};
  auto it = matrix_variable_entry_to_selection_matrix_id->find(
      matrix_variable_entry.id());
  if (it != matrix_variable_entry_to_selection_matrix_id->end()) {
    *E_index = it->second;
  } else {
    const MSKint32t row = matrix_variable_entry.row_index();
    const MSKint32t col = matrix_variable_entry.col_index();
    const MSKrealt val = row == col ? 1.0 : 0.5;
    rescode =
        MSK_appendsparsesymmat(*task, matrix_variable_entry.num_matrix_rows(),
                               1, &row, &col, &val, E_index);
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
    matrix_variable_entry_to_selection_matrix_id->emplace_hint(
        it, matrix_variable_entry.id(), *E_index);
  }
  return rescode;
}

// Add the product c * X̅(i, j) to a constraint.
// This function should be called only if that Mosek matrix variable X̅ appear
// for only once in this constraint. Otherwise we should call
// AddLinearConstraintToMosek, which first collects all the
// entries X̅(i, j) belonging to this matrix variable X̅ in this constraint, and
// then forms a matrix A, such that <A, X̅> contains the weighted sum of all
// entries of X̅ in this constraint.
MSKrescodee AddScalarTimesMatrixVariableEntryToMosek(
    MSKint32t constraint_index,
    const MatrixVariableEntry& matrix_variable_entry, MSKrealt scalar,
    std::unordered_map<MatrixVariableEntry::Id, MSKint64t>*
        matrix_variable_entry_to_selection_matrix_id,
    MSKtask_t* task) {
  MSKrescodee rescode{MSK_RES_OK};
  MSKint64t E_index;
  rescode = AddMatrixVariableEntryCoefficientMatrixIfNonExistent(
      matrix_variable_entry, matrix_variable_entry_to_selection_matrix_id,
      &E_index, task);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  rescode = MSK_putbaraij(*task, constraint_index,
                          matrix_variable_entry.bar_matrix_index(), 1, &E_index,
                          &scalar);
  return rescode;
}

// Determine the sense of each constraint. The sense can be equality constraint,
// less than, greater than, or bounded on both side.
MSKrescodee SetMosekLinearConstraintBound(
    MSKtask_t task, int linear_constraint_index, double lower, double upper,
    LinearConstraintBoundType bound_type) {
  MSKrescodee rescode{MSK_RES_OK};
  switch (bound_type) {
    case LinearConstraintBoundType::kEquality: {
      rescode = MSK_putconbound(task, linear_constraint_index, MSK_BK_FX, lower,
                                lower);
      if (rescode != MSK_RES_OK) {
        return rescode;
      }
      break;
    }
    case LinearConstraintBoundType::kInequality: {
      if (std::isinf(lower) && std::isinf(upper)) {
        DRAKE_DEMAND(lower < 0 && upper > 0);
        rescode = MSK_putconbound(task, linear_constraint_index, MSK_BK_FR,
                                  -MSK_INFINITY, MSK_INFINITY);
      } else if (std::isinf(lower) && !std::isinf(upper)) {
        rescode = MSK_putconbound(task, linear_constraint_index, MSK_BK_UP,
                                  -MSK_INFINITY, upper);
      } else if (!std::isinf(lower) && std::isinf(upper)) {
        rescode = MSK_putconbound(task, linear_constraint_index, MSK_BK_LO,
                                  lower, MSK_INFINITY);
      } else {
        rescode = MSK_putconbound(task, linear_constraint_index, MSK_BK_RA,
                                  lower, upper);
      }
      if (rescode != MSK_RES_OK) {
        return rescode;
      }
      break;
    }
  }
  return rescode;
}

// Add the linear constraint lower <= A * decision_vars + B * slack_vars <=
// upper.
MSKrescodee AddLinearConstraintToMosek(
    const MathematicalProgram& prog, const Eigen::SparseMatrix<double>& A,
    const Eigen::SparseMatrix<double>& B, const Eigen::VectorXd& lower,
    const Eigen::VectorXd& upper,
    const VectorX<symbolic::Variable>& decision_vars,
    const std::vector<MSKint32t>& slack_vars_mosek_indices,
    LinearConstraintBoundType bound_type,
    const std::unordered_map<int, MatrixVariableEntry>&
        decision_variable_index_to_mosek_matrix_variable,
    const std::unordered_map<int, int>&
        decision_variable_index_to_mosek_nonmatrix_variable,
    std::unordered_map<MatrixVariableEntry::Id, MSKint64t>*
        matrix_variable_entry_to_selection_matrix_id,
    MSKtask_t task) {
  MSKrescodee rescode{MSK_RES_OK};
  DRAKE_ASSERT(lower.rows() == upper.rows());
  DRAKE_ASSERT(A.rows() == lower.rows() && A.cols() == decision_vars.rows());
  DRAKE_ASSERT(B.rows() == lower.rows() &&
               B.cols() == static_cast<int>(slack_vars_mosek_indices.size()));
  int num_mosek_constraint{};
  rescode = MSK_getnumcon(task, &num_mosek_constraint);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  rescode = MSK_appendcons(task, lower.rows());
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  for (int i = 0; i < lower.rows(); ++i) {
    // It is important that AddLinearConstraintToMosek keeps the order of the
    // linear constraint the same as in lower <= A * decision_vars + B *
    // slack_vars <= upper. When we specify the dual variable indices, we rely
    // on this order.
    rescode = SetMosekLinearConstraintBound(task, num_mosek_constraint + i,
                                            lower(i), upper(i), bound_type);
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
  }
  // (A * decision_vars + B * slack_var)(i) = (Aₓ * x)(i) + ∑ⱼ <A̅ᵢⱼ, X̅ⱼ>
  // where we decompose [decision_vars; slack_var] to mosek nonmatrix variable
  // x and mosek matrix variable X̅.

  // Ax_subi, Ax_subj, Ax_valij are the triplets format of matrix Aₓ.
  std::vector<MSKint32t> Ax_subi, Ax_subj;
  std::vector<MSKrealt> Ax_valij;
  Ax_subi.reserve(A.nonZeros() + B.nonZeros());
  Ax_subj.reserve(A.nonZeros() + B.nonZeros());
  Ax_valij.reserve(A.nonZeros() + B.nonZeros());

  // mosek_matrix_variable_entries stores all the matrix variables used in this
  // newly added linear constraint.
  // mosek_matrix_variable_entries[j] contains all the entries in that matrix
  // variable X̅ⱼ that show up in this new linear constraint.
  // This map is used to compute the symmetric matrix A̅ᵢⱼ in the term
  // <A̅ᵢⱼ, X̅ⱼ>.
  std::unordered_map<MSKint64t, std::vector<MatrixVariableEntry>>
      mosek_matrix_variable_entries;

  if (A.nonZeros() != 0) {
    std::vector<int> decision_var_indices(decision_vars.rows());
    // decision_vars[mosek_matrix_variable_in_decision_var[i]] is a mosek matrix
    // variable.
    std::vector<int> mosek_matrix_variable_in_decision_var;
    mosek_matrix_variable_in_decision_var.reserve(decision_vars.rows());
    for (int i = 0; i < decision_vars.rows(); ++i) {
      decision_var_indices[i] =
          prog.FindDecisionVariableIndex(decision_vars(i));
      const auto it_mosek_matrix_variable =
          decision_variable_index_to_mosek_matrix_variable.find(
              decision_var_indices[i]);
      if (it_mosek_matrix_variable !=
          decision_variable_index_to_mosek_matrix_variable.end()) {
        // This decision variable is a matrix variable.
        // Denote the matrix variables as X̅. Add the corresponding matrix
        // E̅ₘₙ, such that <E̅ₘₙ, X̅> = X̅(m, n) if such matrix has not been
        // added to Mosek yet.
        mosek_matrix_variable_in_decision_var.push_back(i);
        const MatrixVariableEntry& matrix_variable_entry =
            it_mosek_matrix_variable->second;
        MSKint64t E_mn_index;
        rescode = AddMatrixVariableEntryCoefficientMatrixIfNonExistent(
            matrix_variable_entry, matrix_variable_entry_to_selection_matrix_id,
            &E_mn_index, &task);
        if (rescode != MSK_RES_OK) {
          return rescode;
        }
        mosek_matrix_variable_entries[matrix_variable_entry.bar_matrix_index()]
            .push_back(matrix_variable_entry);
      } else {
        const int mosek_nonmatrix_variable =
            decision_variable_index_to_mosek_nonmatrix_variable.at(
                decision_var_indices[i]);
        // Add the linear term Aₓ * x to the constraint.
        for (Eigen::SparseMatrix<double>::InnerIterator it(A, i); it; ++it) {
          Ax_subi.push_back(num_mosek_constraint + it.row());
          Ax_subj.push_back(mosek_nonmatrix_variable);
          Ax_valij.push_back(it.value());
        }
      }
    }
    if (!mosek_matrix_variable_entries.empty()) {
      // A̅ᵢⱼ is a weighted sum of the symmetrix matrix E stored inside Mosek.
      // bar_A[i][j] stores the indices of E and the weights of E.
      std::vector<std::unordered_map<
          MSKint64t, std::pair<std::vector<MSKint64t>, std::vector<MSKrealt>>>>
          bar_A(lower.rows());
      for (int col_index : mosek_matrix_variable_in_decision_var) {
        const MatrixVariableEntry& matrix_variable_entry =
            decision_variable_index_to_mosek_matrix_variable.at(
                decision_var_indices[col_index]);
        for (Eigen::SparseMatrix<double>::InnerIterator it(A, col_index); it;
             ++it) {
          // If we denote m = matrix_variable_entry.row_index(), n =
          // matrix_variable_entry.col_index(), then
          // bar_A[i][j] = \sum_{col_index} A(i, col_index) * Emn, where j =
          // matrix_variable_entry.bar_matrix_index().
          auto it_bar_A =
              bar_A[it.row()].find(matrix_variable_entry.bar_matrix_index());
          if (it_bar_A != bar_A[it.row()].end()) {
            it_bar_A->second.first.push_back(
                matrix_variable_entry_to_selection_matrix_id->at(
                    matrix_variable_entry.id()));
            it_bar_A->second.second.push_back(it.value());
          } else {
            bar_A[it.row()].emplace_hint(
                it_bar_A, matrix_variable_entry.bar_matrix_index(),
                std::pair<std::vector<MSKint64t>, std::vector<MSKrealt>>(
                    {matrix_variable_entry_to_selection_matrix_id->at(
                        matrix_variable_entry.id())},
                    {it.value()}));
          }
        }
      }
      for (int i = 0; i < lower.rows(); ++i) {
        for (const auto& j_sub_weights : bar_A[i]) {
          // Now compute the matrix A̅ᵢⱼ.
          const MSKint32t j = j_sub_weights.first;
          const std::vector<MSKint64t>& sub = j_sub_weights.second.first;
          const std::vector<MSKrealt>& weights = j_sub_weights.second.second;
          rescode = MSK_putbaraij(task, num_mosek_constraint + i, j, sub.size(),
                                  sub.data(), weights.data());
          if (rescode != MSK_RES_OK) {
            return rescode;
          }
        }
      }
    }
  }
  if (B.nonZeros() != 0) {
    for (int i = 0; i < B.cols(); ++i) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(B, i); it; ++it) {
        Ax_subi.push_back(num_mosek_constraint + it.row());
        Ax_subj.push_back(slack_vars_mosek_indices[i]);
        Ax_valij.push_back(it.value());
      }
    }
  }
  rescode = MSK_putaijlist(task, Ax_subi.size(), Ax_subi.data(), Ax_subj.data(),
                           Ax_valij.data());
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  int num_mosek_constraints_after{};
  rescode = MSK_getnumcon(task, &num_mosek_constraints_after);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  DRAKE_DEMAND(num_mosek_constraints_after ==
               num_mosek_constraint + lower.rows());
  return rescode;
}

// Add LinearConstraints and LinearEqualityConstraints to the Mosek task.
template <typename C>
MSKrescodee AddLinearConstraintsFromBindings(
    MSKtask_t* task, const std::vector<Binding<C>>& constraint_list,
    LinearConstraintBoundType bound_type, const MathematicalProgram& prog,
    const std::unordered_map<int, MatrixVariableEntry>&
        decision_variable_index_to_mosek_matrix_variable,
    const std::unordered_map<int, int>&
        decision_variable_index_to_mosek_nonmatrix_variable,
    std::unordered_map<MatrixVariableEntry::Id, MSKint64t>*
        matrix_variable_entry_to_selection_matrix_id,
    std::unordered_map<Binding<C>, ConstraintDualIndices>* dual_indices) {
  MSKrescodee rescode{MSK_RES_OK};
  for (const auto& binding : constraint_list) {
    const auto& constraint = binding.evaluator();
    const Eigen::MatrixXd& A = constraint->A();
    const Eigen::VectorXd& lb = constraint->lower_bound();
    const Eigen::VectorXd& ub = constraint->upper_bound();
    Eigen::SparseMatrix<double> B_zero(A.rows(), 0);
    B_zero.setZero();
    int num_linear_constraints{-1};
    rescode = MSK_getnumcon(*task, &num_linear_constraints);
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
    rescode = AddLinearConstraintToMosek(
        prog, A.sparseView(), B_zero, lb, ub, binding.variables(), {},
        bound_type, decision_variable_index_to_mosek_matrix_variable,
        decision_variable_index_to_mosek_nonmatrix_variable,
        matrix_variable_entry_to_selection_matrix_id, *task);
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
    ConstraintDualIndices constraint_dual_indices(lb.rows());
    for (int i = 0; i < lb.rows(); ++i) {
      constraint_dual_indices[i].type = DualVarType::kLinearConstraint;
      constraint_dual_indices[i].index = num_linear_constraints + i;
    }
    dual_indices->emplace(binding, constraint_dual_indices);
  }
  return rescode;
}

MSKrescodee AddLinearConstraints(
    const MathematicalProgram& prog,
    const std::unordered_map<int, MatrixVariableEntry>&
        decision_variable_to_mosek_matrix_variable,
    const std::unordered_map<int, int>&
        decision_variable_to_mosek_nonmatrix_variable,
    std::unordered_map<MatrixVariableEntry::Id, MSKint64t>*
        matrix_variable_entry_to_selection_matrix_id,
    std::unordered_map<Binding<LinearConstraint>, ConstraintDualIndices>*
        linear_con_dual_indices,
    std::unordered_map<Binding<LinearEqualityConstraint>,
                       ConstraintDualIndices>* lin_eq_con_dual_indices,
    MSKtask_t* task) {
  MSKrescodee rescode = AddLinearConstraintsFromBindings(
      task, prog.linear_equality_constraints(),
      LinearConstraintBoundType::kEquality, prog,
      decision_variable_to_mosek_matrix_variable,
      decision_variable_to_mosek_nonmatrix_variable,
      matrix_variable_entry_to_selection_matrix_id, lin_eq_con_dual_indices);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  rescode = AddLinearConstraintsFromBindings(
      task, prog.linear_constraints(), LinearConstraintBoundType::kInequality,
      prog, decision_variable_to_mosek_matrix_variable,
      decision_variable_to_mosek_nonmatrix_variable,
      matrix_variable_entry_to_selection_matrix_id, linear_con_dual_indices);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }

  return rescode;
}

// Add the bounds on the decision variables in @p prog. Note that if a decision
// variable in positive definite matrix has a bound, we need to add new linear
// constraint to mosek to bound that variable.
MSKrescodee AddBoundingBoxConstraints(
    const MathematicalProgram& prog,
    const std::unordered_map<int, MatrixVariableEntry>&
        decision_variable_index_to_mosek_matrix_variable,
    const std::unordered_map<int, int>&
        decision_variable_index_to_mosek_nonmatrix_variable,
    std::unordered_map<MatrixVariableEntry::Id, MSKint64t>*
        matrix_variable_entry_to_selection_matrix_id,
    std::unordered_map<Binding<BoundingBoxConstraint>,
                       std::pair<ConstraintDualIndices, ConstraintDualIndices>>*
        dual_indices,
    MSKtask_t* task) {
  int num_decision_vars = prog.num_vars();
  std::vector<double> x_lb(num_decision_vars,
                           -std::numeric_limits<double>::infinity());
  std::vector<double> x_ub(num_decision_vars,
                           std::numeric_limits<double>::infinity());
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
    } else {
      rescode_bound =
          MSK_putvarbound(*task, mosek_var_index, MSK_BK_RA, lower, upper);
    }
    return rescode_bound;
  };

  MSKrescodee rescode = MSK_RES_OK;
  // bounded_matrix_var_indices[i] is the index of the variable
  // bounded_matrix_vars[i] in prog.
  std::vector<int> bounded_matrix_var_indices;
  bounded_matrix_var_indices.reserve(prog.num_vars());
  for (int i = 0; i < num_decision_vars; i++) {
    auto it1 = decision_variable_index_to_mosek_nonmatrix_variable.find(i);
    if (it1 != decision_variable_index_to_mosek_nonmatrix_variable.end()) {
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
  // Add the constraint lower ≤ <A̅ₘₙ, X̅> ≤ upper, where <A̅ₘₙ, X̅> = X̅(m, n)
  const int bounded_matrix_var_count =
      static_cast<int>(bounded_matrix_var_indices.size());
  Eigen::VectorXd bounded_matrix_vars_lower(bounded_matrix_var_count);
  Eigen::VectorXd bounded_matrix_vars_upper(bounded_matrix_var_count);
  VectorX<symbolic::Variable> bounded_matrix_vars(bounded_matrix_var_count);
  // var_in_bounded_matrix_vars[var.get_id()] is the index of a variable var in
  // bounded_matrix_vars, namely
  // bounded_matrix_vars[var_in_bounded_matrix_vars[var.get_id()] = var
  std::unordered_map<symbolic::Variable::Id, int> var_in_bounded_matrix_vars;
  var_in_bounded_matrix_vars.reserve(bounded_matrix_var_count);
  for (int i = 0; i < bounded_matrix_var_count; ++i) {
    bounded_matrix_vars_lower(i) = x_lb[bounded_matrix_var_indices[i]];
    bounded_matrix_vars_upper(i) = x_ub[bounded_matrix_var_indices[i]];
    bounded_matrix_vars(i) =
        prog.decision_variable(bounded_matrix_var_indices[i]);
    var_in_bounded_matrix_vars.emplace(
        prog.decision_variable(bounded_matrix_var_indices[i]).get_id(), i);
  }

  Eigen::SparseMatrix<double> A_eye(bounded_matrix_var_count,
                                    bounded_matrix_var_count);
  A_eye.setIdentity();
  Eigen::SparseMatrix<double> B_zero(bounded_matrix_var_count, 0);
  B_zero.setZero();

  // Make sure after calling AddLinearConstraintToMosek, the number of newly
  // added linear constraints is bounded_matrix_var_count. This is important
  // because we determine the index of the dual variable for bounds on matrix
  // variables based on the order of adding linear constraints in
  // AddLinearConstraintToMosek.
  int num_linear_constraints_before = 0;
  rescode = MSK_getnumcon(*task, &num_linear_constraints_before);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  rescode = AddLinearConstraintToMosek(
      prog, A_eye, B_zero, bounded_matrix_vars_lower, bounded_matrix_vars_upper,
      bounded_matrix_vars, {}, LinearConstraintBoundType::kInequality,
      decision_variable_index_to_mosek_matrix_variable,
      decision_variable_index_to_mosek_nonmatrix_variable,
      matrix_variable_entry_to_selection_matrix_id, *task);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  int num_linear_constraints_after = 0;
  rescode = MSK_getnumcon(*task, &num_linear_constraints_after);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  DRAKE_DEMAND(num_linear_constraints_after ==
               num_linear_constraints_before + bounded_matrix_var_count);
  // Add dual variables.
  for (const auto& binding : prog.bounding_box_constraints()) {
    ConstraintDualIndices lower_bound_duals(binding.variables().rows());
    ConstraintDualIndices upper_bound_duals(binding.variables().rows());
    for (int i = 0; i < binding.variables().rows(); ++i) {
      const int var_index =
          prog.FindDecisionVariableIndex(binding.variables()(i));
      auto it1 =
          decision_variable_index_to_mosek_nonmatrix_variable.find(var_index);
      int dual_variable_index_if_active = -1;
      if (it1 != decision_variable_index_to_mosek_nonmatrix_variable.end()) {
        // Add the dual variables for the constraint registered as bounds on
        // Mosek non-matrix variables.
        lower_bound_duals[i].type = DualVarType::kVariableBound;
        upper_bound_duals[i].type = DualVarType::kVariableBound;
        dual_variable_index_if_active = it1->second;
      } else {
        // This variable is a Mosek matrix variable. The bound on this variable
        // is imposed as a linear constraint.
        DRAKE_DEMAND(decision_variable_index_to_mosek_matrix_variable.count(
                         var_index) > 0);
        lower_bound_duals[i].type = DualVarType::kLinearConstraint;
        upper_bound_duals[i].type = DualVarType::kLinearConstraint;
        const int linear_constraint_index =
            num_linear_constraints_before +
            var_in_bounded_matrix_vars.at(binding.variables()(i).get_id());
        dual_variable_index_if_active = linear_constraint_index;
      }
      if (binding.evaluator()->lower_bound()(i) == x_lb[var_index]) {
        // The lower bound can be active.
        lower_bound_duals[i].index = dual_variable_index_if_active;
      } else {
        // The lower bound can't be active.
        lower_bound_duals[i].index = -1;
      }
      if (binding.evaluator()->upper_bound()(i) == x_ub[var_index]) {
        // The upper bound can be active.
        upper_bound_duals[i].index = dual_variable_index_if_active;
      } else {
        // The upper bound can't be active.
        upper_bound_duals[i].index = -1;
      }
    }
    dual_indices->try_emplace(binding, lower_bound_duals, upper_bound_duals);
  }
  return rescode;
}

/*
 * This is the helper function to add three types of conic constraints
 * 1. A Lorentz cone constraint:
 *    z = A*x+b
 *    z0 >= sqrt(z1^2 + .. zN^2)
 * 2. A rotated Lorentz cone constraint:
 *    z = A*x+b
 *    z0*z1 >= z2^2 + .. + zN^2,
 *    z0 >= 0, z1 >=0
 * 3. An exonential cone constraint:
 *    z = A*x+b
 *    z0 >= z1 * exp(z2 / z1)
 * Mosek does not allow two cones to share variables. To overcome this,
 * we will add a new set of variable (z0, ..., zN)
 */
template <typename C>
MSKrescodee AddConeConstraints(
    const MathematicalProgram& prog,
    const std::vector<Binding<C>>& cone_constraints,
    const std::unordered_map<int, MatrixVariableEntry>&
        decision_variable_index_to_mosek_matrix_variable,
    const std::unordered_map<int, int>&
        decision_variable_index_to_mosek_nonmatrix_variable,
    std::unordered_map<MatrixVariableEntry::Id, MSKint64t>*
        matrix_variable_entry_to_selection_matrix_id,
    std::unordered_map<Binding<C>, ConstraintDualIndices>* dual_indices,
    MSKtask_t* task) {
  static_assert(std::is_same_v<C, LorentzConeConstraint> ||
                    std::is_same_v<C, RotatedLorentzConeConstraint> ||
                    std::is_same_v<C, ExponentialConeConstraint>,
                "Should be either Lorentz cone constraint, rotated Lorentz "
                "cone or exponential cone constraint");
  const bool is_rotated_cone = std::is_same_v<C, RotatedLorentzConeConstraint>;
  MSKrescodee rescode = MSK_RES_OK;
  for (auto const& binding : cone_constraints) {
    const auto& A = binding.evaluator()->A();
    const auto& b = binding.evaluator()->b();
    const int num_z = A.rows();
    MSKint32t num_mosek_vars = 0;
    rescode = MSK_getnumvar(*task, &num_mosek_vars);
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
    rescode = MSK_appendvars(*task, num_z);
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
    std::vector<MSKint32t> new_var_indices(num_z);
    for (int i = 0; i < num_z; ++i) {
      new_var_indices[i] = num_mosek_vars + i;
      rescode = MSK_putvarbound(*task, new_var_indices[i], MSK_BK_FR,
                                -MSK_INFINITY, MSK_INFINITY);
      if (rescode != MSK_RES_OK) {
        return rescode;
      }
    }
    MSKconetypee cone_type;
    if (std::is_same_v<C, LorentzConeConstraint>) {
      cone_type = MSK_CT_QUAD;
    } else if (std::is_same_v<C, RotatedLorentzConeConstraint>) {
      cone_type = MSK_CT_RQUAD;
    } else if (std::is_same_v<C, ExponentialConeConstraint>) {
      cone_type = MSK_CT_PEXP;
    } else {
      DRAKE_UNREACHABLE();
    }
    // The dual cone has the same size as the nonlinear cones (Lorentz cone,
    // rotated Lorentz cone, exponential cone, etc).
    ConstraintDualIndices duals(num_z);
    for (int i = 0; i < num_z; ++i) {
      duals[i].type = DualVarType::kNonlinearConic;
      duals[i].index = num_mosek_vars + i;
    }
    dual_indices->emplace(binding, duals);
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
    // If using Lorentz cone or exponential cone,
    // Add the linear constraint
    //   z0 = a0^T * x + b0;
    // If using rotated Lorentz cone, add the linear constraint
    // 2*z0 = a0^T * x + b0
    const Eigen::SparseMatrix<double> A_sparse = -A;
    Eigen::SparseMatrix<double> B_sparse(num_z, num_z);
    B_sparse.setIdentity();
    B_sparse.coeffRef(0, 0) = is_rotated_cone ? 2.0 : 1.0;
    std::vector<MSKint32t> z_mosek_indices(num_z);
    for (int i = 0; i < num_z; ++i) {
      z_mosek_indices[i] = num_mosek_vars + i;
    }
    rescode = AddLinearConstraintToMosek(
        prog, A_sparse, B_sparse, b, b, binding.variables(), z_mosek_indices,
        LinearConstraintBoundType::kEquality,
        decision_variable_index_to_mosek_matrix_variable,
        decision_variable_index_to_mosek_nonmatrix_variable,
        matrix_variable_entry_to_selection_matrix_id, *task);
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
  }
  // Expect rescode == MSK_RES_OK.
  return rescode;
}

MSKrescodee AddPositiveSemidefiniteConstraints(const MathematicalProgram& prog,
                                               MSKtask_t* task) {
  MSKrescodee rescode = MSK_RES_OK;
  std::vector<MSKint32t> bar_var_dimension;
  bar_var_dimension.reserve(prog.positive_semidefinite_constraints().size());
  for (const auto& binding : prog.positive_semidefinite_constraints()) {
    bar_var_dimension.push_back(binding.evaluator()->matrix_rows());
  }

  rescode = MSK_appendbarvars(*task, bar_var_dimension.size(),
                              bar_var_dimension.data());
  return rescode;
}

MSKrescodee AddLinearMatrixInequalityConstraint(
    const MathematicalProgram& prog,
    const std::unordered_map<int, MatrixVariableEntry>&
        decision_variable_index_to_mosek_matrix_variable,
    const std::unordered_map<int, int>&
        decision_variable_index_to_mosek_nonmatrix_variable,
    std::unordered_map<MatrixVariableEntry::Id, MSKint64t>*
        matrix_variable_entry_to_selection_matrix_id,
    MSKtask_t* task) {
  // 1. Create the matrix variable X_bar.
  // 2. Add the constraint
  //     x1 * F1(m, n) + ... + xk * Fk(m, n) + <E_mn, X_bar> = -F0(m, n)
  //    where E_mn is a symmetric matrix, the matrix inner product <E_mn, X_bar>
  //    = -X_bar(m, n).
  //    This linear constraint can be written as [F1_lower F2_lower ...
  //    Fk_lower] * [x1; ...; xk] - X_bar_lower = -F0_lower
  MSKrescodee rescode = MSK_RES_OK;
  for (const auto& binding : prog.linear_matrix_inequality_constraints()) {
    int num_linear_constraint = 0;
    rescode = MSK_getnumcon(*task, &num_linear_constraint);
    if (rescode != MSK_RES_OK) {
      return rescode;
    }

    const int rows = binding.evaluator()->matrix_rows();
    rescode = MSK_appendbarvars(*task, 1, &rows);
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
    MSKint32t bar_X_index;
    rescode = MSK_getnumbarvar(*task, &bar_X_index);
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
    bar_X_index -= 1;
    // Now form the matrix F_lower = [F1_lower F2_lower ... Fk_lower]
    std::vector<Eigen::Triplet<double>> F_lower_triplets;
    const int num_lower_entries = rows * (rows + 1) / 2;
    F_lower_triplets.reserve(
        num_lower_entries * static_cast<int>(binding.evaluator()->F().size()) -
        1);
    int lower_index = 0;
    for (int k = 1; k < static_cast<int>(binding.evaluator()->F().size());
         ++k) {
      lower_index = 0;
      for (int j = 0; j < rows; ++j) {
        for (int i = j; i < rows; ++i) {
          if (std::abs(binding.evaluator()->F()[k](i, j)) >
              Eigen::NumTraits<double>::epsilon()) {
            F_lower_triplets.emplace_back(lower_index, k - 1,
                                          binding.evaluator()->F()[k](i, j));
          }
          lower_index++;
        }
      }
    }
    Eigen::SparseMatrix<double> F_lower(num_lower_entries,
                                        binding.variables().rows());
    F_lower.setFromTriplets(F_lower_triplets.begin(), F_lower_triplets.end());

    Eigen::VectorXd bound(num_lower_entries);
    lower_index = 0;
    for (int j = 0; j < rows; ++j) {
      for (int i = j; i < rows; ++i) {
        bound(lower_index++) = -binding.evaluator()->F()[0](i, j);
      }
    }

    rescode = AddLinearConstraintToMosek(
        prog, F_lower, Eigen::SparseMatrix<double>(num_lower_entries, 0), bound,
        bound, binding.variables(), {}, LinearConstraintBoundType::kEquality,
        decision_variable_index_to_mosek_matrix_variable,
        decision_variable_index_to_mosek_nonmatrix_variable,
        matrix_variable_entry_to_selection_matrix_id, *task);
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
    // Now add <Eₘₙ, X̅> = -X̅(m,n) to the linear constraint
    lower_index = 0;
    for (int j = 0; j < rows; ++j) {
      for (int i = j; i < rows; ++i) {
        const MSKrealt val = i == j ? -1.0 : -0.5;
        MSKint64t E_index;
        rescode =
            MSK_appendsparsesymmat(*task, rows, 1, &i, &j, &val, &E_index);
        if (rescode != MSK_RES_OK) {
          return rescode;
        }
        const MSKrealt weights{1.0};
        rescode = MSK_putbaraij(*task, num_linear_constraint + lower_index,
                                bar_X_index, 1, &E_index, &weights);
        if (rescode != MSK_RES_OK) {
          return rescode;
        }
        lower_index++;
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

MSKrescodee AddLinearCost(
    const Eigen::SparseVector<double>& linear_coeff,
    const VectorX<symbolic::Variable>& linear_vars,
    const MathematicalProgram& prog,
    const std::unordered_map<int, MatrixVariableEntry>&
        decision_variable_index_to_mosek_matrix_variable,
    const std::unordered_map<int, int>&
        decision_variable_index_to_mosek_nonmatrix_variable,
    MSKtask_t* task) {
  MSKrescodee rescode = MSK_RES_OK;
  // The cost linear_coeff' * linear_vars could be written as
  // c' * x + ∑ᵢ <C̅ᵢ, X̅ᵢ>, where X̅ᵢ is the i'th matrix variable stored
  // inside Mosek, x are the non-matrix variables in Mosek. Mosek API
  // requires adding the cost for matrix and non-matrix variables separately.
  int num_bar_var = 0;
  rescode = MSK_getnumbarvar(*task, &num_bar_var);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  std::vector<std::vector<Eigen::Triplet<double>>> C_bar_lower_triplets(
      num_bar_var);
  for (Eigen::SparseVector<double>::InnerIterator it(linear_coeff); it; ++it) {
    const symbolic::Variable& var = linear_vars(it.row());
    const int var_index = prog.FindDecisionVariableIndex(var);
    auto it1 =
        decision_variable_index_to_mosek_nonmatrix_variable.find(var_index);
    if (it1 != decision_variable_index_to_mosek_nonmatrix_variable.end()) {
      // This variable is a non-matrix variable.
      rescode =
          MSK_putcj(*task, static_cast<MSKint32t>(it1->second), it.value());
      if (rescode != MSK_RES_OK) {
        return rescode;
      }
    } else {
      // Aggregate the matrix C̅ᵢ
      auto it2 =
          decision_variable_index_to_mosek_matrix_variable.find(var_index);
      DRAKE_DEMAND(it2 !=
                   decision_variable_index_to_mosek_matrix_variable.end());
      C_bar_lower_triplets[it2->second.bar_matrix_index()].emplace_back(
          it2->second.row_index(), it2->second.col_index(),
          it2->second.row_index() == it2->second.col_index() ? it.value()
                                                             : it.value() / 2);
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
          *task, matrix_rows, Ci_bar_lower_rows.size(),
          Ci_bar_lower_rows.data(), Ci_bar_lower_cols.data(),
          Ci_bar_lower_values.data(), &Ci_bar_index);
      if (rescode != MSK_RES_OK) {
        return rescode;
      }
      // Now add the cost <C̅ᵢ, X̅ᵢ>
      const MSKrealt weight{1.0};
      rescode = MSK_putbarcj(*task, i, 1, &Ci_bar_index, &weight);
    }
  }
  return rescode;
}

MSKrescodee AddQuadraticCostAsLinearCost(
    const Eigen::SparseMatrix<double>& Q_lower,
    const VectorX<symbolic::Variable>& quadratic_vars,
    const MathematicalProgram& prog,
    const std::unordered_map<int, MatrixVariableEntry>&
        decision_variable_index_to_mosek_matrix_variable,
    const std::unordered_map<int, int>&
        decision_variable_index_to_mosek_nonmatrix_variable,
    std::unordered_map<MatrixVariableEntry::Id, MSKint64t>*
        matrix_variable_entry_to_selection_matrix_id,
    MSKtask_t* task) {
  // We can add the quaratic cost 0.5 * quadratic_vars' * Q * quadratic_vars as
  // a linear cost with a rotated Lorentz cone constraint. To do so, we
  // introduce a slack variable s, with the rotated Lorentz cone constraint
  // 2s >= | L * quadratic_vars|²,
  // where L'L = Q. We then minimize s. In Mosek, we
  // add new variables z, such that
  // z[0] = 1
  // z[1] = s
  // z[2:] = L * quadratic_vars
  // And z is in the rotated Lorentz cone.
  MSKrescodee rescode = MSK_RES_OK;
  // Unfortunately the sparse Cholesky decomposition in Eigen requires GPL
  // license, which is incompatible with Drake's license, so we convert the
  // sparse Q_lower matrix to a dense matrix, and use the dense Cholesky
  // decomposition instead.
  const Eigen::MatrixXd L = math::DecomposePSDmatrixIntoXtransposeTimesX(
      Eigen::MatrixXd(Q_lower), std::numeric_limits<double>::epsilon());
  MSKint32t num_mosek_vars = 0;
  rescode = MSK_getnumvar(*task, &num_mosek_vars);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  const int num_z = L.rows() + 2;
  rescode = MSK_appendvars(*task, num_z);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  std::vector<MSKint32t> new_var_indices(num_z);
  for (int i = 0; i < num_z; ++i) {
    new_var_indices[i] = num_mosek_vars + i;
    if (i >= 1) {
      rescode = MSK_putvarbound(*task, new_var_indices[i], MSK_BK_FR,
                                -MSK_INFINITY, MSK_INFINITY);
      if (rescode != MSK_RES_OK) {
        return rescode;
      }
    }
  }
  // Add the constraint z(0) = 1
  rescode = MSK_putvarbound(*task, new_var_indices[0], MSK_BK_FX, 1., 1.);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  // Add the rotated Lorentz cone constraint.
  rescode =
      MSK_appendcone(*task, MSK_CT_RQUAD, 0.0, num_z, new_var_indices.data());
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  // Now add the linear constraint
  // z[2:] = L * quadratic_vars
  std::vector<MSKint32t> z2_mosek_indices(L.rows());
  for (int i = 0; i < L.rows(); ++i) {
    z2_mosek_indices[i] = new_var_indices[i + 2];
  }
  Eigen::SparseMatrix<double> B_sparse(L.rows(), L.rows());
  B_sparse.setIdentity();
  rescode = AddLinearConstraintToMosek(
      prog, -L.sparseView(), B_sparse, Eigen::VectorXd::Zero(L.rows()),
      Eigen::VectorXd::Zero(L.rows()), quadratic_vars, z2_mosek_indices,
      LinearConstraintBoundType::kEquality,
      decision_variable_index_to_mosek_matrix_variable,
      decision_variable_index_to_mosek_nonmatrix_variable,
      matrix_variable_entry_to_selection_matrix_id, *task);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }

  // Now add the linear cost on s = z[1]
  rescode = MSK_putcj(*task, new_var_indices[1], 1.);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  return rescode;
}

MSKrescodee AddQuadraticCost(
    const Eigen::SparseMatrix<double>& Q_quadratic_vars,
    const VectorX<symbolic::Variable>& quadratic_vars,
    const MathematicalProgram& prog,
    const std::unordered_map<int, int>&
        decision_variable_index_to_mosek_nonmatrix_variable,
    MSKtask_t* task) {
  MSKrescodee rescode = MSK_RES_OK;
  // Add the quadratic cost 0.5 * quadratic_vars' * Q_quadratic_vars *
  // quadratic_vars to Mosek. Q_lower_triplets correspond to the matrix that
  // multiplies with all the non-matrix decision variables in Mosek, not with
  // `quadratic_vars`. We need to map each variable in `quadratic_vars` to its
  // index in Mosek non-matrix variables.
  std::vector<int> var_indices(quadratic_vars.rows());
  for (int i = 0; i < quadratic_vars.rows(); ++i) {
    var_indices[i] = decision_variable_index_to_mosek_nonmatrix_variable.at(
        prog.FindDecisionVariableIndex(quadratic_vars(i)));
  }
  std::vector<Eigen::Triplet<double>> Q_lower_triplets;
  for (int j = 0; j < Q_quadratic_vars.outerSize(); ++j) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(Q_quadratic_vars, j); it;
         ++it) {
      Q_lower_triplets.emplace_back(var_indices[it.row()],
                                    var_indices[it.col()], it.value());
    }
  }
  std::vector<MSKint32t> qrow, qcol;
  std::vector<double> qval;
  const int xDim = decision_variable_index_to_mosek_nonmatrix_variable.size();
  ConvertTripletsToVectors(Q_lower_triplets, xDim, xDim, &qrow, &qcol, &qval);
  const int Q_nnz = static_cast<int>(qrow.size());
  rescode = MSK_putqobj(*task, Q_nnz, qrow.data(), qcol.data(), qval.data());
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  return rescode;
}

MSKrescodee AddCosts(const MathematicalProgram& prog,
                     const std::unordered_map<int, MatrixVariableEntry>&
                         decision_variable_index_to_mosek_matrix_variable,
                     const std::unordered_map<int, int>&
                         decision_variable_index_to_mosek_nonmatrix_variable,
                     std::unordered_map<MatrixVariableEntry::Id, MSKint64t>*
                         matrix_variable_entry_to_selection_matrix_id,
                     MSKtask_t* task) {
  // Add the cost in the form 0.5 * x' * Q_all * x + linear_coeff' * x + ∑ᵢ <C̅ᵢ,
  // X̅ᵢ>, where X̅ᵢ is the i'th matrix variable stored inside Mosek.

  // First aggregate all the linear and quadratic costs.
  Eigen::SparseMatrix<double> Q_lower;
  VectorX<symbolic::Variable> quadratic_vars;
  Eigen::SparseVector<double> linear_coeff;
  VectorX<symbolic::Variable> linear_vars;
  double constant_cost;
  AggregateQuadraticAndLinearCosts(prog.quadratic_costs(), prog.linear_costs(),
                                   &Q_lower, &quadratic_vars, &linear_coeff,
                                   &linear_vars, &constant_cost);

  MSKrescodee rescode = MSK_RES_OK;
  // Add linear cost.
  rescode =
      AddLinearCost(linear_coeff, linear_vars, prog,
                    decision_variable_index_to_mosek_matrix_variable,
                    decision_variable_index_to_mosek_nonmatrix_variable, task);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  if (!prog.quadratic_costs().empty()) {
    if (prog.lorentz_cone_constraints().empty() &&
        prog.rotated_lorentz_cone_constraints().empty() &&
        prog.linear_matrix_inequality_constraints().empty() &&
        prog.positive_semidefinite_constraints().empty() &&
        prog.exponential_cone_constraints().empty()) {
      rescode = AddQuadraticCost(
          Q_lower, quadratic_vars, prog,
          decision_variable_index_to_mosek_nonmatrix_variable, task);

      if (rescode != MSK_RES_OK) {
        return rescode;
      }
    } else {
      rescode = AddQuadraticCostAsLinearCost(
          Q_lower, quadratic_vars, prog,
          decision_variable_index_to_mosek_matrix_variable,
          decision_variable_index_to_mosek_nonmatrix_variable,
          matrix_variable_entry_to_selection_matrix_id, task);
      if (rescode != MSK_RES_OK) {
        return rescode;
      }
    }
  }

  // Provide constant / fixed cost.
  MSK_putcfix(*task, constant_cost);

  return rescode;
}

MSKrescodee SpecifyVariableType(
    const MathematicalProgram& prog,
    const std::unordered_map<int, MatrixVariableEntry>&
        decision_variable_index_to_mosek_matrix_variable,
    const std::unordered_map<int, int>&
        decision_variable_index_to_mosek_nonmatrix_variable,
    MSKtask_t* task, bool* with_integer_or_binary_variable) {
  MSKrescodee rescode = MSK_RES_OK;
  for (const auto& decision_variable_mosek_variable :
       decision_variable_index_to_mosek_nonmatrix_variable) {
    const int mosek_variable_index = decision_variable_mosek_variable.second;
    switch (prog.decision_variable(decision_variable_mosek_variable.first)
                .get_type()) {
      case MathematicalProgram::VarType::INTEGER: {
        rescode = MSK_putvartype(*task, mosek_variable_index, MSK_VAR_TYPE_INT);
        if (rescode != MSK_RES_OK) {
          return rescode;
        }
        *with_integer_or_binary_variable = true;
        break;
      }
      case MathematicalProgram::VarType::BINARY: {
        *with_integer_or_binary_variable = true;
        rescode = MSK_putvartype(*task, mosek_variable_index, MSK_VAR_TYPE_INT);
        if (rescode != MSK_RES_OK) {
          return rescode;
        }
        double xi_lb = NAN;
        double xi_ub = NAN;
        MSKboundkeye bound_key;
        rescode = MSK_getvarbound(*task, mosek_variable_index, &bound_key,
                                  &xi_lb, &xi_ub);
        if (rescode != MSK_RES_OK) {
          return rescode;
        }
        xi_lb = std::max(0.0, xi_lb);
        xi_ub = std::min(1.0, xi_ub);
        rescode = MSK_putvarbound(*task, mosek_variable_index, MSK_BK_RA, xi_lb,
                                  xi_ub);
        if (rescode != MSK_RES_OK) {
          return rescode;
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
  for (const auto& decision_variable_mosek_matrix_variable :
       decision_variable_index_to_mosek_matrix_variable) {
    const auto& decision_variable =
        prog.decision_variable(decision_variable_mosek_matrix_variable.first);
    if (decision_variable.get_type() !=
        MathematicalProgram::VarType::CONTINUOUS) {
      throw std::invalid_argument("The variable " +
                                  decision_variable.get_name() +
                                  "is a positive semidefinite matrix variable, "
                                  "but it doesn't have continuous type.");
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
        decision_variable_index_to_mosek_matrix_variable,
    std::unordered_map<int, int>*
        decision_variable_index_to_mosek_nonmatrix_variable,
    std::unordered_map<int, std::vector<MatrixVariableEntry>>*
        matrix_variable_entries_for_same_decision_variable) {
  // Each PositiveSemidefiniteConstraint will add one matrix variable to Mosek.
  int psd_constraint_count = 0;
  for (const auto& psd_constraint : prog.positive_semidefinite_constraints()) {
    // The bounded variables of a psd constraint is the "flat" version of the
    // symmetrix matrix variables, stacked column by column. We only need to
    // store the lower triangular part of this symmetric matrix in Mosek.
    const int matrix_rows = psd_constraint.evaluator()->matrix_rows();
    for (int j = 0; j < matrix_rows; ++j) {
      for (int i = j; i < matrix_rows; ++i) {
        const MatrixVariableEntry matrix_variable_entry(psd_constraint_count, i,
                                                        j, matrix_rows);
        const int decision_variable_index = prog.FindDecisionVariableIndex(
            psd_constraint.variables()(j * matrix_rows + i));
        auto it = decision_variable_index_to_mosek_matrix_variable->find(
            decision_variable_index);
        if (it == decision_variable_index_to_mosek_matrix_variable->end()) {
          // This variable has not been registered as a mosek matrix variable
          // before.
          decision_variable_index_to_mosek_matrix_variable->emplace_hint(
              it, decision_variable_index, matrix_variable_entry);
        } else {
          // This variable has been registered as a mosek matrix variable
          // already. This matrix variable entry will be registered into
          // matrix_variable_entries_for_same_decision_variable.
          auto it_same_decision_variable =
              matrix_variable_entries_for_same_decision_variable->find(
                  decision_variable_index);
          if (it_same_decision_variable !=
              matrix_variable_entries_for_same_decision_variable->end()) {
            it_same_decision_variable->second.push_back(matrix_variable_entry);
          } else {
            matrix_variable_entries_for_same_decision_variable->emplace_hint(
                it_same_decision_variable, decision_variable_index,
                std::vector<MatrixVariableEntry>(
                    {it->second, matrix_variable_entry}));
          }
        }
      }
    }
    psd_constraint_count++;
  }
  // All the non-matrix variables in @p prog is stored in another vector inside
  // Mosek.
  int nonmatrix_variable_count = 0;
  for (int i = 0; i < prog.num_vars(); ++i) {
    if (decision_variable_index_to_mosek_matrix_variable->count(i) == 0) {
      decision_variable_index_to_mosek_nonmatrix_variable->emplace(
          i, nonmatrix_variable_count++);
    }
  }
}

/**
 * Some entries in Mosek matrix variables might correspond to the same decision
 * variable in MathematicalProgram. Add the equality constraint between these
 * matrix variable entries.
 */
MSKrescodee AddEqualityConstraintBetweenMatrixVariablesForSameDecisionVariable(
    const std::unordered_map<int, std::vector<MatrixVariableEntry>>&
        matrix_variable_entries_for_same_decision_variable,
    std::unordered_map<MatrixVariableEntry::Id, MSKint64t>*
        matrix_variable_entry_to_selection_matrix_id,
    MSKtask_t* task) {
  MSKrescodee rescode{MSK_RES_OK};
  for (const auto& pair : matrix_variable_entries_for_same_decision_variable) {
    int num_mosek_constraint;
    rescode = MSK_getnumcon(*task, &num_mosek_constraint);
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
    const auto& matrix_variable_entries = pair.second;
    const int num_matrix_variable_entries =
        static_cast<int>(matrix_variable_entries.size());
    DRAKE_ASSERT(num_matrix_variable_entries >= 2);
    rescode = MSK_appendcons(*task, num_matrix_variable_entries - 1);
    if (rescode != MSK_RES_OK) {
      return rescode;
    }

    for (int i = 1; i < num_matrix_variable_entries; ++i) {
      const int linear_constraint_index = num_mosek_constraint + i - 1;
      if (matrix_variable_entries[0].bar_matrix_index() !=
          matrix_variable_entries[i].bar_matrix_index()) {
        // We add the constraint X̅ᵢ(m, n) - X̅ⱼ(p, q) = 0 where the index of
        // the bar matrix is different (i ≠ j).
        rescode = AddScalarTimesMatrixVariableEntryToMosek(
            linear_constraint_index, matrix_variable_entries[0], 1.0,
            matrix_variable_entry_to_selection_matrix_id, task);
        if (rescode != MSK_RES_OK) {
          return rescode;
        }
        rescode = AddScalarTimesMatrixVariableEntryToMosek(
            linear_constraint_index, matrix_variable_entries[i], -1.0,
            matrix_variable_entry_to_selection_matrix_id, task);
        if (rescode != MSK_RES_OK) {
          return rescode;
        }
      } else {
        // We add the constraint that X̅ᵢ(m, n) - X̅ᵢ(p, q) = 0, where the index
        // of the bar matrix (i) are the same. So the constraint is written as
        // <A, X̅ᵢ> = 0, where
        // A(m, n) = 1 if m == n else 0.5
        // A(p, q) = -1 if p == q else -0.5
        std::array<MSKint64t, 2> E_indices{};
        rescode = AddMatrixVariableEntryCoefficientMatrixIfNonExistent(
            matrix_variable_entries[0],
            matrix_variable_entry_to_selection_matrix_id, &(E_indices[0]),
            task);
        if (rescode != MSK_RES_OK) {
          return rescode;
        }
        rescode = AddMatrixVariableEntryCoefficientMatrixIfNonExistent(
            matrix_variable_entries[i],
            matrix_variable_entry_to_selection_matrix_id, &(E_indices[1]),
            task);
        if (rescode != MSK_RES_OK) {
          return rescode;
        }

        // weights[0] = A(m, n), weights[1] = A(p, q).
        std::array<MSKrealt, 2> weights{};
        weights[0] = matrix_variable_entries[0].row_index() ==
                             matrix_variable_entries[0].col_index()
                         ? 1.0
                         : 0.5;
        weights[1] = matrix_variable_entries[i].row_index() ==
                             matrix_variable_entries[i].col_index()
                         ? -1.0
                         : -0.5;

        rescode = MSK_putbaraij(*task, linear_constraint_index,
                                matrix_variable_entries[0].bar_matrix_index(),
                                2, E_indices.data(), weights.data());
        if (rescode != MSK_RES_OK) {
          return rescode;
        }
      }
      rescode =
          SetMosekLinearConstraintBound(*task, linear_constraint_index, 0, 0,
                                        LinearConstraintBoundType::kEquality);
      if (rescode != MSK_RES_OK) {
        return rescode;
      }
    }
  }
  return rescode;
}

// @param slx Mosek dual variables for variable lower bound. See
// https://docs.mosek.com/9.2/capi/alphabetic-functionalities.html#mosek.task.getslx
// @param sux Mosek dual variables for variable upper bound. See
// https://docs.mosek.com/9.2/capi/alphabetic-functionalities.html#mosek.task.getsux
// @param slc Mosek dual variables for linear constraint lower bound. See
// https://docs.mosek.com/9.2/capi/alphabetic-functionalities.html#mosek.task.getslc
// @param suc Mosek dual variables for linear constraint upper bound. See
// https://docs.mosek.com/9.2/capi/alphabetic-functionalities.html#mosek.task.getsuc
void SetBoundingBoxDualSolution(
    const std::vector<Binding<BoundingBoxConstraint>>& constraints,
    const std::vector<MSKrealt>& slx, const std::vector<MSKrealt>& sux,
    const std::vector<MSKrealt>& slc, const std::vector<MSKrealt>& suc,
    const std::unordered_map<
        Binding<BoundingBoxConstraint>,
        std::pair<ConstraintDualIndices, ConstraintDualIndices>>&
        bb_con_dual_indices,
    MathematicalProgramResult* result) {
  auto set_dual_sol = [](int mosek_dual_lower_index, int mosek_dual_upper_index,
                         const std::vector<MSKrealt>& mosek_dual_lower,
                         const std::vector<MSKrealt>& mosek_dual_upper,
                         double* dual_sol) {
    const double dual_lower = mosek_dual_lower_index == -1
                                  ? 0.
                                  : mosek_dual_lower[mosek_dual_lower_index];
    const double dual_upper = mosek_dual_upper_index == -1
                                  ? 0.
                                  : mosek_dual_upper[mosek_dual_upper_index];
    // Mosek defines all dual solutions as non-negative. However we use
    // "reduced cost" as the dual solution, so the dual solution for a
    // lower bound should be non-negative, while the dual solution for
    // an upper bound should be non-positive.
    if (dual_lower > dual_upper) {
      // We use the larger dual as the active one.
      *dual_sol = dual_lower;
    } else {
      *dual_sol = -dual_upper;
    }
  };
  for (const auto& binding : constraints) {
    ConstraintDualIndices lower_bound_duals, upper_bound_duals;
    std::tie(lower_bound_duals, upper_bound_duals) =
        bb_con_dual_indices.at(binding);
    Eigen::VectorXd dual_sol =
        Eigen::VectorXd::Zero(binding.evaluator()->num_vars());
    for (int i = 0; i < binding.variables().rows(); ++i) {
      switch (lower_bound_duals[i].type) {
        case DualVarType::kVariableBound: {
          set_dual_sol(lower_bound_duals[i].index, upper_bound_duals[i].index,
                       slx, sux, &(dual_sol(i)));
          break;
        }
        case DualVarType::kLinearConstraint: {
          set_dual_sol(lower_bound_duals[i].index, upper_bound_duals[i].index,
                       slc, suc, &(dual_sol(i)));
          break;
        }
        default: {
          throw std::runtime_error(
              "The dual variable for a BoundingBoxConstraint lower bound can "
              "only be slx or slc.");
        }
      }
    }
    result->set_dual_solution(binding, dual_sol);
  }
}

template <typename C>
void SetLinearConstraintDualSolution(
    const std::vector<Binding<C>>& bindings, const std::vector<MSKrealt>& slc,
    const std::vector<MSKrealt>& suc,
    const std::unordered_map<Binding<C>, ConstraintDualIndices>&
        linear_con_dual_indices,
    MathematicalProgramResult* result) {
  for (const auto& binding : bindings) {
    const ConstraintDualIndices duals = linear_con_dual_indices.at(binding);
    Eigen::VectorXd dual_sol =
        Eigen::VectorXd::Zero(binding.evaluator()->num_constraints());
    for (int i = 0; i < dual_sol.rows(); ++i) {
      DRAKE_DEMAND(duals[i].type == DualVarType::kLinearConstraint);
      // Mosek defines all dual solutions as non-negative. However we use
      // "reduced cost" as the dual solution, so the dual solution for a
      // lower bound should be non-negative, while the dual solution for
      // an upper bound should be non-positive.
      if (slc[duals[i].index] > suc[duals[i].index]) {
        dual_sol[i] = slc[duals[i].index];
      } else {
        dual_sol[i] = -suc[duals[i].index];
      }
    }
    result->set_dual_solution(binding, dual_sol);
  }
}

template <typename C>
void SetNonlinearConstraintDualSolution(
    const std::vector<Binding<C>>& bindings, const std::vector<MSKrealt>& snx,
    const std::unordered_map<Binding<C>, ConstraintDualIndices>& dual_indices,
    MathematicalProgramResult* result) {
  for (const auto& binding : bindings) {
    const ConstraintDualIndices duals = dual_indices.at(binding);
    Eigen::VectorXd dual_sol = Eigen::VectorXd::Zero(duals.size());
    for (int i = 0; i < dual_sol.rows(); ++i) {
      DRAKE_DEMAND(duals[i].type == DualVarType::kNonlinearConic);
      dual_sol[i] = snx[duals[i].index];
    }
    result->set_dual_solution(binding, dual_sol);
  }
}

MSKrescodee SetDualSolution(
    const MSKtask_t& task, MSKsoltypee which_sol,
    const MathematicalProgram& prog,
    const std::unordered_map<
        Binding<BoundingBoxConstraint>,
        std::pair<ConstraintDualIndices, ConstraintDualIndices>>&
        bb_con_dual_indices,
    const std::unordered_map<Binding<LinearConstraint>, ConstraintDualIndices>&
        linear_con_dual_indices,
    const std::unordered_map<Binding<LinearEqualityConstraint>,
                             ConstraintDualIndices>& lin_eq_con_dual_indices,
    const std::unordered_map<Binding<LorentzConeConstraint>,
                             ConstraintDualIndices>& lorentz_cone_dual_indices,
    const std::unordered_map<Binding<RotatedLorentzConeConstraint>,
                             ConstraintDualIndices>&
        rotated_lorentz_cone_dual_indices,
    const std::unordered_map<Binding<ExponentialConeConstraint>,
                             ConstraintDualIndices>& exp_cone_dual_indices,
    MathematicalProgramResult* result) {
  // TODO(hongkai.dai): support other types of constraints, like linear
  // constraint, second order cone constraint, etc.
  MSKrescodee rescode{MSK_RES_OK};
  if (which_sol != MSK_SOL_ITG) {
    // Mosek cannot return dual solution if the solution type is MSK_SOL_ITG
    // (which stands for mixed integer optimizer), see
    // https://docs.mosek.com/9.2/capi/accessing-solution.html#available-solutions
    int num_mosek_vars{0};
    rescode = MSK_getnumvar(task, &num_mosek_vars);
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
    // Mosek dual variables for variable lower bounds (slx) and upper bounds
    // (sux). Refer to
    // https://docs.mosek.com/9.2/capi/alphabetic-functionalities.html#mosek.task.getsolution
    // for more explanation.
    std::vector<MSKrealt> slx(num_mosek_vars);
    std::vector<MSKrealt> sux(num_mosek_vars);
    rescode = MSK_getslx(task, which_sol, slx.data());
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
    rescode = MSK_getsux(task, which_sol, sux.data());
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
    int num_linear_constraints{0};
    rescode = MSK_getnumcon(task, &num_linear_constraints);
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
    // Mosek dual variables for linear constraints lower bounds (slc) and upper
    // bounds (suc). Refer to
    // https://docs.mosek.com/9.2/capi/alphabetic-functionalities.html#mosek.task.getsolution
    // for more explanation.
    std::vector<MSKrealt> slc(num_linear_constraints);
    std::vector<MSKrealt> suc(num_linear_constraints);
    rescode = MSK_getslc(task, which_sol, slc.data());
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
    rescode = MSK_getsuc(task, which_sol, suc.data());
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
    // Set the duals for the bounding box constraint.
    SetBoundingBoxDualSolution(prog.bounding_box_constraints(), slx, sux, slc,
                               suc, bb_con_dual_indices, result);
    // Set the duals for the linear constraint.
    SetLinearConstraintDualSolution(prog.linear_constraints(), slc, suc,
                                    linear_con_dual_indices, result);
    // Set the duals for the linear equality constraint.
    SetLinearConstraintDualSolution(prog.linear_equality_constraints(), slc,
                                    suc, lin_eq_con_dual_indices, result);
    // Set the duals for the nonlinear conic constraint.
    // Mosek provides the dual solution for nonlinear conic constraints only if
    // the program is solved through interior point approach.
    if (which_sol == MSK_SOL_ITR) {
      std::vector<MSKrealt> snx(num_mosek_vars);
      rescode = MSK_getsnx(task, which_sol, snx.data());
      if (rescode != MSK_RES_OK) {
        return rescode;
      }
      SetNonlinearConstraintDualSolution(prog.lorentz_cone_constraints(), snx,
                                         lorentz_cone_dual_indices, result);
      SetNonlinearConstraintDualSolution(
          prog.rotated_lorentz_cone_constraints(), snx,
          rotated_lorentz_cone_dual_indices, result);
      SetNonlinearConstraintDualSolution(prog.exponential_cone_constraints(),
                                         snx, exp_cone_dual_indices, result);
    }
  }
  return rescode;
}

// Throws a runtime error if the mosek option is set incorrectly.
template <typename T>
void ThrowForInvalidOption(MSKrescodee rescode, const std::string& option,
                           const T& val) {
  if (rescode != MSK_RES_OK) {
    const std::string mosek_version =
        fmt::format("{}.{}", MSK_VERSION_MAJOR, MSK_VERSION_MINOR);
    throw std::runtime_error(fmt::format(
        "MosekSolver(): cannot set Mosek option '{option}' to value '{value}', "
        "response code {code}, check "
        "https://docs.mosek.com/{version}/capi/response-codes.html for the "
        "meaning of the response code, check "
        "https://docs.mosek.com/{version}/capi/param-groups.html for allowable "
        "values in C++, or "
        "https://docs.mosek.com/{version}/pythonapi/param-groups.html "
        "for allowable values in python.",
        fmt::arg("option", option), fmt::arg("value", val),
        fmt::arg("code", rescode), fmt::arg("version", mosek_version)));
  }
}

}  // anonymous namespace

/*
 * Implements RAII for a Mosek license / environment.
 */
class MosekSolver::License {
 public:
  License() {
    if (!MosekSolver::is_enabled()) {
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
  // https://docs.mosek.com/8.1/cxxfusion/solving-parallel.html sharing
  // an env used between threads is safe (not mentioned in 9.2 documentation),
  // but nothing mentions thread-safety when allocating the environment. We can
  // safeguard against this ambiguity by using GetScopedSingleton for basic
  // thread-safety when acquiring / releasing the license.
  return GetScopedSingleton<MosekSolver::License>();
}

bool MosekSolver::is_available() { return true; }

void MosekSolver::DoSolve(const MathematicalProgram& prog,
                          const Eigen::VectorXd& initial_guess,
                          const SolverOptions& merged_options,
                          MathematicalProgramResult* result) const {
  if (!prog.GetVariableScaling().empty()) {
    static const logging::Warn log_once(
      "MosekSolver doesn't support the feature of variable scaling.");
  }

  // num_decision_vars is the total number of decision variables in @p prog. It
  // includes both the matrix variables (for psd matrix variables) and
  // non-matrix variables.
  const int num_decision_vars = prog.num_vars();
  MSKrescodee rescode{MSK_RES_OK};

  if (!license_) {
    license_ = AcquireLicense();
  }
  MSKenv_t env = license_->mosek_env();

  // Mosek treats matrix variable (variables in a psd matrix) in a special
  // manner, but MathematicalProgram doesn't. Hence we need to pick out the
  // matrix and non-matrix variables in @p prog, and record how they will be
  // stored in Mosek.
  std::unordered_map<int, MatrixVariableEntry>
      decision_variable_index_to_mosek_matrix_variable;
  std::unordered_map<int, int>
      decision_variable_index_to_mosek_nonmatrix_variable;
  // Multiple entries in Mosek matrix variables could correspond to the same
  // decision variable. We will need to add linear equality constraints to
  // equate these entries.
  std::unordered_map<int, std::vector<MatrixVariableEntry>>
      matrix_variable_entries_for_same_decision_variable;
  MapProgramDecisionVariableToMosekVariable(
      prog, &decision_variable_index_to_mosek_matrix_variable,
      &decision_variable_index_to_mosek_nonmatrix_variable,
      &matrix_variable_entries_for_same_decision_variable);
  DRAKE_ASSERT(
      static_cast<int>(
          decision_variable_index_to_mosek_matrix_variable.size() +
          decision_variable_index_to_mosek_nonmatrix_variable.size()) ==
      prog.num_vars());
  // The number of non-matrix variables in @p prog.
  const int num_nonmatrix_vars_in_prog =
      decision_variable_index_to_mosek_nonmatrix_variable.size();
  // For each entry of the matrix variable X̅ᵢ(m,n), if this entry is also used
  // in the cost or constraint, then we will use the term <E̅ₘₙ,X̅ᵢ>, where
  // the inner product <E̅ₘₙ, X̅ᵢ> = X̅ᵢ(m,n). The "selection matrix" E̅ₘₙ helps
  // to select the (m, n)'th entry of the psd matrix variable. E̅ₘₙ is stored
  // inside Mosek with a unique ID.
  // matrix_variable_entry_to_selection_matrix_id maps the matrix variable
  // entry to the ID of E̅ₘₙ.
  // TODO(hongkai.dai): do not map the matrix variable entry to the selection
  // matrix, instead maps the tuple (matrix_size, row_index, column_index) to
  // the selection matrix. This will create fewer selection matrices, when
  // multiple matrix variables have the same size.
  std::unordered_map<MatrixVariableEntry::Id, MSKint64t>
      matrix_variable_entry_to_selection_matrix_id;

  // Create the optimization task.
  // task is initialized as a null pointer, same as in Mosek's documentation
  // https://docs.mosek.com/9.2/capi/design.html#hello-world-in-mosek
  MSKtask_t task = nullptr;
  rescode = MSK_maketask(env, 0, num_nonmatrix_vars_in_prog, &task);
  ScopeExit guard([&task]() { MSK_deletetask(&task); });

  // Set the options (parameters).
  for (const auto& double_options : merged_options.GetOptionsDouble(id())) {
    if (rescode == MSK_RES_OK) {
      rescode = MSK_putnadouparam(task, double_options.first.c_str(),
                                  double_options.second);
      ThrowForInvalidOption(rescode, double_options.first,
                            double_options.second);
    }
  }
  for (const auto& int_options : merged_options.GetOptionsInt(id())) {
    if (rescode == MSK_RES_OK) {
      rescode = MSK_putnaintparam(task, int_options.first.c_str(),
                                  int_options.second);
      ThrowForInvalidOption(rescode, int_options.first, int_options.second);
    }
  }
  std::optional<std::string> msk_writedata;
  for (const auto& str_options : merged_options.GetOptionsStr(id())) {
    if (rescode == MSK_RES_OK) {
      if (str_options.first == "writedata") {
        if (str_options.second != "") {
          msk_writedata = str_options.second;
        }
      } else {
        rescode = MSK_putnastrparam(task, str_options.first.c_str(),
                                    str_options.second.c_str());
        ThrowForInvalidOption(rescode, str_options.first, str_options.second);
      }
    }
  }

  // Always check if rescode is MSK_RES_OK before we call any Mosek functions.
  // If it is not MSK_RES_OK, then bypasses everything and exits.
  if (rescode == MSK_RES_OK) {
    rescode = MSK_appendvars(task, num_nonmatrix_vars_in_prog);
  }
  // Add positive semidefinite constraint. This also adds Mosek matrix
  // variables.
  if (rescode == MSK_RES_OK) {
    rescode = AddPositiveSemidefiniteConstraints(prog, &task);
  }
  // Add the constraint that Mosek matrix variable entries corresponding to the
  // same decision variables should all be equal.
  if (rescode == MSK_RES_OK) {
    rescode =
        AddEqualityConstraintBetweenMatrixVariablesForSameDecisionVariable(
            matrix_variable_entries_for_same_decision_variable,
            &matrix_variable_entry_to_selection_matrix_id, &task);
  }
  // Add costs
  if (rescode == MSK_RES_OK) {
    rescode = AddCosts(prog, decision_variable_index_to_mosek_matrix_variable,
                       decision_variable_index_to_mosek_nonmatrix_variable,
                       &matrix_variable_entry_to_selection_matrix_id, &task);
  }

  // We store the dual variable indices for each bounding box constraint.
  // bb_con_dual_indices[constraint] returns the pair (lower_bound_indices,
  // upper_bound_indices), where lower_bound_indices are the indices of the
  // lower bound dual variables, and upper_bound_indices are the indices of the
  // upper bound dual variables.
  std::unordered_map<Binding<BoundingBoxConstraint>,
                     std::pair<ConstraintDualIndices, ConstraintDualIndices>>
      bb_con_dual_indices;
  // Add bounding box constraints on decision variables.
  if (rescode == MSK_RES_OK) {
    rescode = AddBoundingBoxConstraints(
        prog, decision_variable_index_to_mosek_matrix_variable,
        decision_variable_index_to_mosek_nonmatrix_variable,
        &matrix_variable_entry_to_selection_matrix_id, &bb_con_dual_indices,
        &task);
  }
  // Specify binary variables.
  bool with_integer_or_binary_variable = false;
  if (rescode == MSK_RES_OK) {
    rescode = SpecifyVariableType(
        prog, decision_variable_index_to_mosek_matrix_variable,
        decision_variable_index_to_mosek_nonmatrix_variable, &task,
        &with_integer_or_binary_variable);
  }
  // Add linear constraints.
  std::unordered_map<Binding<LinearConstraint>, ConstraintDualIndices>
      linear_con_dual_indices;
  std::unordered_map<Binding<LinearEqualityConstraint>, ConstraintDualIndices>
      lin_eq_con_dual_indices;
  if (rescode == MSK_RES_OK) {
    rescode = AddLinearConstraints(
        prog, decision_variable_index_to_mosek_matrix_variable,
        decision_variable_index_to_mosek_nonmatrix_variable,
        &matrix_variable_entry_to_selection_matrix_id, &linear_con_dual_indices,
        &lin_eq_con_dual_indices, &task);
  }

  // Add Lorentz cone constraints.
  std::unordered_map<Binding<LorentzConeConstraint>, ConstraintDualIndices>
      lorentz_cone_dual_indices;
  if (rescode == MSK_RES_OK) {
    rescode =
        AddConeConstraints(prog, prog.lorentz_cone_constraints(),
                           decision_variable_index_to_mosek_matrix_variable,
                           decision_variable_index_to_mosek_nonmatrix_variable,
                           &matrix_variable_entry_to_selection_matrix_id,
                           &lorentz_cone_dual_indices, &task);
  }

  // Add rotated Lorentz cone constraints.
  std::unordered_map<Binding<RotatedLorentzConeConstraint>,
                     ConstraintDualIndices>
      rotated_lorentz_cone_dual_indices;
  if (rescode == MSK_RES_OK) {
    rescode =
        AddConeConstraints(prog, prog.rotated_lorentz_cone_constraints(),
                           decision_variable_index_to_mosek_matrix_variable,
                           decision_variable_index_to_mosek_nonmatrix_variable,
                           &matrix_variable_entry_to_selection_matrix_id,
                           &rotated_lorentz_cone_dual_indices, &task);
  }

  // Add linear matrix inequality constraints.
  if (rescode == MSK_RES_OK) {
    rescode = AddLinearMatrixInequalityConstraint(
        prog, decision_variable_index_to_mosek_matrix_variable,
        decision_variable_index_to_mosek_nonmatrix_variable,
        &matrix_variable_entry_to_selection_matrix_id, &task);
  }

  // Add exponential cone constraints.
  std::unordered_map<Binding<ExponentialConeConstraint>, ConstraintDualIndices>
      exp_cone_dual_indices;
  if (rescode == MSK_RES_OK) {
    rescode =
        AddConeConstraints(prog, prog.exponential_cone_constraints(),
                           decision_variable_index_to_mosek_matrix_variable,
                           decision_variable_index_to_mosek_nonmatrix_variable,
                           &matrix_variable_entry_to_selection_matrix_id,
                           &exp_cone_dual_indices, &task);
  }

  // log file.
  const bool print_to_console = merged_options.get_print_to_console();
  const std::string print_file_name = merged_options.get_print_file_name();
  // Refer to https://docs.mosek.com/9.2/capi/solver-io.html#stream-logging
  // for Mosek stream logging.
  // First we check if the user wants to print to both the console and the file.
  // If true, throw an error BEFORE we create the log file through
  // MSK_linkfiletotaskstream. Otherwise we might create the log file but cannot
  // close it.
  if (print_to_console && !print_file_name.empty()) {
    throw std::runtime_error(
        "MosekSolver::Solve(): cannot print to both the console and the log "
        "file.");
  } else if (print_to_console) {
    rescode = MSK_linkfunctotaskstream(task, MSK_STREAM_LOG, nullptr, printstr);
  } else if (!print_file_name.empty()) {
    rescode = MSK_linkfiletotaskstream(task, MSK_STREAM_LOG,
                                       print_file_name.c_str(), 0);
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
      rescode = MSK_getnumvar(task, &num_mosek_vars);
    }
    for (int i = 0; i < prog.num_vars(); ++i) {
      auto it = decision_variable_index_to_mosek_nonmatrix_variable.find(i);
      if (it != decision_variable_index_to_mosek_nonmatrix_variable.end()) {
        const auto var_type = prog.decision_variable(i).get_type();
        if (var_type == MathematicalProgram::VarType::INTEGER ||
            var_type == MathematicalProgram::VarType::BINARY) {
          if (rescode == MSK_RES_OK) {
            const MSKrealt initial_guess_i = initial_guess(i);
            rescode = MSK_putxxslice(task, MSK_SOL_ITG, it->second,
                                     it->second + 1, &initial_guess_i);
          }
        }
      }
    }
  }

  result->set_solution_result(SolutionResult::kUnknownError);
  // Run optimizer.
  if (rescode == MSK_RES_OK) {
    // TODO(hongkai.dai@tri.global): add trmcode to the returned struct.
    MSKrescodee trmcode;  // termination code
    rescode = MSK_optimizetrm(task, &trmcode);
  }

  if (rescode == MSK_RES_OK && msk_writedata.has_value()) {
    rescode = MSK_writedata(task, msk_writedata.value().c_str());
  }

  // Determines the solution type.
  MSKsoltypee solution_type;
  if (with_integer_or_binary_variable) {
    solution_type = MSK_SOL_ITG;
  } else if (prog.quadratic_costs().empty() &&
             prog.lorentz_cone_constraints().empty() &&
             prog.rotated_lorentz_cone_constraints().empty() &&
             prog.positive_semidefinite_constraints().empty() &&
             prog.linear_matrix_inequality_constraints().empty() &&
             prog.exponential_cone_constraints().empty()) {
    solution_type = MSK_SOL_BAS;
  } else {
    solution_type = MSK_SOL_ITR;
  }

  MSKsolstae solution_status{MSK_SOL_STA_UNKNOWN};
  if (rescode == MSK_RES_OK) {
    if (rescode == MSK_RES_OK) {
      rescode = MSK_getsolsta(task, solution_type, &solution_status);
    }
    if (rescode == MSK_RES_OK) {
      switch (solution_status) {
        case MSK_SOL_STA_OPTIMAL:
        case MSK_SOL_STA_INTEGER_OPTIMAL:
        case MSK_SOL_STA_PRIM_FEAS: {
          result->set_solution_result(SolutionResult::kSolutionFound);
          MSKint32t num_mosek_vars;
          rescode = MSK_getnumvar(task, &num_mosek_vars);
          DRAKE_ASSERT(rescode == MSK_RES_OK);
          Eigen::VectorXd mosek_sol_vector(num_mosek_vars);
          rescode = MSK_getxx(task, solution_type, mosek_sol_vector.data());
          MSKint32t num_bar_x;
          rescode = MSK_getnumbarvar(task, &num_bar_x);
          DRAKE_ASSERT(rescode == MSK_RES_OK);
          std::vector<Eigen::VectorXd> mosek_bar_x_sol(num_bar_x);
          for (int i = 0; i < num_bar_x; ++i) {
            MSKint32t bar_xi_dim;
            rescode = MSK_getdimbarvarj(task, i, &bar_xi_dim);
            DRAKE_ASSERT(rescode == MSK_RES_OK);
            mosek_bar_x_sol[i].resize(bar_xi_dim * (bar_xi_dim + 1) / 2);
            rescode =
                MSK_getbarxj(task, solution_type, i, mosek_bar_x_sol[i].data());
          }
          DRAKE_ASSERT(rescode == MSK_RES_OK);
          Eigen::VectorXd sol_vector(num_decision_vars);
          for (int i = 0; i < num_decision_vars; ++i) {
            auto it1 =
                decision_variable_index_to_mosek_nonmatrix_variable.find(i);
            if (it1 !=
                decision_variable_index_to_mosek_nonmatrix_variable.end()) {
              sol_vector(i) = mosek_sol_vector(it1->second);
            } else {
              auto it2 =
                  decision_variable_index_to_mosek_matrix_variable.find(i);
              sol_vector(i) = mosek_bar_x_sol[it2->second.bar_matrix_index()](
                  it2->second.IndexInLowerTrianglePart());
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
          rescode = SetDualSolution(
              task, solution_type, prog, bb_con_dual_indices,
              linear_con_dual_indices, lin_eq_con_dual_indices,
              lorentz_cone_dual_indices, rotated_lorentz_cone_dual_indices,
              exp_cone_dual_indices, result);
          DRAKE_ASSERT(rescode == MSK_RES_OK);
          break;
        }
        case MSK_SOL_STA_DUAL_INFEAS_CER: {
          result->set_solution_result(SolutionResult::kDualInfeasible);
          break;
        }
        case MSK_SOL_STA_PRIM_INFEAS_CER: {
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
}

}  // namespace solvers
}  // namespace drake
