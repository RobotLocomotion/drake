#include "drake/solvers/mosek_solver_internal.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <limits>

#include "drake/common/never_destroyed.h"
#include "drake/math/quadratic_form.h"
#include "drake/solvers/aggregate_costs_constraints.h"

namespace drake {
namespace solvers {
namespace internal {
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

size_t MatrixVariableEntry::get_next_id() {
  static never_destroyed<std::atomic<int>> next_id(0);
  return next_id.access()++;
}

MosekSolverProgram::MosekSolverProgram(const MathematicalProgram& prog,
                                       MSKenv_t env)
    : map_decision_var_to_mosek_var_{prog} {
  // Create the optimization task.
  // task is initialized as a null pointer, same as in Mosek's documentation
  // https://docs.mosek.com/10.0/capi/design.html#hello-world-in-mosek
  task_ = nullptr;
  MSK_maketask(env, 0,
               map_decision_var_to_mosek_var_
                   .decision_variable_to_mosek_nonmatrix_variable.size(),
               &task_);
}

MosekSolverProgram::~MosekSolverProgram() {
  MSK_deletetask(&task_);
}

MSKrescodee
MosekSolverProgram::AddMatrixVariableEntryCoefficientMatrixIfNonExistent(
    const MatrixVariableEntry& matrix_variable_entry, MSKint64t* E_index) {
  MSKrescodee rescode{MSK_RES_OK};
  auto it = matrix_variable_entry_to_selection_matrix_id_.find(
      matrix_variable_entry.id());
  if (it != matrix_variable_entry_to_selection_matrix_id_.end()) {
    *E_index = it->second;
  } else {
    const MSKint32t row = matrix_variable_entry.row_index();
    const MSKint32t col = matrix_variable_entry.col_index();
    const MSKrealt val = row == col ? 1.0 : 0.5;
    rescode =
        MSK_appendsparsesymmat(task_, matrix_variable_entry.num_matrix_rows(),
                               1, &row, &col, &val, E_index);
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
    matrix_variable_entry_to_selection_matrix_id_.emplace_hint(
        it, matrix_variable_entry.id(), *E_index);
  }
  return rescode;
}

MSKrescodee MosekSolverProgram::AddScalarTimesMatrixVariableEntryToMosek(
    MSKint32t constraint_index,
    const MatrixVariableEntry& matrix_variable_entry, MSKrealt scalar) {
  MSKrescodee rescode{MSK_RES_OK};
  MSKint64t E_index;
  rescode = AddMatrixVariableEntryCoefficientMatrixIfNonExistent(
      matrix_variable_entry, &E_index);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  rescode = MSK_putbaraij(task_, constraint_index,
                          matrix_variable_entry.bar_matrix_index(), 1, &E_index,
                          &scalar);
  return rescode;
}

// Determine the sense of each constraint. The sense can be equality constraint,
// less than, greater than, or bounded on both side.
MSKrescodee MosekSolverProgram::SetMosekLinearConstraintBound(
    int linear_constraint_index, double lower, double upper,
    LinearConstraintBoundType bound_type) {
  MSKrescodee rescode{MSK_RES_OK};
  switch (bound_type) {
    case LinearConstraintBoundType::kEquality: {
      rescode = MSK_putconbound(task_, linear_constraint_index, MSK_BK_FX,
                                lower, lower);
      if (rescode != MSK_RES_OK) {
        return rescode;
      }
      break;
    }
    case LinearConstraintBoundType::kInequality: {
      if (std::isinf(lower) && std::isinf(upper)) {
        DRAKE_DEMAND(lower < 0 && upper > 0);
        rescode = MSK_putconbound(task_, linear_constraint_index, MSK_BK_FR,
                                  -MSK_INFINITY, MSK_INFINITY);
      } else if (std::isinf(lower) && !std::isinf(upper)) {
        rescode = MSK_putconbound(task_, linear_constraint_index, MSK_BK_UP,
                                  -MSK_INFINITY, upper);
      } else if (!std::isinf(lower) && std::isinf(upper)) {
        rescode = MSK_putconbound(task_, linear_constraint_index, MSK_BK_LO,
                                  lower, MSK_INFINITY);
      } else {
        rescode = MSK_putconbound(task_, linear_constraint_index, MSK_BK_RA,
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
MSKrescodee MosekSolverProgram::AddLinearConstraintToMosek(
    const MathematicalProgram& prog, const Eigen::SparseMatrix<double>& A,
    const Eigen::SparseMatrix<double>& B, const Eigen::VectorXd& lower,
    const Eigen::VectorXd& upper,
    const VectorX<symbolic::Variable>& decision_vars,
    const std::vector<MSKint32t>& slack_vars_mosek_indices,
    LinearConstraintBoundType bound_type) {
  MSKrescodee rescode{MSK_RES_OK};
  DRAKE_ASSERT(lower.rows() == upper.rows());
  DRAKE_ASSERT(A.rows() == lower.rows() && A.cols() == decision_vars.rows());
  DRAKE_ASSERT(B.rows() == lower.rows() &&
               B.cols() == static_cast<int>(slack_vars_mosek_indices.size()));
  int num_mosek_constraint{};
  rescode = MSK_getnumcon(task_, &num_mosek_constraint);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  rescode = MSK_appendcons(task_, lower.rows());
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  for (int i = 0; i < lower.rows(); ++i) {
    // It is important that AddLinearConstraintToMosek keeps the order of the
    // linear constraint the same as in lower <= A * decision_vars + B *
    // slack_vars <= upper. When we specify the dual variable indices, we rely
    // on this order.
    rescode = SetMosekLinearConstraintBound(num_mosek_constraint + i, lower(i),
                                            upper(i), bound_type);
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
  }
  // (A * decision_vars + B * slack_var)(i) = (Aₓ * x)(i) + ∑ⱼ <A̅ᵢⱼ, X̅ⱼ>
  // where we decompose [decision_vars; slack_var] to Mosek nonmatrix variable
  // x and Mosek matrix variable X̅.

  // Ax_subi, Ax_subj, Ax_valij are the triplets format of matrix Aₓ.
  std::vector<MSKint32t> Ax_subi, Ax_subj;
  std::vector<MSKrealt> Ax_valij;
  std::vector<std::unordered_map<
      MSKint64t, std::pair<std::vector<MSKint64t>, std::vector<MSKrealt>>>>
      bar_A;

  rescode =
      ParseLinearExpression(prog, A, B, decision_vars, slack_vars_mosek_indices,
                            &Ax_subi, &Ax_subj, &Ax_valij, &bar_A);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  if (!bar_A.empty()) {
    for (int i = 0; i < lower.rows(); ++i) {
      for (const auto& [j, sub_weights] : bar_A[i]) {
        // Now compute the matrix A̅ᵢⱼ.
        const std::vector<MSKint64t>& sub = sub_weights.first;
        const std::vector<MSKrealt>& weights = sub_weights.second;
        rescode = MSK_putbaraij(task_, num_mosek_constraint + i, j, sub.size(),
                                sub.data(), weights.data());
        if (rescode != MSK_RES_OK) {
          return rescode;
        }
      }
    }
  }
  // Add num_mosek_constraint to each entry of Ax_subi;
  for (int i = 0; i < static_cast<int>(Ax_subi.size()); ++i) {
    Ax_subi[i] += num_mosek_constraint;
  }
  rescode = MSK_putaijlist(task_, Ax_subi.size(), Ax_subi.data(),
                           Ax_subj.data(), Ax_valij.data());
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  int num_mosek_constraints_after{};
  rescode = MSK_getnumcon(task_, &num_mosek_constraints_after);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  DRAKE_DEMAND(num_mosek_constraints_after ==
               num_mosek_constraint + lower.rows());
  return rescode;
}

MSKrescodee MosekSolverProgram::ParseLinearExpression(
    const solvers::MathematicalProgram& prog,
    const Eigen::SparseMatrix<double>& A, const Eigen::SparseMatrix<double>& B,
    const VectorX<symbolic::Variable>& decision_vars,
    const std::vector<MSKint32t>& slack_vars_mosek_indices,
    std::vector<MSKint32t>* F_subi, std::vector<MSKint32t>* F_subj,
    std::vector<MSKrealt>* F_valij,
    std::vector<std::unordered_map<
        MSKint64t, std::pair<std::vector<MSKint64t>, std::vector<MSKrealt>>>>*
        bar_F) {
  // First check if decision_vars contains duplication.
  // Since the duplication doesn't happen very often, we focus on improving the
  // speed of the no-duplication case.
  const symbolic::Variables decision_vars_set(decision_vars);
  if (static_cast<int>(decision_vars_set.size()) == decision_vars.rows()) {
    return this->ParseLinearExpressionNoDuplication(
        prog, A, B, decision_vars, slack_vars_mosek_indices, F_subi, F_subj,
        F_valij, bar_F);
  } else {
    Eigen::SparseMatrix<double> A_unique;
    VectorX<symbolic::Variable> unique_decision_vars;
    AggregateDuplicateVariables(A, decision_vars, &A_unique,
                                &unique_decision_vars);
    return this->ParseLinearExpressionNoDuplication(
        prog, A_unique, B, unique_decision_vars, slack_vars_mosek_indices,
        F_subi, F_subj, F_valij, bar_F);
  }
}

MSKrescodee MosekSolverProgram::ParseLinearExpressionNoDuplication(
    const solvers::MathematicalProgram& prog,
    const Eigen::SparseMatrix<double>& A, const Eigen::SparseMatrix<double>& B,
    const VectorX<symbolic::Variable>& decision_vars,
    const std::vector<MSKint32t>& slack_vars_mosek_indices,
    std::vector<MSKint32t>* F_subi, std::vector<MSKint32t>* F_subj,
    std::vector<MSKrealt>* F_valij,
    std::vector<std::unordered_map<
        MSKint64t, std::pair<std::vector<MSKint64t>, std::vector<MSKrealt>>>>*
        bar_F) {
  MSKrescodee rescode{MSK_RES_OK};
  DRAKE_ASSERT(A.rows() == B.rows());
  DRAKE_ASSERT(A.cols() == decision_vars.rows());
  DRAKE_ASSERT(B.cols() == static_cast<int>(slack_vars_mosek_indices.size()));

  // (A * decision_vars + B * slack_var)(i) = (F * x)(i) + ∑ⱼ <F̅ᵢⱼ, X̅ⱼ>
  // where we decompose [decision_vars; slack_var] to Mosek nonmatrix variable
  // x and Mosek matrix variable X̅.

  F_subi->reserve(A.nonZeros() + B.nonZeros());
  F_subj->reserve(A.nonZeros() + B.nonZeros());
  F_valij->reserve(A.nonZeros() + B.nonZeros());

  // mosek_matrix_variable_entries stores all the matrix variables used in this
  // newly added linear constraint.
  // mosek_matrix_variable_entries[j] contains all the entries in that matrix
  // variable X̅ⱼ that show up in this new linear constraint.
  // This map is used to compute the symmetric matrix F̅ᵢⱼA̅ᵢⱼ in the term
  // <F̅ᵢⱼA̅ᵢⱼ, X̅ⱼ>.
  std::unordered_map<MSKint64t, std::vector<MatrixVariableEntry>>
      mosek_matrix_variable_entries;
  if (A.nonZeros() != 0) {
    std::vector<int> decision_var_indices(decision_vars.rows());
    // decision_vars[mosek_matrix_variable_in_decision_var[i]] is a Mosek matrix
    // variable.
    std::vector<int> mosek_matrix_variable_in_decision_var;
    mosek_matrix_variable_in_decision_var.reserve(decision_vars.rows());
    for (int i = 0; i < decision_vars.rows(); ++i) {
      decision_var_indices[i] =
          prog.FindDecisionVariableIndex(decision_vars(i));
      const auto it_mosek_matrix_variable =
          decision_variable_to_mosek_matrix_variable().find(
              decision_var_indices[i]);
      if (it_mosek_matrix_variable !=
          decision_variable_to_mosek_matrix_variable().end()) {
        // This decision variable is a matrix variable.
        // Denote the matrix variables as X̅. Add the corresponding matrix
        // E̅ₘₙ, such that <E̅ₘₙ, X̅> = X̅(m, n) if such matrix has not been
        // added to Mosek yet.
        mosek_matrix_variable_in_decision_var.push_back(i);
        const MatrixVariableEntry& matrix_variable_entry =
            it_mosek_matrix_variable->second;
        MSKint64t E_mn_index;
        rescode = AddMatrixVariableEntryCoefficientMatrixIfNonExistent(
            matrix_variable_entry, &E_mn_index);
        if (rescode != MSK_RES_OK) {
          return rescode;
        }
        mosek_matrix_variable_entries[matrix_variable_entry.bar_matrix_index()]
            .push_back(matrix_variable_entry);
      } else {
        const int mosek_nonmatrix_variable =
            decision_variable_to_mosek_nonmatrix_variable().at(
                decision_var_indices[i]);
        for (Eigen::SparseMatrix<double>::InnerIterator it(A, i); it; ++it) {
          F_subi->push_back(it.row());
          F_subj->push_back(mosek_nonmatrix_variable);
          F_valij->push_back(it.value());
        }
      }
    }
    if (!mosek_matrix_variable_entries.empty()) {
      bar_F->resize(A.rows());
      for (int col_index : mosek_matrix_variable_in_decision_var) {
        const MatrixVariableEntry& matrix_variable_entry =
            decision_variable_to_mosek_matrix_variable().at(
                decision_var_indices[col_index]);
        for (Eigen::SparseMatrix<double>::InnerIterator it(A, col_index); it;
             ++it) {
          // If we denote m = matrix_variable_entry.row_index(), n =
          // matrix_variable_entry.col_index(), then
          // bar_F[i][j] = \sum_{col_index} A(i, col_index) * Emn, where j =
          // matrix_variable_entry.bar_matrix_index().
          auto it_bar_F =
              (*bar_F)[it.row()].find(matrix_variable_entry.bar_matrix_index());
          if (it_bar_F != (*bar_F)[it.row()].end()) {
            it_bar_F->second.first.push_back(
                matrix_variable_entry_to_selection_matrix_id_.at(
                    matrix_variable_entry.id()));
            it_bar_F->second.second.push_back(it.value());
          } else {
            (*bar_F)[it.row()].emplace_hint(
                it_bar_F, matrix_variable_entry.bar_matrix_index(),
                std::pair<std::vector<MSKint64t>, std::vector<MSKrealt>>(
                    {matrix_variable_entry_to_selection_matrix_id_.at(
                        matrix_variable_entry.id())},
                    {it.value()}));
          }
        }
      }
    }
  }
  if (B.nonZeros() != 0) {
    for (int i = 0; i < B.cols(); ++i) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(B, i); it; ++it) {
        F_subi->push_back(it.row());
        F_subj->push_back(slack_vars_mosek_indices[i]);
        F_valij->push_back(it.value());
      }
    }
  }

  return rescode;
}

MSKrescodee MosekSolverProgram::AddLinearConstraints(
    const MathematicalProgram& prog,
    std::unordered_map<Binding<LinearConstraint>, ConstraintDualIndices>*
        linear_con_dual_indices,
    std::unordered_map<Binding<LinearEqualityConstraint>,
                       ConstraintDualIndices>* lin_eq_con_dual_indices) {
  MSKrescodee rescode = AddLinearConstraintsFromBindings(
      prog.linear_equality_constraints(), LinearConstraintBoundType::kEquality,
      prog, lin_eq_con_dual_indices);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  rescode = AddLinearConstraintsFromBindings(
      prog.linear_constraints(), LinearConstraintBoundType::kInequality, prog,
      linear_con_dual_indices);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }

  return rescode;
}

// Add the bounds on the decision variables in @p prog. Note that if a decision
// variable in positive definite matrix has a bound, we need to add new linear
// constraint to Mosek to bound that variable.
MSKrescodee MosekSolverProgram::AddBoundingBoxConstraints(
    const MathematicalProgram& prog,
    std::unordered_map<Binding<BoundingBoxConstraint>,
                       std::pair<ConstraintDualIndices, ConstraintDualIndices>>*
        dual_indices) {
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

  auto add_variable_bound_in_mosek = [this](int mosek_var_index, double lower,
                                            double upper) {
    MSKrescodee rescode_bound{MSK_RES_OK};
    if (std::isinf(lower) && std::isinf(upper)) {
      rescode_bound = MSK_putvarbound(this->task_, mosek_var_index, MSK_BK_FR,
                                      -MSK_INFINITY, MSK_INFINITY);
    } else if (std::isinf(lower) && !std::isinf(upper)) {
      rescode_bound = MSK_putvarbound(this->task_, mosek_var_index, MSK_BK_UP,
                                      -MSK_INFINITY, upper);
    } else if (!std::isinf(lower) && std::isinf(upper)) {
      rescode_bound = MSK_putvarbound(this->task_, mosek_var_index, MSK_BK_LO,
                                      lower, MSK_INFINITY);
    } else {
      rescode_bound = MSK_putvarbound(this->task_, mosek_var_index, MSK_BK_RA,
                                      lower, upper);
    }
    return rescode_bound;
  };

  MSKrescodee rescode = MSK_RES_OK;
  // bounded_matrix_var_indices[i] is the index of the variable
  // bounded_matrix_vars[i] in prog.
  std::vector<int> bounded_matrix_var_indices;
  bounded_matrix_var_indices.reserve(prog.num_vars());
  for (int i = 0; i < num_decision_vars; i++) {
    auto it1 = decision_variable_to_mosek_nonmatrix_variable().find(i);
    if (it1 != decision_variable_to_mosek_nonmatrix_variable().end()) {
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
  rescode = MSK_getnumcon(this->task_, &num_linear_constraints_before);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  rescode = AddLinearConstraintToMosek(
      prog, A_eye, B_zero, bounded_matrix_vars_lower, bounded_matrix_vars_upper,
      bounded_matrix_vars, {}, LinearConstraintBoundType::kInequality);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  int num_linear_constraints_after = 0;
  rescode = MSK_getnumcon(this->task_, &num_linear_constraints_after);
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
          decision_variable_to_mosek_nonmatrix_variable().find(var_index);
      int dual_variable_index_if_active = -1;
      if (it1 != decision_variable_to_mosek_nonmatrix_variable().end()) {
        // Add the dual variables for the constraint registered as bounds on
        // Mosek non-matrix variables.
        lower_bound_duals[i].type = DualVarType::kVariableBound;
        upper_bound_duals[i].type = DualVarType::kVariableBound;
        dual_variable_index_if_active = it1->second;
      } else {
        // This variable is a Mosek matrix variable. The bound on this variable
        // is imposed as a linear constraint.
        DRAKE_DEMAND(
            decision_variable_to_mosek_matrix_variable().count(var_index) > 0);
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

MSKrescodee MosekSolverProgram::AddAffineConeConstraint(
    const MathematicalProgram& prog, const Eigen::SparseMatrix<double>& A,
    const Eigen::SparseMatrix<double>& B,
    const VectorX<symbolic::Variable>& decision_vars,
    const std::vector<MSKint32t>& slack_vars_mosek_indices,
    const Eigen::VectorXd& c, MSKconetypee cone_type, MSKint64t* acc_index) {
  MSKrescodee rescode = MSK_RES_OK;
  std::vector<MSKint32t> F_subi;
  std::vector<MSKint32t> F_subj;
  std::vector<MSKrealt> F_valij;
  std::vector<std::unordered_map<
      MSKint64t, std::pair<std::vector<MSKint64t>, std::vector<MSKrealt>>>>
      bar_F;
  // Get the dimension of this affine expression.
  const int afe_dim = A.rows();
  this->ParseLinearExpression(prog, A, B, decision_vars,
                              slack_vars_mosek_indices, &F_subi, &F_subj,
                              &F_valij, &bar_F);
  // Get the total number of affine expressions.
  MSKint64t num_afe{0};
  rescode = MSK_getnumafe(task_, &num_afe);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  rescode = MSK_appendafes(task_, afe_dim);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  // Now increase F_subi by num_afe and add F matrix to Mosek affine
  // expressions.
  std::vector<MSKint64t> F_subi_increased(F_subi.size());
  for (int i = 0; i < static_cast<int>(F_subi.size()); ++i) {
    F_subi_increased[i] = F_subi[i] + num_afe;
  }
  rescode = MSK_putafefentrylist(task_, F_subi_increased.size(),
                                 F_subi_increased.data(), F_subj.data(),
                                 F_valij.data());
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  // Add g vector.
  rescode = MSK_putafegslice(task_, num_afe, num_afe + afe_dim, c.data());
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  // Now handle the part in the affine expression that involves the mosek matrix
  // variables.
  // bar_F is non-empty only if this affine expression involves the mosek matrix
  // variables. We need to check if bar_F is non-empty, so that we can call
  // bar_F[i] in the inner loop.
  if (!bar_F.empty()) {
    for (int i = 0; i < afe_dim; ++i) {
      for (const auto& [j, sub_weights] : bar_F[i]) {
        const std::vector<MSKint64t>& sub = sub_weights.first;
        rescode = MSK_putafebarfentry(task_, num_afe + i, j, sub.size(),
                                      sub.data(), sub_weights.second.data());
        if (rescode != MSK_RES_OK) {
          return rescode;
        }
      }
    }
  }
  // Create the domain.
  MSKint64t dom_idx;
  switch (cone_type) {
    case MSK_CT_QUAD: {
      rescode = MSK_appendquadraticconedomain(task_, afe_dim, &dom_idx);
      break;
    }
    case MSK_CT_RQUAD: {
      rescode = MSK_appendrquadraticconedomain(task_, afe_dim, &dom_idx);
      break;
    }
    case MSK_CT_PEXP: {
      rescode = MSK_appendprimalexpconedomain(task_, &dom_idx);
      break;
    }
    default: {
      throw std::runtime_error("MosekSolverProgram: unsupported cone type.");
    }
  }
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  // Obtain the affine cone index.
  rescode = MSK_getnumacc(task_, acc_index);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  // Add the actual affine cone constraint.
  rescode = MSK_appendaccseq(task_, dom_idx, afe_dim, num_afe, nullptr);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  return rescode;
}

MSKrescodee MosekSolverProgram::AddPositiveSemidefiniteConstraints(
    const MathematicalProgram& prog,
    std::unordered_map<Binding<PositiveSemidefiniteConstraint>, MSKint32t>*
        psd_barvar_indices) {
  MSKrescodee rescode = MSK_RES_OK;
  // First get the current number of bar matrix vars before appending the new
  // ones.
  MSKint32t numbarvar;
  rescode = MSK_getnumbarvar(task_, &numbarvar);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  std::vector<MSKint32t> bar_var_dimension;
  bar_var_dimension.reserve(prog.positive_semidefinite_constraints().size());
  int psd_count = 0;
  for (const auto& binding : prog.positive_semidefinite_constraints()) {
    bar_var_dimension.push_back(binding.evaluator()->matrix_rows());
    psd_barvar_indices->emplace(binding, numbarvar + psd_count);
    psd_count++;
  }

  rescode = MSK_appendbarvars(task_, bar_var_dimension.size(),
                              bar_var_dimension.data());
  return rescode;
}

MSKrescodee MosekSolverProgram::AddLinearMatrixInequalityConstraint(
    const MathematicalProgram& prog) {
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
    rescode = MSK_getnumcon(task_, &num_linear_constraint);
    if (rescode != MSK_RES_OK) {
      return rescode;
    }

    const int rows = binding.evaluator()->matrix_rows();
    rescode = MSK_appendbarvars(task_, 1, &rows);
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
    MSKint32t bar_X_index;
    rescode = MSK_getnumbarvar(task_, &bar_X_index);
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
        bound, binding.variables(), {}, LinearConstraintBoundType::kEquality);
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
            MSK_appendsparsesymmat(task_, rows, 1, &i, &j, &val, &E_index);
        if (rescode != MSK_RES_OK) {
          return rescode;
        }
        const MSKrealt weights{1.0};
        rescode = MSK_putbaraij(task_, num_linear_constraint + lower_index,
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

MSKrescodee MosekSolverProgram::AddLinearCost(
    const Eigen::SparseVector<double>& linear_coeff,
    const VectorX<symbolic::Variable>& linear_vars,
    const MathematicalProgram& prog) {
  MSKrescodee rescode = MSK_RES_OK;
  // The cost linear_coeff' * linear_vars could be written as
  // c' * x + ∑ᵢ <C̅ᵢ, X̅ᵢ>, where X̅ᵢ is the i'th matrix variable stored
  // inside Mosek, x are the non-matrix variables in Mosek. Mosek API
  // requires adding the cost for matrix and non-matrix variables separately.
  int num_bar_var = 0;
  rescode = MSK_getnumbarvar(task_, &num_bar_var);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  std::vector<std::vector<Eigen::Triplet<double>>> C_bar_lower_triplets(
      num_bar_var);
  for (Eigen::SparseVector<double>::InnerIterator it(linear_coeff); it; ++it) {
    const symbolic::Variable& var = linear_vars(it.row());
    const int var_index = prog.FindDecisionVariableIndex(var);
    auto it1 = decision_variable_to_mosek_nonmatrix_variable().find(var_index);
    if (it1 != decision_variable_to_mosek_nonmatrix_variable().end()) {
      // This variable is a non-matrix variable.
      rescode =
          MSK_putcj(task_, static_cast<MSKint32t>(it1->second), it.value());
      if (rescode != MSK_RES_OK) {
        return rescode;
      }
    } else {
      // Aggregate the matrix C̅ᵢ
      auto it2 = decision_variable_to_mosek_matrix_variable().find(var_index);
      DRAKE_DEMAND(it2 != decision_variable_to_mosek_matrix_variable().end());
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
      rescode = MSK_getdimbarvarj(task_, i, &matrix_rows);
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
          task_, matrix_rows, Ci_bar_lower_rows.size(),
          Ci_bar_lower_rows.data(), Ci_bar_lower_cols.data(),
          Ci_bar_lower_values.data(), &Ci_bar_index);
      if (rescode != MSK_RES_OK) {
        return rescode;
      }
      // Now add the cost <C̅ᵢ, X̅ᵢ>
      const MSKrealt weight{1.0};
      rescode = MSK_putbarcj(task_, i, 1, &Ci_bar_index, &weight);
    }
  }
  return rescode;
}

MSKrescodee MosekSolverProgram::AddQuadraticCostAsLinearCost(
    const Eigen::SparseMatrix<double>& Q_lower,
    const VectorX<symbolic::Variable>& quadratic_vars,
    const MathematicalProgram& prog) {
  // We can add the quaratic cost 0.5 * quadratic_vars' * Q * quadratic_vars as
  // a linear cost with a rotated Lorentz cone constraint. To do so, we
  // introduce a slack variable s, with the rotated Lorentz cone constraint
  // 2s >= | L * quadratic_vars|²,
  // where L'L = Q. We then minimize s.
  MSKrescodee rescode = MSK_RES_OK;
  // Unfortunately the sparse Cholesky decomposition in Eigen requires GPL
  // license, which is incompatible with Drake's license, so we convert the
  // sparse Q_lower matrix to a dense matrix, and use the dense Cholesky
  // decomposition instead.
  const Eigen::MatrixXd L = math::DecomposePSDmatrixIntoXtransposeTimesX(
      Eigen::MatrixXd(Q_lower), std::numeric_limits<double>::epsilon());
  MSKint32t num_mosek_vars = 0;
  rescode = MSK_getnumvar(task_, &num_mosek_vars);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  // Add a new variable s.
  rescode = MSK_appendvars(task_, 1);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  const MSKint32t s_index = num_mosek_vars;
  // Put bound on s as s >= 0.
  rescode = MSK_putvarbound(task_, s_index, MSK_BK_FR, 0, MSK_INFINITY);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  // Add the constraint that
  // [  1] = [0 0] * [s] + [1]
  // [  s] = [1 0]   [x]   [0]
  // [L*x]   [0 L]         [0]
  // is in the rotated Lorentz cone, where we denote x = quadratic_vars.
  // First add the affine expression
  // [0 0] * [s] + [1]
  // [1 0] * [x]   [0]
  // [0 L]         [0]
  // L_bar = [0]
  //         [0]
  //         [L]
  Eigen::MatrixXd L_bar = Eigen::MatrixXd::Zero(L.rows() + 2, L.cols());
  L_bar.bottomRows(L.rows()) = L;
  const Eigen::SparseMatrix<double> L_bar_sparse = L_bar.sparseView();
  // s_coeff = [0;1;...;0]
  Eigen::SparseMatrix<double> s_coeff(L.rows() + 2, 1);
  std::array<Eigen::Triplet<double>, 1> s_coeff_triplets;
  s_coeff_triplets[0] = Eigen::Triplet<double>(1, 0, 1);
  s_coeff.setFromTriplets(s_coeff_triplets.begin(), s_coeff_triplets.end());
  MSKint64t acc_index;
  // g = [1;0;0;...;0]
  Eigen::VectorXd g = Eigen::VectorXd::Zero(L.rows() + 2);
  g(0) = 1;
  std::vector<MSKint32t> slack_vars(1);
  slack_vars[0] = s_index;
  rescode =
      this->AddAffineConeConstraint(prog, L_bar_sparse, s_coeff, quadratic_vars,
                                    slack_vars, g, MSK_CT_RQUAD, &acc_index);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  // Now add the linear cost on s
  rescode = MSK_putcj(task_, s_index, 1.);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  return rescode;
}

MSKrescodee MosekSolverProgram::AddQuadraticCost(
    const Eigen::SparseMatrix<double>& Q_quadratic_vars,
    const VectorX<symbolic::Variable>& quadratic_vars,
    const MathematicalProgram& prog) {
  MSKrescodee rescode = MSK_RES_OK;
  // Add the quadratic cost 0.5 * quadratic_vars' * Q_quadratic_vars *
  // quadratic_vars to Mosek. Q_lower_triplets correspond to the matrix that
  // multiplies with all the non-matrix decision variables in Mosek, not with
  // `quadratic_vars`. We need to map each variable in `quadratic_vars` to its
  // index in Mosek non-matrix variables.
  std::vector<int> var_indices(quadratic_vars.rows());
  for (int i = 0; i < quadratic_vars.rows(); ++i) {
    var_indices[i] = decision_variable_to_mosek_nonmatrix_variable().at(
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
  const int xDim = decision_variable_to_mosek_nonmatrix_variable().size();
  ConvertTripletsToVectors(Q_lower_triplets, xDim, xDim, &qrow, &qcol, &qval);
  const int Q_nnz = static_cast<int>(qrow.size());
  rescode = MSK_putqobj(task_, Q_nnz, qrow.data(), qcol.data(), qval.data());
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  return rescode;
}

MSKrescodee MosekSolverProgram::AddCosts(const MathematicalProgram& prog) {
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
  rescode = AddLinearCost(linear_coeff, linear_vars, prog);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  if (!prog.quadratic_costs().empty()) {
    if (prog.lorentz_cone_constraints().empty() &&
        prog.rotated_lorentz_cone_constraints().empty() &&
        prog.linear_matrix_inequality_constraints().empty() &&
        prog.positive_semidefinite_constraints().empty() &&
        prog.exponential_cone_constraints().empty()) {
      rescode = AddQuadraticCost(Q_lower, quadratic_vars, prog);

      if (rescode != MSK_RES_OK) {
        return rescode;
      }
    } else {
      rescode = AddQuadraticCostAsLinearCost(Q_lower, quadratic_vars, prog);
      if (rescode != MSK_RES_OK) {
        return rescode;
      }
    }
  }

  // Provide constant / fixed cost.
  MSK_putcfix(task_, constant_cost);

  return rescode;
}

MSKrescodee MosekSolverProgram::SpecifyVariableType(
    const MathematicalProgram& prog, bool* with_integer_or_binary_variables) {
  MSKrescodee rescode = MSK_RES_OK;
  for (const auto& decision_variable_mosek_variable :
       decision_variable_to_mosek_nonmatrix_variable()) {
    const int mosek_variable_index = decision_variable_mosek_variable.second;
    switch (prog.decision_variable(decision_variable_mosek_variable.first)
                .get_type()) {
      case MathematicalProgram::VarType::INTEGER: {
        rescode = MSK_putvartype(task_, mosek_variable_index, MSK_VAR_TYPE_INT);
        if (rescode != MSK_RES_OK) {
          return rescode;
        }
        *with_integer_or_binary_variables = true;
        break;
      }
      case MathematicalProgram::VarType::BINARY: {
        *with_integer_or_binary_variables = true;
        rescode = MSK_putvartype(task_, mosek_variable_index, MSK_VAR_TYPE_INT);
        if (rescode != MSK_RES_OK) {
          return rescode;
        }
        double xi_lb = NAN;
        double xi_ub = NAN;
        MSKboundkeye bound_key;
        rescode = MSK_getvarbound(task_, mosek_variable_index, &bound_key,
                                  &xi_lb, &xi_ub);
        if (rescode != MSK_RES_OK) {
          return rescode;
        }
        xi_lb = std::max(0.0, xi_lb);
        xi_ub = std::min(1.0, xi_ub);
        rescode = MSK_putvarbound(task_, mosek_variable_index, MSK_BK_RA, xi_lb,
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
       decision_variable_to_mosek_matrix_variable()) {
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

MapDecisionVariableToMosekVariable::MapDecisionVariableToMosekVariable(
    const MathematicalProgram& prog) {
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
        auto it = decision_variable_to_mosek_matrix_variable.find(
            decision_variable_index);
        if (it == decision_variable_to_mosek_matrix_variable.end()) {
          // This variable has not been registered as a mosek matrix variable
          // before.
          decision_variable_to_mosek_matrix_variable.emplace_hint(
              it, decision_variable_index, matrix_variable_entry);
        } else {
          // This variable has been registered as a mosek matrix variable
          // already. This matrix variable entry will be registered into
          // matrix_variable_entries_for_same_decision_variable.
          auto it_same_decision_variable =
              matrix_variable_entries_for_same_decision_variable.find(
                  decision_variable_index);
          if (it_same_decision_variable !=
              matrix_variable_entries_for_same_decision_variable.end()) {
            it_same_decision_variable->second.push_back(matrix_variable_entry);
          } else {
            matrix_variable_entries_for_same_decision_variable.emplace_hint(
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
    if (decision_variable_to_mosek_matrix_variable.count(i) == 0) {
      decision_variable_to_mosek_nonmatrix_variable.emplace(
          i, nonmatrix_variable_count++);
    }
  }
  DRAKE_DEMAND(
      static_cast<int>(decision_variable_to_mosek_matrix_variable.size() +
                       decision_variable_to_mosek_nonmatrix_variable.size()) ==
      prog.num_vars());
}

MSKrescodee MosekSolverProgram::
    AddEqualityConstraintBetweenMatrixVariablesForSameDecisionVariable() {
  MSKrescodee rescode{MSK_RES_OK};
  for (const auto& pair :
       matrix_variable_entries_for_same_decision_variable()) {
    int num_mosek_constraint;
    rescode = MSK_getnumcon(task_, &num_mosek_constraint);
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
    const auto& matrix_variable_entries = pair.second;
    const int num_matrix_variable_entries =
        static_cast<int>(matrix_variable_entries.size());
    DRAKE_ASSERT(num_matrix_variable_entries >= 2);
    rescode = MSK_appendcons(task_, num_matrix_variable_entries - 1);
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
            linear_constraint_index, matrix_variable_entries[0], 1.0);
        if (rescode != MSK_RES_OK) {
          return rescode;
        }
        rescode = AddScalarTimesMatrixVariableEntryToMosek(
            linear_constraint_index, matrix_variable_entries[i], -1.0);
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
            matrix_variable_entries[0], &(E_indices[0]));
        if (rescode != MSK_RES_OK) {
          return rescode;
        }
        rescode = AddMatrixVariableEntryCoefficientMatrixIfNonExistent(
            matrix_variable_entries[i], &(E_indices[1]));
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

        rescode = MSK_putbaraij(task_, linear_constraint_index,
                                matrix_variable_entries[0].bar_matrix_index(),
                                2, E_indices.data(), weights.data());
        if (rescode != MSK_RES_OK) {
          return rescode;
        }
      }
      rescode = SetMosekLinearConstraintBound(
          linear_constraint_index, 0, 0, LinearConstraintBoundType::kEquality);
      if (rescode != MSK_RES_OK) {
        return rescode;
      }
    }
  }
  return rescode;
}

MSKrescodee MosekSolverProgram::SetPositiveSemidefiniteConstraintDualSolution(
    const MathematicalProgram& prog,
    const std::unordered_map<Binding<PositiveSemidefiniteConstraint>,
                             MSKint32t>& psd_barvar_indices,
    MSKsoltypee whichsol, MathematicalProgramResult* result) const {
  MSKrescodee rescode = MSK_RES_OK;
  for (const auto& psd_constraint : prog.positive_semidefinite_constraints()) {
    // Get the bar index of the Mosek matrix var.
    const auto it = psd_barvar_indices.find(psd_constraint);
    if (it == psd_barvar_indices.end()) {
      throw std::runtime_error(
          "SetPositiveSemidefiniteConstraintDualSolution: this positive "
          "semidefinite constraint has not been "
          "registered in Mosek as a matrix variable. This should not happen, "
          "please post an issue on Drake: "
          "https://github.com/RobotLocomotion/drake/issues/new.");
    }
    const auto bar_index = it->second;
    // barsj stores the lower triangular values of the psd matrix (as the dual
    // solution).
    std::vector<MSKrealt> barsj(
        psd_constraint.evaluator()->matrix_rows() *
        (psd_constraint.evaluator()->matrix_rows() + 1) / 2);
    rescode = MSK_getbarsj(task_, whichsol, bar_index, barsj.data());
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
    // Copy barsj to dual_lower. We don't use barsj directly since MSKrealt
    // might be a different data type from double.
    Eigen::VectorXd dual_lower(barsj.size());
    for (int i = 0; i < dual_lower.rows(); ++i) {
      dual_lower(i) = barsj[i];
    }
    result->set_dual_solution(psd_constraint, dual_lower);
  }
  return rescode;
}

namespace {
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

// This function is used to print information for each iteration to the console,
// it will show PRSTATUS, PFEAS, DFEAS, etc. For more information, check out
// https://docs.mosek.com/10.0/capi/solver-io.html. This printstr is copied
// directly from https://docs.mosek.com/10.0/capi/solver-io.html#stream-logging.
void MSKAPI printstr(void*, const char str[]) {
  printf("%s", str);
}

}  // namespace

MSKrescodee MosekSolverProgram::UpdateOptions(
    const SolverOptions& merged_options, const SolverId mosek_id,
    bool* print_to_console, std::string* print_file_name,
    std::optional<std::string>* msk_writedata) {
  MSKrescodee rescode{MSK_RES_OK};
  for (const auto& double_options : merged_options.GetOptionsDouble(mosek_id)) {
    if (rescode == MSK_RES_OK) {
      rescode = MSK_putnadouparam(task_, double_options.first.c_str(),
                                  double_options.second);
      ThrowForInvalidOption(rescode, double_options.first,
                            double_options.second);
    }
  }
  for (const auto& int_options : merged_options.GetOptionsInt(mosek_id)) {
    if (rescode == MSK_RES_OK) {
      rescode = MSK_putnaintparam(task_, int_options.first.c_str(),
                                  int_options.second);
      ThrowForInvalidOption(rescode, int_options.first, int_options.second);
    }
  }
  for (const auto& str_options : merged_options.GetOptionsStr(mosek_id)) {
    if (rescode == MSK_RES_OK) {
      if (str_options.first == "writedata") {
        if (str_options.second != "") {
          msk_writedata->emplace(str_options.second);
        }
      } else {
        rescode = MSK_putnastrparam(task_, str_options.first.c_str(),
                                    str_options.second.c_str());
        ThrowForInvalidOption(rescode, str_options.first, str_options.second);
      }
    }
  }
  // log file.
  *print_to_console = merged_options.get_print_to_console();
  *print_file_name = merged_options.get_print_file_name();
  // Refer to https://docs.mosek.com/10.0/capi/solver-io.html#stream-logging
  // for Mosek stream logging.
  // First we check if the user wants to print to both the console and the file.
  // If true, throw an error BEFORE we create the log file through
  // MSK_linkfiletotaskstream. Otherwise we might create the log file but cannot
  // close it.
  if (*print_to_console && !print_file_name->empty()) {
    throw std::runtime_error(
        "MosekSolver::Solve(): cannot print to both the console and the log "
        "file.");
  } else if (*print_to_console) {
    if (rescode == MSK_RES_OK) {
      rescode =
          MSK_linkfunctotaskstream(task_, MSK_STREAM_LOG, nullptr, printstr);
    }
  } else if (!print_file_name->empty()) {
    if (rescode == MSK_RES_OK) {
      rescode = MSK_linkfiletotaskstream(task_, MSK_STREAM_LOG,
                                         print_file_name->c_str(), 0);
    }
  }

  return rescode;
}

MSKrescodee MosekSolverProgram::SetDualSolution(
    MSKsoltypee which_sol, const MathematicalProgram& prog,
    const std::unordered_map<
        Binding<BoundingBoxConstraint>,
        std::pair<ConstraintDualIndices, ConstraintDualIndices>>&
        bb_con_dual_indices,
    const DualMap<LinearConstraint>& linear_con_dual_indices,
    const DualMap<LinearEqualityConstraint>& lin_eq_con_dual_indices,
    const std::unordered_map<Binding<LorentzConeConstraint>, MSKint64t>&
        lorentz_cone_acc_indices,
    const std::unordered_map<Binding<RotatedLorentzConeConstraint>, MSKint64t>&
        rotated_lorentz_cone_acc_indices,
    const std::unordered_map<Binding<ExponentialConeConstraint>, MSKint64t>&
        exp_cone_acc_indices,
    const std::unordered_map<Binding<PositiveSemidefiniteConstraint>,
                             MSKint32t>& psd_barvar_indices,
    MathematicalProgramResult* result) const {
  // TODO(hongkai.dai): support other types of constraints, like linear
  // constraint, second order cone constraint, etc.
  MSKrescodee rescode{MSK_RES_OK};
  if (which_sol == MSK_SOL_ITG) {
    // Mosek cannot return dual solution if the solution type is MSK_SOL_ITG
    // (which stands for mixed integer optimizer), see
    // https://docs.mosek.com/10.0/capi/accessing-solution.html#available-solutions
    return rescode;
  }
  int num_mosek_vars{0};
  rescode = MSK_getnumvar(task_, &num_mosek_vars);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  // Mosek dual variables for variable lower bounds (slx) and upper bounds
  // (sux). Refer to
  // https://docs.mosek.com/10.0/capi/alphabetic-functionalities.html#mosek.task.getsolution
  // for more explanation.
  std::vector<MSKrealt> slx(num_mosek_vars);
  std::vector<MSKrealt> sux(num_mosek_vars);
  rescode = MSK_getslx(task_, which_sol, slx.data());
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  rescode = MSK_getsux(task_, which_sol, sux.data());
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  int num_linear_constraints{0};
  rescode = MSK_getnumcon(task_, &num_linear_constraints);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  // Mosek dual variables for linear constraints lower bounds (slc) and upper
  // bounds (suc). Refer to
  // https://docs.mosek.com/10.0/capi/alphabetic-functionalities.html#mosek.task.getsolution
  // for more explanation.
  std::vector<MSKrealt> slc(num_linear_constraints);
  std::vector<MSKrealt> suc(num_linear_constraints);
  rescode = MSK_getslc(task_, which_sol, slc.data());
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  rescode = MSK_getsuc(task_, which_sol, suc.data());
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
  SetLinearConstraintDualSolution(prog.linear_equality_constraints(), slc, suc,
                                  lin_eq_con_dual_indices, result);
  // Set the duals for the nonlinear conic constraint.
  // Mosek provides the dual solution for nonlinear conic constraints only if
  // the program is solved through interior point approach.
  if (which_sol == MSK_SOL_ITR) {
    std::vector<MSKrealt> snx(num_mosek_vars);
    rescode = MSK_getsnx(task_, which_sol, snx.data());
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
    rescode = SetAffineConeConstraintDualSolution(
        prog.lorentz_cone_constraints(), task_, which_sol,
        lorentz_cone_acc_indices, result);
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
    rescode = SetAffineConeConstraintDualSolution(
        prog.rotated_lorentz_cone_constraints(), task_, which_sol,
        rotated_lorentz_cone_acc_indices, result);
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
    rescode = SetAffineConeConstraintDualSolution(
        prog.exponential_cone_constraints(), task_, which_sol,
        exp_cone_acc_indices, result);
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
  }
  rescode = SetPositiveSemidefiniteConstraintDualSolution(
      prog, psd_barvar_indices, which_sol, result);
  if (rescode != MSK_RES_OK) {
    return rescode;
  }
  return rescode;
}

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

}  // namespace internal
}  // namespace solvers
}  // namespace drake
