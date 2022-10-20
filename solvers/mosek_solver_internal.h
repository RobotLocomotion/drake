#pragma once

// For external users, please do not include this header file. It only exists so
// that we can expose the internals to mosek_solver_internal_test.cc

#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <mosek.h>

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"

namespace drake {
namespace solvers {
namespace internal {
// Mosek treats psd matrix variables in a special manner.
// Check https://docs.mosek.com/10.0/capi/tutorial-sdo-shared.html for more
// details. To summarize, Mosek stores a positive semidefinite (psd) matrix
// variable as a "bar var" (as called in Mosek's API, for example
// https://docs.mosek.com/10.0/capi/tutorial-sdo-shared.html). Inside Mosek, it
// accesses each of the psd matrix variable with a unique ID. Moreover, the
// Mosek user cannot access the entries of the psd matrix variable individually;
// instead, the user can only access the matrix X̅ as a whole. To impose
// linear constraints on psd matrix variable X̅ entries, the user must specify a
// "coefficient matrix" A̅ that multiplies X̅ (using the matrix inner product) to
// yield the linear constraint lower ≤ <A̅, X̅> ≤ upper. For example, to impose
// the constraint X̅(0, 0) + X̅(1, 0) = 1, where X̅ is a 2 x 2 psd matrix, Mosek
// requires the user to write it as <A̅, X̅> = 1, where
// A̅ = ⌈1   0.5⌉
//     ⌊0.5   0⌋
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
  static size_t get_next_id();
  MSKint64t bar_matrix_index_;
  MSKint32t row_index_;
  MSKint32t col_index_;
  int num_matrix_rows_;
  Id id_;
};

// Mosek stores dual variable in different categories, called slc, suc, slx, sux
// and snx. Refer to
// https://docs.mosek.com/10.0/capi/alphabetic-functionalities.html#mosek.task.getsolution
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

template <typename ConstraintType>
using DualMap =
    std::unordered_map<Binding<ConstraintType>, ConstraintDualIndices>;

enum class LinearConstraintBoundType {
  kEquality,
  kInequality,
};

// Mosek treats matrix variables (variables in the psd matrix) in a special
// manner, while MathematicalProgram doesn't. Hence we need to pick out the
// variables in prog.positive_semidefinite_constraint(), record how they will
// be stored in Mosek, and also how the remaining non-matrix variable will be
// stored in Mosek. Note that we only loop through
// PositiveSemidefiniteConstraint, not LinearMatrixInequalityConstraint.
struct MapDecisionVariableToMosekVariable {
 public:
  explicit MapDecisionVariableToMosekVariable(const MathematicalProgram& prog);

  std::unordered_map<int, MatrixVariableEntry>
      decision_variable_to_mosek_matrix_variable{};
  std::unordered_map<int, int> decision_variable_to_mosek_nonmatrix_variable{};
  // Multiple entries in Mosek matrix variables could correspond to the same
  // decision variable. We will need to add linear equality constraints to
  // equate these entries.
  std::unordered_map<int, std::vector<MatrixVariableEntry>>
      matrix_variable_entries_for_same_decision_variable{};
};

// MosekSolverProgram is a temporary object used by our MosekSolver
// implementation that is created (and destroyed) once per Solve() operation. It
// provides individual, program-specific helper functions to translate a
// MathematicalProgram into Mosek's API. We've separated it from
// MosekSolver::DoSolve both to make it easier to understand and to unit test
// each function one by one.
class MosekSolverProgram {
 public:
  MosekSolverProgram(const MathematicalProgram& prog, MSKenv_t env);

  ~MosekSolverProgram();

  // If a matrix variable entry X̅(m, n) appears in a cost or a constraint
  // (except the psd constraint), then we need a matrix Eₘₙ stored inside Mosek,
  // such that <Eₘₙ, X̅> = X̅(m, n). In this function we add the symmetric matrix
  // Eₘₙ into Mosek, and record the index of Eₘₙ in Mosek.
  // @param[out] E_idx The index of Eₘₙ for @p matrix_variable_entry.
  MSKrescodee AddMatrixVariableEntryCoefficientMatrixIfNonExistent(
      const MatrixVariableEntry& matrix_variable_entry, MSKint64t* E_index);

  // Add the product c * X̅(i, j) to a constraint.
  // This function should be called only if that Mosek matrix variable X̅ appear
  // only once in this constraint. Otherwise we should call
  // AddLinearConstraintToMosek, which first collects all the
  // entries X̅(i, j) belonging to this matrix variable X̅ in this constraint, and
  // then forms a matrix A, such that <A, X̅> contains the weighted sum of all
  // entries of X̅ in this constraint.
  MSKrescodee AddScalarTimesMatrixVariableEntryToMosek(
      MSKint32t constraint_index,
      const MatrixVariableEntry& matrix_variable_entry, MSKrealt scalar);

  // Determine the sense of each constraint. The sense can be equality
  // constraint, less than, greater than, or bounded on both side.
  MSKrescodee SetMosekLinearConstraintBound(
      int linear_constraint_index, double lower, double upper,
      LinearConstraintBoundType bound_type);

  // Add the linear constraint lower <= A * decision_vars + B * slack_vars <=
  // upper.
  MSKrescodee AddLinearConstraintToMosek(
      const MathematicalProgram& prog, const Eigen::SparseMatrix<double>& A,
      const Eigen::SparseMatrix<double>& B, const Eigen::VectorXd& lower,
      const Eigen::VectorXd& upper,
      const VectorX<symbolic::Variable>& decision_vars,
      const std::vector<MSKint32t>& slack_vars_mosek_indices,
      LinearConstraintBoundType bound_type);

  // Convert the expression A * decicion_vars + B * slack_vars to Mosek affine
  // expression format
  // ∑ⱼ fᵢⱼ xⱼ + ∑ⱼ<F̅ᵢⱼ, X̅ⱼ>
  // Please refer to
  // https://docs.mosek.com/latest/capi/tutorial-acc-optimizer.html for an
  // introduction on Mosek affine expression, and
  // https://docs.mosek.com/latest/capi/tutorial-sdo-shared.html for the affine
  // expression with matrix variables.
  // F̅ᵢⱼ is a weighted sum of the symmetric matrix E stored inside Mosek.
  // bar_F[i][j] stores the indices of E and the weights of E.
  // namely F̅ᵢⱼ = ∑ₖbar_F[i][j][k].second* E[bar_F[i][j][k].first]
  MSKrescodee ParseLinearExpression(
      const solvers::MathematicalProgram& prog,
      const Eigen::SparseMatrix<double>& A,
      const Eigen::SparseMatrix<double>& B,
      const VectorX<symbolic::Variable>& decision_vars,
      const std::vector<MSKint32t>& slack_vars_mosek_indices,
      std::vector<MSKint32t>* F_subi, std::vector<MSKint32t>* F_subj,
      std::vector<MSKrealt>* F_valij,
      std::vector<std::unordered_map<
          MSKint64t, std::pair<std::vector<MSKint64t>, std::vector<MSKrealt>>>>*
          bar_F);

  // Add LinearConstraints and LinearEqualityConstraints to the Mosek task.
  // @param[out] dual_indices maps each linear constraint to its dual variable
  // indices.
  template <typename C>
  MSKrescodee AddLinearConstraintsFromBindings(
      const std::vector<Binding<C>>& constraint_list,
      LinearConstraintBoundType bound_type, const MathematicalProgram& prog,
      std::unordered_map<Binding<C>, ConstraintDualIndices>* dual_indices);

  // @param[out] lin_eq_con_dual_indices maps each linear equality constraint to
  // its dual variable indices.
  MSKrescodee AddLinearConstraints(
      const MathematicalProgram& prog,
      std::unordered_map<Binding<LinearConstraint>, ConstraintDualIndices>*
          linear_con_dual_indices,
      std::unordered_map<Binding<LinearEqualityConstraint>,
                         ConstraintDualIndices>* lin_eq_con_dual_indices);

  // Add the bounds on the decision variables in @p prog. Note that if a
  // decision variable in positive definite matrix has a bound, we need to add
  // new linear constraint to Mosek to bound that variable.
  // @param[out] dual_indices Map each bounding box constraint to its dual
  // variable indices.
  MSKrescodee AddBoundingBoxConstraints(
      const MathematicalProgram& prog,
      std::unordered_map<Binding<BoundingBoxConstraint>,
                         std::pair<ConstraintDualIndices,
                                   ConstraintDualIndices>>* dual_indices);

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
   * @param[out] dual_indices Maps each conic constraint to its dual variable
   * indices.
   */
  template <typename C>
  MSKrescodee AddConeConstraints(
      const MathematicalProgram& prog,
      const std::vector<Binding<C>>& cone_constraints,
      std::unordered_map<Binding<C>, ConstraintDualIndices>* dual_indices);

  // @param[out] psd_barvar_indices maps each psd constraint to Mosek matrix
  // variable
  MSKrescodee AddPositiveSemidefiniteConstraints(
      const MathematicalProgram& prog,
      std::unordered_map<Binding<PositiveSemidefiniteConstraint>, MSKint32t>*
          psd_barvar_indices);

  MSKrescodee AddLinearMatrixInequalityConstraint(
      const MathematicalProgram& prog);

  MSKrescodee AddLinearCost(const Eigen::SparseVector<double>& linear_coeff,
                            const VectorX<symbolic::Variable>& linear_vars,
                            const MathematicalProgram& prog);

  MSKrescodee AddQuadraticCostAsLinearCost(
      const Eigen::SparseMatrix<double>& Q_lower,
      const VectorX<symbolic::Variable>& quadratic_vars,
      const MathematicalProgram& prog);

  MSKrescodee AddQuadraticCost(
      const Eigen::SparseMatrix<double>& Q_quadratic_vars,
      const VectorX<symbolic::Variable>& quadratic_vars,
      const MathematicalProgram& prog);

  MSKrescodee AddCosts(const MathematicalProgram& prog);

  // @param[out] with_integer_or_binary_variables True if the program has
  // integer or binary variables.
  MSKrescodee SpecifyVariableType(const MathematicalProgram& prog,
                                  bool* with_integer_or_binary_variables);

  // Some entries in Mosek matrix variables might correspond to the same
  // decision variable in MathematicalProgram. Add the equality constraint
  // between these matrix variable entries.
  MSKrescodee
  AddEqualityConstraintBetweenMatrixVariablesForSameDecisionVariable();

  // @param[in/out] result Update the dual solution for psd constraints in
  // result.
  MSKrescodee SetPositiveSemidefiniteConstraintDualSolution(
      const MathematicalProgram& prog,
      const std::unordered_map<Binding<PositiveSemidefiniteConstraint>,
                               MSKint32t>& psd_barvar_indices,
      MSKsoltypee whichsol, MathematicalProgramResult* result) const;

  // @param[in/out] result Update the dual solution in result.
  MSKrescodee SetDualSolution(
      MSKsoltypee which_sol, const MathematicalProgram& prog,
      const std::unordered_map<
          Binding<BoundingBoxConstraint>,
          std::pair<ConstraintDualIndices, ConstraintDualIndices>>&
          bb_con_dual_indices,
      const DualMap<LinearConstraint>& linear_con_dual_indices,
      const DualMap<LinearEqualityConstraint>& lin_eq_con_dual_indices,
      const DualMap<LorentzConeConstraint>& lorentz_cone_dual_indices,
      const DualMap<RotatedLorentzConeConstraint>&
          rotated_lorentz_cone_dual_indices,
      const DualMap<ExponentialConeConstraint>& exp_cone_dual_indices,
      const std::unordered_map<Binding<PositiveSemidefiniteConstraint>,
                               MSKint32t>& psd_barvar_indices,
      MathematicalProgramResult* result) const;

  // @param[out] print_to_console Set to true if solver options requires
  // printing the log to the console.
  // @param[out] print_file_name Set to the name of the print file store in
  // solver options. If solver options doesn't store the print file name, then
  // set *print_file_name to an empty string.
  // @param[out] msk_writedata If solver options stores the file for writing
  // data, then put the file name to msk_writedata for later use.
  MSKrescodee UpdateOptions(const SolverOptions& solver_options,
                            SolverId mosek_id, bool* print_to_console,
                            std::string* print_file_name,
                            std::optional<std::string>* msk_writedata);

  MSKtask_t task() const { return task_; }

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
  const std::unordered_map<MatrixVariableEntry::Id, MSKint64t>&
  matrix_variable_entry_to_selection_matrix_id() const {
    return matrix_variable_entry_to_selection_matrix_id_;
  }

  const std::unordered_map<int, MatrixVariableEntry>&
  decision_variable_to_mosek_matrix_variable() const {
    return map_decision_var_to_mosek_var_
        .decision_variable_to_mosek_matrix_variable;
  }

  const std::unordered_map<int, int>&
  decision_variable_to_mosek_nonmatrix_variable() const {
    return map_decision_var_to_mosek_var_
        .decision_variable_to_mosek_nonmatrix_variable;
  }

  const std::unordered_map<int, std::vector<MatrixVariableEntry>>&
  matrix_variable_entries_for_same_decision_variable() const {
    return map_decision_var_to_mosek_var_
        .matrix_variable_entries_for_same_decision_variable;
  }

 private:
  MSKtask_t task_{nullptr};
  std::unordered_map<MatrixVariableEntry::Id, MSKint64t>
      matrix_variable_entry_to_selection_matrix_id_{};
  const MapDecisionVariableToMosekVariable map_decision_var_to_mosek_var_;
};

template <typename C>
MSKrescodee MosekSolverProgram::AddLinearConstraintsFromBindings(
    const std::vector<Binding<C>>& constraint_list,
    LinearConstraintBoundType bound_type, const MathematicalProgram& prog,
    std::unordered_map<Binding<C>, ConstraintDualIndices>* dual_indices) {
  MSKrescodee rescode{MSK_RES_OK};
  for (const auto& binding : constraint_list) {
    const auto& constraint = binding.evaluator();
    const Eigen::SparseMatrix<double>& A = constraint->get_sparse_A();
    const Eigen::VectorXd& lb = constraint->lower_bound();
    const Eigen::VectorXd& ub = constraint->upper_bound();
    Eigen::SparseMatrix<double> B_zero(A.rows(), 0);
    B_zero.setZero();
    int num_linear_constraints{-1};
    rescode = MSK_getnumcon(task_, &num_linear_constraints);
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
    rescode = AddLinearConstraintToMosek(prog, A, B_zero, lb, ub,
                                         binding.variables(), {}, bound_type);
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

template <typename C>
MSKrescodee MosekSolverProgram::AddConeConstraints(
    const MathematicalProgram& prog,
    const std::vector<Binding<C>>& cone_constraints,
    std::unordered_map<Binding<C>, ConstraintDualIndices>* dual_indices) {
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
    rescode = MSK_getnumvar(task_, &num_mosek_vars);
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
    rescode = MSK_appendvars(task_, num_z);
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
    std::vector<MSKint32t> new_var_indices(num_z);
    for (int i = 0; i < num_z; ++i) {
      new_var_indices[i] = num_mosek_vars + i;
      rescode = MSK_putvarbound(task_, new_var_indices[i], MSK_BK_FR,
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
        MSK_appendcone(task_, cone_type, 0.0, num_z, new_var_indices.data());
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
    // With this difference in rotated Lorentz cone, the first row of
    // constraint z = A * x + b needs special treatment. If using Lorentz cone
    // or exponential cone, Add the linear constraint
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
    rescode = AddLinearConstraintToMosek(prog, A_sparse, B_sparse, b, b,
                                         binding.variables(), z_mosek_indices,
                                         LinearConstraintBoundType::kEquality);
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
  }
  // Expect rescode == MSK_RES_OK.
  return rescode;
}

// @param slx Mosek dual variables for variable lower bound. See
// https://docs.mosek.com/10.0/capi/alphabetic-functionalities.html#mosek.task.getslx
// @param sux Mosek dual variables for variable upper bound. See
// https://docs.mosek.com/10.0/capi/alphabetic-functionalities.html#mosek.task.getsux
// @param slc Mosek dual variables for linear constraint lower bound. See
// https://docs.mosek.com/10.0/capi/alphabetic-functionalities.html#mosek.task.getslc
// @param suc Mosek dual variables for linear constraint upper bound. See
// https://docs.mosek.com/10.0/capi/alphabetic-functionalities.html#mosek.task.getsuc
void SetBoundingBoxDualSolution(
    const std::vector<Binding<BoundingBoxConstraint>>& constraints,
    const std::vector<MSKrealt>& slx, const std::vector<MSKrealt>& sux,
    const std::vector<MSKrealt>& slc, const std::vector<MSKrealt>& suc,
    const std::unordered_map<
        Binding<BoundingBoxConstraint>,
        std::pair<ConstraintDualIndices, ConstraintDualIndices>>&
        bb_con_dual_indices,
    MathematicalProgramResult* result);

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

}  // namespace internal
}  // namespace solvers
}  // namespace drake
