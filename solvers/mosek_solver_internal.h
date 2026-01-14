#pragma once

// For external users, please do not include this header file. It only exists so
// that we can expose the internals to mosek_solver_internal_test.cc

#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <mosek.h>

#include "drake/solvers/constraint.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"
#include "drake/solvers/specific_options.h"

namespace drake {
namespace solvers {
namespace internal {
// Mosek treats psd matrix variables in a special manner.
// Check https://docs.mosek.com/11.1/capi/tutorial-sdo-shared.html for more
// details. To summarize, Mosek stores a positive semidefinite (psd) matrix
// variable as a "bar var" (as called in Mosek's API, for example
// https://docs.mosek.com/11.1/capi/tutorial-sdo-shared.html). Inside Mosek, it
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
// https://docs.mosek.com/11.1/capi/alphabetic-functionalities.html#mosek.task.getsolution
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
  // https://docs.mosek.com/11.1/capi/tutorial-acc-optimizer.html for an
  // introduction on Mosek affine expression, and
  // https://docs.mosek.com/11.1/capi/tutorial-sdo-shared.html for the affine
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

  // Same as ParseLinearExpression, but assumes that each entry in
  // `decision_vars` is unique.
  MSKrescodee ParseLinearExpressionNoDuplication(
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
  // decision variable in positive definite matrix (with >= 2 rows) has a bound,
  // we need to add new linear constraint to Mosek to bound that variable.
  // @param[out] bbcon_dual_indices Map each bounding box constraint to its dual
  // variable indices.
  // @param[out] scalar_psd_dual_indices Map each 1x1
  // PositiveSemidefiniteConstraint to its dual variable index.
  MSKrescodee AddVariableBounds(
      const MathematicalProgram& prog,
      std::unordered_map<Binding<BoundingBoxConstraint>,
                         std::pair<ConstraintDualIndices,
                                   ConstraintDualIndices>>* bbcon_dual_indices,
      std::unordered_map<Binding<PositiveSemidefiniteConstraint>,
                         std::pair<ConstraintDualIndex, ConstraintDualIndex>>*
          scalar_psd_dual_indices);

  // Add the quadratic constraints in @p prog.
  // @param[out] dual_indices Map each quadratic constraint to its dual variable
  // index.
  MSKrescodee AddQuadraticConstraints(
      const MathematicalProgram& prog,
      std::unordered_map<Binding<QuadraticConstraint>, MSKint64t>*
          dual_indices);

  /**
   Adds the constraint that A * decision_vars + B * slack_vars + c is in an
   affine cone.
   @param[out] acc_index The index of the newly added affine cone.
   */
  MSKrescodee AddAffineConeConstraint(
      const MathematicalProgram& prog, const Eigen::SparseMatrix<double>& A,
      const Eigen::SparseMatrix<double>& B,
      const VectorX<symbolic::Variable>& decision_vars,
      const std::vector<MSKint32t>& slack_vars_mosek_indices,
      const Eigen::VectorXd& c, MSKdomaintypee cone_type, MSKint64t* acc_index);

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
   * @param[out] acc_indices Maps each conic constraint to its Affine Cone
   * Constraint indices.
   */
  template <typename C>
  MSKrescodee AddConeConstraints(
      const MathematicalProgram& prog,
      const std::vector<Binding<C>>& cone_constraints,
      std::unordered_map<Binding<C>, MSKint64t>* acc_indices);

  // Add positive semidefinite constraints to Mosek.
  // Normally (for psd matrix with > 2 rows), we use Mosek's "bar variable"
  // which is constrained to be positive semidefinite;
  // for a psd matrix with 1 row (namely a scalar >= 0), we just impose a lower
  // bound on that scalar (in AddVariableBounds).
  // @param[out] psd_barvar_indices maps each psd constraint to Mosek matrix
  // variable
  MSKrescodee AddPositiveSemidefiniteConstraints(
      const MathematicalProgram& prog,
      std::unordered_map<Binding<PositiveSemidefiniteConstraint>, MSKint32t>*
          psd_barvar_indices);

  // Adds positive semidefinite constraints on a 2x2 matrix to Mosek.
  // [x(0) x(1)] is psd
  // [x(1) x(2)]
  // is imposed as
  // [x(0), x(2), x(1)] in the rotated Lorentz cone.
  MSKrescodee Add2x2PositiveSemidefiniteConstraints(
      const MathematicalProgram& prog,
      std::unordered_map<Binding<PositiveSemidefiniteConstraint>, MSKint64t>*
          acc_indices);

  // Add linear matrix inequality (LMI) constraints as affine cone constraints
  // (acc), return the indices of the added affine cone constraints.
  // Note that when the LMI constraint is on a 1x1 matrix (a scalar), then we
  // impose it as a linear inequality constraint; when the LMI constraint is on
  // a 2x2 matrix, then we impose it as a rotated Lorentz cone constraint; when
  // the matrix size is >= 3, then we impose it as a PSD cone constraint.
  MSKrescodee AddLinearMatrixInequalityConstraint(
      const MathematicalProgram& prog,
      std::unordered_map<Binding<LinearMatrixInequalityConstraint>, MSKint64t>*
          lmi_acc_indices);

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

  // Adds the L2 norm cost min |C*x+d|₂ to Mosek.
  MSKrescodee AddL2NormCost(const Eigen::SparseMatrix<double>& C,
                            const Eigen::VectorXd& d,
                            const VectorX<symbolic::Variable>& x,
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
      const std::unordered_map<Binding<QuadraticConstraint>, MSKint64t>&
          quadratic_constraint_dual_indices,
      const std::unordered_map<Binding<LorentzConeConstraint>, MSKint64t>&
          lorentz_cone_acc_indices,
      const std::unordered_map<Binding<RotatedLorentzConeConstraint>,
                               MSKint64t>& rotated_lorentz_cone_acc_indices,
      const std::unordered_map<Binding<LinearMatrixInequalityConstraint>,
                               MSKint64t>& lmi_acc_indices,
      const std::unordered_map<Binding<ExponentialConeConstraint>, MSKint64t>&
          exp_cone_acc_indices,
      const std::unordered_map<Binding<PositiveSemidefiniteConstraint>,
                               MSKint32t>& psd_barvar_indices,
      const std::unordered_map<
          Binding<PositiveSemidefiniteConstraint>,
          std::pair<ConstraintDualIndex, ConstraintDualIndex>>&
          scalar_psd_dual_indices,
      const std::unordered_map<Binding<PositiveSemidefiniteConstraint>,
                               MSKint64t>&
          twobytwo_psd_constraint_cone_acc_indices,
      MathematicalProgramResult* result) const;

  // @param[in] options The options to copy into our task. It is mutable so
  // that we can mutate it for efficiency; the data may be invalid afterwards,
  // so the caller should not use it for anything after we return.
  // @param[out] is_printing Set to true iff solver is printing to console or
  // file, to indicate we want more details when possible.
  // @param[out] msk_writedata If solver options stores the file for writing
  // data, then put the file name to msk_writedata for later use.
  void UpdateOptions(internal::SpecificOptions* options, bool* is_printing,
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
    std::unordered_map<Binding<C>, MSKint64t>* acc_indices) {
  static_assert(std::is_same_v<C, LorentzConeConstraint> ||
                    std::is_same_v<C, RotatedLorentzConeConstraint> ||
                    std::is_same_v<C, ExponentialConeConstraint>,
                "Should be either Lorentz cone constraint, rotated Lorentz "
                "cone or exponential cone constraint");
  const bool is_rotated_cone = std::is_same_v<C, RotatedLorentzConeConstraint>;
  MSKrescodee rescode = MSK_RES_OK;
  for (auto const& binding : cone_constraints) {
    MSKint64t acc_index;
    if (is_rotated_cone) {
      // Drake's RotatedLorentzConeConstraint imposes z₀ * z₁ ≥ z₂² + z₃² + ...
      // zₙ₋₁² where z being an affine expression of x, while Mosek's rotated
      // quadratic cone imposes 2 * z₀ * z₁ ≥ z₂² + z₃² + ... zₙ₋₁² (notice the
      // factor of 2). Hence we will impose the constraint that  the vector
      // ⌈0.5*(A.row(0)*x + b(0))⌉
      // |     A.row(1)*x + b(1) |
      // |           ...         |
      // ⌊  A.row(n-1)*x + b(n-1)⌋
      // is in Mosek's rotated quadratic cone. Namely we multiply 0.5 to the
      // first row of A, b.
      Eigen::SparseMatrix<double> A = binding.evaluator()->A();
      for (int k = 0; k < A.outerSize(); ++k) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(A, k); it; ++it) {
          if (it.row() == 0) {
            it.valueRef() *= 0.5;
          }
        }
      }
      Eigen::VectorXd b = binding.evaluator()->b();
      b(0) *= 0.5;
      rescode = this->AddAffineConeConstraint(
          prog, A, Eigen::SparseMatrix<double>(A.rows(), 0),
          binding.variables(), {}, b, MSK_DOMAIN_RQUADRATIC_CONE, &acc_index);
      if (rescode != MSK_RES_OK) {
        return rescode;
      }
    } else {
      MSKdomaintypee cone_type;
      if (std::is_same_v<C, LorentzConeConstraint>) {
        cone_type = MSK_DOMAIN_QUADRATIC_CONE;
      } else if (std::is_same_v<C, ExponentialConeConstraint>) {
        cone_type = MSK_DOMAIN_PRIMAL_EXP_CONE;
      }
      rescode = this->AddAffineConeConstraint(
          prog, binding.evaluator()->A(),
          Eigen::SparseMatrix<double>(binding.evaluator()->A().rows(), 0),
          binding.variables(), {}, binding.evaluator()->b(), cone_type,
          &acc_index);
      if (rescode != MSK_RES_OK) {
        return rescode;
      }
    }
    acc_indices->emplace(binding, acc_index);
  }
  return rescode;
}

// @param slx Mosek dual variables for variable lower bound. See
// https://docs.mosek.com/11.1/capi/alphabetic-functionalities.html#mosek.task.getslx
// @param sux Mosek dual variables for variable upper bound. See
// https://docs.mosek.com/11.1/capi/alphabetic-functionalities.html#mosek.task.getsux
// @param slc Mosek dual variables for linear constraint lower bound. See
// https://docs.mosek.com/11.1/capi/alphabetic-functionalities.html#mosek.task.getslc
// @param suc Mosek dual variables for linear constraint upper bound. See
// https://docs.mosek.com/11.1/capi/alphabetic-functionalities.html#mosek.task.getsuc
void SetVariableBoundsDualSolution(
    const std::vector<Binding<BoundingBoxConstraint>>& bb_constraints,
    const std::vector<Binding<PositiveSemidefiniteConstraint>>& psd_constraints,
    const std::vector<MSKrealt>& slx, const std::vector<MSKrealt>& sux,
    const std::vector<MSKrealt>& slc, const std::vector<MSKrealt>& suc,
    const std::unordered_map<
        Binding<BoundingBoxConstraint>,
        std::pair<ConstraintDualIndices, ConstraintDualIndices>>&
        bb_con_dual_indices,
    const std::unordered_map<
        Binding<PositiveSemidefiniteConstraint>,
        std::pair<ConstraintDualIndex, ConstraintDualIndex>>&
        scalar_psd_con_dual_indices,
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

// @param slc Mosek dual variables for linear and quadratic constraint lower
// bound. See
// https://docs.mosek.com/11.1/capi/alphabetic-functionalities.html#mosek.task.getslc
// @param suc Mosek dual variables for linear and quadratic constraint upper
// bound. See
// https://docs.mosek.com/11.1/capi/alphabetic-functionalities.html#mosek.task.getsuc
void SetQuadraticConstraintDualSolution(
    const std::vector<Binding<QuadraticConstraint>>& constraints,
    const std::vector<MSKrealt>& slc, const std::vector<MSKrealt>& suc,
    const std::unordered_map<Binding<QuadraticConstraint>, MSKint64t>&
        quadratic_constraint_dual_indices,
    MathematicalProgramResult* result);

/*
 * Sets the dual solution for the affine cone constraints.
 * @param bindings The constraint for which the dual solution will be set.
 * @param task The Mosek task.
 * @param whichsol The solution type. See
 * https://docs.mosek.com/11.1/capi/constants.html#mosek.soltype
 * @param acc_indices Maps each constraint to the affine cone constraint
 * indices.
 * @param[out] result The dual solution in `result` will be set.
 */
template <typename C>
MSKrescodee SetAffineConeConstraintDualSolution(
    const std::vector<Binding<C>>& bindings, MSKtask_t task,
    MSKsoltypee whichsol,
    const std::unordered_map<Binding<C>, MSKint64t>& acc_indices,
    MathematicalProgramResult* result) {
  for (const auto& binding : bindings) {
    if constexpr (std::is_same_v<C, PositiveSemidefiniteConstraint>) {
      // For PositiveSemidefiniteConstraint, it is only treated as an affine
      // cone constraint when the matrix has size 2x2. For other cases they are
      // treated differently: for 1x1 matrix it is treated as a lower bound on
      // the scalar variable. For matrix with more than 2 rows it is treated as
      // a Mosek "bar" matrix.
      if (binding.evaluator()->matrix_rows() != 2) {
        continue;
      }
    }
    const MSKint64t acc_index = acc_indices.at(binding);
    MSKint64t acc_dim;
    auto rescode = MSK_getaccn(task, acc_index, &acc_dim);
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
    Eigen::VectorXd dual_sol = Eigen::VectorXd::Zero(acc_dim);
    rescode = MSK_getaccdoty(task, whichsol, acc_index, dual_sol.data());
    if (rescode != MSK_RES_OK) {
      return rescode;
    }
    if constexpr (std::is_same_v<C, RotatedLorentzConeConstraint>) {
      // Drake's rotated Lorentz cone constraint on z = Ax+b is
      // K_drake={ z | z₀z₁≥ z₂² + ... zₙ₋₁², z₀≥0, z₁≥0}
      // On the other hand, Mosek's rotated Lorentz cone constraint has a
      // multiplier of 2
      // K_mosek={ z | 2z₀z₁≥ z₂² + ... zₙ₋₁², z₀≥0, z₁≥0} Hence
      // we can write K_drake = C*K_mosek where C = diag([2, 1, ..., 1]). By
      // duality we know K_drake_dual = C⁻ᵀ*K_mosek_dual, where K_drake_dual is
      // the dual cone of K_drake, likewise K_mosek_dual is the dual cone of
      // K_mosek.
      dual_sol(0) *= 0.5;
    }
    if constexpr (std::is_same_v<C, LinearMatrixInequalityConstraint>) {
      const int matrix_rows = binding.evaluator()->matrix_rows();
      if (matrix_rows == 2) {
        // We impose the LMI constraint as a rotated Quadratic cone constraint
        // in Mosek. For a constraint that
        // [y0 y1]
        // [y1 y2]
        // is in psd, this is equivalent to
        // [y0/2, y2, y1] in Mosek's rotated quadratic cone.
        // If we denote the dual solution of the Mosek's rotated quadratic cone
        // as [z0, z2, z1], then by the duality theory, the complementary
        // slackness (namely the inner product between the primal and the dual)
        // should be the same for both the PSD cone and the rotated quadratic
        // cone, hence the dual solution for the PSD constraint is
        // [z0/2 z1/2]
        // [z1/2   z2]
        // We store the lower triangular part of the dual PSD matrix as the dual
        // solution, namely [z0/2, z1/2, z2]
        dual_sol(0) *= 0.5;
        const double tmp = dual_sol(1);
        dual_sol(1) = dual_sol(2) / 2;
        dual_sol(2) = tmp;
      } else {
        // The dual solution returned by Mosek is the lower triangular part of
        // the psd matrix, but the off-diagonal terms are scaled by sqrt(2). We
        // need to scale the off-diagonal terms back.
        int dual_sol_entry_count = 0;
        const double sqrt2 = std::sqrt(2);
        for (int j = 0; j < binding.evaluator()->matrix_rows(); ++j) {
          for (int i = j; i < binding.evaluator()->matrix_rows(); ++i) {
            if (i != j) {
              dual_sol(dual_sol_entry_count) /= sqrt2;
            }
            ++dual_sol_entry_count;
          }
        }
      }
    } else if constexpr (std::is_same_v<C, PositiveSemidefiniteConstraint>) {
      if (binding.evaluator()->matrix_rows() == 2) {
        // The PSD constraint on 2x2 matrix
        // [x(0) x(1)]
        // [x(1) x(2)]
        // is imposed as a rotated Lorentz cone
        // [x(0), x(2), x(1)] in rotated Lorentz cone.
        // Or equivalently [0.5 * x(0), x(2), x(1)] is in Mosek's rotated
        // Quadratic cone. So if the dual of Mosek's rotated Quadratic cone is
        // [y(0), y(1), y(2)], then the dual of the PSD cone is
        // [0.5*y(0)   0.5*y(2)]
        // [0.5*y(2)       y(1)]
        // (Because the complementarity slackness, which is the inner-product of
        // the primal and dual should be the same for both PSD cone constraint
        // and Mosek's rotated Quadratic cone constraint).
        // We return the lower-triangular part of the dual matrix as
        // [0.5*y(0), 0.5*y(2) ,y(1)]
        dual_sol(0) *= 0.5;
        const double tmp = dual_sol(1);
        dual_sol(1) = dual_sol(2) * 0.5;
        dual_sol(2) = tmp;
      }
    }
    result->set_dual_solution(binding, dual_sol);
  }
  return MSK_RES_OK;
}

}  // namespace internal
}  // namespace solvers
}  // namespace drake
