#pragma once

#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "drake/common/drake_copyable.h"
#include "drake/common/type_safe_index.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
namespace internal {
/**
 * X is a block diagonal matrix in SDPA format. EntryInX stores the information
 * of one entry in this block-diagonal matrix X.
 */
struct EntryInX {
  EntryInX(int block_index_in, int row_index_in_block_in,
           int column_index_in_block_in, int X_start_row_in)
      : block_index{block_index_in},
        row_index_in_block(row_index_in_block_in),
        column_index_in_block(column_index_in_block_in),
        X_start_row(X_start_row_in) {}

  // block_index is 0-indexed.
  int block_index;
  // The row and column indices of the entry in this block. Both row/column
  // indices are 0-indexed.
  int row_index_in_block;
  int column_index_in_block;
  // The starting row index of this block in X. This is 0-indexed.
  int X_start_row;
};

enum class BlockType {
  kMatrix,
  kDiagonal,
};

struct BlockInX {
  BlockInX(BlockType block_type_in, int num_rows_in)
      : block_type{block_type_in}, num_rows{num_rows_in} {}
  BlockType block_type;
  int num_rows;
};

/**
 * Refer to @ref map_decision_variable_to_sdpa
 * When the decision variable either (or both) finite lower or upper bound (with
 * the two bounds not equal), we need to record the sign of the coefficient
 * before y.
 */
enum class Sign {
  kPositive,
  kNegative,
};

/**
 * @anchor map_decision_variable_to_sdpa Map decision variable to SDPA
 * When MathematicalProgram formulates a semidefinite program (SDP), we can
 * convert MathematicalProgram to a standard format for SDP, namely the SDPA
 * format. Each of the variable x in MathematicalProgram might be mapped to a
 * variable in the SDPA free format, depending on the following conditions.
 * 1. If the variable x has no lower nor upper bound, it is mapped to a free
 *    variable in SDPA free format.
 * 2. If the variable x only has a finite lower bound, and an infinite upper
 *    bound, then we will replace x by lower + y, where y is a diagonal entry
 *    in X in SDPA free format.
 * 3. If the variable x only has a finite upper bound, and an infinite lower
 *    bound, then we will replace x by upper - y, where y is a diagonal entry
 *    in X in SDPA free format.
 * 4. If the variable x has both finite upper and lower bounds, and these bounds
 *    are not equal, then we replace x by lower + y1, and introduce another
 *    constraint y1 + y2 = upper - lower, where both y1 and y2 are diagonal
 *    entries in X in SDPA free format.
 * 5. If the variable x has equal lower and upper bound, then we replace x with
 *    the double value of lower bound.
 * A MathematicalProgram decision variable can be replaced by coeff_sign * y +
 * offset, where y is a diagonal entry in SDPA X matrix.
 */
struct DecisionVariableInSdpaX {
  DecisionVariableInSdpaX(Sign coeff_sign_m, double offset_m,
                          EntryInX entry_in_X_m)
      : coeff_sign{coeff_sign_m}, offset{offset_m}, entry_in_X{entry_in_X_m} {}

  DecisionVariableInSdpaX(Sign coeff_sign_m, double offset_m, int block_index,
                          int row_index_in_block, int col_index_in_block,
                          int block_start_row)
      : coeff_sign(coeff_sign_m),
        offset(offset_m),
        entry_in_X(block_index, row_index_in_block, col_index_in_block,
                   block_start_row) {}
  Sign coeff_sign;
  double offset;
  EntryInX entry_in_X;
};

/**
 * SDPA format with free variables.
 *
 *     max tr(C * X) + dᵀs
 *     s.t tr(Aᵢ*X) + bᵢᵀs = gᵢ
 *         X ≽ 0
 *         s is free.
 */
class SdpaFreeFormat {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SdpaFreeFormat)

  explicit SdpaFreeFormat(const MathematicalProgram& prog);

  ~SdpaFreeFormat();

  const std::vector<BlockInX>& X_blocks() const { return X_blocks_; }

  using FreeVariableIndex = TypeSafeIndex<class FreeVariableTag>;

  const std::vector<std::variant<DecisionVariableInSdpaX, FreeVariableIndex,
                                 double, std::nullptr_t>>&
  prog_var_in_sdpa() const {
    return prog_var_in_sdpa_;
  }

  const std::vector<std::vector<Eigen::Triplet<double>>>& A_triplets() const {
    return A_triplets_;
  }

  const std::vector<Eigen::Triplet<double>>& B_triplets() const {
    return B_triplets_;
  }

  /** The right-hand side of the linear equality constraints. */
  const Eigen::VectorXd& g() const { return g_; }

  int num_X_rows() const { return num_X_rows_; }

  int num_free_variables() const { return num_free_variables_; }

  const std::vector<Eigen::Triplet<double>>& C_triplets() const {
    return C_triplets_;
  }

  const std::vector<Eigen::Triplet<double>>& d_triplets() const {
    return d_triplets_;
  }

  const std::vector<Eigen::SparseMatrix<double>>& A() const { return A_; }

  const Eigen::SparseMatrix<double>& B() const { return B_; }

  const Eigen::SparseMatrix<double>& C() const { return C_; }

  const Eigen::SparseMatrix<double>& d() const { return d_; }

  /** The SDPA format doesn't include the constant term in the cost, but
   * MathematicalProgram does. We store the constant cost term here.
   */
  double constant_min_cost_term() const { return constant_min_cost_term_; }

 private:
  // Go through all the positive semidefinite constraint in @p prog, and
  // register the corresponding blocks in matrix X for the bound variables of
  // each PositiveSemidefiniteConstraint. It is possible for two
  // PositiveSemidefiniteConstraint bindings to have overlapping decision
  // variables, or for a single PositiveSemidefiniteConstraint to have duplicate
  // decision variables. We need to impose equality constraints on these entries
  // in X. We use entries_in_X_for_same_decision_variable to record these
  // entries in X, so that we can impose the equality constraints later.
  void DeclareXforPositiveSemidefiniteConstraints(
      const MathematicalProgram& prog,
      std::unordered_map<symbolic::Variable::Id, std::vector<EntryInX>>*
          entries_in_X_for_same_decision_variable);

  // Some entries in X correspond to the same decision variables. We need to add
  // the equality constraint on these entries.
  void AddEqualityConstraintOnXEntriesForSameDecisionVariable(
      const std::unordered_map<symbolic::Variable::Id, std::vector<EntryInX>>&
          entries_in_X_for_same_decision_variable);

  // Adds a linear equality constraint
  // coeff_prog_vars' * prog_vars + coeff_X_entries' * X_entries +
  // coeff_free_vars' * free_vars = rhs.
  // @param coeff_prog_vars The coefficients for program decision variables that
  // appear in X.
  // @param prog_vars_indices  The indices of the MathematicalProgram decision
  // variables in X that appear in this constraint.
  // @param coeff_X_entries The coefficients for @p X_entries.
  // @param X_entries The entries in X that show up in the linear equality
  // constraint, X_entries and prog_vars_indices should not overlap.
  // @param coeff_free_vars The coefficients of free variables.
  // @param free_vars_indices The indices of the free variables show up in this
  // constraint, these free variables are not the decision variables in the
  // MathematicalProgram.
  // @param rhs The right-hand side of the linear equality constraint.
  void AddLinearEqualityConstraint(
      const std::vector<double>& coeff_prog_vars,
      const std::vector<int>& prog_vars_indices,
      const std::vector<double>& coeff_X_entries,
      const std::vector<EntryInX>& X_entries,
      const std::vector<double>& coeff_free_vars,
      const std::vector<FreeVariableIndex>& free_vars_indices, double rhs);

  void RegisterMathematicalProgramDecisionVariables(
      const MathematicalProgram& prog);

  // Registers the program decision variable with index `variable_index` into
  // SDPA and adds lower and upper bounds on that variable.
  // This function should only be called within
  // RegisterMathematicalProgramDecisionVariables().
  // We might need to add more slack variables into the block diagonal matrix X,
  // so as to incorporate the bounds as equality constraints.
  // @param block_index The index of the block in X that this new variable
  // belongs to.
  // @param[in/out] new_X_var_count The number of variables in this block before
  // and after registering this decision variable.
  void RegisterSingleMathematicalProgramDecisionVariable(double lower_bound,
                                                         double upper_bound,
                                                         int variable_index,
                                                         int block_index,
                                                         int* new_X_var_count);

  // Add the bounds on a variable that has been registered.
  // This function should only be called within
  // RegisterMathematicalProgramDecisionVariables().
  // We might need to add more slack variables into the block diagonal matrix X,
  // so as to incorporate the bounds as equality constraints.
  // @param block_index The index of the block in X that the new slack variables
  // belongs to.
  // @param[in/out] new_X_var_count The number of variables in this block before
  // and after adding the variable bounds.
  void AddBoundsOnRegisteredDecisionVariable(double lower_bound,
                                             double upper_bound,
                                             int variable_index,
                                             int block_index,
                                             int* new_X_var_count);

  // Sum up all the linear costs in prog, store the result in SDPA free format.
  void AddLinearCostsFromProgram(const MathematicalProgram& prog);

  // Add both the linear constraints lower <= a'x <= upper and the linear
  // equality constraints a'x = rhs to SDPA free format. */
  void AddLinearConstraintsFromProgram(const MathematicalProgram& prog);

  template <typename Constraint>
  void AddLinearConstraintsHelper(
      const MathematicalProgram& prog,
      const Binding<Constraint>& linear_constraint, bool is_equality_constraint,
      int* linear_constraint_slack_entry_in_X_count);

  void AddLinearMatrixInequalityConstraints(const MathematicalProgram& prog);

  void AddLorentzConeConstraints(const MathematicalProgram& prog);

  void AddRotatedLorentzConeConstraints(const MathematicalProgram& prog);

  // Called at the end of the constructor.
  void Finalize();

  // X_blocks_ just stores the size and category of each block in the
  // block-diagonal matrix X.
  std::vector<BlockInX> X_blocks_;

  std::vector<Eigen::Triplet<double>> C_triplets_;
  std::vector<Eigen::Triplet<double>> d_triplets_;
  // gᵢ is the i-th entry of g.
  Eigen::VectorXd g_;
  // A_triplets_[i] describes the nonzero entries in Aᵢ
  std::vector<std::vector<Eigen::Triplet<double>>> A_triplets_;
  // bᵢ is the i-th column of B. B_triplets records the nonzero entries in B.
  std::vector<Eigen::Triplet<double>> B_triplets_;

  /**
   * Depending on the bounds and whether the variable appears in a PSD cone, a
   * MathematicalProgram decision variable can be either an entry in X, a free
   * variable, or a double constant in SDPA free format.
   * We use std::nullptr_t to indicate that this variable hasn't been registered
   * into SDPA free format yet.
   */
  std::vector<std::variant<DecisionVariableInSdpaX, FreeVariableIndex, double,
                      std::nullptr_t>>
      prog_var_in_sdpa_;

  int num_X_rows_{0};
  int num_free_variables_{0};
  double constant_min_cost_term_{0.0};

  std::vector<Eigen::SparseMatrix<double>> A_;
  Eigen::SparseMatrix<double> C_;
  Eigen::SparseMatrix<double> B_;
  Eigen::SparseMatrix<double> d_;
};
}  // namespace internal

/**
 * SDPA is a format to record an SDP problem
 *
 *     max tr(C*X)
 *     s.t tr(Aᵢ*X) = gᵢ
 *         X ≽ 0
 * or the dual of the problem
 *
 *     min gᵀy
 *     s.t ∑ᵢ yᵢAᵢ - C ≽ 0
 * where X is a symmetric block diagonal matrix.
 * The format is described in http://plato.asu.edu/ftp/sdpa_format.txt. Many
 * solvers, such as CSDP, DSDP, SDPA, sedumi and SDPT3, accept an SDPA format
 * file as the input.
 * This function reads a MathematicalProgram that can be formulated as above,
 * and write an SDPA file.
 * @param prog a program that contains an optimization program.
 * @param file_name The name of the file, note that the extension will be added
 * automatically.
 * @retval is_success. Returns true if we can generate the SDPA file. The
 * failure could be
 * 1. @p prog cannot be captured by the formulation above.
 * 2. @p prog cannot create a file with the given name, etc.
 */
bool GenerateSDPA(const MathematicalProgram& prog,
                  const std::string& file_name);
}  // namespace solvers
}  // namespace drake
