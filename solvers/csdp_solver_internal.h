#pragma once

// For external users, please do not include this header file. It only exists so
// that we can expose the internals to csdp_solver_internal_test.cc

#include <unordered_map>
#include <vector>

namespace csdp {
extern "C" {
// TODO(Jeremy.Nimmer): include this header as <csdp/declarations.h>
#include <declarations.h>
}  // extern C
}  // namespace csdp

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "drake/common/drake_copyable.h"
#include "drake/common/type_safe_index.h"
#include "drake/solvers/csdp_solver.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
namespace internal {
/**
 * X is a block diagonal matrix in CSDP. EntryInX stores the information of one
 * entry in this block-diagonal matrix X.
 */
struct EntryInX {
  EntryInX(int block_index_in, int row_index_in_block_in,
           int column_index_in_block_in, int X_start_row_in)
      : block_index{block_index_in},
        row_index_in_block(row_index_in_block_in),
        column_index_in_block(column_index_in_block_in),
        X_start_row(X_start_row_in) {}

  // block_index is 0-indexed.
  const int block_index;
  // The row and column indices of the entry in this block. Both row/column
  // indices are 0-indexed.
  const int row_index_in_block;
  const int column_index_in_block;
  // The starting row index of this block in X. This is 0-indexed.
  const int X_start_row;
};

struct BlockInX {
  BlockInX(csdp::blockcat blockcategory_in, int num_rows_in)
      : blockcategory{blockcategory_in}, num_rows{num_rows_in} {}
  csdp::blockcat blockcategory;
  int num_rows;
};

/**
 * SDPA format with free variables.
 * max tr(C * X) + dᵀs
 * s.t tr(Aᵢ*X) + bᵢᵀs = gᵢ
 *     X ≽ 0
 *     s is free.
 */
class SdpaFreeFormat {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SdpaFreeFormat)

  explicit SdpaFreeFormat(const MathematicalProgram& prog);

  ~SdpaFreeFormat();

  const std::vector<BlockInX>& X_blocks() const { return X_blocks_; }

  const std::unordered_map<int, EntryInX>& map_prog_var_index_to_entry_in_X()
      const {
    return map_prog_var_index_to_entry_in_X_;
  }

  using DecisionVariableIndex = TypeSafeIndex<class MathematicalProgramTag>;
  using FreeVariableIndex = TypeSafeIndex<class FreeVariableTag>;

  const std::unordered_map<DecisionVariableIndex, FreeVariableIndex>&
  map_prog_var_index_to_s_index() const {
    return map_prog_var_index_to_s_index_;
  }

  const std::vector<std::vector<Eigen::Triplet<double>>>& A_triplets() const {
    return A_triplets_;
  }

  const std::vector<Eigen::Triplet<double>>& B_triplets() const {
    return B_triplets_;
  }

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

  double constant_min_cost_term() const { return constant_min_cost_term_; }

  /**
   * If SdpaFreeFormat.num_free_variables() == 0 (i.e., it doesn't have free
   * variable s), then convert the problem to CSDP problem data format.
   */
  void GenerateCsdpProblemDataWithoutFreeVariables(
      csdp::blockmatrix* C, double** b,
      csdp::constraintmatrix** constraints) const;

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
  void AddEqualityConstraintOnXentriesForSameDecisionVariable(
      const std::unordered_map<symbolic::Variable::Id, std::vector<EntryInX>>&
          entries_in_X_for_same_decision_variable);

  void AddLinearEqualityConstraint(const std::vector<double>& a,
                                   const std::vector<EntryInX>& X_entries,
                                   const std::vector<double>& b,
                                   const std::vector<int>& s_indices,
                                   double rhs);

  void RegisterMathematicalProgramDecisionVariable(
      const MathematicalProgram& prog);

  /** Sum up all the linear costs in prog, store the result in SDPA free format.
   */
  void AddLinearCosts(const MathematicalProgram& prog);

  /** Add both the linear constraints lower <= a'x <= upper and the linear
   * equality constraints a'x = rhs to SDPA free format. */
  void AddLinearConstraints(const MathematicalProgram& prog);

  template <typename Constraint>
  void AddLinearConstraintsHelper(
      const MathematicalProgram& prog,
      const Binding<Constraint>& linear_constraint, bool is_equality_constraint,
      int* linear_constraint_slack_entry_in_X_count);

  void AddLinearMatrixInequalityConstraints(const MathematicalProgram& prog);

  void Finalize();

  // X_blocks_ just stores the size and category of each block in the
  // block-diagonal matrix X.
  std::vector<BlockInX> X_blocks_;

  std::vector<Eigen::Triplet<double>> C_triplets_;
  std::vector<Eigen::Triplet<double>> d_triplets_;
  // gᵢ is the i-th entry of g.
  Eigen::VectorXd g_;
  // A_triplets_[i] is the nonzero entries in Aᵢ
  std::vector<std::vector<Eigen::Triplet<double>>> A_triplets_;
  // bᵢ is the i-th column of B. B_triplets records the nonzero entries in B.
  std::vector<Eigen::Triplet<double>> B_triplets_;
  // Some of the decision variables in MathematicalProgram will be in X.
  // map_prog_var_index_to_X_entry_ maps the index of a variable in
  // MathematicalProgram to its index in X.
  std::unordered_map<int, EntryInX> map_prog_var_index_to_entry_in_X_;

  // Some of the decision variables in MathematicalProgram will be in s.
  // map_prog_var_index_to_s_index_ maps the index of a variable in
  // MathematicalProgram to its index in s.
  std::unordered_map<DecisionVariableIndex, FreeVariableIndex>
      map_prog_var_index_to_s_index_;

  int num_X_rows_{0};
  int num_free_variables_{0};
  // The SDPA format doesn't include the constant term in the cost, but
  // MathematicalProgram does. We store the constant cost term here.
  double constant_min_cost_term_{0.0};

  std::vector<Eigen::SparseMatrix<double>> A_;
  Eigen::SparseMatrix<double> C_;
  Eigen::SparseMatrix<double> B_;
  Eigen::SparseMatrix<double> d_;
};

/**
 * For a problem
 * max tr(C * X)
 * s.t tr(Ai * X) = rhs_i
 *     X ≽ 0
 * We are given C, Ai, rhs in the Eigen sparse matrix format, convert these data
 * to CSDP format.
 */
void ConvertSparseMatrixFormattToCsdpProblemData(
    const std::vector<BlockInX>& X_blocks, const Eigen::SparseMatrix<double>& C,
    const std::vector<Eigen::SparseMatrix<double>> A,
    const Eigen::VectorXd& rhs, csdp::blockmatrix* C_csdp, double** rhs_csdp,
    csdp::constraintmatrix** constraints);

void ConvertCsdpBlockMatrixtoEigen(const csdp::blockmatrix& X_csdp,
                                   Eigen::SparseMatrix<double>* X);

void FreeCsdpProblemData(int num_constraints, csdp::blockmatrix C_csdp,
                         double* rhs_csdp, csdp::constraintmatrix* constraints);

}  // namespace internal
}  // namespace solvers
}  // namespace drake
