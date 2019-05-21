#include "drake/solvers/csdp_solver.h"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/SparseQR>

#include "drake/solvers/csdp_solver_internal.h"

namespace drake {
namespace solvers {
namespace internal {

const double kInf = std::numeric_limits<double>::infinity();

SdpaFreeFormat::~SdpaFreeFormat() {}

void SdpaFreeFormat::DeclareXforPositiveSemidefiniteConstraints(
    const MathematicalProgram& prog,
    std::unordered_map<symbolic::Variable::Id, std::vector<EntryInX>>*
        entries_in_X_for_same_decision_variable) {
  int num_blocks = 0;
  for (const auto& binding : prog.positive_semidefinite_constraints()) {
    // The bound variables are the column-stacked vector of the psd matrix. We
    // only record the upper-diagonal entries in the psd matrix.
    int psd_matrix_variable_index = 0;
    const int matrix_rows = binding.evaluator()->matrix_rows();
    X_blocks_.emplace_back(csdp::MATRIX, matrix_rows);
    for (int j = 0; j < matrix_rows; ++j) {
      for (int i = 0; i <= j; ++i) {
        const symbolic::Variable& psd_ij_var =
            binding.variables()(psd_matrix_variable_index++);
        const int psd_ij_var_index = prog.FindDecisionVariableIndex(psd_ij_var);
        const auto it1 =
            map_prog_var_index_to_entry_in_X_.find(psd_ij_var_index);
        const EntryInX psd_ij_entry_in_X(num_blocks, i, j, num_X_rows_);
        if (it1 == map_prog_var_index_to_entry_in_X_.end()) {
          // This variable has not been registered into X. Now register this
          // variable, by adding it to map_prog_var_index_to_entry_in_X_.
          map_prog_var_index_to_entry_in_X_.emplace_hint(it1, psd_ij_var_index,
                                                         psd_ij_entry_in_X);
        } else {
          // This variable has been registered into X. We need to add the
          // equality constraint between all X entries corresponding to this
          // variable.

          // First find if there exists equality constraint on this variable
          // already.
          const auto it2 = entries_in_X_for_same_decision_variable->find(
              psd_ij_var.get_id());
          if (it2 == entries_in_X_for_same_decision_variable->end()) {
            // There does not exist equality constraint on this variable yet.
            entries_in_X_for_same_decision_variable->emplace_hint(
                it2, psd_ij_var.get_id(),
                std::vector<EntryInX>({it1->second, psd_ij_entry_in_X}));
          } else {
            // We found equality constraint on this variable, append the vector
            // containing all X entries that correspond to this variable.
            it2->second.push_back(psd_ij_entry_in_X);
          }
        }
      }
      psd_matrix_variable_index += matrix_rows - j - 1;
    }
    num_blocks++;
    num_X_rows_ += matrix_rows;
  }
}

void SdpaFreeFormat::AddLinearEqualityConstraint(
    const std::vector<double>& a, const std::vector<EntryInX>& X_entries,
    const std::vector<double>& b, const std::vector<int>& s_indices,
    double rhs) {
  DRAKE_ASSERT(a.size() == X_entries.size());
  DRAKE_ASSERT(b.size() == s_indices.size());
  const int constraint_index = static_cast<int>(A_triplets_.size());
  std::vector<Eigen::Triplet<double>> Ai_triplets;
  // If the entries are off-diagonal entries of X, then it needs two
  // coefficients in Ai, for both the upper and lower diagonal part.
  Ai_triplets.reserve(static_cast<int>(a.size()) * 2);
  for (int i = 0; i < static_cast<int>(a.size()); ++i) {
    if (X_entries[i].row_index_in_block == X_entries[i].column_index_in_block) {
      // diagonal term in X.
      Ai_triplets.emplace_back(
          X_entries[i].X_start_row + X_entries[i].row_index_in_block,
          X_entries[i].X_start_row + X_entries[i].column_index_in_block, a[i]);
    } else {
      // off-diagnal term in X. Add two coefficients in Ai.
      Ai_triplets.emplace_back(
          X_entries[i].X_start_row + X_entries[i].row_index_in_block,
          X_entries[i].X_start_row + X_entries[i].column_index_in_block,
          a[i] / 2);
      Ai_triplets.emplace_back(
          X_entries[i].X_start_row + X_entries[i].column_index_in_block,
          X_entries[i].X_start_row + X_entries[i].row_index_in_block, a[i] / 2);
    }
  }
  A_triplets_.push_back(Ai_triplets);
  if (!b.empty()) {
    for (int i = 0; i < static_cast<int>(b.size()); ++i) {
      B_triplets_.emplace_back(constraint_index, s_indices[i], b[i]);
    }
  }
  g_.conservativeResize(g_.rows() + 1);
  g_(constraint_index) = rhs;
}

void SdpaFreeFormat::AddEqualityConstraintOnXentriesForSameDecisionVariable(
    const std::unordered_map<symbolic::Variable::Id, std::vector<EntryInX>>&
        entries_in_X_for_same_decision_variable) {
  for (const auto& item : entries_in_X_for_same_decision_variable) {
    const auto& Xentries = item.second;
    DRAKE_ASSERT(Xentries.size() >= 2);
    const std::vector<double> a{{1, -1}};
    for (int i = 1; i < static_cast<int>(Xentries.size()); ++i) {
      AddLinearEqualityConstraint(a, {Xentries[0], Xentries[i]}, {}, {}, 0.0);
    }
  }
}

void SdpaFreeFormat::RegisterMathematicalProgramDecisionVariable(
    const MathematicalProgram& prog) {
  // Go through all the decision variables in @p prog. If the decision variable
  // has not been registered in SDPA, then register it now. We take the
  // following steps:
  // 1. If the variable has a lower bound of 0, then add this variable to X.
  // 2. If the variable has a lower bound of -inf, then add this variable to s.
  // 3. If the variable x has a lower bound that is neither 0 nor -inf, then add
  // a linear equality constraint x - y = lower_bound, add x to s, and y to X.
  // 4. If the variable has an upper bound of +inf, do nothing.
  // 5. If the variable x has a finite upper bound, then add a linear equality
  // constraint x + y = upper_bound, add x to s and y to X.

  // We first need to store the variable lower and upper bounds to two vectors.
  Eigen::VectorXd lower_bound =
      Eigen::VectorXd::Constant(prog.num_vars(), -kInf);
  Eigen::VectorXd upper_bound =
      Eigen::VectorXd::Constant(prog.num_vars(), kInf);
  for (const auto& bounding_box : prog.bounding_box_constraints()) {
    for (int i = 0; i < bounding_box.variables().rows(); ++i) {
      const int variable_index =
          prog.FindDecisionVariableIndex(bounding_box.variables()(i));
      lower_bound(variable_index) =
          std::max(lower_bound(variable_index),
                   bounding_box.evaluator()->lower_bound()(i));
      upper_bound(variable_index) =
          std::min(upper_bound(variable_index),
                   bounding_box.evaluator()->upper_bound()(i));
    }
  }

  // Now go through each of the decision variables in prog.

  const int block_index = static_cast<int>(X_blocks_.size());
  int new_X_var_count = 0;
  for (int i = 0; i < prog.num_vars(); ++i) {
    auto it_x = map_prog_var_index_to_entry_in_X_.find(i);
    bool is_x_in_X = it_x != map_prog_var_index_to_entry_in_X_.end();
    if (!is_x_in_X) {
      if (lower_bound(i) == 0) {
        // Register this variable to X.
        const EntryInX x_entry_in_X(block_index, new_X_var_count,
                                    new_X_var_count, num_X_rows_);
        new_X_var_count++;
        map_prog_var_index_to_entry_in_X_.emplace_hint(it_x, i, x_entry_in_X);
        it_x = map_prog_var_index_to_entry_in_X_.find(i);
        is_x_in_X = true;
      }
    }
    if (!is_x_in_X) {
      const int x_index_in_s = num_free_variables_;
      num_free_variables_++;
      map_prog_var_index_to_s_index_.emplace(DecisionVariableIndex(i),
                                             FreeVariableIndex(x_index_in_s));
      if (lower_bound(i) != upper_bound(i)) {
        if (!std::isinf(lower_bound(i))) {
          // lower bound is neither 0 nor -inf. Add the linear equality x - y =
          // lower_bound, add x to free variable s, and y to X.
          const EntryInX y_entry_in_X(block_index, new_X_var_count,
                                      new_X_var_count, num_X_rows_);
          new_X_var_count++;
          AddLinearEqualityConstraint({-1.0}, {y_entry_in_X}, {1.0},
                                      {x_index_in_s}, lower_bound(i));
        }

        if (!std::isinf(upper_bound(i))) {
          // If this variable x has a finite upper bound, then introduce a
          // slack variable y, y >= 0, with the constraint x + y = upper_bound.
          const EntryInX y_entry_in_X(block_index, new_X_var_count,
                                      new_X_var_count, num_X_rows_);
          new_X_var_count++;
          AddLinearEqualityConstraint({1.0}, {y_entry_in_X}, {1.0},
                                      {x_index_in_s}, upper_bound(i));
        }
      } else {
        // lower_bound(i) == upper_bound(i). We will add a single linear
        // constraint x(i) == lower_bound(i).
        AddLinearEqualityConstraint({}, {}, {1.0}, {x_index_in_s},
                                    lower_bound(i));
      }
    } else {
      if (lower_bound(i) != upper_bound(i)) {
        if (!std::isinf(lower_bound(i)) && lower_bound(i) != 0) {
          // lower bound is neither 0 nor -inf. Add the linear equality x - y =
          // lower_bound, add x to free variable s, and y to X.
          const EntryInX y_entry_in_X(block_index, new_X_var_count,
                                      new_X_var_count, num_X_rows_);
          new_X_var_count++;
          AddLinearEqualityConstraint({1.0, -1.0}, {it_x->second, y_entry_in_X},
                                      {}, {}, lower_bound(i));
        }
        if (!std::isinf(upper_bound(i))) {
          // If this variable x has a finite upper bound, then introduce a
          // slack variable y, y >= 0, with the constraint x + y = upper_bound.
          const EntryInX y_entry_in_X(block_index, new_X_var_count,
                                      new_X_var_count, num_X_rows_);
          new_X_var_count++;
          AddLinearEqualityConstraint({1.0, 1.0}, {it_x->second, y_entry_in_X},
                                      {}, {}, upper_bound(i));
        }
      } else {
        // lower_bound(i) == upper_bound(i). We will add a single linear
        // constraint x(i) == lower_bound(i).
        AddLinearEqualityConstraint({1}, {it_x->second}, {}, {},
                                    lower_bound(i));
      }
    }
  }
  if (new_X_var_count > 0) {
    X_blocks_.emplace_back(csdp::DIAG, new_X_var_count);
    num_X_rows_ += new_X_var_count;
  }
}

void SdpaFreeFormat::AddLinearCosts(const MathematicalProgram& prog) {
  for (const auto& linear_cost : prog.linear_costs()) {
    for (int i = 0; i < linear_cost.variables().rows(); ++i) {
      // The negation sign is because in CSDP format, the objective is to
      // maximize the cost, while in MathematicalProgram, the objective is to
      // minimize the cost.
      const double coeff = -linear_cost.evaluator()->a()(i);
      if (coeff != 0) {
        // Only adds the cost if the cost coefficient is non-zero.
        const int var_index =
            prog.FindDecisionVariableIndex(linear_cost.variables()(i));
        const auto it_X = map_prog_var_index_to_entry_in_X_.find(var_index);
        if (it_X != map_prog_var_index_to_entry_in_X_.end()) {
          if (it_X->second.row_index_in_block ==
              it_X->second.column_index_in_block) {
            // This is a diagonal entry in X.
            C_triplets_.emplace_back(
                it_X->second.X_start_row + it_X->second.row_index_in_block,
                it_X->second.X_start_row + it_X->second.column_index_in_block,
                coeff);
          } else {
            C_triplets_.emplace_back(
                it_X->second.X_start_row + it_X->second.row_index_in_block,
                it_X->second.X_start_row + it_X->second.column_index_in_block,
                coeff / 2);
            C_triplets_.emplace_back(
                it_X->second.X_start_row + it_X->second.column_index_in_block,
                it_X->second.X_start_row + it_X->second.row_index_in_block,
                coeff / 2);
          }
        } else {
          // The decision variable is not an entry in X. Then it should be an
          // entry in s.
          const auto it_s = map_prog_var_index_to_s_index_.find(
              DecisionVariableIndex(var_index));
          DRAKE_THROW_UNLESS(it_s != map_prog_var_index_to_s_index_.end());
          d_triplets_.emplace_back(it_s->second, 0, coeff);
        }
      }
    }
    constant_min_cost_term_ += linear_cost.evaluator()->b();
  }
}

template <typename Constraint>
void SdpaFreeFormat::AddLinearConstraintsHelper(
    const MathematicalProgram& prog,
    const Binding<Constraint>& linear_constraint, bool is_equality_constraint,
    int* linear_constraint_slack_entry_in_X_count) {
  const std::vector<int> var_indices =
      prog.FindDecisionVariableIndices(linear_constraint.variables());
  std::vector<decltype(map_prog_var_index_to_entry_in_X_)::const_iterator>
      var_it_in_X(var_indices.size());
  std::vector<decltype(map_prog_var_index_to_s_index_)::const_iterator>
      var_it_in_s(var_indices.size());
  for (int i = 0; i < static_cast<int>(var_indices.size()); ++i) {
    var_it_in_X[i] = map_prog_var_index_to_entry_in_X_.find(
        DecisionVariableIndex(var_indices[i]));
    var_it_in_s[i] = var_it_in_X[i] == map_prog_var_index_to_entry_in_X_.end()
                         ? map_prog_var_index_to_s_index_.find(
                               DecisionVariableIndex(var_indices[i]))
                         : map_prog_var_index_to_s_index_.end();
  }
  // Go through each row of the constraint.
  for (int i = 0; i < linear_constraint.evaluator()->num_constraints(); ++i) {
    const bool does_lower_equal_upper_in_this_row =
        is_equality_constraint ||
        linear_constraint.evaluator()->lower_bound()(i) ==
            linear_constraint.evaluator()->upper_bound()(i);
    std::vector<double> a, b;
    // If this row's lower and upper bound are the same, then we only need to
    // append one row to SDPA format, namely tr(Ai * X) + bi' * s = rhs,
    // Otherwise, we need to append two rows to SDPA format
    // tr(Ai_lower * X_lower) + bi' * s = lower_bound
    // tr(Ai_upper * X_upper) + bi' * s = upper_bound
    // and append two diagonal entries into X as slack variables.
    b.reserve(var_indices.size());
    // add 1 because we might need a slack variable for inequality constraint.
    a.reserve(var_indices.size() + 1);
    std::vector<EntryInX> X_entries;
    X_entries.reserve(var_indices.size() + 1);
    std::vector<int> s_indices;
    s_indices.reserve(var_indices.size());
    for (int j = 0; j < linear_constraint.variables().rows(); ++j) {
      if (linear_constraint.evaluator()->A()(i, j) != 0) {
        if (var_it_in_X[j] != map_prog_var_index_to_entry_in_X_.end()) {
          a.push_back(linear_constraint.evaluator()->A()(i, j));
          X_entries.push_back(var_it_in_X[j]->second);
        } else {
          b.push_back(linear_constraint.evaluator()->A()(i, j));
          s_indices.push_back(var_it_in_s[j]->second);
        }
      }
    }
    if (does_lower_equal_upper_in_this_row) {
      // Add the equality constraint.
      AddLinearEqualityConstraint(
          a, X_entries, b, s_indices,
          linear_constraint.evaluator()->lower_bound()(i));
    } else {
      // First add the constraint and slack variable for the lower bound.
      if (!std::isinf(linear_constraint.evaluator()->lower_bound()(i))) {
        a.push_back(-1);
        X_entries.emplace_back(static_cast<int>(X_blocks_.size()),
                               *linear_constraint_slack_entry_in_X_count,
                               *linear_constraint_slack_entry_in_X_count,
                               num_X_rows_);
        (*linear_constraint_slack_entry_in_X_count)++;
        AddLinearEqualityConstraint(
            a, X_entries, b, s_indices,
            linear_constraint.evaluator()->lower_bound()(i));
        a.pop_back();
        X_entries.pop_back();
      }
      // Now add the constraint and slack variable for the upper bound.
      if (!std::isinf(linear_constraint.evaluator()->upper_bound()(i))) {
        a.push_back(1);
        X_entries.emplace_back(static_cast<int>(X_blocks_.size()),
                               *linear_constraint_slack_entry_in_X_count,
                               *linear_constraint_slack_entry_in_X_count,
                               num_X_rows_);
        (*linear_constraint_slack_entry_in_X_count)++;
        AddLinearEqualityConstraint(
            a, X_entries, b, s_indices,
            linear_constraint.evaluator()->upper_bound()(i));
      }
    }
  }
}

void SdpaFreeFormat::AddLinearConstraints(const MathematicalProgram& prog) {
  int linear_constraint_slack_entry_in_X_count = 0;
  for (const auto& linear_eq_constraint : prog.linear_equality_constraints()) {
    AddLinearConstraintsHelper(prog, linear_eq_constraint, true,
                               &linear_constraint_slack_entry_in_X_count);
  }
  for (const auto& linear_constraint : prog.linear_constraints()) {
    AddLinearConstraintsHelper(prog, linear_constraint, false,
                               &linear_constraint_slack_entry_in_X_count);
  }
  if (linear_constraint_slack_entry_in_X_count > 0) {
    num_X_rows_ += linear_constraint_slack_entry_in_X_count;
    X_blocks_.emplace_back(csdp::DIAG,
                           linear_constraint_slack_entry_in_X_count);
  }
}

void SdpaFreeFormat::AddLinearMatrixInequalityConstraints(
    const MathematicalProgram& prog) {
  for (const auto& lmi_constraint :
       prog.linear_matrix_inequality_constraints()) {
    // Add the constraint that F1 * x1 + ... + Fn * xn - X_slack = -F0 and
    // X_slack is psd.
    const std::vector<int> var_indices =
        prog.FindDecisionVariableIndices(lmi_constraint.variables());
    std::vector<decltype(map_prog_var_index_to_entry_in_X_)::const_iterator>
        it_var_in_X(var_indices.size());
    std::vector<decltype(map_prog_var_index_to_s_index_)::const_iterator>
        it_var_in_s(var_indices.size());
    for (int i = 0; i < lmi_constraint.variables().rows(); ++i) {
      it_var_in_X[i] = map_prog_var_index_to_entry_in_X_.find(var_indices[i]);
      it_var_in_s[i] = it_var_in_X[i] == map_prog_var_index_to_entry_in_X_.end()
                           ? map_prog_var_index_to_s_index_.find(
                                 DecisionVariableIndex(var_indices[i]))
                           : map_prog_var_index_to_s_index_.end();
    }
    const std::vector<Eigen::MatrixXd>& F = lmi_constraint.evaluator()->F();
    for (int j = 0; j < lmi_constraint.evaluator()->matrix_rows(); ++j) {
      for (int i = 0; i <= j; ++i) {
        std::vector<double> a, b;
        std::vector<EntryInX> X_entries;
        std::vector<int> s_indices;
        a.reserve(F.size());
        b.reserve(F.size());
        X_entries.reserve(F.size());
        s_indices.reserve(F.size());
        for (int k = 1; k < static_cast<int>(F.size()); ++k) {
          if (F[k](i, j) != 0) {
            if (it_var_in_X[k - 1] != map_prog_var_index_to_entry_in_X_.end()) {
              a.push_back(F[k](i, j));
              X_entries.push_back(it_var_in_X[k - 1]->second);
            } else {
              b.push_back(F[k](i, j));
              s_indices.push_back(it_var_in_s[k - 1]->second);
            }
          }
        }
        a.push_back(-1);
        X_entries.emplace_back(static_cast<int>(X_blocks_.size()), i, j,
                               num_X_rows_);
        AddLinearEqualityConstraint(a, X_entries, b, s_indices, -F[0](i, j));
      }
    }

    X_blocks_.emplace_back(csdp::MATRIX,
                           lmi_constraint.evaluator()->matrix_rows());
    num_X_rows_ += lmi_constraint.evaluator()->matrix_rows();
  }
}

void SdpaFreeFormat::Finalize() {
  A_.reserve(A_triplets_.size());
  for (int i = 0; i < static_cast<int>(A_triplets_.size()); ++i) {
    A_.emplace_back(num_X_rows_, num_X_rows_);
    A_.back().setFromTriplets(A_triplets_[i].begin(), A_triplets_[i].end());
  }
  B_.resize(static_cast<int>(A_triplets_.size()), num_free_variables_);
  B_.setFromTriplets(B_triplets_.begin(), B_triplets_.end());
  C_.resize(num_X_rows_, num_X_rows_);
  C_.setFromTriplets(C_triplets_.begin(), C_triplets_.end());
  d_.resize(num_free_variables_, 1);
  d_.setFromTriplets(d_triplets_.begin(), d_triplets_.end());
}

SdpaFreeFormat::SdpaFreeFormat(const MathematicalProgram& prog) {
  std::unordered_map<symbolic::Variable::Id, std::vector<EntryInX>>
      entries_in_X_for_same_decision_variable;
  DeclareXforPositiveSemidefiniteConstraints(
      prog, &entries_in_X_for_same_decision_variable);

  AddEqualityConstraintOnXentriesForSameDecisionVariable(
      entries_in_X_for_same_decision_variable);

  RegisterMathematicalProgramDecisionVariable(prog);

  AddLinearCosts(prog);

  AddLinearConstraints(prog);

  AddLinearMatrixInequalityConstraints(prog);

  Finalize();
}

void ConvertSparseMatrixFormattToCsdpProblemData(
    const std::vector<BlockInX>& X_blocks, const Eigen::SparseMatrix<double>& C,
    const std::vector<Eigen::SparseMatrix<double>> A,
    const Eigen::VectorXd& rhs, csdp::blockmatrix* C_csdp, double** rhs_csdp,
    csdp::constraintmatrix** constraints) {
  const int num_X_rows = C.rows();
  DRAKE_ASSERT(C.cols() == num_X_rows);
  DRAKE_ASSERT(static_cast<int>(A.size()) == rhs.rows());

  // maps the row index in X to the block index. Both the row index and the
  // block index are 0-indexed.
  std::vector<int> X_row_to_block_index(num_X_rows);
  std::vector<int> block_start_rows(static_cast<int>(X_blocks.size()));
  int row_count = 0;
  for (int i = 0; i < static_cast<int>(X_blocks.size()); ++i) {
    block_start_rows[i] = row_count;
    for (int row = row_count; row < row_count + X_blocks[i].num_rows; ++row) {
      X_row_to_block_index[row] = i;
    }
    row_count += static_cast<int>(X_blocks[i].num_rows);
  }

  C_csdp->nblocks = static_cast<int>(X_blocks.size());
  C_csdp->blocks = static_cast<struct csdp::blockrec*>(
      /* We need to add 1 here because CSDP is uses Fortran 1-indexed, so the
         0'th block is wasted. */
      malloc((C_csdp->nblocks + 1) * sizeof(struct csdp::blockrec)));
  for (int m = 0; m < C_csdp->nblocks; ++m) {
    // CSDP uses Fortran index, so we need to add 1.
    C_csdp->blocks[m + 1].blockcategory = X_blocks[m].blockcategory;
    C_csdp->blocks[m + 1].blocksize = X_blocks[m].num_rows;
    if (X_blocks[m].blockcategory == csdp::MATRIX) {
      C_csdp->blocks[m + 1].data.mat = static_cast<double*>(
          malloc(X_blocks[m].num_rows * X_blocks[m].num_rows * sizeof(double)));
      for (int j = 0; j < X_blocks[m].num_rows; ++j) {
        // First fill in this column with 0, and then we will go through the
        // non-zero entries (stored inside  C) to set the value of the
        // corresponding entries in C_csdp.
        for (int i = 0; i < X_blocks[m].num_rows; ++i) {
          C_csdp->blocks[m + 1]
              .data.mat[ijtok(i + 1, j + 1, X_blocks[m].num_rows)] = 0;
        }
        for (Eigen::SparseMatrix<double>::InnerIterator it(
                 C, block_start_rows[m] + j);
             it; ++it) {
          C_csdp->blocks[m + 1]
              .data.mat[ijtok(it.row() - block_start_rows[m] + 1, j + 1,
                              X_blocks[m].num_rows)] = it.value();
        }
      }
    } else if (X_blocks[m].blockcategory == csdp::DIAG) {
      C_csdp->blocks[m + 1].data.vec = static_cast<double*>(
          // CSDP uses Fortran 1-index array, so the 0'th entry is wasted.
          malloc((X_blocks[m].num_rows + 1) * sizeof(double)));
      for (int j = 0; j < X_blocks[m].num_rows; ++j) {
        C_csdp->blocks[m + 1].data.vec[j + 1] = 0.0;
        for (Eigen::SparseMatrix<double>::InnerIterator it(
                 C, block_start_rows[m] + j);
             it; ++it) {
          DRAKE_ASSERT(it.row() == it.col());
          C_csdp->blocks[m + 1].data.vec[j + 1] = it.value();
        }
      }
    } else {
      throw std::runtime_error(
          "ConvertSparseMatrixFormattToCsdpProblemData() only support MATRIX "
          "or DIAG blocks.");
    }
  }

  // copy rhs
  *rhs_csdp = static_cast<double*>(malloc((rhs.rows() + 1) * sizeof(double)));
  for (int i = 0; i < rhs.rows(); ++i) {
    (*rhs_csdp)[i + 1] = rhs(i);
  }

  // Copy constraints.
  *constraints = static_cast<struct csdp::constraintmatrix*>(
      malloc((static_cast<int>(A.size()) + 1) *
             sizeof(struct csdp::constraintmatrix)));
  struct csdp::sparseblock* blockptr;
  for (int constraint_index = 0; constraint_index < static_cast<int>(A.size());
       ++constraint_index) {
    (*constraints)[constraint_index + 1].blocks = nullptr;
    // Start from the last block in the block-diagonal matrix
    // A[constraint_index], we add each block in the reverse order.
    for (int block_index = static_cast<int>(X_blocks.size() - 1);
         block_index >= 0; --block_index) {
      std::vector<Eigen::Triplet<double>> A_block_triplets;
      // CSDP only stores the non-zero entries in the upper-triangular part of
      // each block. Also the row and column indices in A_block_triplets are the
      // indices within THIS block, not the indices in the whole matrix
      // A[constraint_index].
      A_block_triplets.reserve((X_blocks[block_index].num_rows + 1) *
                               X_blocks[block_index].num_rows / 2);
      for (int col_index = block_start_rows[block_index];
           col_index <
           block_start_rows[block_index] + X_blocks[block_index].num_rows;
           ++col_index) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(A[constraint_index],
                                                           col_index);
             it; ++it) {
          if (it.row() > it.col()) {
            break;
          }
          A_block_triplets.emplace_back(
              it.row() - block_start_rows[block_index] + 1,
              it.col() - block_start_rows[block_index] + 1, it.value());
        }
      }
      if (!A_block_triplets.empty()) {
        blockptr = static_cast<struct csdp::sparseblock*>(
            malloc(sizeof(struct csdp::sparseblock)));
        // CSDP uses Fortran 1-indexed array.
        blockptr->blocknum = block_index + 1;
        blockptr->blocksize = X_blocks[block_index].num_rows;
        // CSDP uses Fortran 1-indexed array.
        blockptr->constraintnum = constraint_index + 1;
        blockptr->next = nullptr;
        blockptr->nextbyblock = nullptr;
        blockptr->entries = static_cast<double*>(malloc(
            (1 + static_cast<int>(A_block_triplets.size())) * sizeof(double)));
        blockptr->iindices = static_cast<int*>(malloc(
            (1 + static_cast<int>(A_block_triplets.size())) * sizeof(int)));
        blockptr->jindices = static_cast<int*>(malloc(
            (1 + static_cast<int>(A_block_triplets.size())) * sizeof(int)));
        blockptr->numentries = static_cast<int>(A_block_triplets.size());
        for (int i = 0; i < blockptr->numentries; ++i) {
          blockptr->iindices[i + 1] = A_block_triplets[i].row();
          blockptr->jindices[i + 1] = A_block_triplets[i].col();
          blockptr->entries[i + 1] = A_block_triplets[i].value();
        }
        // Insert this block into the linked list of
        // constraints[constraint_index + 1] blocks.
        blockptr->next = (*constraints)[constraint_index + 1].blocks;
        (*constraints)[constraint_index + 1].blocks = blockptr;
      }
    }
  }
}

void SdpaFreeFormat::GenerateCsdpProblemDataWithoutFreeVariables(
    csdp::blockmatrix* C_csdp, double** rhs_csdp,
    csdp::constraintmatrix** constraints) const {
  if (num_free_variables_ == 0) {
    Eigen::SparseMatrix<double> C(num_X_rows_, num_X_rows_);
    C.setFromTriplets(C_triplets_.begin(), C_triplets_.end());
    std::vector<Eigen::SparseMatrix<double>> A;
    A.reserve(A_triplets_.size());
    for (int i = 0; i < static_cast<int>(A_triplets_.size()); ++i) {
      A.emplace_back(num_X_rows_, num_X_rows_);
      A[static_cast<int>(A.size()) - 1].setFromTriplets(A_triplets_[i].begin(),
                                                        A_triplets_[i].end());
    }
    ConvertSparseMatrixFormattToCsdpProblemData(X_blocks_, C, A, g_, C_csdp,
                                                rhs_csdp, constraints);
  } else {
    throw std::runtime_error(
        "SdpaFreeFormat::GenerateCsdpProblemDataWithoutFreeVariables(): the "
        "formulation has free variables, you should't call this method.");
  }
}

void ConvertCsdpBlockMatrixtoEigen(const csdp::blockmatrix& X_csdp,
                                   Eigen::SparseMatrix<double>* X) {
  int num_X_nonzero_entries = 0;
  for (int i = 0; i < X_csdp.nblocks; ++i) {
    if (X_csdp.blocks[i + 1].blockcategory == csdp::MATRIX) {
      num_X_nonzero_entries +=
          X_csdp.blocks[i + 1].blocksize * X_csdp.blocks[i + 1].blocksize;
    } else if (X_csdp.blocks[i + 1].blockcategory == csdp::DIAG) {
      num_X_nonzero_entries += X_csdp.blocks[i + 1].blocksize;
    }
  }
  std::vector<Eigen::Triplet<double>> X_triplets;
  X_triplets.reserve(num_X_nonzero_entries);
  int X_row_count = 0;
  for (int k = 0; k < X_csdp.nblocks; ++k) {
    if (X_csdp.blocks[k + 1].blockcategory == csdp::MATRIX) {
      for (int i = 0; i < X_csdp.blocks[k + 1].blocksize; ++i) {
        for (int j = 0; j < X_csdp.blocks[k + 1].blocksize; ++j) {
          X_triplets.emplace_back(
              X_row_count + i, X_row_count + j,
              X_csdp.blocks[k + 1].data.mat[ijtok(
                  i + 1, j + 1, X_csdp.blocks[k + 1].blocksize)]);
        }
      }
    } else if (X_csdp.blocks[k + 1].blockcategory == csdp::DIAG) {
      for (int i = 0; i < X_csdp.blocks[k + 1].blocksize; ++i) {
        X_triplets.emplace_back(X_row_count + i, X_row_count + i,
                                X_csdp.blocks[k + 1].data.vec[i + 1]);
      }
    } else {
      throw std::runtime_error(
          "ConvertCsdpBlockMatrixtoEigen(): unknown block matrix type.");
    }
    X_row_count += X_csdp.blocks[k + 1].blocksize;
  }
  X->resize(X_row_count, X_row_count);
  X->setFromTriplets(X_triplets.begin(), X_triplets.end());
}

void SetCsdpSolverDetails(int csdp_ret, double pobj, double dobj, int y_size,
                          double* y, const csdp::blockmatrix& Z,
                          CsdpSolverDetails* solver_details) {
  solver_details->return_code = csdp_ret;
  solver_details->primal_objective = pobj;
  solver_details->dual_objective = dobj;
  solver_details->y_val.resize(y_size);
  for (int i = 0; i < y_size; ++i) {
    solver_details->y_val(i) = y[i + 1];
  }
  ConvertCsdpBlockMatrixtoEigen(Z, &(solver_details->Z_val));
}

SolutionResult ConvertCsdpReturnToSolutionResult(int csdp_ret) {
  switch (csdp_ret) {
    case 0:
    case 3:
      return SolutionResult::kSolutionFound;
    case 1:
      return SolutionResult::kInfeasibleConstraints;
    case 2:
      return SolutionResult::kDualInfeasible;
    case 4:
      return SolutionResult::kIterationLimit;
    default:
      return SolutionResult::kUnknownError;
  }
}

void SetProgramSolutionFromX(const SdpaFreeFormat& sdpa_free_format,
                             const csdp::blockmatrix& X,
                             Eigen::VectorXd* prog_sol) {
  for (const auto& item : sdpa_free_format.map_prog_var_index_to_entry_in_X()) {
    const int var_index = item.first;
    const EntryInX& X_entry = item.second;
    if (X.blocks[X_entry.block_index + 1].blockcategory == csdp::MATRIX) {
      (*prog_sol)(var_index) = X.blocks[X_entry.block_index + 1].data.mat[ijtok(
          X_entry.row_index_in_block + 1, X_entry.column_index_in_block + 1,
          X.blocks[X_entry.block_index + 1].blocksize)];
    } else if (X.blocks[X_entry.block_index + 1].blockcategory == csdp::DIAG) {
      (*prog_sol)(var_index) = X.blocks[X_entry.block_index + 1]
                                   .data.vec[X_entry.row_index_in_block + 1];
    } else {
      throw std::runtime_error(
          "SolveProgramWithNoFreeVariables(): unsupported block type.");
    }
  }
}

void SolveProgramWithNoFreeVariables(const MathematicalProgram& prog,
                                     const SdpaFreeFormat& sdpa_free_format,
                                     MathematicalProgramResult* result) {
  DRAKE_ASSERT(sdpa_free_format.num_free_variables() == 0);

  csdp::blockmatrix C;
  double* rhs;
  csdp::constraintmatrix* constraints{nullptr};
  sdpa_free_format.GenerateCsdpProblemDataWithoutFreeVariables(&C, &rhs,
                                                               &constraints);

  struct csdp::blockmatrix X, Z;
  double* y;
  csdp::initsoln(sdpa_free_format.num_X_rows(), sdpa_free_format.g().rows(), C,
                 rhs, constraints, &X, &y, &Z);
  double pobj, dobj;
  const int ret = csdp::easy_sdp(
      sdpa_free_format.num_X_rows(), sdpa_free_format.g().rows(), C, rhs,
      constraints, -sdpa_free_format.constant_min_cost_term(), &X, &y, &Z,
      &pobj, &dobj);

  // Set solver details.
  CsdpSolverDetails& solver_details =
      result->SetSolverDetailsType<CsdpSolverDetails>();
  SetCsdpSolverDetails(ret, pobj, dobj, sdpa_free_format.g().rows(), y, Z,
                       &solver_details);
  // Retrieve the information back to result
  // Since CSDP solves a mazimization problem max -cost, where "cost" is the
  // minimization cost in MathematicalProgram, we need to negate the cost.
  result->set_optimal_cost(-pobj);
  result->set_solution_result(ConvertCsdpReturnToSolutionResult(ret));
  Eigen::VectorXd prog_sol(prog.num_vars());
  SetProgramSolutionFromX(sdpa_free_format, X, &prog_sol);
  result->set_x_val(prog_sol);

  csdp::free_prob(sdpa_free_format.num_X_rows(), sdpa_free_format.g().rows(), C,
                  rhs, constraints, X, y, Z);
}

void FreeCsdpProblemData(int num_constraints, csdp::blockmatrix C_csdp,
                         double* rhs_csdp,
                         csdp::constraintmatrix* constraints) {
  // This function is copied from the source code in csdp/lib/freeprob.c
  free(rhs_csdp);
  csdp::free_mat(C_csdp);
  csdp::sparseblock* ptr;
  csdp::sparseblock* oldptr;
  if (constraints != nullptr) {
    for (int i = 1; i <= num_constraints; ++i) {
      ptr = constraints[i].blocks;
      while (ptr != nullptr) {
        free(ptr->entries);
        free(ptr->iindices);
        free(ptr->jindices);
        oldptr = ptr;
        ptr = ptr->next;
        free(oldptr);
      }
    }
    free(constraints);
  }
}
}  // namespace internal

void CsdpSolver::DoSolve(const MathematicalProgram& prog,
                         const Eigen::VectorXd&, const SolverOptions&,
                         MathematicalProgramResult* result) const {
  result->set_solver_id(CsdpSolver::id());
  const internal::SdpaFreeFormat sdpa_free_format(prog);
  if (sdpa_free_format.num_free_variables() == 0) {
    internal::SolveProgramWithNoFreeVariables(prog, sdpa_free_format, result);
  }
}

bool CsdpSolver::is_available() { return true; }
}  // namespace solvers
}  // namespace drake
