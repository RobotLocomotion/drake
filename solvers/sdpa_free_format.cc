#include "drake/solvers/sdpa_free_format.h"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/SparseQR>

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
    X_blocks_.emplace_back(BlockType::kMatrix, matrix_rows);
    for (int j = 0; j < matrix_rows; ++j) {
      for (int i = 0; i <= j; ++i) {
        const symbolic::Variable& psd_ij_var =
            binding.variables()(psd_matrix_variable_index++);
        const int psd_ij_var_index = prog.FindDecisionVariableIndex(psd_ij_var);
        const bool has_var_registered = !(holds_alternative<std::nullptr_t>(
            prog_var_in_sdpa_[psd_ij_var_index]));
        const EntryInX psd_ij_entry_in_X(num_blocks, i, j, num_X_rows_);
        if (!has_var_registered) {
          // This variable has not been registered into X. Now register this
          // variable, by adding it to prog_var_in_sdpa_
          prog_var_in_sdpa_[psd_ij_var_index].emplace<DecisionVariableInSdpaX>(
              Sign::kPositive, 0, psd_ij_entry_in_X);
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
                std::vector<EntryInX>({get<DecisionVariableInSdpaX>(
                                           prog_var_in_sdpa_[psd_ij_var_index])
                                           .entry_in_X,
                                       psd_ij_entry_in_X}));
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

void AddTermToTriplets(const EntryInX& entry_in_X, double coeff,
                       std::vector<Eigen::Triplet<double>>* triplets) {
  if (entry_in_X.row_index_in_block == entry_in_X.column_index_in_block) {
    triplets->emplace_back(
        entry_in_X.X_start_row + entry_in_X.row_index_in_block,
        entry_in_X.X_start_row + entry_in_X.column_index_in_block, coeff);
  } else {
    triplets->emplace_back(
        entry_in_X.X_start_row + entry_in_X.row_index_in_block,
        entry_in_X.X_start_row + entry_in_X.column_index_in_block, coeff / 2);
    triplets->emplace_back(
        entry_in_X.X_start_row + entry_in_X.column_index_in_block,
        entry_in_X.X_start_row + entry_in_X.row_index_in_block, coeff / 2);
  }
}

void SdpaFreeFormat::AddLinearEqualityConstraint(
    const std::vector<double>& a, const std::vector<int>& prog_vars_indices,
    const std::vector<double>& b, const std::vector<EntryInX>& X_entries,
    const std::vector<double>& c,
    const std::vector<FreeVariableIndex>& s_indices, double rhs) {
  DRAKE_ASSERT(a.size() == prog_vars_indices.size());
  DRAKE_ASSERT(b.size() == X_entries.size());
  DRAKE_ASSERT(c.size() == s_indices.size());
  const int constraint_index = static_cast<int>(A_triplets_.size());
  std::vector<Eigen::Triplet<double>> Ai_triplets;
  // If the entries are off-diagonal entries of X, then it needs two
  // coefficients in Ai, for both the upper and lower diagonal part.
  Ai_triplets.reserve(static_cast<int>(a.size() + b.size()) * 2);
  g_.conservativeResize(g_.rows() + 1);
  g_(constraint_index) = rhs;
  for (int i = 0; i < static_cast<int>(a.size()); ++i) {
    if (a[i] != 0) {
      const int prog_var_index = prog_vars_indices[i];
      if (holds_alternative<DecisionVariableInSdpaX>(
              prog_var_in_sdpa_[prog_var_index])) {
        // This variable is an entry in X.
        const auto& decision_var_in_X =
            get<DecisionVariableInSdpaX>(prog_var_in_sdpa_[prog_var_index]);
        g_(constraint_index) -= a[i] * decision_var_in_X.offset;
        const double coeff =
            decision_var_in_X.coeff_sign == Sign::kPositive ? a[i] : -a[i];
        AddTermToTriplets(decision_var_in_X.entry_in_X, coeff, &Ai_triplets);
      } else if (holds_alternative<double>(prog_var_in_sdpa_[prog_var_index])) {
        // This variable has a constant value.
        const double var_value = get<double>(prog_var_in_sdpa_[prog_var_index]);
        g_(constraint_index) -= a[i] * var_value;
      } else if (holds_alternative<FreeVariableIndex>(
                     prog_var_in_sdpa_[prog_var_index])) {
        B_triplets_.emplace_back(
            constraint_index,
            get<FreeVariableIndex>(prog_var_in_sdpa_[prog_var_index]), a[i]);
      } else {
        throw std::runtime_error(
            "SdpaFreeFormat::AddLinearEqualityConstraint() : this decision "
            "variable is not an entry in X or s, and is not a constant.");
      }
    }
  }
  for (int i = 0; i < static_cast<int>(b.size()); ++i) {
    if (b[i] != 0) {
      AddTermToTriplets(X_entries[i], b[i], &Ai_triplets);
    }
  }
  A_triplets_.push_back(Ai_triplets);
  if (!b.empty()) {
    for (int i = 0; i < static_cast<int>(c.size()); ++i) {
      B_triplets_.emplace_back(constraint_index, s_indices[i], c[i]);
    }
  }
}

void SdpaFreeFormat::AddEqualityConstraintOnXentriesForSameDecisionVariable(
    const std::unordered_map<symbolic::Variable::Id, std::vector<EntryInX>>&
        entries_in_X_for_same_decision_variable) {
  for (const auto& item : entries_in_X_for_same_decision_variable) {
    const auto& Xentries = item.second;
    DRAKE_ASSERT(Xentries.size() >= 2);
    const std::vector<double> a{{1, -1}};
    for (int i = 1; i < static_cast<int>(Xentries.size()); ++i) {
      AddLinearEqualityConstraint({}, {}, a, {Xentries[0], Xentries[i]}, {}, {},
                                  0.0);
    }
  }
}

void SdpaFreeFormat::RegisterMathematicalProgramDecisionVariable(
    const MathematicalProgram& prog) {
  // Go through all the decision variables in @p prog. If the decision variable
  // has not been registered in SDPA, then register it now. Refer to @ref
  // prog_var_in_sdpa_ for different types.
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
    // The variable might have been registered when we go through the positive
    // semidefinite constraint.
    bool has_var_registered =
        !holds_alternative<std::nullptr_t>(prog_var_in_sdpa_[i]);
    // First check if this variable has either finite lower or upper bound. If
    // not, then the variable is a free variable in SDPA.
    if (!has_var_registered) {
      if (std::isinf(lower_bound(i)) && std::isinf(upper_bound(i))) {
        // This is a free variable.
        prog_var_in_sdpa_[i].emplace<FreeVariableIndex>(num_free_variables_);
        num_free_variables_++;
      } else {
        if (!std::isinf(lower_bound(i)) && std::isinf(upper_bound(i))) {
          // lower <= x.
          prog_var_in_sdpa_[i].emplace<DecisionVariableInSdpaX>(
              Sign::kPositive, lower_bound(i), block_index, new_X_var_count,
              new_X_var_count, num_X_rows_);
          new_X_var_count++;
        } else if (std::isinf(lower_bound(i)) && !std::isinf(upper_bound(i))) {
          // x <= upper.
          prog_var_in_sdpa_[i].emplace<DecisionVariableInSdpaX>(
              Sign::kNegative, upper_bound(i), block_index, new_X_var_count,
              new_X_var_count, num_X_rows_);
          new_X_var_count++;
        } else if (lower_bound(i) == upper_bound(i)) {
          // x == bound
          prog_var_in_sdpa_[i].emplace<double>(lower_bound(i));
        } else {
          // lower <= x <= upper
          // x will be replaced as y + lower, where y is an entry in X.
          prog_var_in_sdpa_[i].emplace<DecisionVariableInSdpaX>(
              Sign::kPositive, lower_bound(i), block_index, new_X_var_count,
              new_X_var_count, num_X_rows_);
          // Add another slack variables z, with the constraint x + z = upper
          AddLinearEqualityConstraint(
              {1}, {i}, {1.0},
              {EntryInX(block_index, new_X_var_count + 1, new_X_var_count + 1,
                        num_X_rows_)},
              {}, {}, upper_bound(i));
          new_X_var_count += 2;
        }
      }
    } else {
      // This variable has been registered as a variable in SDPA. It should have
      // been registered as an entry in X.
      if (holds_alternative<DecisionVariableInSdpaX>(prog_var_in_sdpa_[i])) {
        if (std::isinf(lower_bound(i)) && std::isinf(upper_bound(i))) {
          // no finite bounds.
          continue;
        } else if (!std::isinf(lower_bound(i)) && std::isinf(upper_bound(i))) {
          // lower <= x
          // Adds a slack variable y as a diagonal entry in X, with the
          // constraint x - y = lower
          AddLinearEqualityConstraint({1}, {i}, {-1.0},
                                      {EntryInX(block_index, new_X_var_count,
                                                new_X_var_count, num_X_rows_)},
                                      {}, {}, lower_bound(i));
          new_X_var_count++;
        } else if (std::isinf(lower_bound(i)) && !std::isinf(upper_bound(i))) {
          // x <= upper
          // Adds a slack variable y as a diagonal entry in X, with the
          // constraint x + y = upper
          AddLinearEqualityConstraint({1.0}, {i}, {1.0},
                                      {EntryInX(block_index, new_X_var_count,
                                                new_X_var_count, num_X_rows_)},
                                      {}, {}, upper_bound(i));
          new_X_var_count++;
        } else if (lower_bound(i) == upper_bound(i)) {
          // Add the constraint x == bound.
          AddLinearEqualityConstraint({1.0}, {i}, {}, {}, {}, {},
                                      lower_bound(i));
        } else {
          // lower <= x <= upper
          // Adds two slack variables y1, y2 in X, with the linear equality
          // constraint x - y1 = lower and x + y2 <= upper
          AddLinearEqualityConstraint({1.0}, {i}, {-1.0},
                                      {EntryInX(block_index, new_X_var_count,
                                                new_X_var_count, num_X_rows_)},
                                      {}, {}, lower_bound(i));
          AddLinearEqualityConstraint(
              {1.0}, {i}, {1.0},
              {EntryInX(block_index, new_X_var_count + 1, new_X_var_count + 1,
                        num_X_rows_)},
              {}, {}, upper_bound(i));
          new_X_var_count += 2;
        }
      } else {
        throw std::runtime_error(
            "SdpaFreeFormat::RegisterMathematicalProgramDecisionVariable(): "
            "the registed variable should be an entry in X");
      }
    }
  }
  if (new_X_var_count > 0) {
    X_blocks_.emplace_back(BlockType::kDiagonal, new_X_var_count);
    num_X_rows_ += new_X_var_count;
  }
}

void SdpaFreeFormat::AddLinearCostsFromProgram(
    const MathematicalProgram& prog) {
  for (const auto& linear_cost : prog.linear_costs()) {
    for (int i = 0; i < linear_cost.variables().rows(); ++i) {
      // The negation sign is because in CSDP format, the objective is to
      // maximize the cost, while in MathematicalProgram, the objective is to
      // minimize the cost.
      double coeff = -linear_cost.evaluator()->a()(i);
      if (coeff != 0) {
        // Only adds the cost if the cost coefficient is non-zero.
        const int var_index =
            prog.FindDecisionVariableIndex(linear_cost.variables()(i));
        if (holds_alternative<DecisionVariableInSdpaX>(
                prog_var_in_sdpa_[var_index])) {
          const auto& decision_var_in_X =
              get<DecisionVariableInSdpaX>(prog_var_in_sdpa_[var_index]);
          coeff =
              decision_var_in_X.coeff_sign == Sign::kPositive ? coeff : -coeff;
          constant_min_cost_term_ +=
              linear_cost.evaluator()->a()(i) * decision_var_in_X.offset;
          AddTermToTriplets(decision_var_in_X.entry_in_X, coeff, &C_triplets_);
        } else if (holds_alternative<double>(prog_var_in_sdpa_[var_index])) {
          const double val = get<double>(prog_var_in_sdpa_[var_index]);
          constant_min_cost_term_ += linear_cost.evaluator()->a()(i) * val;
        } else if (holds_alternative<FreeVariableIndex>(
                       prog_var_in_sdpa_[var_index])) {
          const FreeVariableIndex& s_index =
              get<FreeVariableIndex>(prog_var_in_sdpa_[var_index]);
          d_triplets_.emplace_back(s_index, 0, coeff);
        } else {
          throw std::runtime_error(
              "SdpaFreeFormat::AddLinearCostFromProgram() only supports "
              "DecisionVariableInSdpaX, double or FreeVariableIndex.");
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
    a.reserve(var_indices.size());
    b.reserve(1);
    std::vector<int> decision_var_indices_in_X;
    decision_var_indices_in_X.reserve(var_indices.size());
    std::vector<EntryInX> X_entries;
    X_entries.reserve(1);
    std::vector<int> s_indices;
    for (int j = 0; j < linear_constraint.variables().rows(); ++j) {
      if (linear_constraint.evaluator()->A()(i, j) != 0) {
        a.push_back(linear_constraint.evaluator()->A()(i, j));
        decision_var_indices_in_X.push_back(var_indices[j]);
      }
    }
    if (does_lower_equal_upper_in_this_row) {
      // Add the equality constraint.
      AddLinearEqualityConstraint(
          a, decision_var_indices_in_X, {}, {}, {}, {},
          linear_constraint.evaluator()->lower_bound()(i));
    } else {
      // First add the constraint and slack variable for the lower bound.
      if (!std::isinf(linear_constraint.evaluator()->lower_bound()(i))) {
        AddLinearEqualityConstraint(
            a, decision_var_indices_in_X, {-1},
            {EntryInX(static_cast<int>(X_blocks_.size()),
                      *linear_constraint_slack_entry_in_X_count,
                      *linear_constraint_slack_entry_in_X_count, num_X_rows_)},
            {}, {}, linear_constraint.evaluator()->lower_bound()(i));
        (*linear_constraint_slack_entry_in_X_count)++;
      }
      // Now add the constraint and slack variable for the upper bound.
      if (!std::isinf(linear_constraint.evaluator()->upper_bound()(i))) {
        AddLinearEqualityConstraint(
            a, decision_var_indices_in_X, {1},
            {EntryInX(static_cast<int>(X_blocks_.size()),
                      *linear_constraint_slack_entry_in_X_count,
                      *linear_constraint_slack_entry_in_X_count, num_X_rows_)},
            {}, {}, linear_constraint.evaluator()->upper_bound()(i));
        (*linear_constraint_slack_entry_in_X_count)++;
      }
    }
  }
}

void SdpaFreeFormat::AddLinearConstraintsFromProgram(
    const MathematicalProgram& prog) {
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
    X_blocks_.emplace_back(BlockType::kDiagonal,
                           linear_constraint_slack_entry_in_X_count);
  }
}

void SdpaFreeFormat::AddLinearMatrixInequalityConstraints(
    const MathematicalProgram& prog) {
  for (const auto& lmi_constraint :
       prog.linear_matrix_inequality_constraints()) {
    const std::vector<int> var_indices =
        prog.FindDecisionVariableIndices(lmi_constraint.variables());
    // Add the constraint that F1 * x1 + ... + Fn * xn - X_slack = -F0 and
    // X_slack is psd.
    const std::vector<Eigen::MatrixXd>& F = lmi_constraint.evaluator()->F();
    for (int j = 0; j < lmi_constraint.evaluator()->matrix_rows(); ++j) {
      for (int i = 0; i <= j; ++i) {
        std::vector<double> a;
        a.reserve(static_cast<int>(F.size()) - 1);
        for (int k = 1; k < static_cast<int>(F.size()); ++k) {
          a.push_back(F[k](i, j));
        }
        AddLinearEqualityConstraint(
            a, var_indices, {-1},
            {EntryInX(static_cast<int>(X_blocks_.size()), i, j, num_X_rows_)},
            {}, {}, -F[0](i, j));
      }
    }

    X_blocks_.emplace_back(BlockType::kMatrix,
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
  prog_var_in_sdpa_.reserve(prog.num_vars());
  for (int i = 0; i < prog.num_vars(); ++i) {
    prog_var_in_sdpa_.emplace_back(nullptr);
  }
  std::unordered_map<symbolic::Variable::Id, std::vector<EntryInX>>
      entries_in_X_for_same_decision_variable;
  DeclareXforPositiveSemidefiniteConstraints(
      prog, &entries_in_X_for_same_decision_variable);

  AddEqualityConstraintOnXentriesForSameDecisionVariable(
      entries_in_X_for_same_decision_variable);

  RegisterMathematicalProgramDecisionVariable(prog);

  AddLinearCostsFromProgram(prog);

  AddLinearConstraintsFromProgram(prog);

  AddLinearMatrixInequalityConstraints(prog);

  Finalize();
}
}  // namespace internal
}  // namespace solvers
}  // namespace drake
