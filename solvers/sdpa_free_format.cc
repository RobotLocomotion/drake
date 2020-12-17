#include "drake/solvers/sdpa_free_format.h"

#include <algorithm>
#include <fstream>
#include <initializer_list>
#include <iomanip>
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
        const bool has_var_registered =
            !(std::holds_alternative<std::nullptr_t>(
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
                std::vector<EntryInX>({std::get<DecisionVariableInSdpaX>(
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
    const std::vector<double>& coeff_prog_vars,
    const std::vector<int>& prog_vars_indices,
    const std::vector<double>& coeff_X_entries,
    const std::vector<EntryInX>& X_entries,
    const std::vector<double>& coeff_free_vars,
    const std::vector<FreeVariableIndex>& free_vars_indices, double rhs) {
  DRAKE_ASSERT(coeff_prog_vars.size() == prog_vars_indices.size());
  DRAKE_ASSERT(coeff_X_entries.size() == X_entries.size());
  DRAKE_ASSERT(coeff_free_vars.size() == free_vars_indices.size());
  const int constraint_index = static_cast<int>(A_triplets_.size());
  std::vector<Eigen::Triplet<double>> Ai_triplets;
  // If the entries are off-diagonal entries of X, then it needs two
  // coefficients in Ai, for both the upper and lower diagonal part.
  Ai_triplets.reserve(
      static_cast<int>(coeff_prog_vars.size() + coeff_X_entries.size()) * 2);
  g_.conservativeResize(g_.rows() + 1);
  g_(constraint_index) = rhs;
  for (int i = 0; i < static_cast<int>(coeff_prog_vars.size()); ++i) {
    if (coeff_prog_vars[i] != 0) {
      const int prog_var_index = prog_vars_indices[i];
      if (std::holds_alternative<DecisionVariableInSdpaX>(
              prog_var_in_sdpa_[prog_var_index])) {
        // This variable is an entry in X.
        const auto& decision_var_in_X =
            std::get<DecisionVariableInSdpaX>(
                prog_var_in_sdpa_[prog_var_index]);
        g_(constraint_index) -= coeff_prog_vars[i] * decision_var_in_X.offset;
        const double coeff = decision_var_in_X.coeff_sign == Sign::kPositive
                                 ? coeff_prog_vars[i]
                                 : -coeff_prog_vars[i];
        AddTermToTriplets(decision_var_in_X.entry_in_X, coeff, &Ai_triplets);
      } else if (std::holds_alternative<double>(
          prog_var_in_sdpa_[prog_var_index])) {
        // This variable has a constant value.
        const double var_value = std::get<double>(
            prog_var_in_sdpa_[prog_var_index]);
        g_(constraint_index) -= coeff_prog_vars[i] * var_value;
      } else if (std::holds_alternative<FreeVariableIndex>(
                     prog_var_in_sdpa_[prog_var_index])) {
        // This variable is a free variable (no lower nor upper bound).
        B_triplets_.emplace_back(
            constraint_index,
            std::get<FreeVariableIndex>(prog_var_in_sdpa_[prog_var_index]),
            coeff_prog_vars[i]);
      } else {
        throw std::runtime_error(
            "SdpaFreeFormat::AddLinearEqualityConstraint() : this decision "
            "variable is not an entry in X or s, and is not a constant.");
      }
    }
  }
  // Adds coeff_X_entries * X_entries.
  for (int i = 0; i < static_cast<int>(coeff_X_entries.size()); ++i) {
    if (coeff_X_entries[i] != 0) {
      AddTermToTriplets(X_entries[i], coeff_X_entries[i], &Ai_triplets);
    }
  }
  A_triplets_.push_back(Ai_triplets);
  // Adds coeff_free_vars * free_vars.
  if (!coeff_X_entries.empty()) {
    for (int i = 0; i < static_cast<int>(coeff_free_vars.size()); ++i) {
      B_triplets_.emplace_back(constraint_index, free_vars_indices[i],
                               coeff_free_vars[i]);
    }
  }
}

void SdpaFreeFormat::AddEqualityConstraintOnXEntriesForSameDecisionVariable(
    const std::unordered_map<symbolic::Variable::Id, std::vector<EntryInX>>&
        entries_in_X_for_same_decision_variable) {
  for (const auto& item : entries_in_X_for_same_decision_variable) {
    const auto& Xentries = item.second;
    DRAKE_ASSERT(Xentries.size() >= 2);
    const std::vector<double> a{{1, -1}};
    for (int i = 1; i < static_cast<int>(Xentries.size()); ++i) {
      AddLinearEqualityConstraint({} /* coefficients for prog_vars */,
                                  {} /* empty prog_vars */, a,
                                  {Xentries[0], Xentries[i]}, {}, {}, 0.0);
    }
  }
}

void SdpaFreeFormat::RegisterSingleMathematicalProgramDecisionVariable(
    double lower_bound, double upper_bound, int variable_index, int block_index,
    int* new_X_var_count) {
  // First check if this variable has either finite lower or upper bound. If
  // not, then the variable is a free variable in SDPA.
  if (std::isinf(lower_bound) && std::isinf(upper_bound)) {
    // This is a free variable.
    prog_var_in_sdpa_[variable_index].emplace<FreeVariableIndex>(
        num_free_variables_);
    num_free_variables_++;
  } else {
    if (!std::isinf(lower_bound) && std::isinf(upper_bound)) {
      // lower <= x.
      prog_var_in_sdpa_[variable_index].emplace<DecisionVariableInSdpaX>(
          Sign::kPositive, lower_bound, block_index, *new_X_var_count,
          *new_X_var_count, num_X_rows_);
      (*new_X_var_count)++;
    } else if (std::isinf(lower_bound) && !std::isinf(upper_bound)) {
      // x <= upper.
      prog_var_in_sdpa_[variable_index].emplace<DecisionVariableInSdpaX>(
          Sign::kNegative, upper_bound, block_index, *new_X_var_count,
          *new_X_var_count, num_X_rows_);
      (*new_X_var_count)++;
    } else if (lower_bound == upper_bound) {
      // x == bound.
      prog_var_in_sdpa_[variable_index].emplace<double>(lower_bound);
    } else {
      // lower <= x <= upper.
      // x will be replaced with y + lower, where y is an entry in X.
      prog_var_in_sdpa_[variable_index].emplace<DecisionVariableInSdpaX>(
          Sign::kPositive, lower_bound, block_index, *new_X_var_count,
          *new_X_var_count, num_X_rows_);
      // Add another slack variables z, with the constraint x + z = upper.
      AddLinearEqualityConstraint({1}, {variable_index}, {1.0},
                                  {EntryInX(block_index, *new_X_var_count + 1,
                                            *new_X_var_count + 1, num_X_rows_)},
                                  {}, {}, upper_bound);
      (*new_X_var_count) += 2;
    }
  }
}

void SdpaFreeFormat::AddBoundsOnRegisteredDecisionVariable(
    double lower_bound, double upper_bound, int variable_index, int block_index,
    int* new_X_var_count) {
  // This variable has been registered as a variable in SDPA. It should have
  // been registered as an entry in X.
  if (std::holds_alternative<DecisionVariableInSdpaX>(
          prog_var_in_sdpa_[variable_index])) {
    if (std::isinf(lower_bound) && std::isinf(upper_bound)) {
      // no finite bounds.
      return;
    } else if (!std::isinf(lower_bound) && std::isinf(upper_bound)) {
      // lower <= x.
      // Adds a slack variable y as a diagonal entry in X, with the
      // constraint x - y = lower.
      AddLinearEqualityConstraint({1}, {variable_index}, {-1.0},
                                  {EntryInX(block_index, *new_X_var_count,
                                            *new_X_var_count, num_X_rows_)},
                                  {}, {}, lower_bound);
      (*new_X_var_count)++;
    } else if (std::isinf(lower_bound) && !std::isinf(upper_bound)) {
      // x <= upper.
      // Adds a slack variable y as a diagonal entry in X, with the
      // constraint x + y = upper.
      AddLinearEqualityConstraint({1.0}, {variable_index}, {1.0},
                                  {EntryInX(block_index, *new_X_var_count,
                                            *new_X_var_count, num_X_rows_)},
                                  {}, {}, upper_bound);
      (*new_X_var_count)++;
    } else if (lower_bound == upper_bound) {
      // Add the constraint x == bound.
      AddLinearEqualityConstraint({1.0}, {variable_index}, {}, {}, {}, {},
                                  lower_bound);
    } else {
      // lower <= x <= upper.
      // Adds two slack variables y1, y2 in X, with the linear equality
      // constraint x - y1 = lower and x + y2 <= upper.
      AddLinearEqualityConstraint({1.0}, {variable_index}, {-1.0},
                                  {EntryInX(block_index, *new_X_var_count,
                                            *new_X_var_count, num_X_rows_)},
                                  {}, {}, lower_bound);
      AddLinearEqualityConstraint(
          {1.0}, {variable_index}, {1.0},
          {EntryInX(block_index, (*new_X_var_count) + 1, (*new_X_var_count) + 1,
                    num_X_rows_)},
          {}, {}, upper_bound);
      (*new_X_var_count) += 2;
    }
  } else {
    throw std::runtime_error(
        "SdpaFreeFormat::AddBoundsOnRegisteredDecisionVariable(): "
        "the registered variable should be an entry in X.");
  }
}

void SdpaFreeFormat::RegisterMathematicalProgramDecisionVariables(
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
        !std::holds_alternative<std::nullptr_t>(prog_var_in_sdpa_[i]);
    // First check if this variable has either finite lower or upper bound. If
    // not, then the variable is a free variable in SDPA.
    if (!has_var_registered) {
      RegisterSingleMathematicalProgramDecisionVariable(


          lower_bound(i), upper_bound(i), i, block_index, &new_X_var_count);
    } else {
      AddBoundsOnRegisteredDecisionVariable(lower_bound(i), upper_bound(i), i,
                                            block_index, &new_X_var_count);
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
      // The negation sign is because in SDPA format, the objective is to
      // maximize the cost, while in MathematicalProgram, the objective is to
      // minimize the cost.
      double coeff = -linear_cost.evaluator()->a()(i);
      if (coeff != 0) {
        // Only adds the cost if the cost coefficient is non-zero.
        const int var_index =
            prog.FindDecisionVariableIndex(linear_cost.variables()(i));
        if (std::holds_alternative<DecisionVariableInSdpaX>(
                prog_var_in_sdpa_[var_index])) {
          const auto& decision_var_in_X =
              std::get<DecisionVariableInSdpaX>(prog_var_in_sdpa_[var_index]);
          coeff =
              decision_var_in_X.coeff_sign == Sign::kPositive ? coeff : -coeff;
          constant_min_cost_term_ +=
              linear_cost.evaluator()->a()(i) * decision_var_in_X.offset;
          AddTermToTriplets(decision_var_in_X.entry_in_X, coeff, &C_triplets_);
        } else if (std::holds_alternative<double>(
            prog_var_in_sdpa_[var_index])) {
          const double val = std::get<double>(prog_var_in_sdpa_[var_index]);
          constant_min_cost_term_ += linear_cost.evaluator()->a()(i) * val;
        } else if (std::holds_alternative<FreeVariableIndex>(
                       prog_var_in_sdpa_[var_index])) {
          const FreeVariableIndex& s_index =
              std::get<FreeVariableIndex>(prog_var_in_sdpa_[var_index]);
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

// A Lorentz cone constraint z₀ ≥ sqrt(z₁² + ... + zₙ²), where z = A*x+b, can
// be rewritten as a positive semidefinite constraint
// ⎡ z₀ z₁ ... zₙ ⎤
// ⎢ z₁ z₀  0  0  ⎥  is positive semidefinite.
// ⎢    ....      ⎥
// ⎣ zₙ 0   0  z₀ ⎦
void SdpaFreeFormat::AddLorentzConeConstraints(
    const MathematicalProgram& prog) {
  for (const auto& lorentz_cone_constraint : prog.lorentz_cone_constraints()) {
    const int num_block_rows = lorentz_cone_constraint.evaluator()->A().rows();
    const int num_decision_vars = lorentz_cone_constraint.variables().rows();
    const std::vector<int> prog_vars_indices =
        prog.FindDecisionVariableIndices(lorentz_cone_constraint.variables());

    // Add the linear constraint that all the diagonal terms of the new block
    // matrix equals to z0.
    std::vector<double> a;
    // The last entry in X_entries would be the diagonal term in the new block.
    a.reserve(num_decision_vars);
    // We need to impose the linear equality constraint
    // lorentz_cone_constraint.evaluator()->A().row(0) *
    // lorentz_cone_constraint.variables() - new_block(i, i) =
    // -lorentz_cone_constraint.evaluator()->b()(0).
    // So we first fill in a, b, X_entries and s_indices with the term from
    // lorentz_cone_constraint.evaluator()->A().row(0) *
    // lorentz_cone_constraint.variables()
    for (int i = 0; i < num_decision_vars; ++i) {
      const double coeff = lorentz_cone_constraint.evaluator()->A_dense()(0, i);
      a.push_back(coeff);
    }

    // For each diagonal entry in the new block matrix, we need to add
    // -new_block(i, i) to the left-hand side of the equality constraint.
    for (int i = 0; i < num_block_rows; ++i) {
      AddLinearEqualityConstraint(
          a, prog_vars_indices, {-1.0},
          {EntryInX(static_cast<int>(X_blocks_.size()), i, i, num_X_rows_)}, {},
          {}, -lorentz_cone_constraint.evaluator()->b()(0));
    }

    // Now we add the linear equality constraint arising from the first row of
    // the new block lorentz_cone_constraint.evaluator()->A().row(i) *
    // lorentz_cone_constraint.variables() +
    // lorentz_cone_constraint.evaluator()->b()(i) = new_block(0, i) for i
    for (int i = 1; i < num_block_rows; ++i) {
      a.clear();
      a.reserve(num_decision_vars);
      for (int j = 0; j < num_decision_vars; ++j) {
        const double coeff =
            lorentz_cone_constraint.evaluator()->A_dense()(i, j);
        a.push_back(coeff);
      }
      // Add the term -new_block(0, i)
      AddLinearEqualityConstraint(
          a, prog_vars_indices, {-1},
          {EntryInX(static_cast<int>(X_blocks_.size()), 0, i, num_X_rows_)}, {},
          {}, -lorentz_cone_constraint.evaluator()->b()(i));
    }

    // Now add the constraint that many entries in this new block is 0.
    for (int i = 1; i < num_block_rows; ++i) {
      for (int j = 1; j < i; ++j) {
        AddLinearEqualityConstraint(
            {}, {}, {1.0},
            {EntryInX(static_cast<int>(X_blocks_.size()), i, j, num_X_rows_)},
            {}, {}, 0);
      }
    }

    X_blocks_.emplace_back(BlockType::kMatrix, num_block_rows);
    num_X_rows_ += num_block_rows;
  }
}

// A vector z in rotated Lorentz cone (i.e.,z₀≥0, z₁≥0, z₀z₁≥ sqrt(z₂² + ...
// zₙ²), where z = Ax+b ) is equivalent to the following positive semidefinite
// constraint
// ⎡ z₀ z₂ z₃ ... zₙ ⎤
// ⎢ z₂ z₁ 0  ...  0 ⎥
// ⎢ z₃ 0 z₁ 0 ... 0 ⎥ is positive semidefinite.
// ⎢ z₄ 0 0 z₁ 0   0 ⎥
// ⎢      ...        ⎥
// ⎢ zₙ₋₁ ...   z₁ 0 ⎥
// ⎣ zₙ 0 ... 0    z₁⎦
void SdpaFreeFormat::AddRotatedLorentzConeConstraints(
    const MathematicalProgram& prog) {
  for (const auto& rotated_lorentz_constraint :
       prog.rotated_lorentz_cone_constraints()) {
    const int z_size = rotated_lorentz_constraint.evaluator()->A().rows();
    // The number of rows in the new block matrix
    const int num_block_rows = z_size - 1;
    const int num_decision_vars = rotated_lorentz_constraint.variables().rows();
    const std::vector<int> prog_vars_indices = prog.FindDecisionVariableIndices(
        rotated_lorentz_constraint.variables());
    // First add the equality constraint arising from the first row of the PSD
    // constraint. rotated_lorentz_constraint.evaluator()->A().row(j) *
    // rotated_lorentz_constraint.variables() +
    // rotated_lorentz_constraint.evaluator()->b()(j) = new_X(0, i);
    // where j = 0 if i = 0, and j = i + 1 otherwise.
    for (int i = 0; i < num_block_rows; ++i) {
      std::vector<double> a;
      a.reserve(num_decision_vars);
      const int j = i == 0 ? 0 : i + 1;
      for (int k = 0; k < num_decision_vars; ++k) {
        a.push_back(rotated_lorentz_constraint.evaluator()->A_dense()(j, k));
      }
      // Add the term -new_X(0, i)
      AddLinearEqualityConstraint(
          a, prog_vars_indices, {-1},
          {EntryInX(static_cast<int>(X_blocks_.size()), 0, i, num_X_rows_)}, {},
          {}, -rotated_lorentz_constraint.evaluator()->b()(j));
    }

    // Add the linear constraint
    // rotated_lorentz_constraint.evaluator()->A().row(1) *
    // rotated_lorentz_constraint.variables() +
    // rotated_lorentz_constraint.evaluator()->b()(1) = new_X(i, i) for i >= 1
    std::vector<double> a;
    a.reserve(num_decision_vars);
    for (int i = 0; i < num_decision_vars; ++i) {
      a.push_back(rotated_lorentz_constraint.evaluator()->A_dense()(1, i));
    }
    for (int i = 1; i < num_block_rows; ++i) {
      // Add the term -new_block(i, i).
      AddLinearEqualityConstraint(
          a, prog_vars_indices, {-1},
          {EntryInX(static_cast<int>(X_blocks_.size()), i, i, num_X_rows_)}, {},
          {}, -rotated_lorentz_constraint.evaluator()->b()(1));
    }

    // Now add the constraint that new_X(i, j) = 0 for j >= 2 and 1 <= i < j
    for (int j = 2; j < num_block_rows; ++j) {
      for (int i = 1; i < j; ++i) {
        AddLinearEqualityConstraint(
            {}, {}, {1},
            {EntryInX(static_cast<int>(X_blocks_.size()), i, j, num_X_rows_)},
            {}, {}, 0);
      }
    }

    X_blocks_.emplace_back(BlockType::kMatrix, num_block_rows);
    num_X_rows_ += num_block_rows;
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
  ProgramAttributes solver_capabilities(std::initializer_list<ProgramAttribute>{
      ProgramAttribute::kLinearCost, ProgramAttribute::kLinearConstraint,
      ProgramAttribute::kLinearEqualityConstraint,
      ProgramAttribute::kLorentzConeConstraint,
      ProgramAttribute::kRotatedLorentzConeConstraint,
      ProgramAttribute::kPositiveSemidefiniteConstraint});
  if (!AreRequiredAttributesSupported(prog.required_capabilities(),
                                      solver_capabilities)) {
    throw std::invalid_argument(
        "SdpaFreeFormat(): the program cannot be formulated as an SDP "
        "in the standard form.\n");
  }
  prog_var_in_sdpa_.reserve(prog.num_vars());
  for (int i = 0; i < prog.num_vars(); ++i) {
    prog_var_in_sdpa_.emplace_back(nullptr);
  }
  std::unordered_map<symbolic::Variable::Id, std::vector<EntryInX>>
      entries_in_X_for_same_decision_variable;
  DeclareXforPositiveSemidefiniteConstraints(
      prog, &entries_in_X_for_same_decision_variable);

  AddEqualityConstraintOnXEntriesForSameDecisionVariable(
      entries_in_X_for_same_decision_variable);

  RegisterMathematicalProgramDecisionVariables(prog);

  AddLinearCostsFromProgram(prog);

  AddLinearConstraintsFromProgram(prog);

  AddLinearMatrixInequalityConstraints(prog);

  AddLorentzConeConstraints(prog);

  AddRotatedLorentzConeConstraints(prog);

  Finalize();
}

void SdpaFreeFormat::RemoveFreeVariableByNullspaceApproach(
    Eigen::SparseMatrix<double>* C_hat,
    std::vector<Eigen::SparseMatrix<double>>* A_hat, Eigen::VectorXd* rhs_hat,
    Eigen::VectorXd* y_hat,
    Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>>*
        QR_B) const {
  DRAKE_ASSERT(this->num_free_variables() != 0);
  const int num_constraints = static_cast<int>(this->A_triplets().size());
  // Do a QR decomposition on B to find the null space of Bᵀ.
  QR_B->analyzePattern(this->B());
  QR_B->factorize(this->B());
  if (QR_B->info() != Eigen::Success) {
    throw std::runtime_error(
        "SdpaFreeFormat::RemoveFreeVariableByNullspaceApproach(): cannot "
        "perform QR decomposition of B. Please try the other method "
        "kTwoSlackVariables.");
  }
  // BP = [Q₁ Q₂] * [R; 0], so the nullspace of Bᵀ is Q₂
  Eigen::SparseMatrix<double> Q;
  Q = QR_B->matrixQ();
  const Eigen::SparseMatrix<double> N =
      Q.rightCols(this->B().rows() - QR_B->rank());
  *rhs_hat = N.transpose() * this->g();

  A_hat->clear();
  A_hat->reserve(N.cols());
  for (int i = 0; i < N.cols(); ++i) {
    A_hat->emplace_back(this->num_X_rows(), this->num_X_rows());
    A_hat->back().setZero();
    for (Eigen::SparseMatrix<double>::InnerIterator it_N(N, i); it_N; ++it_N) {
      A_hat->back() += it_N.value() * this->A()[it_N.row()];
    }
  }

  const Eigen::SparseMatrix<double> B_t = this->B().transpose();
  Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>>
      qr_B_t;
  qr_B_t.compute(B_t);
  if (qr_B_t.info() != Eigen::Success) {
    throw std::runtime_error(
        "RemoveFreeVariableByNullspaceApproach():QR "
        "decomposition on B.transpose() fails\n");
  }
  *y_hat = qr_B_t.solve(Eigen::VectorXd(this->d()));
  *C_hat = this->C();
  for (int i = 0; i < num_constraints; ++i) {
    *C_hat -= (*y_hat)(i) * this->A()[i];
  }
}

void SdpaFreeFormat::RemoveFreeVariableByTwoSlackVariablesApproach(
    std::vector<internal::BlockInX>* X_hat_blocks,
    std::vector<Eigen::SparseMatrix<double>>* A_hat,
    Eigen::SparseMatrix<double>* C_hat) const {
  *X_hat_blocks = this->X_blocks();
  X_hat_blocks->emplace_back(internal::BlockType::kDiagonal,
                             2 * this->num_free_variables());
  const int num_X_hat_rows =
      this->num_X_rows() + 2 * this->num_free_variables();
  std::vector<std::vector<Eigen::Triplet<double>>> A_hat_triplets =
      this->A_triplets();
  for (int j = 0; j < this->num_free_variables(); ++j) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(this->B(), j); it;
         ++it) {
      const int i = it.row();
      // Add the entry in Âᵢ that multiplies with sⱼ.
      A_hat_triplets[i].emplace_back(this->num_X_rows() + j,
                                     this->num_X_rows() + j, it.value());
      A_hat_triplets[i].emplace_back(
          this->num_X_rows() + this->num_free_variables() + j,
          this->num_X_rows() + this->num_free_variables() + j, -it.value());
    }
  }
  A_hat->clear();
  A_hat->reserve(this->A().size());
  for (int i = 0; i < static_cast<int>(this->A().size()); ++i) {
    A_hat->emplace_back(num_X_hat_rows, num_X_hat_rows);
    A_hat->back().setFromTriplets(A_hat_triplets[i].begin(),
                                  A_hat_triplets[i].end());
  }
  // Add the entry in Ĉ that multiplies with sᵢ
  std::vector<Eigen::Triplet<double>> C_hat_triplets = this->C_triplets();
  for (Eigen::SparseMatrix<double>::InnerIterator it(this->d(), 0); it; ++it) {
    const int i = it.row();
    C_hat_triplets.emplace_back(this->num_X_rows() + i, this->num_X_rows() + i,
                                it.value());
    C_hat_triplets.emplace_back(
        this->num_X_rows() + this->num_free_variables() + i,
        this->num_X_rows() + this->num_free_variables() + i, -it.value());
  }
  *C_hat = Eigen::SparseMatrix<double>(num_X_hat_rows, num_X_hat_rows);
  C_hat->setFromTriplets(C_hat_triplets.begin(), C_hat_triplets.end());
}

void SdpaFreeFormat::RemoveFreeVariableByLorentzConeSlackApproach(
    std::vector<internal::BlockInX>* X_hat_blocks,
    std::vector<Eigen::SparseMatrix<double>>* A_hat, Eigen::VectorXd* rhs_hat,
    Eigen::SparseMatrix<double>* C_hat) const {
  *X_hat_blocks = this->X_blocks();
  X_hat_blocks->emplace_back(internal::BlockType::kMatrix,
                             this->num_free_variables() + 1);
  const int num_X_hat_rows =
      this->num_X_rows() + this->num_free_variables() + 1;
  std::vector<std::vector<Eigen::Triplet<double>>> A_hat_triplets =
      this->A_triplets();
  // Âᵢ = diag(Aᵢ, B̂ᵢ)
  // where B̂ᵢ = ⎡0  bᵢᵀ/2⎤
  //            ⎣bᵢ/2   0⎦
  for (int j = 0; j < this->num_free_variables(); ++j) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(this->B(), j); it;
         ++it) {
      const int i = it.row();
      // Add the entry in Âᵢ that multiplies with sⱼ.
      A_hat_triplets[i].emplace_back(
          this->num_X_rows(), this->num_X_rows() + j + 1, it.value() / 2);
      A_hat_triplets[i].emplace_back(this->num_X_rows() + j + 1,
                                     this->num_X_rows(), it.value() / 2);
    }
  }
  const int num_linear_eq_constr =
      this->A().size() + (num_free_variables_ * (num_free_variables_ + 1)) / 2;
  A_hat->clear();
  A_hat->reserve(num_linear_eq_constr);
  for (int i = 0; i < static_cast<int>(this->A().size()); ++i) {
    A_hat->emplace_back(num_X_hat_rows, num_X_hat_rows);
    A_hat->back().setFromTriplets(A_hat_triplets[i].begin(),
                                  A_hat_triplets[i].end());
  }
  // The new right hand side is the same as the old one, except padding 0's
  // in the end.
  *rhs_hat = Eigen::VectorXd::Zero(num_linear_eq_constr);
  rhs_hat->head(this->A().size()) = this->g();

  for (int i = 0; i < num_free_variables_; ++i) {
    // Append the constraint Y(i + 1, i + 1) = Y(0, 0).
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(2);
    triplets.emplace_back(num_X_rows_, num_X_rows_, -1);
    triplets.emplace_back(num_X_rows_ + i + 1, num_X_rows_ + i + 1, 1);
    A_hat->emplace_back(num_X_hat_rows, num_X_hat_rows);
    A_hat->back().setFromTriplets(triplets.begin(), triplets.end());
    for (int j = i + 1; j < num_free_variables_; ++j) {
      // Append the constraint Y(i+1, j+1) = 0
      triplets.clear();
      triplets.reserve(2);
      triplets.emplace_back(num_X_rows_ + i + 1, num_X_rows_ + j + 1, 0.5);
      triplets.emplace_back(num_X_rows_ + j + 1, num_X_rows_ + i + 1, 0.5);
      A_hat->emplace_back(num_X_hat_rows, num_X_hat_rows);
      A_hat->back().setFromTriplets(triplets.begin(), triplets.end());
    }
  }
  // Add the entry in Ĉ that multiplies with sᵢ.
  std::vector<Eigen::Triplet<double>> C_hat_triplets = this->C_triplets();
  for (Eigen::SparseMatrix<double>::InnerIterator it(this->d(), 0); it; ++it) {
    const int i = it.row();
    C_hat_triplets.emplace_back(this->num_X_rows(), this->num_X_rows() + i + 1,
                                it.value() / 2);
    C_hat_triplets.emplace_back(this->num_X_rows() + i + 1, this->num_X_rows(),
                                it.value() / 2);
  }
  *C_hat = Eigen::SparseMatrix<double>(num_X_hat_rows, num_X_hat_rows);
  C_hat->setFromTriplets(C_hat_triplets.begin(), C_hat_triplets.end());
}

namespace {
bool GenerateSdpaImpl(const std::vector<BlockInX>& X_blocks,
                      const Eigen::SparseMatrix<double>& C,
                      const std::vector<Eigen::SparseMatrix<double>>& A,
                      const Eigen::Ref<const Eigen::VectorXd>& g,
                      const std::string& file_name) {
  const int num_X_rows = C.rows();
  DRAKE_DEMAND(C.cols() == num_X_rows);
  std::ofstream sdpa_file;
  sdpa_file.open(file_name + ".dat-s", std::ios::out | std::ios::trunc);
  if (sdpa_file.is_open()) {
    // First line, number of constraints.
    sdpa_file << g.rows() << "\n";
    // Second line, number of blocks in X.
    sdpa_file << X_blocks.size() << "\n";
    // Third line, size of each block.
    for (const auto& X_block : X_blocks) {
      switch (X_block.block_type) {
        case internal::BlockType::kMatrix: {
          sdpa_file << X_block.num_rows;
          break;
        }
        case internal::BlockType::kDiagonal: {
          // Negative value indates diagonal block according to SDPA format.
          sdpa_file << -X_block.num_rows;
          break;
        }
      }
      sdpa_file << " ";
    }
    sdpa_file << "\n";
    // Forth line, the right-hand side of the constraint g.
    std::stringstream g_stream;
    g_stream << std::setprecision(20);
    g_stream << g.transpose() << "\n";
    sdpa_file << g_stream.str();
    // block_start_rows[i] records the starting row index of the i'th block in
    // X. row_to_block_indices[i] records the index of the block that X(i, i)
    // belongs to.
    std::vector<int> block_start_rows(X_blocks.size());
    std::vector<int> row_to_block_indices(num_X_rows);
    int X_row_count = 0;
    for (int i = 0; i < static_cast<int>(X_blocks.size()); ++i) {
      block_start_rows[i] = X_row_count;
      for (int j = X_row_count; j < X_row_count + X_blocks[i].num_rows; ++j) {
        row_to_block_indices[j] = i;
      }
      X_row_count += X_blocks[i].num_rows;
    }
    // The non-zero entries in C
    for (int i = 0; i < num_X_rows; ++i) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(C, i); it; ++it) {
        if (it.row() <= it.col()) {
          const int block_start_row = block_start_rows[row_to_block_indices[i]];
          sdpa_file << 0 /* 0 for cost matrix C */ << " "
                    << row_to_block_indices[i] +
                           1 /* block number, starts from 1 */
                    << " "
                    << it.row() - block_start_row +
                           1 /* block row index, starts from 1*/
                    << " "
                    << i - block_start_row +
                           1 /* block column index, starts from 1*/
                    << " " << std::setprecision(20) << it.value() << "\n";
        }
      }
    }
    // The remaining lines are for A
    for (int i = 0; i < static_cast<int>(A.size()); ++i) {
      for (int j = 0; j < num_X_rows; ++j) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(A[i], j); it; ++it) {
          if (it.row() <= it.col()) {
            const int block_start_row =
                block_start_rows[row_to_block_indices[j]];
            sdpa_file << i + 1 /* constraint number, starts from 1 */ << " "
                      << row_to_block_indices[j] +
                             1 /* block number, starts from 1 */
                      << " " << it.row() - block_start_row + 1 << " "
                      << j - block_start_row + 1 << std::setprecision(20) << " "
                      << it.value() << "\n";
          }
        }
      }
    }

  } else {
    std::cout << "GenerateSDPA(): Cannot open the file " << file_name
              << ".dat-s\n";
    return false;
  }
  sdpa_file.close();
  return true;
}
}  // namespace

}  // namespace internal

bool GenerateSDPA(const MathematicalProgram& prog, const std::string& file_name,
                  RemoveFreeVariableMethod method) {
  const internal::SdpaFreeFormat sdpa_free_format(prog);
  if (sdpa_free_format.num_free_variables() == 0) {
    return internal::GenerateSdpaImpl(
        sdpa_free_format.X_blocks(), sdpa_free_format.C(), sdpa_free_format.A(),
        sdpa_free_format.g(), file_name);
  }
  switch (method) {
    case RemoveFreeVariableMethod::kNullspace: {
      Eigen::SparseMatrix<double> C_hat;
      std::vector<Eigen::SparseMatrix<double>> A_hat;
      Eigen::VectorXd rhs_hat, y_hat;
      Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>>
          QR_B;
      sdpa_free_format.RemoveFreeVariableByNullspaceApproach(
          &C_hat, &A_hat, &rhs_hat, &y_hat, &QR_B);
      return internal::GenerateSdpaImpl(sdpa_free_format.X_blocks(), C_hat,
                                        A_hat, rhs_hat, file_name);
      break;
    }
    case RemoveFreeVariableMethod::kTwoSlackVariables: {
      std::vector<internal::BlockInX> X_hat_blocks;
      std::vector<Eigen::SparseMatrix<double>> A_hat;
      Eigen::SparseMatrix<double> C_hat;
      sdpa_free_format.RemoveFreeVariableByTwoSlackVariablesApproach(
          &X_hat_blocks, &A_hat, &C_hat);
      return internal::GenerateSdpaImpl(X_hat_blocks, C_hat, A_hat,
                                        sdpa_free_format.g(), file_name);
    }
    case RemoveFreeVariableMethod::kLorentzConeSlack: {
      std::vector<internal::BlockInX> X_hat_blocks;
      std::vector<Eigen::SparseMatrix<double>> A_hat;
      Eigen::VectorXd rhs_hat;
      Eigen::SparseMatrix<double> C_hat;
      sdpa_free_format.RemoveFreeVariableByLorentzConeSlackApproach(
          &X_hat_blocks, &A_hat, &rhs_hat, &C_hat);
      return internal::GenerateSdpaImpl(X_hat_blocks, C_hat, A_hat, rhs_hat,
                                        file_name);
    }
  }
  return false;
}
}  // namespace solvers
}  // namespace drake
