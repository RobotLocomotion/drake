#include "drake/solvers/evaluator_base.h"

#include <set>

using std::make_shared;
using std::shared_ptr;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace solvers {
namespace {
// Check if each entry of gradient_sparsity_pattern is within [0, rows) and [0,
// cols), and if there are any repeated entries in gradient_sparsity_pattern.
void CheckGradientSparsityPattern(
    const std::vector<std::pair<int, int>>& gradient_sparsity_pattern, int rows,
    int cols) {
  std::set<std::pair<int, int>> nonzero_entries;
  for (const auto& nonzero_entry : gradient_sparsity_pattern) {
    if (nonzero_entry.first < 0 || nonzero_entry.first >= rows) {
      throw std::invalid_argument(
          "Constraint::SetSparsityPattern(): row index out of range.");
    }
    if (nonzero_entry.second < 0 || nonzero_entry.second >= cols) {
      throw std::invalid_argument(
          "Constraint::SetSparsityPattern(): column index out of range.");
    }
    auto it = nonzero_entries.find(nonzero_entry);
    if (it != nonzero_entries.end()) {
      throw std::invalid_argument(
          "Constraint::SetSparsityPatten(): was given entries with repeated "
          "values.");
    }
    nonzero_entries.insert(it, nonzero_entry);
  }
}
}  // namespace

void EvaluatorBase::SetGradientSparsityPattern(
    const std::vector<std::pair<int, int>>& gradient_sparsity_pattern) {
  if (kDrakeAssertIsArmed) {
    CheckGradientSparsityPattern(gradient_sparsity_pattern, num_outputs(),
                                 num_vars());
  }
  gradient_sparsity_pattern_.emplace(gradient_sparsity_pattern);
}

void PolynomialEvaluator::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                 Eigen::VectorXd* y) const {
  double_evaluation_point_temp_.clear();
  for (size_t i = 0; i < poly_vars_.size(); i++) {
    double_evaluation_point_temp_[poly_vars_[i]] = x[i];
  }
  y->resize(num_outputs());
  for (int i = 0; i < num_outputs(); i++) {
    (*y)[i] =
        polynomials_[i].EvaluateMultivariate(double_evaluation_point_temp_);
  }
}

void PolynomialEvaluator::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                 AutoDiffVecXd* y) const {
  taylor_evaluation_point_temp_.clear();
  for (size_t i = 0; i < poly_vars_.size(); i++) {
    taylor_evaluation_point_temp_[poly_vars_[i]] = x[i];
  }
  y->resize(num_outputs());
  for (int i = 0; i < num_outputs(); i++) {
    (*y)[i] =
        polynomials_[i].EvaluateMultivariate(taylor_evaluation_point_temp_);
  }
}

}  // namespace solvers
}  // namespace drake
