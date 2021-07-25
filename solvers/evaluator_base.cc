#include "drake/solvers/evaluator_base.h"

#include <set>

#include "drake/common/drake_throw.h"
#include "drake/common/nice_type_name.h"

using std::make_shared;
using std::shared_ptr;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace solvers {

std::ostream& EvaluatorBase::Display(
    std::ostream& os, const VectorX<symbolic::Variable>& vars) const {
  const int num_vars = this->num_vars();
  DRAKE_THROW_UNLESS(vars.rows() == num_vars || num_vars == Eigen::Dynamic);
  return this->DoDisplay(os, vars);
}

std::ostream& EvaluatorBase::Display(std::ostream& os) const {
  if (this->num_vars() == Eigen::Dynamic) {
    symbolic::Variable dynamic_var("dynamic_sized_variable");
    return this->DoDisplay(os, Vector1<symbolic::Variable>(dynamic_var));
  }
  return this->DoDisplay(
      os, symbolic::MakeVectorContinuousVariable(this->num_vars(), "$"));
}

std::ostream& EvaluatorBase::DoDisplay(
    std::ostream& os, const VectorX<symbolic::Variable>& vars) const {
  // Display the evaluator's most derived type name.
  os << NiceTypeName::RemoveNamespaces(NiceTypeName::Get(*this));

  // Append the description (when provided).
  const std::string& description = get_description();
  if (!description.empty()) {
    os << " described as '" << description << "'";
  }

  // Append the bound decision variables (when provided).
  const int vars_rows = vars.rows();
  os << " with " << vars_rows << " decision variables";
  for (int i = 0; i < vars_rows; ++i) {
    os << " " << vars(i).get_name();
  }
  os << "\n";

  return os;
}

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

std::ostream& operator<<(std::ostream& os, const EvaluatorBase& e) {
  return e.Display(os);
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
