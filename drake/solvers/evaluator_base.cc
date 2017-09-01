#include "drake/solvers/evaluator_base.h"

using std::make_shared;
using std::shared_ptr;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace solvers {

void PolynomialEvaluator::DoEval(const Eigen::Ref<const Eigen::VectorXd> &x,
                                 Eigen::VectorXd &y) const {
  double_evaluation_point_temp_.clear();
  for (size_t i = 0; i < poly_vars_.size(); i++) {
    double_evaluation_point_temp_[poly_vars_[i]] = x[i];
  }
  y.resize(num_outputs());
  for (int i = 0; i < num_outputs(); i++) {
    y[i] = polynomials_[i].EvaluateMultivariate(double_evaluation_point_temp_);
  }
}

void PolynomialEvaluator::DoEval(const Eigen::Ref<const AutoDiffVecXd> &x,
                                 AutoDiffVecXd &y) const {
  taylor_evaluation_point_temp_.clear();
  for (size_t i = 0; i < poly_vars_.size(); i++) {
    taylor_evaluation_point_temp_[poly_vars_[i]] = x[i];
  }
  y.resize(num_outputs());
  for (int i = 0; i < num_outputs(); i++) {
    y[i] = polynomials_[i].EvaluateMultivariate(taylor_evaluation_point_temp_);
  }
}

}  // namespace solvers
}  // namespace drake
