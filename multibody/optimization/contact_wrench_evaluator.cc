#include "drake/multibody/optimization/contact_wrench_evaluator.h"

namespace drake {
namespace multibody {
template <typename T, typename U>
void ContactWrenchFromForceInWorldFrameEvaluator::DoEvalGeneric(
    const Eigen::Ref<const VectorX<T>>& x, VectorX<U>* y) const {
  y->resize(6);
  y->template head<3>().setZero();
  const auto lambda_val = lambda(x);
  (*y)(3) = static_cast<U>(lambda_val(0));
  (*y)(4) = static_cast<U>(lambda_val(1));
  (*y)(5) = static_cast<U>(lambda_val(2));
}

void ContactWrenchFromForceInWorldFrameEvaluator::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const {
  DoEvalGeneric(x, y);
}

void ContactWrenchFromForceInWorldFrameEvaluator::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) const {
  DoEvalGeneric(x, y);
}

void ContactWrenchFromForceInWorldFrameEvaluator::DoEval(
    const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
    VectorX<symbolic::Expression>* y) const {
  DoEvalGeneric(x, y);
}
}  // namespace multibody
}  // namespace drake
