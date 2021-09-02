#include "drake/solvers/test_utilities/check_constraint_eval_nonsymbolic.h"

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/compute_numerical_gradient.h"

namespace drake {
namespace solvers {
namespace test {
/** Compare the result between Eval<double> and Eval<AutoDiffXd>. Also compare
    the gradient in Eval<AutoDiffXd> with a numerical approximation. */
void CheckConstraintEvalNonsymbolic(
    const Constraint& constraint,
    const Eigen::Ref<const AutoDiffVecXd>& x_autodiff, double tol) {
  const Eigen::VectorXd x_double{math::ExtractValue(x_autodiff)};
  Eigen::VectorXd y_double;
  constraint.Eval(x_double, &y_double);
  AutoDiffVecXd y_autodiff;
  constraint.Eval(x_autodiff, &y_autodiff);
  EXPECT_TRUE(CompareMatrices(y_double, math::ExtractValue(y_autodiff), tol));
  std::function<void(const Eigen::Ref<const Eigen::VectorXd>&,
                     Eigen::VectorXd*)>
      constraint_eval =
          [&constraint](const Eigen::Ref<const Eigen::VectorXd>& x,
                        Eigen::VectorXd* y) { return constraint.Eval(x, y); };
  const auto J = math::ComputeNumericalGradient(
      constraint_eval, x_double,
      math::NumericalGradientOption{math::NumericalGradientMethod::kCentral});
  EXPECT_TRUE(CompareMatrices(J * math::ExtractGradient(x_autodiff),
                              math::ExtractGradient(y_autodiff),
                              std::sqrt(tol)));
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
