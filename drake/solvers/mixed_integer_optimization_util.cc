#include "drake/solvers/mixed_integer_optimization_util.h"

#include "drake/math/gray_code.h"

namespace drake {
namespace solvers {
VectorXDecisionVariable AddLogarithmicSOS2Constraint(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorX<symbolic::Expression>>& lambda,
    const std::string& binary_variable_name) {
  const int num_lambda = lambda.rows();
  const int num_interval = num_lambda - 1;
  const int num_binary_vars = CeilLog2(num_interval);
  auto y = prog->NewBinaryVariables(num_binary_vars, binary_variable_name);
  AddLogarithmicSOS2Constraint(prog, lambda, y);
  return y;
}

void AddLogarithmicSOS2Constraint(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorX<symbolic::Expression>>& lambda,
    const Eigen::Ref<const VectorXDecisionVariable>& y) {
  const int num_lambda = lambda.rows();
  for (int i = 0; i < num_lambda; ++i) {
    prog->AddLinearConstraint(lambda(i) >= 0);
    prog->AddLinearConstraint(lambda(i) <= 1);
  }
  prog->AddLinearConstraint(lambda.sum() == 1);
  const int num_interval = num_lambda - 1;
  const int num_binary_vars = CeilLog2(num_interval);
  const auto gray_codes =
      math::CalculateReflectedGrayCodes(num_binary_vars);
  DRAKE_ASSERT(y.rows() == num_binary_vars);
  for (int j = 0; j < num_binary_vars; ++j) {
    symbolic::Expression lambda_sum1 = gray_codes(0, j) == 1 ? lambda(0) : 0;
    symbolic::Expression lambda_sum2 = gray_codes(0, j) == 0 ? lambda(0) : 0;
    for (int i = 1; i < num_lambda - 1; ++i) {
      lambda_sum1 +=
          (gray_codes(i - 1, j) == 1 && gray_codes(i, j) == 1) ? lambda(i) : 0;
      lambda_sum2 +=
          (gray_codes(i - 1, j) == 0 && gray_codes(i, j) == 0) ? lambda(i) : 0;
    }
    lambda_sum1 +=
        gray_codes(num_lambda - 2, j) == 1 ? lambda(num_lambda - 1) : 0;
    lambda_sum2 +=
        gray_codes(num_lambda - 2, j) == 0 ? lambda(num_lambda - 1) : 0;
    prog->AddLinearConstraint(lambda_sum1 <= y(j));
    prog->AddLinearConstraint(lambda_sum2 <= 1 - y(j));
  }
}

void AddBilinearProductMcCormickEnvelopeSOS2(
    MathematicalProgram* prog,
    const symbolic::Variable& x,
    const symbolic::Variable& y,
    const symbolic::Expression& w,
    const Eigen::Ref<const Eigen::VectorXd>& phi_x,
    const Eigen::Ref<const Eigen::VectorXd>& phi_y,
    const Eigen::Ref<const VectorXDecisionVariable>& Bx,
    const Eigen::Ref<const VectorXDecisionVariable>& By) {
  DRAKE_ASSERT(Bx.rows() == CeilLog2(phi_x.rows() - 1));
  DRAKE_ASSERT(By.rows() == CeilLog2(phi_y.rows() - 1));
  const int num_phi_x = phi_x.rows();
  const int num_phi_y = phi_y.rows();
  auto lambda = prog->NewContinuousVariables(num_phi_x, num_phi_y, "lambda");
  prog->AddBoundingBoxConstraint(0, 1, lambda);

  symbolic::Expression x_convex_combination{0};
  symbolic::Expression y_convex_combination{0};
  symbolic::Expression w_convex_combination{0};
  for (int i = 0; i < num_phi_x; ++i) {
    for (int j = 0; j < num_phi_y; ++j) {
      x_convex_combination += lambda(i, j) * phi_x(i);
      y_convex_combination += lambda(i, j) * phi_y(j);
      w_convex_combination += lambda(i, j) * phi_x(i) * phi_y(j);
    }
  }
  prog->AddLinearConstraint(x == x_convex_combination);
  prog->AddLinearConstraint(y == y_convex_combination);
  prog->AddLinearConstraint(w == w_convex_combination);

  AddLogarithmicSOS2Constraint(prog, lambda.cast<symbolic::Expression>().rowwise().sum(), Bx);
  AddLogarithmicSOS2Constraint(prog, lambda.cast<symbolic::Expression>().colwise().sum().transpose(), By);
}
}  // namespace solvers
}  // namespace drake
