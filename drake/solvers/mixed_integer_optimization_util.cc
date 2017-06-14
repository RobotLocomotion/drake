#include "drake/solvers/mixed_integer_optimization_util.h"

namespace drake {
namespace solvers {
namespace internal {
int GrayCodeToInteger(const Eigen::Ref<const Eigen::VectorXi>& gray_code) {
  // This implementation is based on
  // https://testbook.com/blog/conversion-from-gray-code-to-binary-code-and-vice-versa/
  int digit = gray_code(0);
  int ret = digit ? 1 << (gray_code.size() - 1) : 0;
  for (int i = 0; i < gray_code.size() - 1; ++i) {
    digit ^= gray_code(i + 1);
    ret |= digit ? 1 << (gray_code.size() - i - 2) : 0;
  }
  return ret;
}

Eigen::MatrixXi CalculateReflectedGrayCodes(int k) {
  int num_codes = k == 0 ? 0 : 1 << k;
  Eigen::MatrixXi return_codes(num_codes, k);
  return_codes.setZero();
  // TODO(hongkai.dai): implement this part more efficiently.
  for (int i = 0; i < num_codes; ++i) {
    int gray_code = i ^ (i >> 1);
    for (int j = 0; j < k; ++j) {
      return_codes(i, j) = (gray_code & (1 << (k - j - 1))) >> (k - j - 1);
    }
  }
  return return_codes;
}
}  // namespace internal

VectorXDecisionVariable AddLogarithmicSOS2Constraint(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorX<symbolic::Expression>>& lambda,
    const std::string& binary_variable_name) {
  const int num_lambda = lambda.rows();
  for (int i = 0; i < num_lambda; ++i) {
    prog->AddLinearConstraint(lambda(i) >= 0);
    prog->AddLinearConstraint(lambda(i) <= 1);
  }
  const int num_interval = num_lambda - 1;
  const int num_binary_vars = CeilLog2(num_interval);
  const auto gray_codes =
      internal::CalculateReflectedGrayCodes(num_binary_vars);
  auto y = prog->NewBinaryVariables(num_binary_vars, binary_variable_name);
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
  return y;
}
}  // namespace solvers
}  // namespace drake
