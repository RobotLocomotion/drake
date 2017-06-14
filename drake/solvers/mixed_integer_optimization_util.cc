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
  for (int i = 0; i < num_codes; i++) {
    for (int j = 0; j < k; j++) {
      // The jth digit cycles
      // 2^j times off, then 2^j times on.
      // The jth digit goes through a quarter-period
      // (2^j / 2) off first.
      int quarter_period_time = 1 << (j);
      int half_period_time = 2 << (j);
      int full_period_time = 4 << (j);
      if ((i + quarter_period_time) % full_period_time >= half_period_time) {
        return_codes(i, j) = 1;
      }
    }
  }
  return return_codes;
}
}  // namespace internal

}  // namespace solvers
}  // namespace drake