#include "drake/math/gray_code.h"

namespace drake {
namespace math {
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
}  // namespace math
}  // namespace drake
