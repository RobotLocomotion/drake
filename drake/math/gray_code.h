#pragma once

#include <Eigen/Core>

namespace drake {
namespace math {
template<int NumDigits>
struct GrayCodesMatrix {
  typedef typename Eigen::Matrix<int, NumDigits <= 0 ? Eigen::Dynamic : 1 << NumDigits, NumDigits <= 0 ? Eigen::Dynamic : NumDigits> type;
};

template<int NumDigits = 0>
typename GrayCodesMatrix<NumDigits>::type
CalculateReflectedGrayCodes(int num_digits = NumDigits) {
  typename GrayCodesMatrix<NumDigits>::type gray_codes(1 << num_digits, num_digits);
  int num_codes = gray_codes.rows();
  gray_codes.setZero();
  // TODO(hongkai.dai): implement this part more efficiently.
  for (int i = 0; i < num_codes; ++i) {
    int gray_code = i ^ (i >> 1);
    for (int j = 0; j < num_digits; ++j) {
      gray_codes(i, j) = (gray_code & (1 << (num_digits - j - 1))) >> (num_digits - j - 1);
    }
  }
  return gray_codes;
}

/**
 * Converts the Gray code to an integer.
 * (0, 0) -> 0
 * (0, 1) -> 1
 * (1, 1) -> 2
 * (1, 0) -> 3
 * @param gray_code The N-digit Gray code, where N is gray_code.rows()
 * @return The integer represented by the Gray code `gray_code`.
 */
int GrayCodeToInteger(const Eigen::Ref<const Eigen::VectorXi>& gray_code);
}  // namespace math
}  // namespace drake
