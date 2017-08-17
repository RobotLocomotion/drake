#pragma once

#include <Eigen/Core>

#include "drake/common/drake_assert.h"

namespace drake {
namespace math {
/**
 * GrayCodesMatrix::type returns an Eigen matrix of integers. The size of this
 * matrix is determined by the number of digits in the Gray code.
 */
template<int NumDigits>
struct GrayCodesMatrix {
  static_assert(NumDigits >= 0 && NumDigits <= 30, "NumDigits out of range.");
  typedef typename Eigen::Matrix<int, NumDigits == 0 ? 0 : 1 << NumDigits,
                                 NumDigits>
      type;
};

template<>
struct GrayCodesMatrix<Eigen::Dynamic> {
  typedef typename Eigen::MatrixXi type;
};

/**
 * Returns a matrix whose i'th row is the Gray code for integer i.
 * @tparam NumDigits The number of digits in the Gray code.
 * @param num_digits The number of digits in the Gray code.
 * @return M. M is a matrix of size 2ᵏ x k, where `k` is `num_digits`.
 * M.row(i) is the Gray code for integer i.
 */
template<int NumDigits = Eigen::Dynamic>
typename GrayCodesMatrix<NumDigits>::type
CalculateReflectedGrayCodes(int num_digits = NumDigits) {
  DRAKE_DEMAND(num_digits >= 0);
  int num_codes = num_digits <= 0 ? 0 : 1 << num_digits;
  typename GrayCodesMatrix<NumDigits>::type gray_codes(num_codes, num_digits);
  // TODO(hongkai.dai): implement this part more efficiently.
  for (int i = 0; i < num_codes; ++i) {
    int gray_code = i ^ (i >> 1);
    for (int j = 0; j < num_digits; ++j) {
      gray_codes(i, j) =
          (gray_code & (1 << (num_digits - j - 1))) >> (num_digits - j - 1);
    }
  }
  return gray_codes;
}

/**
 * Converts the Gray code to an integer. For example
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
