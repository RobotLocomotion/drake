#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/text_logging.h"

namespace drake {

enum class MatrixCompareType { absolute, relative };

/**
 * Compares two matrices to determine whether they are equal to within a certain
 * threshold.
 *
 * @param m1 The first matrix to compare.
 * @param m2 The second matrix to compare.
 * @param tolerance The tolerance for determining equivalence.
 * @param compare_type Whether the tolereance is absolute or relative.
 * @param explanation A pointer to a string variable for saving an explanation
 * of why @p m1 and @p m2 are unequal. This parameter is optional and defaults
 * to `nullptr`. If this is `nullptr` and @p m1 and @p m2 are not equal, an
 * explanation is logged as an error message.
 * @return true if the two matrices are equal based on the specified tolerance.
 */
template <typename DerivedA, typename DerivedB>
[[nodiscard]] ::testing::AssertionResult CompareMatrices(
    const Eigen::MatrixBase<DerivedA>& m1,
    const Eigen::MatrixBase<DerivedB>& m2, double tolerance = 0.0,
    MatrixCompareType compare_type = MatrixCompareType::absolute) {
  if (m1.rows() != m2.rows() || m1.cols() != m2.cols()) {
    return ::testing::AssertionFailure()
           << "Matrix size mismatch: (" << m1.rows() << " x " << m1.cols()
           << " vs. " << m2.rows() << " x " << m2.cols() << ")";
  }

  for (int ii = 0; ii < m1.rows(); ii++) {
    for (int jj = 0; jj < m1.cols(); jj++) {
      // First handle the corner cases of positive infinity, negative infinity,
      // and NaN
      const auto both_positive_infinity =
          m1(ii, jj) == std::numeric_limits<double>::infinity() &&
          m2(ii, jj) == std::numeric_limits<double>::infinity();

      const auto both_negative_infinity =
          m1(ii, jj) == -std::numeric_limits<double>::infinity() &&
          m2(ii, jj) == -std::numeric_limits<double>::infinity();

      using std::isnan;
      const auto both_nan = isnan(m1(ii, jj)) && isnan(m2(ii, jj));

      if (both_positive_infinity || both_negative_infinity || both_nan)
        continue;

      // Check for case where one value is NaN and the other is not
      if ((isnan(m1(ii, jj)) && !isnan(m2(ii, jj))) ||
          (!isnan(m1(ii, jj)) && isnan(m2(ii, jj)))) {
        return ::testing::AssertionFailure() << "NaN mismatch at (" << ii
                                             << ", " << jj << "):\nm1 =\n"
                                             << m1 << "\nm2 =\n"
                                             << m2;
      }

      // Determine whether the difference between the two matrices is less than
      // the tolerance.
      using std::abs;
      const auto delta = abs(m1(ii, jj) - m2(ii, jj));

      if (compare_type == MatrixCompareType::absolute) {
        // Perform comparison using absolute tolerance.

        if (delta > tolerance) {
          return ::testing::AssertionFailure()
                 << "Values at (" << ii << ", " << jj
                 << ") exceed tolerance: " << m1(ii, jj) << " vs. "
                 << m2(ii, jj) << ", diff = " << delta
                 << ", tolerance = " << tolerance << "\nm1 =\n"
                 << m1 << "\nm2 =\n"
                 << m2 << "\ndelta=\n"
                 << (m1 - m2);
        }
      } else {
        // Perform comparison using relative tolerance, see:
        // http://realtimecollisiondetection.net/blog/?p=89
        using std::max;
        const auto max_value = max(abs(m1(ii, jj)), abs(m2(ii, jj)));
        const auto relative_tolerance =
            tolerance * max(decltype(max_value){1}, max_value);

        if (delta > relative_tolerance) {
          return ::testing::AssertionFailure()
                 << "Values at (" << ii << ", " << jj
                 << ") exceed tolerance: " << m1(ii, jj) << " vs. "
                 << m2(ii, jj) << ", diff = " << delta
                 << ", tolerance = " << tolerance
                 << ", relative tolerance = " << relative_tolerance
                 << "\nm1 =\n"
                 << m1 << "\nm2 =\n"
                 << m2 << "\ndelta=\n"
                 << (m1 - m2);
        }
      }
    }
  }

  return ::testing::AssertionSuccess() << "m1 =\n"
                                       << m1
                                       << "\nis approximately equal to m2 =\n"
                                       << m2;
}

}  // namespace drake
