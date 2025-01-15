#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/fmt_eigen.h"

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
 * @return true if the two matrices are equal based on the specified tolerance.
 */
template <typename DerivedA, typename DerivedB>
[[nodiscard]] ::testing::AssertionResult CompareMatrices(
    const Eigen::MatrixBase<DerivedA>& m1,
    const Eigen::MatrixBase<DerivedB>& m2, double tolerance = 0.0,
    MatrixCompareType compare_type = MatrixCompareType::absolute) {
  if (m1.rows() != m2.rows() || m1.cols() != m2.cols()) {
    const std::string message =
        fmt::format("Matrix size mismatch: ({} x {} vs. {} x {})", m1.rows(),
                    m1.cols(), m2.rows(), m2.cols());
    return ::testing::AssertionFailure() << message;
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
        const std::string message =
            fmt::format("NaN mismatch at ({}, {}):\nm1 =\n{}\nm2 =\n{}", ii, jj,
                        fmt_eigen(m1), fmt_eigen(m2));
        return ::testing::AssertionFailure() << message;
      }

      // Determine whether the difference between the two matrices is less than
      // the tolerance.
      using std::abs;
      const auto delta = abs(m1(ii, jj) - m2(ii, jj));

      if (compare_type == MatrixCompareType::absolute) {
        // Perform comparison using absolute tolerance.
        if (delta > tolerance) {
          const std::string message = fmt::format(
              "Values at ({}, {}) exceed tolerance: {} vs. {}, diff = {}, "
              "tolerance = {}\nm1 =\n{}\nm2 =\n{}\ndelta=\n{}",
              ii, jj, m1(ii, jj), m2(ii, jj), delta, tolerance, fmt_eigen(m1),
              fmt_eigen(m2), fmt_eigen(m1 - m2));
          return ::testing::AssertionFailure() << message;
        }
      } else {
        // Perform comparison using relative tolerance, see:
        // http://realtimecollisiondetection.net/blog/?p=89
        using std::max;
        const auto max_value = max(abs(m1(ii, jj)), abs(m2(ii, jj)));
        const auto relative_tolerance =
            tolerance * max(decltype(max_value){1}, max_value);
        if (delta > relative_tolerance) {
          const std::string message = fmt::format(
              "Values at ({}, {}) exceed tolerance: {} vs. {}, diff = {}, "
              "tolerance = {}, relative tolerance = {}\nm1 =\n{}\nm2 "
              "=\n{}\ndelta=\n{}",
              ii, jj, m1(ii, jj), m2(ii, jj), delta, tolerance,
              relative_tolerance, fmt_eigen(m1), fmt_eigen(m2),
              fmt_eigen(m1 - m2));
          return ::testing::AssertionFailure() << message;
        }
      }
    }
  }

  const std::string message =
      fmt::format("m1 =\n{}\nis approximately equal to m2 =\n{}", fmt_eigen(m1),
                  fmt_eigen(m2));
  return ::testing::AssertionSuccess() << message;
}

}  // namespace drake
