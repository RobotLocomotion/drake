#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include <Eigen/Dense>

#include "drake/common/drake_compat.h"
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
bool CompareMatrices(
    const Eigen::MatrixBase<DerivedA>& m1,
    const Eigen::MatrixBase<DerivedB>& m2, double tolerance = 0.0,
    MatrixCompareType compare_type = MatrixCompareType::absolute,
    std::string* explanation = nullptr) {
  bool result = true;
  std::string error_message;

  if (m1.rows() != m2.rows() || m1.cols() != m2.cols()) {
    std::stringstream msg;
    msg << "Matrix size mismatch: (" << m1.rows() << " x " << m1.cols()
        << " vs. " << m2.rows() << " x " << m2.cols() << ")";
    error_message = msg.str();
    result = false;
  }

  for (int ii = 0; result && ii < m1.rows(); ii++) {
    for (int jj = 0; result && jj < m1.cols(); jj++) {
      // First handle the corner cases of positive infinity, negative infinity,
      // and NaN
      bool both_positive_infinity =
          m1(ii, jj) == std::numeric_limits<double>::infinity() &&
          m2(ii, jj) == std::numeric_limits<double>::infinity();

      bool both_negative_infinity =
          m1(ii, jj) == -std::numeric_limits<double>::infinity() &&
          m2(ii, jj) == -std::numeric_limits<double>::infinity();

      bool both_nan = std::isnan(m1(ii, jj)) && std::isnan(m2(ii, jj));

      if (both_positive_infinity || both_negative_infinity || both_nan)
        continue;

      // Check for case where one value is NaN and the other is not
      if ((std::isnan(m1(ii, jj)) && !std::isnan(m2(ii, jj))) ||
          (!std::isnan(m1(ii, jj)) && std::isnan(m2(ii, jj)))) {
        std::stringstream msg;
        msg << "NaN missmatch at (" << ii << ", " << jj << "):\nm1 =\n"
            << m1 << "\nm2 =\n"
            << m2;
        error_message = msg.str();
        result = false;
        continue;
      }

      // Determine whether the difference between the two matrices is less than
      // the tolerance.
      double delta = std::abs(m1(ii, jj) - m2(ii, jj));

      if (compare_type == MatrixCompareType::absolute) {
        // Perform comparison using absolute tolerance.

        if (delta > tolerance) {
          std::stringstream msg;
          msg << "Values at (" << ii << ", " << jj
              << ") exceed tolerance: " << m1(ii, jj) << " vs. " << m2(ii, jj)
              << ", diff = " << delta << ", tolerance = " << tolerance
              << "\nm1 =\n"
              << m1 << "\nm2 =\n"
              << m2 << "\ndelta=\n"
              << (m1 - m2);

          error_message = msg.str();
          result = false;
        }
      } else {
        // Perform comparison using relative tolerance, see:
        // http://realtimecollisiondetection.net/blog/?p=89
        double max_value = std::max(std::abs(m1(ii, jj)), std::abs(m2(ii, jj)));
        double relative_tolerance = tolerance * std::max(1.0, max_value);

        if (delta > relative_tolerance) {
          std::stringstream msg;
          msg << "Values at (" << ii << ", " << jj
              << ") exceed tolerance: " << m1(ii, jj) << " vs. " << m2(ii, jj)
              << ", diff = " << delta << ", tolerance = " << tolerance
              << ", relative tolerance = " << relative_tolerance << "\nm1 =\n"
              << m1 << "\nm2 =\n"
              << m2 << "\ndelta=\n"
              << (m1 - m2);
          error_message = msg.str();
          result = false;
        }
      }
    }
  }

  if (!result) {
    if (explanation != nullptr)
      *explanation = error_message;
    else
      drake::log()->error(error_message);
  }

  return result;
}

}  // namespace drake
