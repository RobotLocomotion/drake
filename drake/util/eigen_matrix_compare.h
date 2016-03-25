#ifndef DRAKE_UTIL_EIGEN_MATRIX_COMPARE_H_
#define DRAKE_UTIL_EIGEN_MATRIX_COMPARE_H_

#include <Eigen/Dense>
#include <cmath>

namespace drake {
namespace util {

enum MatrixCompareType {
  absolute,
  relative
};

/**
 * Compares two matrices to determine whether they are equal to within a certain
 * threshold.
 *
 * @param m1 The first matrix to compare.
 * @param m2 The second matrix to compare.
 * @param tolerance The tolerance for determining equivalence.
 * @param compare_type Whether the tolereance is absolute or relative.
 * @param error_msg A string for storing a description of why a comparison is
 * false.
 * @return true if the two matrices match.
 */
template <typename DerivedA, typename DerivedB>
bool CompareMatrices(const Eigen::MatrixBase<DerivedA>& m1,
                           const Eigen::MatrixBase<DerivedB>& m2,
                           double tolerance,
                           MatrixCompareType compare_type,
                           std::string * error_msg) {
  bool result = true;

  if (m1.rows() != m2.rows() || m1.cols() != m2.cols()) {
    std::stringstream msg;
    msg << "Matrix size mismatch: (" << m1.rows() << " x " << m1.cols()
        << " vs. " << m2.rows() << " x " << m2.rows() << ")";
    *error_msg = msg.str();
    result = false;
  }

  for (size_t ii = 0; result && ii < m1.rows(); ii++) {
    for (size_t jj = 0; result && jj < m1.cols(); jj++) {

      // First handle the corner cases of positive infinity, negative infinity,
      // and NaN
      bool both_positive_infinity =
        m1(ii, jj) == std::numeric_limits<double>::infinity() &&
        m2(ii, jj) == std::numeric_limits<double>::infinity();

      bool both_negative_infinity =
        m1(ii, jj) == -std::numeric_limits<double>::infinity() &&
        m2(ii, jj) == -std::numeric_limits<double>::infinity();

      bool both_nan = std::isnan(m1(ii, jj)) && std::isnan(m2(ii, jj));

      if (both_positive_infinity || both_negative_infinity || both_nan){
        continue;
      }

      // Determine whether the difference between the two matrices are less than
      // the tolerance.
      bool is_within_tolerance = false;
      double delta = std::abs(m1(ii, jj) - m2(ii, jj));

      if (compare_type == MatrixCompareType::absolute) {
        // Perform comparison using absolute tolerance.
        is_within_tolerance = delta < tolerance;

        if (!is_within_tolerance) {
          std::stringstream msg;
          msg
            << "Values at (" << ii << ", " << jj << ") exceed tolerance: "
            << m1(ii, jj) << " vs. " << m2(ii, jj) << ", diff = "
            << delta << ", tolerance = " << tolerance << "\nm1 =\n" << m1
            << "\nm2 =\n" << m2 << "\ndelta=\n" << (m1 - m2);

          *error_msg = msg.str();
          result = false;
        }
      } else {
        // Perform comparison using relative tolerance, see:
        // http://realtimecollisiondetection.net/blog/?p=89
        double max_value = std::max(std::abs(m1(ii, jj)),
                                    std::abs(m2(ii, jj)));
        double relative_tolerance = tolerance * std::max(1.0, max_value);

        is_within_tolerance = delta < relative_tolerance;

        if (!is_within_tolerance) {
          std::stringstream msg;
          msg
            << "Values at (" << ii << ", " << jj << ") exceed tolerance: "
            << m1(ii, jj) << " vs. " << m2(ii, jj) << ", diff = "
            << delta << ", tolerance = " << tolerance
            << ", relative tolerance = " << relative_tolerance
            << "\nm1 =\n" << m1 << "\nm2 =\n" << m2 << "\ndelta=\n"
            << (m1 - m2);

          *error_msg = msg.str();
          result = false;
        }
      }
    }
  }

  return result;
}

}  // namespace util
}  // namespace drake

#endif  // DRAKE_UTIL_EIGEN_MATRIX_COMPARE_H_