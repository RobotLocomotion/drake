#ifndef DRAKE_UTIL_TEST_EIGEN_COMPARE_MATRIX_MOCK_H_
#define DRAKE_UTIL_TEST_EIGEN_COMPARE_MATRIX_MOCK_H_

#include "gmock/gmock.h"
#include <Eigen/Dense>

#include "drake/util/eigen_matrix_compare.h"

using ::testing::MakePolymorphicMatcher;
using ::testing::PolymorphicMatcher;
using ::testing::MatcherInterface;
using ::testing::MatchResultListener;

using ::drake::util::MatrixCompareType;

namespace drake {
namespace test {

/**
 * Implements a Google Mock matcher for comparing two Eigen matrices.
 * It supports both relative and absolute tolerances and can handle
 * infinity and NaN corner cases.
 */
template<typename Derived>
class EigenMatrixIsApproximatelyEqualMatcher {
 public:
  explicit EigenMatrixIsApproximatelyEqualMatcher(
    Eigen::MatrixBase<Derived> & mm, double tolerance,
    MatrixCompareType compare_type)
      : mm_(mm),
        tolerance_(tolerance),
        compare_type_(compare_type) { }

  template <typename T>
  bool MatchAndExplain(T& mm,
                       MatchResultListener * listener) const {

    bool result = true;

    if (mm.rows() != mm_.rows()) {
      *listener << "rows do not match: " << mm.rows() << " vs. " << mm_.rows();
      result = false;
    }

    if (result && mm.cols() != mm_.cols()) {
      *listener << "columns do not match: " << mm.cols() << " vs. " << mm_.cols();
      result = false;
    }

    if (result) {
      std::string error_msg;
      result = CompareMatrices(mm, mm_, tolerance_, compare_type_, error_msg);
      if (!result)
        *listener << error_msg;
    }

    return result;
  }

  /**
   * Describes the property of a value matching this matcher.
   */
  virtual void DescribeTo(::std::ostream* os) const {
    *os << "is approximately equal to";
  }

  /**
   * Describes the property of a value NOT matching this matcher.
   */
  virtual void DescribeNegationTo(::std::ostream* os) const {
    *os << "is not approximately equal to";
  }

 private:
  Eigen::MatrixBase<Derived> & mm_;
  double tolerance_;
  MatrixCompareType compare_type_;
};

template <typename Derived>
inline PolymorphicMatcher<EigenMatrixIsApproximatelyEqualMatcher<Derived>>
EigenMatrixIsApproximatelyEqual(Eigen::MatrixBase<Derived>& mm, double tolerance, MatrixCompareType compare_type) {
  return MakePolymorphicMatcher(EigenMatrixIsApproximatelyEqualMatcher<Derived>(mm, tolerance, compare_type));
}

}  // namespace test
}  // namespace drake

#endif
