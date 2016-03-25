#ifndef EIGEN_COMPARE_MATRIX_H_
#define EIGEN_COMPARE_MATRIX_H_

#include "gmock/gmock.h"
#include <Eigen/Dense>

using ::testing::MakePolymorphicMatcher;
using ::testing::PolymorphicMatcher;
using ::testing::MatcherInterface;
using ::testing::MatchResultListener;

template<typename Derived>
class EigenMatrixIsApproximatelyEqualMatcher {
 public:
  explicit EigenMatrixIsApproximatelyEqualMatcher(Eigen::MatrixBase<Derived> & mm)
      : mm_(mm) { }

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

    for (size_t ii = 0; result && ii < mm.rows(); ii++) {
      for (size_t jj = 0; result && jj < mm.cols(); jj++) {

        // TODO: Add logic for handling infinity, NaN, and tolerance
        if (mm(ii,jj) != mm_(ii,jj)) {
          result = false;
        }

        if (!result) {
          *listener << "mismatch in location (" << ii << ", " << jj << "): " << mm(ii, jj) << " vs. " << mm_(ii, jj);
        }
      }
    }

    return result;  // temp!
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
};

template <typename Derived>
inline PolymorphicMatcher<EigenMatrixIsApproximatelyEqualMatcher<Derived>> EigenMatrixIsApproximatelyEqual(Eigen::MatrixBase<Derived>& mm) {
  return MakePolymorphicMatcher(EigenMatrixIsApproximatelyEqualMatcher<Derived>(mm));
}

#endif
