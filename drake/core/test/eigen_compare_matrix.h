#ifndef EIGEN_COMPARE_MATRIX_H_
#define EIGEN_COMPARE_MATRIX_H_

#include "gmock/gmock.h"
#include <Eigen/Dense>

using ::testing::MakeMatcher;
using ::testing::Matcher;
using ::testing::MatcherInterface;
using ::testing::MatchResultListener;

template<typename Derived>
class EigenMatrixIsApproximatelyEqualMatcher : public MatcherInterface<Eigen::MatrixBase<Derived>> {
 public:
  explicit EigenMatrixIsApproximatelyEqualMatcher(Eigen::MatrixBase<Derived> mm) 
      : mm_(mm) { }

  virtual bool MatchAndExplain(Eigen::MatrixBase<Derived> mm, MatchResultListener* listener) const {
    return false;  // temp!
  }

  virtual void DescribeTo(::std::ostream* os) const {
    *os << "is approximately equal to";
  }

  virtual void DescribeNegationTo(::std::ostream* os) const {
    *os << "is not approximately equal to";
  }

 private:
  Eigen::MatrixBase<Derived> mm_;
};

template<typename Derived>
inline Matcher<Eigen::MatrixBase<Derived>> EigenMatrixIsApproximatelyEqual(Eigen::MatrixBase<Derived> mm) {
  return MakeMatcher(new EigenMatrixIsApproximatelyEqualMatcher<Derived>(mm));
}

// class EigenMatrixIsApproximatelyEqualMatcher : public MatcherInterface<double> {
//  public:
//   explicit EigenMatrixIsApproximatelyEqualMatcher(double mm) 
//       : mm_(mm) { }

//   virtual bool MatchAndExplain(double mm, MatchResultListener* listener) const {
//     return mm == mm_;
//   }

//   virtual void DescribeTo(::std::ostream* os) const {
//     *os << "is approximately equal to";
//   }

//   virtual void DescribeNegationTo(::std::ostream* os) const {
//     *os << "is not approximately equal to";
//   }

//  private:
//   double mm_;
// };

// inline Matcher<double> EigenMatrixIsApproximatelyEqual(double mm) {
//   return MakeMatcher(new EigenMatrixIsApproximatelyEqualMatcher(mm));
// }

#endif
