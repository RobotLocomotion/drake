#include <gtest/gtest.h>

#include "drake/common/autodiff.h"

namespace drake {
namespace {
GTEST_TEST(EigenAutodiffTypesTest, CheckingInheritance) {
  typedef double Scalar;
  typedef Eigen::Matrix<Scalar, 2, 2> Deriv;
  typedef Eigen::AutoDiffScalar<Deriv> AD;

  typedef std::numeric_limits<AD> ADLimits;
  typedef std::numeric_limits<Scalar> ScalarLimits;

  bool res = std::is_base_of_v<ScalarLimits, ADLimits>;
  EXPECT_TRUE(res);
}
}  // namespace
}  // namespace drake
