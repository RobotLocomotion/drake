#include "drake/common/eigen_autodiff_types.h"

#include <gtest/gtest.h>

namespace drake {
namespace {
GTEST_TEST(EigenAutodiffTypesTest, CheckingInheritance) {
  typedef double Scalar;
  typedef Eigen::Matrix<Scalar, 2, 2> Deriv;
  typedef Eigen::AutoDiffScalar<Deriv> AD;

  typedef std::numeric_limits<AD> ADLimits;
  typedef std::numeric_limits<Scalar> ScalarLimits;

  bool res = std::is_base_of<ScalarLimits, ADLimits>::value;
  EXPECT_TRUE(res);
}
}  // namespace
}  // namespace drake
