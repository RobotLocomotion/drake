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

GTEST_TEST(EigenAutodiffTypesTest, CacheLine) {
  // The AutoDiff6d must fit in a 64-byte cache line.  It has 1 value and
  // 6 partials.  That's at least 56 bytes, but padding may push it larger.
  EXPECT_LE(sizeof(AutoDiff6d), 64);

  // The derivatives must be aligned within their containing AutoDiff6d.
  const AutoDiff6d scalar;
  const void* const starting_address = &scalar;
  const void* const derivatives_address = &scalar.derivatives();
  EXPECT_EQ(
      static_cast<const char*>(derivatives_address) -
          static_cast<const char*>(starting_address),
      EIGEN_MAX_STATIC_ALIGN_BYTES);
}

}  // namespace
}  // namespace drake
