#include "drake/common/extract_double.h"

#include <stdexcept>

#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <unsupported/Eigen/AutoDiff>

// A non-numeric ScalarType for testing.
namespace { struct NonNumericScalar { }; }
namespace drake {
template <>
struct is_numeric<NonNumericScalar> {
  static constexpr bool value = false;
};
}  // namespace drake

namespace drake {
namespace {

GTEST_TEST(ExtractDoubleTest, BasicTest) {
  // A double works.
  double x = 1.0;
  EXPECT_EQ(ExtractDoubleOrThrow(x), 1.0);

  // A const double works.
  const double y = 2.0;
  EXPECT_EQ(ExtractDoubleOrThrow(y), 2.0);

  // A non-numeric throws.
  NonNumericScalar non_numeric;
  EXPECT_THROW(ExtractDoubleOrThrow(non_numeric), std::exception);
}

}  // namespace
}  // namespace drake
