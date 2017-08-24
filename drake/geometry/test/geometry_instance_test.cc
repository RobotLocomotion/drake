#include "drake/geometry/geometry_instance.h"

#include <gtest/gtest.h>

#include "drake/common/copyable_unique_ptr.h"

namespace drake {
namespace geometry {
namespace {

// Confirms that the instance is copyable.
GTEST_TEST(GeometryInstanceTest, IsCopyable) {
  EXPECT_TRUE(is_copyable_unique_ptr_compatible<GeometryInstance>::value);
}

}  // namespace
}  // namespace geometry
}  // namespace drake
