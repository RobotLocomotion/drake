#include "drake/geometry/geometry_instance.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {
namespace {

using Pose = Isometry3<double>;

// Confirms that the instance is copyable.
GTEST_TEST(GeometryInstanceTest, IsCopyable) {
  EXPECT_TRUE(is_copyable_unique_ptr_compatible<GeometryInstance>::value);
}

}  // namespace
}  // namespace geometry
}  // namespace drake
