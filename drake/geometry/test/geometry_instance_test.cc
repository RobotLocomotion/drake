#include "drake/geometry/geometry_instance.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace {

using std::make_unique;
using std::move;

// Confirms that the instance is copyable.
GTEST_TEST(GeometryInstanceTest, IsCopyable) {
  EXPECT_TRUE(is_copyable_unique_ptr_compatible<GeometryInstance>::value);
}

GTEST_TEST(GeometryInstanceTest, IdCopies) {
  Isometry3<double> pose = Isometry3<double>::Identity();
  auto shape = make_unique<Sphere>(1.0);
  GeometryInstance geometry_a{pose, move(shape)};
  GeometryInstance geometry_b(geometry_a);
  EXPECT_EQ(geometry_a.id(), geometry_b.id());

  shape = make_unique<Sphere>(2.0);
  GeometryInstance geometry_c{pose, move(shape)};
  EXPECT_NE(geometry_a.id(), geometry_c.id());
  geometry_c = geometry_a;
  EXPECT_EQ(geometry_a.id(), geometry_c.id());
}

}  // namespace
}  // namespace geometry
}  // namespace drake
