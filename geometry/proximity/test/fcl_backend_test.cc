#include "drake/geometry/proximity/fcl_backend.h"

#include <map>

#include <fcl/fcl.h>
#include <gtest/gtest.h>

namespace drake {
namespace geometry {
namespace internal {
namespace {

// Every enumerator in fcl::NODE_TYPE has an explicit expectation: the seven
// geometry types Drake registers map to their Drake names, and everything else
// is "Unsupported".
GTEST_TEST(FclBackendTest, NodeTypeName) {
  const std::map<fcl::NODE_TYPE, std::string_view> supported{
      {fcl::GEOM_BOX, "Box"},
      {fcl::GEOM_CAPSULE, "Capsule"},
      {fcl::GEOM_CONVEX, "Convex"},
      {fcl::GEOM_CYLINDER, "Cylinder"},
      {fcl::GEOM_ELLIPSOID, "Ellipsoid"},
      {fcl::GEOM_HALFSPACE, "Halfspace"},
      {fcl::GEOM_SPHERE, "Sphere"},
  };

  int supported_count = 0;
  // NODE_COUNT is FCL's sentinel; it is included to show that it, too, is
  // handled.
  for (int i = fcl::BV_UNKNOWN; i <= fcl::NODE_COUNT; ++i) {
    const auto node_type = static_cast<fcl::NODE_TYPE>(i);
    const std::string_view name = FclBackend::NodeTypeName(node_type);
    const auto iter = supported.find(node_type);
    if (iter != supported.end()) {
      EXPECT_EQ(name, iter->second) << "node type " << i;
      ++supported_count;
    } else {
      EXPECT_EQ(name, "Unsupported") << "node type " << i;
    }
  }
  EXPECT_EQ(supported_count, ssize(supported));
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
