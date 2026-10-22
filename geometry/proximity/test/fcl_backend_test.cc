#include "drake/geometry/proximity/fcl_backend.h"

#include <map>

#include <fcl/fcl.h>
#include <fmt/format.h>
#include <gtest/gtest.h>

namespace drake {
namespace geometry {
namespace internal {
namespace {

// Every enumerator in fcl::NODE_TYPE has an explicit expectation: the seven
// geometry types Drake registers map to their ShapeKind, and everything else
// is kUnsupported.
GTEST_TEST(FclBackendTest, Classify) {
  const std::map<fcl::NODE_TYPE, ShapeKind> supported{
      {fcl::GEOM_BOX, ShapeKind::kBox},
      {fcl::GEOM_CAPSULE, ShapeKind::kCapsule},
      {fcl::GEOM_CONVEX, ShapeKind::kConvex},
      {fcl::GEOM_CYLINDER, ShapeKind::kCylinder},
      {fcl::GEOM_ELLIPSOID, ShapeKind::kEllipsoid},
      {fcl::GEOM_HALFSPACE, ShapeKind::kHalfspace},
      {fcl::GEOM_SPHERE, ShapeKind::kSphere},
  };

  int supported_count = 0;
  // NODE_COUNT is FCL's sentinel; it is included to show that it, too, is
  // handled.
  for (int i = fcl::BV_UNKNOWN; i <= fcl::NODE_COUNT; ++i) {
    SCOPED_TRACE(fmt::format("node type {}", i));
    const auto node_type = static_cast<fcl::NODE_TYPE>(i);
    const ShapeKind kind = FclBackend::Classify(node_type);
    const auto iter = supported.find(node_type);
    if (iter != supported.end()) {
      EXPECT_EQ(kind, iter->second);
      ++supported_count;
    } else {
      EXPECT_EQ(kind, ShapeKind::kUnsupported);
    }
  }
  EXPECT_EQ(supported_count, ssize(supported));
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
