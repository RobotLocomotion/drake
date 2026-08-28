#pragma once

#include <type_traits>

#include "drake/common/unused.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace planning {
namespace continuous_collision {
namespace internal {

/* The closed set of shape classes this package recognizes. The enumerator
values are the row and column indices of the documented-accuracy table in
continuous_collision_checker.cc, so they must stay contiguous from zero.
Anything outside the set is `kUnsupported` and is refused by the oracle's
capability probe, mirroring the throw-on-unknown-shape rule
ComputeBoundingSphere() uses. */
enum class ShapeClass {
  kSphere = 0,
  kBox = 1,
  kCapsule = 2,
  kCylinder = 3,
  kEllipsoid = 4,
  kConvex = 5,
  kMesh = 6,
  kHalfSpace = 7,
  kUnsupported = 8,
};

constexpr int kNumShapeClasses = 9;

/* Classifies `shape` into the set above. */
inline ShapeClass Classify(const geometry::Shape& shape) {
  return shape.Visit<ShapeClass>([](const auto& s) {
    using S = std::decay_t<decltype(s)>;
    unused(s);
    if constexpr (std::is_same_v<S, geometry::Sphere>) {
      return ShapeClass::kSphere;
    } else if constexpr (std::is_same_v<S, geometry::Box>) {
      return ShapeClass::kBox;
    } else if constexpr (std::is_same_v<S, geometry::Capsule>) {
      return ShapeClass::kCapsule;
    } else if constexpr (std::is_same_v<S, geometry::Cylinder>) {
      return ShapeClass::kCylinder;
    } else if constexpr (std::is_same_v<S, geometry::Ellipsoid>) {
      return ShapeClass::kEllipsoid;
    } else if constexpr (std::is_same_v<S, geometry::Convex>) {
      return ShapeClass::kConvex;
    } else if constexpr (std::is_same_v<S, geometry::Mesh>) {
      return ShapeClass::kMesh;
    } else if constexpr (std::is_same_v<S, geometry::HalfSpace>) {
      return ShapeClass::kHalfSpace;
    } else {
      return ShapeClass::kUnsupported;
    }
  });
}

/* True iff `shape` is a HalfSpace. A halfspace is unbounded, so it has no
bounding sphere, and Drake computes signed distance against it only for a
Sphere partner. */
inline bool IsHalfSpace(const geometry::Shape& shape) {
  return Classify(shape) == ShapeClass::kHalfSpace;
}

}  // namespace internal
}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake
