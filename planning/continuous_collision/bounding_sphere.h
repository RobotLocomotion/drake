#pragma once

#include <Eigen/Core>

#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace planning {
namespace continuous_collision {

/** A sphere, expressed in the owning body (link) frame L, that contains a
proximity geometry at every configuration of the body.
@ingroup planning_collision_checker */
struct BoundingSphere {
  /** Sphere center in the body frame. */
  Eigen::Vector3d center_L{Eigen::Vector3d::Zero()};
  double radius{0.0};
};

/** Computes a bounding sphere, in the body frame, of shape `shape` posed at
X_LG in the body frame.

The sphere is centered at the shape's natural center, which is tighter for the
broadphase prefilter than an origin-centered radius. The origin-centered bound
the reach chain needs is ‖center_L‖ + radius, which is sound because the sphere
contains the geometry. Formulas are exact containment per shape:

 - Sphere(r): center X_LG·0, radius r.
 - Box(w,d,h; Drake stores full sizes): box center, radius = half diagonal.
 - Capsule(r, L): center, radius = L/2 + r.
 - Cylinder(r, L): center, radius = √(r² + (L/2)²) (farthest point on a rim).
 - Ellipsoid(a,b,c): center, radius = max(a,b,c).
 - Convex / Mesh: centroid of the convex-hull vertices, radius = max vertex
   distance. The vertices MUST come from the same hull object the proximity
   engine collides (Shape::GetConvexHull()), never from the raw file: the
   engine's hull bakes in scale and degeneracy inflation, and the radius must
   bound the geometry actually checked.

An under-bounding formula produces an unsound λ with no other symptom, so this
function switches on the closed set of supported shape types rather than
falling back to a generic bound.
@throws std::exception on any other shape type, HalfSpace included; half
spaces are handled by dedicated rules, never through a bounding sphere.
@ingroup planning_collision_checker */
BoundingSphere ComputeBoundingSphere(const geometry::Shape& shape,
                                     const math::RigidTransform<double>& X_LG);

}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake
