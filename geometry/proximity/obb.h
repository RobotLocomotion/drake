#pragma once

#include <algorithm>
#include <iostream>
#include <limits>
#include <memory>
#include <stack>
#include <utility>
#include <variant>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/reset_on_copy.h"
#include "drake/geometry/proximity/plane.h"
#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/shape_specification.h"
#include "drake/geometry/utilities.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

/* Axis-aligned bounding box used in BoundingVolumeHierarchy. The box is
 defined in a canonical frame B such that it is centered on Bo and its extents
 are aligned with B's axes. However, the box is posed in a hierarchical frame
 H. Because this is an _axis-aligned_ bounding box, `Bx = Hx`, `By = Hy`, and
 `Bz = Hz`. Therefore the pose of the box is completely captured with p_HoBo_H
 (see center()).

 Because of this, an instance of Aabb is a frame-dependent quantity and should
 be expressed that way. For example, for a mesh measured and expressed in frame
 M, the bounding boxes on its triangles will likely be measured and expressed
 in the same frame.

 ```
 auto mesh_M = ...;
 Aabb bv_M = ...;  // A bounding volume for mesh_M in the same frame.
 ```
 */
class Aabb {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Aabb)

  /* Constructs an axis-aligned bounding box measured and expressed in frame H.

   @param p_HoBo_H      The position vector from the hierarchy frame's origin to
                        the box's canonical origin, expressed in frame H. The
                        box is centered on Bo and aligned with Bx, By, and Bz.
   @param half_width    The _half_ measures of the box in each of the Bx, By,
                        and Bz directions.
   @pre half_width.x(), half_width.y(), half_width.z() are not negative.
  */
  Aabb(Vector3<double> p_HoBoH, Vector3<double> half_width)
      : center_(std::move(p_HoBoH)), half_width_(std::move(half_width)) {
    DRAKE_DEMAND(half_width.x() >= 0.0);
    DRAKE_DEMAND(half_width.y() >= 0.0);
    DRAKE_DEMAND(half_width.z() >= 0.0);

    PadBoundary();
  }

  /* Returns the center of the box -- equivalent to the position vector from
   the hierarchy frame's origin Ho to `this` box's origin Bo: `p_HoBo_H`. */
  const Vector3<double>& center() const { return center_; }

  /* Returns the half_width. */
  const Vector3<double>& half_width() const { return half_width_; }

  /* Returns the upper bounding point: p_HoU_H. */
  Vector3<double> upper() const { return center_ + half_width_; }

  /* Returns the lower bounding point: p_HoL_H. */
  Vector3<double> lower() const { return center_ - half_width_; }

  /* @return Volume of the bounding box.  */
  double CalcVolume() const {
    // Double the three half widths using * 8 instead of repeating * 2 three
    // times to help the compiler out.
    return half_width_[0] * half_width_[1] * half_width_[2] * 8;
  }

  /* Checks whether the two bounding volumes overlap by applying the transform
   between the two boxes and using Gottschalk's OBB overlap test.  */
  static bool HasOverlap(const Aabb& a, const Aabb& b,
                         const math::RigidTransformd& X_AB);

  /* Checks whether bounding volume `bv` intersects the given plane. The
   bounding volume is centered on its canonical frame B and B is posed in the
   corresponding hierarchy frame H; by construction B is aligned with H. The
   plane is defined in frame P.

   The box and plane intersect if _any_ point within the bounding volume has
   zero height (see CalcHeight()).

   @param bv        The bounding box to test.
   @param plane_P   The plane to test against the `bv`. The plane is expressed
                    in Frame P, therefore, to evaluate the height of a point
                    with respect to it, that point must be measured and
                    expressed in P.
   @param X_PH      The relative pose between the hierarchy frame H and the
                    plane frame P.
   @returns `true` if the plane intersects the box.   */
  static bool HasOverlap(const Aabb& bv, const Plane<double>& plane_P,
                         const math::RigidTransformd& X_PH);

  /* Checks whether bounding volume `bv` intersects the given half space. The
   bounding volume is centered on its canonical frame B and B is posed in the
   corresponding hierarchy frame H; by construction B is aligned with H. The
   half space is defined in its canonical frame C (such that the boundary plane
   of the half space is perpendicular to Cz and Co lies on the boundary plane).

   @param bv        The bounding box to test.
   @param hs_C      The half space to test against the `bv`. The half space is
                    expressed in Frame C, therefore, to evaluate the signed
                    distance of a point with respect to it, that point must be
                    measured and expressed in C.
   @param X_CH      The relative pose between the hierarchy frame H and the
                    half space canonical frame C.
   @returns `true` if the half space intersects the box.   */
  static bool HasOverlap(const Aabb& bv, const HalfSpace& hs_C,
                         const math::RigidTransformd& X_CH);

  /* Compares the values of the two Aabb instances for exact equality down to
   the last bit. Assumes that the quantities are measured and expressed in
   the same frame. */
  bool Equal(const Aabb& other) const {
    if (this == &other) return true;
    return center_ == other.center_ &&
           half_width_ == other.half_width_;
  }

 private:
  friend class AabbTester;

  // Pad this box in place by a small amount to ensure there will be no
  // roundoff problems. The amount to pad depends on the default tolerance for
  // this precision, the dimensions, and the position of the box in space.
  // A very large box, or a box that is very far from the origin, must be
  // padded more than a small one at the origin.
  void PadBoundary();

  // Default tolerance for double precision. This is the minimum amount of
  // padding to be added to the boundary, regardless of size or position.
  static constexpr double kTolerance = 2e-14;

  // Center point of the box.
  Vector3<double> center_;
  // Half width extents along each axes.
  Vector3<double> half_width_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
