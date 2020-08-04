#pragma once

#include <set>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/plane.h"
#include "drake/geometry/shape_specification.h"
#include "drake/geometry/utilities.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

/* Oriented bounding box used in Bvh. The box is defined in a canonical
 frame B such that it is centered on Bo and its extents are aligned with
 B's axes. However, the box is posed in a hierarchical frame H (see pose()).

 Because of this, an instance of Obb is a frame-dependent quantity and should
 be expressed that way. For example, for a mesh measured and expressed in frame
 M, the bounding boxes on its triangles will likely be measured and expressed
 in the same frame.

 ```
 auto mesh_M = ...;
 Obb bv_M = ...;  // A bounding volume for mesh_M in the same frame.
 ```
 */
class Obb {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Obb)

  /* Constructs an oriented bounding box measured and expressed in frame H.

   @param X_HB          The pose of the box in the hierarchy frame H.
                        The box is centered on Bo and aligned with Bx, By,
                        and Bz.
   @param half_width    The _half_ measures of the box in each of the Bx, By,
                        and Bz directions.
   @pre half_width.x(), half_width.y(), half_width.z() are not negative.
  */
  Obb(math::RigidTransformd X_HB, Vector3<double> half_width)
      : pose_(std::move(X_HB)), half_width_(std::move(half_width)) {
    DRAKE_DEMAND(half_width.x() >= 0.0);
    DRAKE_DEMAND(half_width.y() >= 0.0);
    DRAKE_DEMAND(half_width.z() >= 0.0);

    PadBoundary();
  }

  /* Returns the center of the box -- equivalent to the position vector from
   the hierarchy frame's origin Ho to `this` box's origin Bo: `p_HoBo_H`. */
  const Vector3<double>& center() const { return pose_.translation(); }

  /* Returns the half_width -- equivalent to the position vector from the
   box's center Bo to the box's upper corner U expressed in the box's frame B:
   `p_BoU_B`. */
  const Vector3<double>& half_width() const { return half_width_; }

  /* Returns the upper bounding point: p_HoU_H. */
  Vector3<double> upper() const {
    return pose_.translation() + pose_.rotation() * half_width_;
  }

  /* Returns the lower bounding point: p_HoL_H. */
  Vector3<double> lower() const {
    return pose_.translation() - pose_.rotation() * half_width_;
  }

  /* Returns the pose X_HB of the box frame B in the hierarchy frame H */
  const math::RigidTransformd& pose() const { return pose_; }

  /* @return Volume of the bounding box.  */
  double CalcVolume() const {
    // Double the three half widths using * 8 instead of repeating * 2 three
    // times to help the compiler out.
    return half_width_[0] * half_width_[1] * half_width_[2] * 8;
  }

  /* Checks whether the two bounding volumes `a` and `b` overlap by applying
   transforms between frames of boxes and hierarchies and using Gottschalk's
   OBB overlap test. Box `a` has its frame A posed in hierarchy frame G, and
   box `b` has its frame B posed in hierarchy frame H. */
  static bool HasOverlap(const Obb& a, const Obb& b,
                         const math::RigidTransformd& X_GH);

  /* Checks whether bounding volume `bv` intersects the given plane. The
   bounding volume is centered on its canonical frame B, and B is posed in the
   corresponding hierarchy frame H. The plane is defined in frame P.

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
  static bool HasOverlap(const Obb& bv, const Plane<double>& plane_P,
                         const math::RigidTransformd& X_PH);

  /* Checks whether bounding volume `bv` intersects the given half space. The
   bounding volume is centered on its canonical frame B, and B is posed in the
   corresponding hierarchy frame H. The half space is defined in its
   canonical frame C (such that the boundary plane of the half space is
   perpendicular to Cz and Co lies on the boundary plane).

   @param bv        The bounding box to test.
   @param hs_C      The half space to test against the `bv`. The half space is
                    expressed in Frame C, therefore, to evaluate the signed
                    distance of a point with respect to it, that point must be
                    measured and expressed in C.
   @param X_CH      The relative pose between the hierarchy frame H and the
                    half space canonical frame C.
   @returns `true` if the half space intersects the box.   */
  static bool HasOverlap(const Obb& bv, const HalfSpace& hs_C,
                         const math::RigidTransformd& X_CH);

  /* Compares the values of the two Obb instances for exact equality down to
   the last bit. Assumes that the quantities are measured and expressed in
   the same frame. */
  bool Equal(const Obb& other) const {
    if (this == &other) return true;
    return pose_.IsExactlyEqualTo(other.pose_) &&
           half_width_ == other.half_width_;
  }

 private:
  friend class ObbTester;

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
  math::RigidTransformd pose_;
  // Half width extents along each axes.
  Vector3<double> half_width_;
};

// Forward declaration, so we can grant friend access to the tester.
template <typename MeshType> class ObbMakerTester;

/* %ObbMaker performs an algorithm to create an oriented bounding box that
 fits a specified set of vertices in a mesh. Do not use %ObbMaker directly;
 please use the wrapper function ComputeObb() instead.

 @tparam MeshType is either SurfaceMesh<double> or VolumeMesh<double>.  */
template <class MeshType>
class ObbMaker {
 public:
  // Specifies the input mesh with frame M and a set of vertices.
  //
  // @pre The specified set of vertices is not empty.
  ObbMaker(const MeshType& mesh_M,
           const std::set<typename MeshType::VertexIndex>& vertices)
      : mesh_M_(mesh_M), vertices_(vertices) {
    DRAKE_DEMAND(vertices_.size() > 0);
  }

  // The output box's frame B is posed in the input mesh frame M.
  Obb Compute() const;

 private:
  /* Calculates an orientation of the bounding box of the specified set of
   vertices in the mesh by a simple principal component analysis. Return
   the rotation R_MB of the box frame B expressed in the mesh frame M. */
  math::RotationMatrixd CalcOrientationByPCA() const;

  /* Calculates an oriented bounding box in a given orientation that fits the
   specified set of vertices in the mesh. The orientation is given by a
   rotation R_MB of the box frame B expressed in the mesh frame M. */
  Obb CalcOrientedBox(const math::RotationMatrixd& R_MB) const;

  /* Numerically calculates the gradient of volume of an oriented bounding
   box as a function of the roll-pitch-yaw angles by a simple forward
   difference. Return the vector (∂v/∂r, ∂v/∂p, ∂v/∂y) where v is the volume,
   and r,p,y are the roll, pitch, yaw angles respectively.  */
  Eigen::Vector3d CalcVolumeGradient(const Obb& box) const;

  /* Locally optimizes volume of the oriented bounding box subject to fitting
   the specified set of vertices in the mesh. */
  Obb OptimizeObbVolume(const Obb& box) const;

  const MeshType& mesh_M_;
  const std::set<typename MeshType::VertexIndex>& vertices_;

  friend class ObbMakerTester<MeshType>;
};

/* Computes an oriented bounding box that fits a specified set of vertices in
 a mesh. It is a convenient wrapper for ObbMaker.

 @param mesh_M   The mesh that owns the vertices expressed in frame M.
 @param vertices The vertices to fit.
 @return obb_M   The oriented bounding box posed in frame M.
 @tparam MeshType is either SurfaceMesh<double> or VolumeMesh<double>.

 @pre The specified set of vertices is not empty.  */
template <class MeshType>
Obb ComputeObb(const MeshType& mesh_M,
               const std::set<typename MeshType::VertexIndex>& vertices) {
  return ObbMaker<MeshType>(mesh_M, vertices).Compute();
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
