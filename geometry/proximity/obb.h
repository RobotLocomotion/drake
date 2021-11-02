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

// Forward declarations.
template <typename> class ObbMaker;
class Aabb;

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

  /** The class used for various creation operations on this bounding volume. */
  template <typename MeshType>
  using Maker = ObbMaker<MeshType>;

  /* Constructs an oriented bounding box measured and expressed in frame H.

   @param X_HB          The pose of the box in the hierarchy frame H.
                        The box is centered on Bo and aligned with Bx, By,
                        and Bz.
   @param half_width    The _half_ measures of the box in each of the Bx, By,
                        and Bz directions.
   @pre half_width.x(), half_width.y(), half_width.z() are not negative.
  */
  Obb(const math::RigidTransformd& X_HB, const Vector3<double>& half_width)
      : pose_(X_HB), half_width_(half_width) {
    DRAKE_DEMAND(half_width.x() >= 0.0);
    DRAKE_DEMAND(half_width.y() >= 0.0);
    DRAKE_DEMAND(half_width.z() >= 0.0);

    PadBoundary();
  }

  /* Returns the center of the box -- equivalent to the position vector from
   the hierarchy frame's origin Ho to `this` box's origin Bo: `p_HoBo_H`. */
  const Vector3<double>& center() const { return pose_.translation(); }

  /* Returns the half_width -- equivalent to the position vector from the
   box's center Bo to the box's first octant (+,+,+) corner U expressed in
   the box's frame B: `p_BoU_B`. */
  const Vector3<double>& half_width() const { return half_width_; }

  /* Returns the pose X_HB of the box frame B in the hierarchy frame H */
  const math::RigidTransformd& pose() const { return pose_; }

  /* @return Volume of the bounding box.  */
  double CalcVolume() const {
    // Double the three half widths using * 8 instead of repeating * 2 three
    // times to help the compiler out.
    return half_width_[0] * half_width_[1] * half_width_[2] * 8;
  }

  /* Reports whether the two oriented bounding boxes `a_G` and `b_H` intersect.
   The poses of `a_G` and `b_H` are defined in their corresponding hierarchy
   frames G and H, respectively.

   @param a_G       The first oriented box.
   @param b_H       The second oriented box.
   @param X_GH      The relative pose between hierarchy frame G and hierarchy
                    frame H.
   @returns `true` if the boxes intersect.   */
  static bool HasOverlap(const Obb& a_G, const Obb& b_H,
                         const math::RigidTransformd& X_GH);

  /* Reports whether oriented bounding box `obb_G` intersects the given
   axis-aligned bounding box `aabb_H`. The poses of `obb_G` and `aabb_H` are
   defined in their corresponding hierarchy frames G and H, respectively.

   @param obb_G     The oriented box.
   @param aabb_H    The axis-aligned box.
   @param X_GH      The relative pose between the obb hierarchy frame G and the
                    aabb hierarchy frame H.
   @returns `true` if the boxes intersect.   */
  static bool HasOverlap(const Obb& obb_G, const Aabb& aabb_H,
                         const math::RigidTransformd& X_GH);

  /* Checks whether bounding volume `bv` intersects the given plane. The
   bounding volume is centered on its canonical frame B, and B is posed in the
   corresponding hierarchy frame H. The plane is defined in frame P.

   The box and plane intersect if _any_ point within the bounding volume has
   zero height (see CalcHeight()).

   @param bv_H      The bounding box to test.
   @param plane_P   The plane to test against the `bv`. The plane is expressed
                    in frame P, therefore, to evaluate the height of a point
                    with respect to it, that point must be measured and
                    expressed in P.
   @param X_PH      The relative pose between the hierarchy frame H and the
                    plane frame P.
   @returns `true` if the plane intersects the box.   */
  static bool HasOverlap(const Obb& bv_H, const Plane<double>& plane_P,
                         const math::RigidTransformd& X_PH);

  /* Checks whether bounding volume `bv` intersects the given half space. The
   bounding volume is centered on its canonical frame B, and B is posed in the
   corresponding hierarchy frame H. The half space is defined in its
   canonical frame C (such that the boundary plane of the half space is
   perpendicular to Cz and Co lies on the boundary plane).

   @param bv_H      The bounding box to test.
   @param hs_C      The half space to test against the `bv`. The half space is
                    expressed in Frame C, therefore, to evaluate the signed
                    distance of a point with respect to it, that point must be
                    measured and expressed in C.
   @param X_CH      The relative pose between the hierarchy frame H and the
                    half space canonical frame C.
   @returns `true` if the half space intersects the box.   */
  static bool HasOverlap(const Obb& bv_H, const HalfSpace& hs_C,
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

  /* Pad this box in place by a small amount to ensure there will be no
   roundoff problems. The amount to pad depends on the default tolerance for
   this precision, the dimensions, and the position of the box in space.
   A very large box, or a box that is very far from the origin, must be
   padded more than a small one at the origin.  */
  void PadBoundary();

  // Default tolerance for double precision. This is the minimum amount of
  // padding to be added to the boundary, regardless of size or position.
  static constexpr double kTolerance = 2e-14;

  // Pose X_HB of the box frame B in the hierarchy frame H. Bo is the box
  // center.
  math::RigidTransformd pose_;
  // Half width extents along each axes.
  Vector3<double> half_width_;
};

// Forward declaration, so we can grant friend access to the tester.
template <typename MeshType> class ObbMakerTester;

/* %ObbMaker performs an algorithm to create an oriented bounding box that
 fits a specified set of vertices in a mesh.

 @tparam MeshType is either TriangleSurfaceMesh<T> or VolumeMesh<T>, where T is
         double or AutoDiffXd.  */
template <class MeshType>
class ObbMaker {
 public:
  // TODO(DamrongGuoy): If we get a performance hit, consider taking
  //  std::vector instead of std::set and put @pre to require callers to sort
  //  and unique before passing it to ObbMaker. Repeated vertices can harm
  //  PCA and slow down the rest of ObbMaker.

  /* Specifies the input mesh with frame M and a set of vertices to fit.
   @param mesh_M   The mesh that owns the vertices expressed in frame M.
   @param vertices The vertices to fit.
   @pre `vertices` is not empty, and each of its entry is in the
        range [0, V), where V is mesh_M.num_vertices().  */
  ObbMaker(const MeshType& mesh_M, const std::set<int>& vertices)
      : mesh_M_(mesh_M), vertices_(vertices) {
    DRAKE_DEMAND(vertices_.size() > 0);
  }

  /* Computes the bounding volume of the vertices specified in the constructor.
   @retval obb_M   The oriented bounding box posed in frame M.  */
  Obb Compute() const;

 private:
  /* Calculates an orientation of the bounding box of the specified set of
   vertices in the mesh by principal component analysis (PCA). Return
   the rotation R_MB of the box frame B expressed in the mesh frame M.

   Bx_M = R_MB.col(0) is the first principal component, By_M = R_MB.col(1) is
   the second principal component, and Bz_M = R_MB.col(2) is the last principal
   component. See https://en.wikipedia.org/wiki/Principal_component_analysis
   for definitions of principal components. Intuitively, the box is longest in
   Bx direction and shortest in Bz direction.

   We cannot always guarantee predictable basis R_MB. The basis is never unique
   and the actual basis returned is dependent on implementation details.
   Some reasons for non-uniqueness:
     - A valid basis can be rotated 180 degrees around an axis and still be a
       valid basis.
     - Certain distributions of the vertices allow for arbitrary orientation
       of the basis vectors. For example, uniformly distributed vertices on
       a circle can have any two basis vectors Bx_M, By_M spanning the plane
       of the circle and have Bz_M orthogonal to such plane.
    */
  math::RotationMatrixd CalcOrientationByPca() const;

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
  const std::set<int>& vertices_;

  friend class ObbMakerTester<MeshType>;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
