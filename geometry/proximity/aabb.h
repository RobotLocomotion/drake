#pragma once

#include <set>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/utilities.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

// Forward declarations.
template <typename> class AabbMaker;
template <typename> class BvhUpdater;
class Obb;

/* Axis-aligned bounding box. The box is defined in a canonical frame B such
 that it is centered on Bo and its extents are aligned with B's axes. However,
 the box is posed in a hierarchical frame H. Because this is an _axis-aligned_
 bounding box, `R_HB = I`. Therefore the pose of the box is completely captured
 with p_HoBo_H (see center()).

 Because of this, an instance of Aabb is a frame-dependent quantity and should
 be expressed that way. For example, for a mesh measured and expressed in frame
 M, the bounding boxes on its triangles will be measured and expressed in the
 same frame.

 ```
 auto mesh_M = ...;
 Aabb bv_M = ...;  // A bounding volume for mesh_M in the same frame.
 ```
 */
class Aabb {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Aabb)

  /* The class used for various creation operations on this bounding volume. */
  template <typename MeshType>
  using Maker = AabbMaker<MeshType>;

  /* Constructs an axis-aligned bounding box measured and expressed in frame H.

   @param p_HoBo_H      The position vector from the hierarchy frame's origin to
                        the box's canonical origin, expressed in frame H. The
                        box is centered on Bo and aligned with Bx, By, and Bz.
   @param half_width    The _half_ measures of the box in each of the Bx, By,
                        and Bz directions. (Also the half measures in the Hx,
                        Hy, and Hz directions because R_HB = I.)
   @pre half_width.x(), half_width.y(), half_width.z() are not negative.
  */
  Aabb(const Vector3<double>& p_HoBo, const Vector3<double>& half_width)
      : center_(p_HoBo), half_width_(half_width) {
    DRAKE_DEMAND(half_width.x() >= 0.0);
    DRAKE_DEMAND(half_width.y() >= 0.0);
    DRAKE_DEMAND(half_width.z() >= 0.0);
  }

  /* Returns the center of the box -- equivalent to the position vector from
   the hierarchy frame's origin Ho to `this` box's origin Bo: `p_HoBo_H`. */
  const Vector3<double>& center() const { return center_; }

  /* Returns the half_width. */
  const Vector3<double>& half_width() const { return half_width_; }

  /* The point on the axis-aligned box with the smallest measures along the Bx-,
   By-, and Bz-directions. */
  Vector3<double> lower() const { return center_ - half_width_; }

  /* The point on the axis-aligned box with the largest measures along the Bx-,
   By-, and Bz-directions. */
  Vector3<double> upper() const { return center_ + half_width_; }

  // TODO(SeanCurtis-TRI): I've added this to be compatible with the Obb in
  //  terms of the generic BVH. The generic BVH sorts the vertices along a
  //  particular axis. For aabb. the axis is always one of the basis axes,
  //  therefore, it is sufficient to simply compare two floats. For Obb, we have
  //  to compare two dot products; I dot each vertex position w.r.t. one of the
  //  axes of the containing Obb. It would be better if this operation weren't
  //  built into the BVH but a function of the geometry type.
  //  The challenge in doing this is *what* I'm sorting. "CentroidPair" is
  //  defined inside the Bvh. I'll need to do some refactoring so that I don't
  //  get a circular dependency or any such thing (Or do things in a weird
  //  template-y fashion).
  /* Returns the pose X_HB of the box frame B in the hierarchy frame H. */
  math::RigidTransformd pose() const { return math::RigidTransformd{center_}; }

  /* @return Volume of the bounding box.  */
  double CalcVolume() const {
    // Double the three half widths using * 8 instead of repeating * 2 three
    // times to help the compiler out.
    return half_width_[0] * half_width_[1] * half_width_[2] * 8;
  }

  /* Reports whether the two axis-aligned bounding boxes `a_G` and `b_H`
   intersect. The poses of `a_G` and `b_H` are defined in their corresponding
   hierarchy frames G and H, respectively.

   @param a_G       The first axis-aligned box.
   @param b_H       The second axis-aligned box.
   @param X_GH      The relative pose between hierarchy frame G and hierarchy
                    frame H.
   @returns `true` if the boxes intersect.   */
  static bool HasOverlap(const Aabb& a_G, const Aabb& b_H,
                         const math::RigidTransformd& X_GH);

  /* Reports whether axis-aligned bounding box `aabb_G` intersects the given
   oriented bounding box `obb_H`. The poses of `aabb_G` and `obb_H` are defined
   in their corresponding hierarchy frames G and H, respectively.

   @param aabb_G   The axis-aligned box.
   @param obb_H     The oriented box.
   @param X_GH      The relative pose between the aabb hierarchy frame G and the
                    obb hierarchy frame H.
   @returns `true` if the boxes intersect.   */
  static bool HasOverlap(const Aabb& aabb_G, const Obb& obb_H,
                         const math::RigidTransformd& X_GH);

  // TODO(SeanCurtis-TRI): Support collision with primitives as appropriate
  //  (see obb.h for an example).

  /* Compares the values of the two Aabb instances for exact equality down to
   the last bit. Assumes that the quantities are measured and expressed in
   the same frame. */
  bool Equal(const Aabb& other) const {
    if (this == &other) return true;
    return center_ == other.center_ &&
           half_width_ == other.half_width_;
  }

 private:
  // Allow the BvhUpdater access to set_bounds().
  template <typename> friend class BvhUpdater;

  // Provides access to the BvhUpdater to refit the Aabb. Sets the extents of
  // the bounding box based on a box spanned by the given `lower` and `upper`
  // corners.
  // @pre lower(i) ≤ upper(i), ∀ i ∈ {0, 1, 2}.
  void set_bounds(const Vector3<double>& lower, const Vector3<double>& upper) {
    center_ = (lower + upper) / 2;
    half_width_ = (upper - lower) / 2;
  }

  // Center point of the box.
  Vector3<double> center_;
  // Half width extents along each axes.
  Vector3<double> half_width_;
};

/* %AabbMaker implements the logic to fit an Aabb to a collection of points.
 The points are the position of a subset of verties in a mesh. The Aabb will
 be measured and expressed in the same frame as the mesh.

 This serves as the interface to Bvh, allowing the Bvh to fit volumes to
 geometry without knowing the details of the bounding volume types.

 @tparam MeshType is either TriangleSurfaceMesh<T> or VolumeMesh<T>, where T is
         double or AutoDiffXd.  */
template <class MeshType>
class AabbMaker {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AabbMaker)

  /* Constructs the maker with the reference mesh and the subset of vertices to
   fit (indicated by corresponding index).

   @param mesh_M     The mesh frame M.
   @param vertices   The subset of vertices to fit.
   @pre `vertices` is not empty, and each of its entry is in the
        range [0, mesh_M.num_vertices()).  */
  AabbMaker(const MeshType& mesh_M, const std::set<int>& vertices)
      : mesh_M_(mesh_M), vertices_(vertices) {
    DRAKE_DEMAND(vertices_.size() > 0);
  }

  /* Computes the bounding volume of the vertices specified in the constructor.
   @retval aabb_M    The axis-aligned bounding box posed in mesh frame M.  */
  Aabb Compute() const;

 private:
  const MeshType& mesh_M_;
  const std::set<int>& vertices_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
