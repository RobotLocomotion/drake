#pragma once

#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/shape_specification.h"
#include "drake/multibody/rational/rational_forward_kinematics.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace geometry {
namespace optimization {

/**
 The supported type of geometries in C-IRIS.
 */
enum class CIrisGeometryType {
  kSphere,
  kPolytope,
  kCylinder,
  kCapsule,
};

enum class PlaneSide {
  kPositive,
  kNegative,
};

/**
 This class contains the necessary information about the collision geometry used
 in C-IRIS. Most notably it transcribes the geometric condition that the
 collision geometry is on one side of the plane to mathematical constraints.
 For the detailed algorithm please refer to the paper
 Certified Polyhedral Decompositions of Collision-Free Configuration Space
 by Hongkai Dai*, Alexandre Amice*, Peter Werner, Annan Zhang and Russ Tedrake.

 @ingroup planning_iris
 */
class CIrisCollisionGeometry {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CIrisCollisionGeometry);

  /**
   @param geometry The actual geometry object. `geometry` must outlive this
   CIrisCollisionGeometry object.
   @param body_index The index of the body to which this geometry is fixed.
   @param id The ID of this geometry.
   @param X_BG The pose of the geometry (G) in the attached body frame (B).
   */
  CIrisCollisionGeometry(const Shape* geometry, multibody::BodyIndex body_index,
                         GeometryId id, math::RigidTransformd X_BG);

  const Shape& geometry() const { return *geometry_; }

  multibody::BodyIndex body_index() const { return body_index_; }

  GeometryId id() const { return id_; }

  const math::RigidTransformd& X_BG() const { return X_BG_; }

  /** Impose the constraint that the geometry is on a given side of the plane
   {x | aᵀx+b=0}.

   For example, to impose the constraint that a polytope is on the positive
   side of the plane, we consider the following constraints
   aᵀp_AVᵢ + b ≥ 1      (1)
   where Vᵢ is the i'th vertex of the polytope.
   (1) says rational functions are non-negative.

   To impose the constraint that a sphere is on positive side of the
   plane, we consider the following constraints
   aᵀp_AS + b ≥ r*|a|         (2a)
   aᵀp_AS + b ≥ 1             (2b)
   where S is the sphere center, r is the sphere radius.

   We can reformulate (2a) as the following constraint
   ⌈aᵀp_AS + b                aᵀ⌉  is psd.           (3)
   ⌊ a        (aᵀp_AS + b)/r²*I₃⌋
   (3) is equivalent to the rational
   ⌈1⌉ᵀ*⌈aᵀp_AS + b               aᵀ⌉*⌈1⌉
   ⌊y⌋  ⌊ a        (aᵀp_AS+ b)/r²*I₃⌋ ⌊y⌋
   is positive.

   @param a The normal vector in the separating plane. a is expressed in frame
   A. Note that `a` doesn't need to have a unit length.
   @param b The constant term in the separating plane.
   @param X_AB_multilinear The pose of the collision geometry body (B) in the
   expressed frame A, written as a multilinear polynomial. This quantity is
   generated from
   RationalForwardKinematics::CalcBodyPoseAsMultilinearPolynomial.
   @param rational_forward_kin This object is constructed with the
   MultibodyPlant containing this collision geometry.
   @param plane_side Whether the geometry is on the positive or negative side of
   the plane.
   @param y_slack The slack variable y in the documentation above, used for
   non-polytopic geometries. For spheres and capsules, y_slack has size 3. For
   cylinders, y_slack has size 2.
   @param[in,out] rationals We append new rational functions to `rationals`.
   If these new rational functions are positive, then the geometry is on a given
   side of the plane.
   @pre rationals != nullptr
   */
  void OnPlaneSide(
      const Vector3<symbolic::Polynomial>& a, const symbolic::Polynomial& b,
      const multibody::RationalForwardKinematics::Pose<symbolic::Polynomial>&
          X_AB_multilinear,
      const multibody::RationalForwardKinematics& rational_forward_kin,
      PlaneSide plane_side, const VectorX<symbolic::Variable>& y_slack,
      std::vector<symbolic::RationalFunction>* rationals) const;

  [[nodiscard]] CIrisGeometryType type() const;

  /**
   Returns the number of rationals in the condition "this geometry is
   on one side of the plane."
   */
  [[nodiscard]] int num_rationals() const;

 private:
  const Shape* geometry_{};
  multibody::BodyIndex body_index_{};
  GeometryId id_;
  math::RigidTransformd X_BG_;
};

/** Computes the signed distance from `collision_geometry` to the half space ℋ,
 where ℋ = {x | aᵀx+b >= 0} if plane_side=PlaneSide::kPositive, and
 ℋ = {x | aᵀx+b <= 0} if plane_side=PlaneSide::kNegative.
 The half space is measured and expressed in the expressed_body's body frame.
 This works for both `collision_geometry` separated from the half space, and
 `collision geometry` in penetration with the halfspace.
 @note `a` does not need to be a unit length vector (but should be non-zero).
 */
[[nodiscard]] double DistanceToHalfspace(
    const CIrisCollisionGeometry& collision_geometry, const Eigen::Vector3d& a,
    double b, multibody::BodyIndex expressed_body, PlaneSide plane_side,
    const multibody::MultibodyPlant<double>& plant,
    const systems::Context<double>& plant_context);
}  // namespace optimization
}  // namespace geometry
}  // namespace drake
