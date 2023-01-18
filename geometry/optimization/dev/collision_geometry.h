#pragma once

#include <optional>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/shape_specification.h"
#include "drake/multibody/rational/rational_forward_kinematics.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace geometry {
namespace optimization {

enum class GeometryType {
  kSphere,
  kPolytope,
  kCylinder,
  kCapsule,
};

enum class PlaneSide {
  kPositive,
  kNegative,
};

/** Returns the other side */
[[nodiscard]] PlaneSide OtherSide(PlaneSide plane_side);

class CollisionGeometry {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CollisionGeometry)

  /**
   @param geometry The actual geometry object.
   @param body_index The index of the body to which this geometry is fixed.
   @param id The ID of this geometry.
   @param X_BG The pose of the geometry (G) in the attached body frame (B).
   */
  CollisionGeometry(const geometry::Shape* geometry,
                    multibody::BodyIndex body_index, geometry::GeometryId id,
                    math::RigidTransformd X_BG);

  const Shape& geometry() const { return *geometry_; }

  multibody::BodyIndex body_index() const { return body_index_; }

  GeometryId id() const { return id_; }

  const math::RigidTransformd& X_BG() const { return X_BG_; }

  /** Impose the constraint that the geometry is on a given side of the plane {x
   | aᵀx+b=0}.

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
   A.
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
   non-polytopic geometries.
   @param[out] rationals The rational functions that need to be
   positive to represent that the geometry is on a given side of the plane.
   */
  void OnPlaneSide(
      const Vector3<symbolic::Polynomial>& a, const symbolic::Polynomial& b,
      const multibody::RationalForwardKinematics::Pose<symbolic::Polynomial>&
          X_AB_multilinear,
      const multibody::RationalForwardKinematics& rational_forward_kin,
      PlaneSide plane_side, const VectorX<symbolic::Variable>& y_slack,
      std::vector<symbolic::RationalFunction>* rationals) const;

  [[nodiscard]] GeometryType type() const;

  /**
   Returns the number of rationals in the condition "this geometry is
   on one side of the plane."
   */
  [[nodiscard]] int num_rationals() const;

 private:
  const Shape* geometry_;
  multibody::BodyIndex body_index_;
  geometry::GeometryId id_;
  math::RigidTransformd X_BG_;
};

/** Computes the signed distance from `collision_geometry` to the halfspace ℋ,
 where ℋ ={ x | aᵀx+b >= 0} if plane_side=PlaneSide::kPositive, and ℋ ={ x |
 aᵀx+b <= 0} if plane_side=PlaneSide::kNegative.
 The halfspace is expressed in the expressed_body's body frame.
 */
[[nodiscard]] double DistanceToHalfspace(
    const CollisionGeometry& collision_geometry, const Eigen::Vector3d& a,
    double b, multibody::BodyIndex expressed_body, PlaneSide plane_side,
    const multibody::MultibodyPlant<double>& plant,
    const systems::Context<double>& plant_context);
}  // namespace optimization
}  // namespace geometry
}  // namespace drake
