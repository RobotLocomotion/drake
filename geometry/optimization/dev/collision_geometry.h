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

  /**
   When we constrain a polytope to be on one side of the plane (for example the
   positive side), we impose the constraint
   aᵀ*p_AVᵢ + b ≥ δ
   aᵀ*p_AC  + b ≥ k * r
   where Vᵢ being the i'th vertex of the polytope. C is the Chebyshev center of
   the polytope, r is the radius (namely the distance from the Chebyshev center
   to the boundary of the polytope), and k is a positive scalar. This function
   returns the value of k.
   */
  static double PolytopeChebyshevRadiusMultiplier() { return 0.5; }

  /**
   To impose the geometric constraint that this collision geometry is on one
   side of a plane {x|aᵀx+b=0} with a certain margin, we can equivalently write
   this constraint with the following conditions
   1. Some rational functions are always non-negative.
   2. A certain vector has length <= 1.

   For example, if this geometry is a sphere with radius r, and we constrain
   that the sphere is on the positive side of the plane with a margin δ, we can
   impose the following constraint
   aᵀ*p_AS(s) + b ≥ r + δ       (1)
   |a| ≤ 1                      (2)
   where p_AS(s) is the position of the sphere center S expressed in a frame A.
   The left hand side of equation (1) is a rational function of indeterminate
   s. Constraint (2) says the vector a has length <= 1.

   Similarly we can write down the conditions for other geometry types,
   including polytopes and capsules.

   Note that when we don't require a separating margin δ, and both this
   geometry and the geometry on the other side of the plane are polytopes, then
   we consider the constraint
   aᵀp_AVᵢ(s) + b ≥ 1 if the polytope is on the positive side of the plane, and
   aᵀp_AVᵢ(s) + b ≤ -1 if the polytope is on the negative side of the plane.
   Note that in this case we don't have the "vector with length <= 1 "
   constraint.

   @param a The normal vector in the separating plane. a is expressed in frame
   A.
   @param b The constant term in the separating plane.
   @param X_AB_multilinear The pose of the collision geometry body (B) in the
   expressed frame A, written as a multilinear polynomial. This quantity is
   generated from
   RationalForwardKinematics::CalcBodyPoseAsMultilinearPolynomial.
   @param rational_forward_kin This object is constructed with the
   MultibodyPlant containing this collision geometry.
   @param separating_margin δ in the documentation above.
   @param plane_side Whether the geometry is on the positive or negative side of
   the plane.
   @param other_side_geometry_type The type of the geometry on the other side of
   the plane.
   @param[out] rationals The rationals that should be positive when the geometry
   is on the designated side of the plane.
   @param[out] unit_length_vector The vector that should have length <= 1 when
   the geometry is on the designated side of the plane.
   */
  // TODO(hongkai.dai): remove query_object from input when we can construct an
  // HPolyhedron directly from vertices.
  void OnPlaneSide(
      const Vector3<symbolic::Polynomial>& a, const symbolic::Polynomial& b,
      const multibody::RationalForwardKinematics::Pose<symbolic::Polynomial>&
          X_AB_multilinear,
      const multibody::RationalForwardKinematics& rational_forward_kin,
      const std::optional<symbolic::Variable>& separating_margin,
      PlaneSide plane_side, std::vector<symbolic::RationalFunction>* rationals,
      std::optional<VectorX<symbolic::Polynomial>>* unit_length_vector) const;

  [[nodiscard]] GeometryType type() const;

  /**
   Returns the number of rationals in the condition "this geometry is on one
   side of the plane."
   */
  [[nodiscard]] int num_rationals_per_side() const;

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
