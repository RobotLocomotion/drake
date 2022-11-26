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

enum class PlaneSide {
  kPositive,
  kNegative,
};

class CollisionGeometry : public ShapeReifier {
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
   including polytopes, cylinders, and capsules.

   Note that when we don't require a separating margin δ, and the geometry is a
   polytope, then we consider the constraint
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
   @param[out] rationals The rationals that should be positive when the geometry
   is on the designated side of the plane.
   @param[out] unit_length_vector The vector that should have length <= 1 when
   the geometry is on the designated side of the plane.
   */
  void OnPlaneSide(
      const Vector3<symbolic::Polynomial>& a, const symbolic::Polynomial& b,
      const multibody::RationalForwardKinematics::Pose<symbolic::Polynomial>&
          X_AB_multilinear,
      const multibody::RationalForwardKinematics& rational_forward_kin,
      const std::optional<symbolic::Variable>& separating_margin,
      PlaneSide plane_side, std::vector<symbolic::RationalFunction>* rationals,
      std::optional<VectorX<symbolic::Polynomial>>* unit_length_vector);

 private:
  // Implements supported shapes for the ShapeReifier interface.
  using ShapeReifier::ImplementGeometry;
  void ImplementGeometry(const Box& box, void* data) final;
  void ImplementGeometry(const Convex& convex, void* data) final;
  void ImplementGeometry(const Sphere& sphere, void* data) final;
  void ImplementGeometry(const Capsule& capsule, void* data) final;
  void ImplementGeometry(const Cylinder& cylinder, void* data) final;
  const Shape* geometry_;
  multibody::BodyIndex body_index_;
  geometry::GeometryId id_;
  math::RigidTransformd X_BG_;
};
}  // namespace optimization
}  // namespace geometry
}  // namespace drake
