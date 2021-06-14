#pragma once

#include <memory>
#include <optional>
#include <utility>

#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace geometry {
namespace optimization {

/** @defgroup geometry_optimization Geometry Optimization
@ingroup geometry
@brief Provides an abstraction for reasoning about geometry in optimization
problems, and using optimization problems to solve geometry problems.

# Relationship to other components in Drake.

SceneGraph handles many of types of geometry (see geometry::Shape).  It is
specialized to 3D and puts a strong emphasis on efficient implementation of a
particular subset of geometry queries, like collision detection and
signed-distance queries.  SceneGraph also provides a lot of valuable tools for
content management, including parsing geometries from file (via
multibody::Parser) and geometry::Role.

MathematicalProgram has many relevant solvers::Cost / solver::Constraint for
reasoning about geometry (e.g. the LorentzCone or even LinearConstraint). The
class and methods in this group add a level of modeling power above these
individual constraints (there are many different types of constraints one would
write given various optimization on these sets).

The geometry::optimization tools support:
- The ability to model sets in arbitrary dimensions, not just 3D.
- Many advanced queries (intersections, Minkowski sum, set containment, etc).
- The ability to use sets in mathematical programs
- The ability to use mathematical programs to perform operations on sets.
- The ability to include the geometry from a SceneGraph into a
  MathematicalProgram.
- The ability to use SceneGraph queries for some of the geometry operations
  (signed distance, collision), when the set is in 3D.
- The ability to use SceneGraph for rendering the geometries when they are in
  3D.

*/

/** Abstract base class for defining a convex set.
@ingroup geometry_optimization
*/
class ConvexSet {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ConvexSet)

  virtual ~ConvexSet() {}

  /** Returns the dimension of the vector space in which the elements of this
  set are evaluated.  Contrast this with the `affine dimension`: the
  dimension of the smallest affine subset of the ambient space that contains
  our set.  For example, if we define a set using `A*x = b`, where `A` has
  linearly independent rows, then the ambient dimension is the dimension of
  `x`, but the affine dimension of the set is `ambient_dimension() -
  rank(A)`. */
  int ambient_dimension() const { return ambient_dimension_; }

  /** Returns true iff the point x is contained in the set. */
  bool PointInSet(const Eigen::Ref<const Eigen::VectorXd>& x,
                  double tol = 1e-8) const {
    return DoPointInSet(x, tol);
  }

  // Note: I would like to return the Binding, but the type is subclass
  // dependent.
  /** Adds a constraint to an existing MathematicalProgram enforcing that the
  point defined by vars is inside the set. */
  void AddPointInSetConstraint(
      solvers::MathematicalProgram* prog,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& vars) const {
    return DoAddPointInSetConstraint(prog, vars);
  }

  /** Constructs a geometry::Shape and a pose of the set in the world frame for
  use in the SceneGraph geometry ecosystem.
  @throws std::exception if this.dimension() != 3 or if the functionality for a
  particular set has not been implemented yet. */
  virtual std::pair<std::unique_ptr<Shape>, math::RigidTransformd>
  ToShapeWithPose() const {
    throw std::runtime_error("Not implemented yet.");
  }

 protected:
  explicit ConvexSet(int ambient_dimension)
      : ambient_dimension_{ambient_dimension} {}

  // Non-virtual interface implementations.
  virtual bool DoPointInSet(const Eigen::Ref<const Eigen::VectorXd>& x,
                            double tol) const = 0;

  virtual void DoAddPointInSetConstraint(
      solvers::MathematicalProgram* prog,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& vars) const = 0;

  int ambient_dimension_{0};
};

/** Implements a polyhedral convex set using the half-space representation:
`{x| A x ≤ b}`.  Note: This set may be unbounded.
@ingroup geometry_optimization
*/
class HPolyhedron : public ConvexSet {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(HPolyhedron)

  /** Constructs the polyhedron.
  @pre A.rows() == b.size().
  */
  HPolyhedron(const Eigen::Ref<const Eigen::MatrixXd>& A,
              const Eigen::Ref<const Eigen::VectorXd>& b);
  virtual ~HPolyhedron() {}

  /** Returns the half-space representation matrix A. */
  const Eigen::MatrixXd& A() const { return A_; }

  /** Returns the half-space representation vector b. */
  const Eigen::VectorXd& b() const { return b_; }

  // TODO(russt): Add recommendation to ToShapeWithPose doc once we have
  // AHPolytope: "If this.dimension() != 3, then consider using the AH polytope
  // representation to project it to 3D."

  // TODO(russt): ToShapeWithPose shouldn't need to go to .obj on disk and back
  // to get this geometry into ProximityEngine nor MeshcatVisualizer.  Consider
  // adding optional vertex/poly to Shape::Convex?

  /** Constructs a geometry::Convex shape description of this polytope, writing
  the mesh to a temporary .obj file on disk.
  @throws std::exception if this.dimension() != 3 */
  // std::pair<std::unique_ptr<Shape>, math::RigidTransformd> ToShapeWithPose()
  // const;

  /** Constructs a new HPolyhedron from a SceneGraph geometry and pose in the @p
  expressed_in frame, obtained via the QueryObject.  If @p expressed_in frame is
  std::nullopt, then it will be expressed in the world frame.
  @throws std::exception the geometry is not a convex polytope. */
  static HPolyhedron MakeFromSceneGraph(
      const QueryObject<double>& query_object, GeometryId geometry_id,
      std::optional<FrameId> expressed_in = std::nullopt);

  // TODO(russt): MakeFromSceneGraphAABB() and MakeFromSceneGraphOBB() pending
  // #15121.
 private:
  bool DoPointInSet(const Eigen::Ref<const Eigen::VectorXd>& x,
                    double tol) const override;

  void DoAddPointInSetConstraint(
      solvers::MathematicalProgram* prog,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& vars)
      const override;

  Eigen::MatrixXd A_{};
  Eigen::VectorXd b_{};
};

/** Implements an ellipsoidal convex set represented by the quadratic form `{x
| (x-center)ᵀAᵀA(x-center) ≤ 1}`.  Note that `A` need not be square; we require
only that the matrix AᵀA is positive semi-definite.

Compare this with an alternative (very useful) parameterization of the
ellipsoid: `{Bu + center | |u|₂ ≤ 1}`, which is an affine scaling of the unit
ball.  This is related to the quadratic form by `B = A⁻¹`, when `A` is
invertible, but the quadratic form can also represent unbounded sets.

Note: the name HyperEllipsoid was taken here to avoid conflicting with
geometry::Ellipsoid and to distinguish that this class supports N dimensions.

@ingroup geometry_optimization
*/
class HyperEllipsoid : public ConvexSet {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(HyperEllipsoid)

  /** Constructs the ellipsoid.
  @pre A.cols() == center.size(). */
  HyperEllipsoid(const Eigen::Ref<const Eigen::MatrixXd>& A,
            const Eigen::Ref<const Eigen::VectorXd>& center);
  virtual ~HyperEllipsoid() {}

  /** Returns the quadratic form matrix A. */
  const Eigen::MatrixXd& A() const { return A_; }

  /** Returns the center of the ellipsoid. */
  const Eigen::VectorXd& center() const { return center_; }

  /** Constructs a Ellipsoid shape description of this set.  Note
  that the choice of ellipsoid is not unique.  This method chooses to order the
  Ellipsoid parameters a ≥ b ≥ c.
  @throws std::exception if this.dimension() != 3
  @throws std::exception if A is not invertible (has any eigenvalue less than
  sqrt(1e-12)). */
  std::pair<std::unique_ptr<Shape>, math::RigidTransformd> ToShapeWithPose()
      const override;

  /** Constructs a new HyperEllipsoid from a SceneGraph
  geometry and pose in the @p expressed_in frame, obtained via the QueryObject.
  If @p expressed_in frame is std::nullopt, then it will be expressed in the
  world frame.
  @throws std::exception if geometry_id does not represent a shape that can be
  described as an HyperEllipsoid. */
  static HyperEllipsoid MakeFromSceneGraph(
      const QueryObject<double>& query_object, GeometryId geometry_id,
      std::optional<FrameId> expressed_in = std::nullopt);

 private:
  bool DoPointInSet(const Eigen::Ref<const Eigen::VectorXd>& x,
                    double tol) const override;

  void DoAddPointInSetConstraint(
      solvers::MathematicalProgram* prog,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& vars)
      const override;

  Eigen::MatrixXd A_{};
  Eigen::VectorXd center_{};
};

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
