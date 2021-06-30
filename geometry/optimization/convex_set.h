#pragma once

#include <memory>
#include <optional>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace geometry {
namespace optimization {

// TODO(russt): Remove the experimental caveat once we've given this a proper
// spin.
/** @defgroup geometry_optimization Geometry Optimization
@ingroup geometry
@brief Provides an abstraction for reasoning about geometry in optimization
problems, and using optimization problems to solve geometry problems.

(Experimental).  Note that the features/designs in this class hierarchy are
considered experimental, and may change without deprecation.

### Relationship to other components in Drake.

SceneGraph handles many types of geometry (see Shape).  It is specialized to 3D
and puts a strong emphasis on efficient implementation of a particular subset
of geometry queries, like collision detection and signed-distance queries.
SceneGraph also provides a lot of valuable tools for content management,
including parsing geometries from file (via multibody::Parser) and Role.

MathematicalProgram has many relevant solvers::Cost / solvers::Constraint for
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

@ingroup geometry
@ingroup solvers
*/

/** Abstract base class for defining a convex set.
@ingroup geometry_optimization
*/
class ConvexSet : public ShapeReifier {
 public:
  virtual ~ConvexSet() {}

  /** Creates a unique deep copy of this set. */
  std::unique_ptr<ConvexSet> Clone() const;

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
                  double tol = 0) const {
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

  /** Constructs a Shape and a pose of the set in the world frame for use in
  the SceneGraph geometry ecosystem.

  @throws std::exception if ambient_dimension() != 3 or if the functionality for
  a particular set has not been implemented yet. */
  std::pair<std::unique_ptr<Shape>, math::RigidTransformd>
  ToShapeWithPose() const {
    DRAKE_DEMAND(ambient_dimension_ == 3);
    return DoToShapeWithPose();
  }

 protected:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ConvexSet)

  /** For use by derived classes to construct a %ConvexSet.

  @param cloner Function pointer to implement Clone(), typically of the form
  `&ConvexSetCloner<Derived>`.

  Here is a typical example:
  @code
   class MyConvexSet final : public ConvexSet {
    public:
     MyConvexSet() : ConvexSet(&ConvexSetCloner<MyConvexSet>, 3) {}
     ...
   };
  @endcode */
  ConvexSet(std::function<std::unique_ptr<ConvexSet>(const ConvexSet&)> cloner,
            int ambient_dimension);

  // Non-virtual interface implementations.
  virtual bool DoPointInSet(const Eigen::Ref<const Eigen::VectorXd>& x,
                            double tol) const = 0;

  virtual void DoAddPointInSetConstraint(
      solvers::MathematicalProgram* prog,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& vars) const = 0;

  virtual std::pair<std::unique_ptr<Shape>, math::RigidTransformd>
  DoToShapeWithPose() const = 0;

  std::function<std::unique_ptr<ConvexSet>(const ConvexSet&)> cloner_;
  int ambient_dimension_{0};
};

/** (Advanced) Implementation helper for ConvexSet::Clone. Refer to the
ConvexSet::ConvexSet() constructor documentation for an example. */
template <typename Derived>
std::unique_ptr<ConvexSet> ConvexSetCloner(const ConvexSet& other) {
  static_assert(std::is_base_of_v<ConvexSet, Derived>,
                "Concrete sets *must* be derived from the ConvexSet class");
  DRAKE_DEMAND(typeid(other) == typeid(Derived));
  const auto& typed_other = static_cast<const Derived&>(other);
  return std::make_unique<Derived>(typed_other);
}

// Forward declaration.
class HyperEllipsoid;

/** Implements a polyhedral convex set using the half-space representation:
`{x| A x ≤ b}`.  Note: This set may be unbounded.
@ingroup geometry_optimization
*/
class HPolyhedron final : public ConvexSet {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(HPolyhedron)

  /** Constructs the polyhedron.
  @pre A.rows() == b.size().
  */
  HPolyhedron(const Eigen::Ref<const Eigen::MatrixXd>& A,
              const Eigen::Ref<const Eigen::VectorXd>& b);

  /** Constructs a new HPolyhedron from a SceneGraph geometry and pose in the
  @p expressed_in frame, obtained via the QueryObject.  If @p expressed_in
  frame is std::nullopt, then it will be expressed in the world frame.

  @throws std::exception the geometry is not a convex polytope. */
  HPolyhedron(const QueryObject<double>& query_object, GeometryId geometry_id,
              std::optional<FrameId> expressed_in = std::nullopt);
  // TODO(russt): Add a method/constructor that would create the geometry using
  // SceneGraph's AABB or OBB representation (for arbitrary objects) pending
  // #15121.

  virtual ~HPolyhedron() {}

  /** Returns the half-space representation matrix A. */
  const Eigen::MatrixXd& A() const { return A_; }

  /** Returns the half-space representation vector b. */
  const Eigen::VectorXd& b() const { return b_; }

  // TODO(russt): Add bool IsBounded() so users can test the precondition.
  /** Solves a semi-definite program to compute the inscribed ellipsoid.
  From Section 8.4.2 in Boyd and Vandenberghe, 2004, we solve
  @verbatim
  max_{C,d} log det (C)
        s.t. |aᵢC|₂ ≤ bᵢ - aᵢd, ∀i
            C ≽ 0
  @endverbatim
  where aᵢ and bᵢ denote the ith row.  This defines the ellipsoid
  E = { Cx + d | |x|₂ ≤ 1}.

  @pre the HPolyhedron is bounded.
  @throws std::exception if the solver fails to solve the problem.
  */
  HyperEllipsoid MaximumVolumeInscribedEllipsoid() const;

  /** Constructs a polyhedron as an axis-aligned box from the lower and upper
   * corners. */
  static HPolyhedron MakeBox(const Eigen::Ref<const Eigen::VectorXd>& lb,
                             const Eigen::Ref<const Eigen::VectorXd>& ub);

  /** Constructs the L∞-norm unit box in @p dim dimensions, {x | |x|∞ <= 1 }.
  This is an axis-aligned box, centered at the origin, with edge length 2. */
  static HPolyhedron MakeUnitBox(int dim);

 private:
  bool DoPointInSet(const Eigen::Ref<const Eigen::VectorXd>& x,
                    double tol) const final;

  void DoAddPointInSetConstraint(
      solvers::MathematicalProgram* prog,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& vars)
      const final;

  // TODO(russt): Implement DoToShapeWithPose.  Currently we don't have a Shape
  // that can consume this output.  The obvious candidate is Convex, that class
  // currently only stores the filename of an .obj file, and I shouldn't need
  // to go to disk and back to get this geometry into ProximityEngine nor
  // MeshcatVisualizer.
  //
  // When we do implement this and also have the AHPolyhedron class, we should
  // add recommendation here that: "If ambient_dimension() != 3, then consider
  // using the AH polytope representation to project it to 3D."
  std::pair<std::unique_ptr<Shape>, math::RigidTransformd> DoToShapeWithPose()
      const final {
    throw std::runtime_error(
      "ToShapeWithPose is not implemented yet for HPolyhedron.  Implementing "
      "this will likely require additional support from the Convex shape "
      "class (to support in-memory mesh data, or file I/O).");
  }

  // Implement support shapes for the ShapeReifier interface.
  using ShapeReifier::ImplementGeometry;
  void ImplementGeometry(const HalfSpace&, void* data) final;
  void ImplementGeometry(const Box& box, void* data) final;
  // TODO(russt): Support ImplementGeometry(const Convex& convex, ...), but
  // currently it would require e.g. digging ReadObjForConvex out of
  // proximity_engine.cc.

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
class HyperEllipsoid final : public ConvexSet {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(HyperEllipsoid)

  /** Constructs the ellipsoid.
  @pre A.cols() == center.size(). */
  HyperEllipsoid(const Eigen::Ref<const Eigen::MatrixXd>& A,
                 const Eigen::Ref<const Eigen::VectorXd>& center);

  /** Constructs a HyperEllipsoid from a SceneGraph geometry and pose in
  the @p expressed_in frame, obtained via the QueryObject. If @p expressed_in
  frame is std::nullopt, then it will be expressed in the world frame.

  @throws std::exception if geometry_id does not represent a shape that can be
  described as an HyperEllipsoid. */
  HyperEllipsoid(const QueryObject<double>& query_object,
                 GeometryId geometry_id,
                 std::optional<FrameId> expressed_in = std::nullopt);

  virtual ~HyperEllipsoid() {}

  /** Returns the quadratic form matrix A. */
  const Eigen::MatrixXd& A() const { return A_; }

  /** Returns the center of the ellipsoid. */
  const Eigen::VectorXd& center() const { return center_; }

  /** Constructs a Ellipsoid shape description of this set.  Note that the
  choice of ellipsoid is not unique.  This method chooses to order the
  Ellipsoid parameters a ≥ b ≥ c.

  @throws std::exception if ambient_dimension() != 3
  @throws std::exception if A is not invertible (has any eigenvalue less than
  sqrt(1e-12)). This tolerance is not carefully tuned (yet).  We use Eigen's
  SelfAdjointEigenSolver to take the eigenvalues of AᵀA; this solver is listed
  as having accuracy "Good":
  https://eigen.tuxfamily.org/dox/group__TopicLinearAlgebraDecompositions.html
  but how does that translate into a numerical precision? */
  using ConvexSet::ToShapeWithPose;

 private:
  bool DoPointInSet(const Eigen::Ref<const Eigen::VectorXd>& x,
                    double tol) const final;

  void DoAddPointInSetConstraint(
      solvers::MathematicalProgram* prog,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& vars)
      const final;

  std::pair<std::unique_ptr<Shape>, math::RigidTransformd> DoToShapeWithPose()
      const final;

  // Implement support shapes for the ShapeReifier interface.
  using ShapeReifier::ImplementGeometry;
  void ImplementGeometry(const Sphere& sphere, void* data) final;
  void ImplementGeometry(const Ellipsoid& ellipsoid, void* data) final;

  Eigen::MatrixXd A_{};
  Eigen::VectorXd center_{};
};

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
