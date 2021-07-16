#pragma once

#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "drake/common/copyable_unique_ptr.h"
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
  virtual ~ConvexSet();

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

  /** Returns true iff the intersection between `this` and `other` is non-empty.
  @throws std::exception if the ambient dimension of `other` is not the same as
  that of `this`.
   */
  bool IntersectsWith(const ConvexSet& other) const;

  /** Returns true iff the set is bounded, e.g. there exists an element-wise
   * finite lower and upper bound for the set.  Note: for some derived classes,
   * this check is trivial, but for others it can require solving an (typically
   * small) optimization problem.  Check the derived class documentation for any
   * notes. */
  bool IsBounded() const { return DoIsBounded(); }

  /** Returns true iff the point x is contained in the set. */
  bool PointInSet(const Eigen::Ref<const Eigen::VectorXd>& x,
                  double tol = 0) const {
    DRAKE_DEMAND(x.size() == ambient_dimension());
    return DoPointInSet(x, tol);
  }

  // Note: I would like to return the Binding, but the type is subclass
  // dependent.
  /** Adds a constraint to an existing MathematicalProgram enforcing that the
  point defined by vars is inside the set. */
  void AddPointInSetConstraints(
      solvers::MathematicalProgram* prog,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& vars) const {
    DRAKE_DEMAND(vars.size() == ambient_dimension());
    return DoAddPointInSetConstraints(prog, vars);
  }

  /** Let S be this convex set.  When S is bounded, this method adds the convex
  constraints to imply
  @verbatim
  x ∈ t S,
  t ≥ 0,
  @endverbatim
  where x is a point in ℜⁿ (with n the ambient_dimension) and t is a scalar.

  When S is unbounded, then the behavior is almost identical, except when t=0.
  In this case, the constraints imply t ≥ 0, x ∈ t S ⊕ rec(S), where rec(S) is
  the recession cone of S (the asymptotic directions in which S is not bounded)
  and ⊕ is the Minkowski sum.  For t > 0, this is equivalent to x ∈ t S, but for
  t = 0, we have only x ∈ rec(S). */
  std::vector<solvers::Binding<solvers::Constraint>>
  AddPointInNonnegativeScalingConstraints(
      solvers::MathematicalProgram* prog,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& x,
      const symbolic::Variable& t) const;

  /** Constructs a Shape and a pose of the set in the world frame for use in
  the SceneGraph geometry ecosystem.

  @throws std::exception if ambient_dimension() != 3 or if the functionality for
  a particular set has not been implemented yet. */
  std::pair<std::unique_ptr<Shape>, math::RigidTransformd> ToShapeWithPose()
      const {
    DRAKE_DEMAND(ambient_dimension_ == 3);
    return DoToShapeWithPose();
  }

  // TODO(russt): Consider adding a set_solver() method here, which determines
  // the solver that any derived class uses if it solves an optimization.

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
  virtual bool DoIsBounded() const = 0;

  virtual bool DoPointInSet(const Eigen::Ref<const Eigen::VectorXd>& x,
                            double tol) const = 0;

  virtual void DoAddPointInSetConstraints(
      solvers::MathematicalProgram* prog,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& vars) const = 0;

  virtual std::vector<solvers::Binding<solvers::Constraint>>
  DoAddPointInNonnegativeScalingConstraints(
      solvers::MathematicalProgram* prog,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& x,
      const symbolic::Variable& t) const = 0;

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

/** Provides the recommended container for passing a collection of ConvexSet
instances. */
typedef std::vector<copyable_unique_ptr<ConvexSet>> ConvexSets;

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
