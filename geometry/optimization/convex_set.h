#pragma once

#include <limits>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_assert.h"
#include "drake/common/parallelism.h"
#include "drake/common/reset_after_move.h"
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

### Relationship to other components in Drake.

SceneGraph handles many types of geometry (see Shape).  It is specialized to 3D
and puts a strong emphasis on efficient implementation of a particular subset
of geometry queries, like collision detection and signed-distance queries.
SceneGraph also provides a lot of valuable tools for content management,
including parsing geometries from file (via multibody::Parser) and Role.

MathematicalProgram has many relevant solvers::Cost / solvers::Constraint for
reasoning about geometry (e.g., the LorentzCone or even LinearConstraint). The
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
@ingroup solvers */

/** The result of a volume calculation from CalcVolumeViaSampling(). */
struct SampledVolume {
  /** The estimated volume of the set. */
  double volume{};

  /** An upper bound for the relative accuracy of the volume estimate. When not
  evaluated, this value is NaN. */
  double rel_accuracy{};

  /** The number of samples used to compute the volume estimate. */
  int num_samples{};
};

/** Abstract base class for defining a convex set.
@ingroup geometry_optimization */
class ConvexSet {
 public:
  virtual ~ConvexSet();

  /** Creates a unique deep copy of this set. */
  std::unique_ptr<ConvexSet> Clone() const { return DoClone(); }

  /** Returns the dimension of the vector space in which the elements of this
  set are evaluated.  Contrast this with the `affine dimension`: the dimension
  of the smallest affine subset of the ambient space that contains our set.
  For example, if we define a set using `A*x = b`, where `A` has linearly
  independent rows, then the ambient dimension is the dimension of `x`, but the
  affine dimension of the set is `ambient_dimension() - rank(A)`. */
  int ambient_dimension() const { return ambient_dimension_; }

  /** Returns true iff the intersection between `this` and `other` is non-empty.
  @throws std::exception if the ambient dimension of `other` is not the same as
  that of `this`. */
  bool IntersectsWith(const ConvexSet& other) const;

  /** Returns true iff the set is bounded, e.g., there exists an element-wise
  finite lower and upper bound for the set.  Note: for some derived classes,
  this check is trivial, but for others it can require solving a number of
  (typically small) optimization problems. Each derived class documents the cost
  of its boundedness test and whether it honors the request for parallelism.
  (Derived classes which do not have a specialized check will, by default, honor
  parallelism requests.) Note that the overhead of multithreading may lead to
  slower runtimes for simple, low-dimensional sets, but can enable major
  speedups for more challenging problems.

  @param parallelism requests the number of cores to use when solving
  mathematical programs to check boundedness, subject to whether a particular
  derived class honors parallelism. */
  bool IsBounded(Parallelism parallelism = Parallelism::None()) const {
    if (ambient_dimension() == 0) {
      return true;
    }
    auto shortcut_result = DoIsBoundedShortcutParallel(parallelism);
    if (shortcut_result.has_value()) {
      return shortcut_result.value();
    }
    shortcut_result = DoIsBoundedShortcut();
    if (shortcut_result.has_value()) {
      return shortcut_result.value();
    }
    return GenericDoIsBounded(parallelism);
  }

  /** Returns true iff the set is empty. Note: for some derived classes, this
  check is trivial, but for others, it can require solving a (typically small)
  optimization problem. Check the derived class documentation for any notes.
  Zero-dimensional sets must be handled specially. There are two possible sets
  in a zero-dimensional space -- the empty set, and the whole set (which is
  simply the "zero vector space", {0}.) For more details, see:
  https://en.wikipedia.org/wiki/Examples_of_vector_spaces#Trivial_or_zero_vector_space
  Zero-dimensional sets are considered to be nonempty by default. Sets
  which can be zero-dimensional and empty must handle this behavior in their
  derived implementation of DoIsEmpty. An example of such a subclass is
  VPolytope. */
  bool IsEmpty() const { return DoIsEmpty(); }

  /** If this set trivially contains exactly one point, returns the value of
  that point. Otherwise, returns nullopt. By "trivially", we mean that
  representation of the set structurally maps to a single point; if checking
  for point-ness would require solving an optimization program, returns nullopt.
  In other words, this is a relatively cheap function to call. */
  std::optional<Eigen::VectorXd> MaybeGetPoint() const {
    if (ambient_dimension() == 0) {
      if (IsEmpty()) {
        return std::nullopt;
      } else {
        return Eigen::VectorXd::Zero(0);
      }
    }
    return DoMaybeGetPoint();
  }

  /** Returns a feasible point within this convex set if it is nonempty,
  and nullopt otherwise. */
  std::optional<Eigen::VectorXd> MaybeGetFeasiblePoint() const {
    if (ambient_dimension() == 0) {
      if (IsEmpty()) {
        return std::nullopt;
      } else {
        return Eigen::VectorXd::Zero(0);
      }
    }
    if (MaybeGetPoint().has_value()) {
      return MaybeGetPoint();
    } else {
      return DoMaybeGetFeasiblePoint();
    }
  }

  /** Returns true iff the point x is contained in the set. If the ambient
  dimension is zero, then if the set is nonempty, the point is trivially in
  the set, and if the set is empty, the point is trivially not in the set. */
  bool PointInSet(const Eigen::Ref<const Eigen::VectorXd>& x,
                  double tol = 0) const {
    DRAKE_THROW_UNLESS(x.size() == ambient_dimension());
    if (ambient_dimension() == 0) {
      return !IsEmpty();
    }
    const auto shortcut_result = DoPointInSetShortcut(x, tol);
    if (shortcut_result.has_value()) {
      return shortcut_result.value();
    }
    return DoPointInSet(x, tol);
  }

  /** Adds a constraint to an existing MathematicalProgram enforcing that the
  point defined by vars is inside the set.
  @return (new_vars, new_constraints) Some of the derived class will add new
  decision variables to enforce this constraint, we return all the newly added
  decision variables as new_vars. The meaning of these new decision variables
  differs in each subclass. If no new variables are added, then we return an
  empty Eigen vector. Also we return all the newly added constraints to `prog`
  through this function.
  @throws std::exception if ambient_dimension() == 0 */
  std::pair<VectorX<symbolic::Variable>,
            std::vector<solvers::Binding<solvers::Constraint>>>
  AddPointInSetConstraints(
      solvers::MathematicalProgram* prog,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& vars) const;

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
  t = 0, we have only x ∈ rec(S).
  @throws std::exception if ambient_dimension() == 0 */
  // TODO(hongkai.dai): return the new variables also.
  std::vector<solvers::Binding<solvers::Constraint>>
  AddPointInNonnegativeScalingConstraints(
      solvers::MathematicalProgram* prog,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& x,
      const symbolic::Variable& t) const;

  /** Let S be this convex set.  When S is bounded, this method adds the convex
  constraints to imply
  @verbatim
  A * x + b ∈ (c' * t + d) S,
  c' * t + d ≥ 0,
  @endverbatim
  where A is an n-by-m matrix (with n the ambient_dimension), b is a vector of
  size n, c is a vector of size p, x is a point in ℜᵐ, and t is a point in ℜᵖ.

  When S is unbounded, then the behavior is almost identical, except when c' *
  t+d=0. In this case, the constraints imply
  @verbatim
  A * x + b ∈ (c' * t + d) S ⊕ rec(S),
  c' * t + d ≥ 0,
  @endverbatim
  where rec(S) is the recession cone of S (the asymptotic directions in which S
  is not bounded) and ⊕ is the Minkowski sum.  For c' * t + d > 0, this is
  equivalent to A * x + b ∈ (c' * t + d) S, but for c' * t + d = 0, we have
  only A * x + b ∈ rec(S).
  @throws std::exception if ambient_dimension() == 0 */
  std::vector<solvers::Binding<solvers::Constraint>>
  AddPointInNonnegativeScalingConstraints(
      solvers::MathematicalProgram* prog,
      const Eigen::Ref<const Eigen::MatrixXd>& A,
      const Eigen::Ref<const Eigen::VectorXd>& b,
      const Eigen::Ref<const Eigen::VectorXd>& c, double d,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& x,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& t) const;

  /** Constructs a Shape and a pose of the set in the world frame for use in
  the SceneGraph geometry ecosystem.
  @throws std::exception if ambient_dimension() != 3 or if the functionality for
  a particular set has not been implemented yet. */
  std::pair<std::unique_ptr<Shape>, math::RigidTransformd> ToShapeWithPose()
      const {
    DRAKE_THROW_UNLESS(ambient_dimension() == 3);
    return DoToShapeWithPose();
  }

  // TODO(russt): Consider adding a set_solver() method here, which determines
  // the solver that any derived class uses if it solves an optimization.

  /** Computes the exact volume for the convex set.
  @note Not every convex set can report an exact volume. In that case, use
  CalcVolumeViaSampling() instead.
  @throws std::exception if `has_exact_volume()` returns `false`.
  @throws if ambient_dimension() == 0. */
  double CalcVolume() const;

  /** Calculates an estimate of the volume of the convex set using sampling and
  performing Monte Carlo integration.
  @note this method is intended to be used for low to moderate dimensions
  (d<15). For larger dimensions, a telescopic product approach has yet to be
  implemented. See, e.g.,
  https://proceedings.mlr.press/v151/chevallier22a/chevallier22a.pdf
  @param generator a random number generator.
  @param desired_rel_accuracy the desired relative accuracy of the volume
  estimate in the sense that the estimated volume is likely to be within the
  interval defined by (1±2*desired_rel_accuracy)*true_volume with probability of
  *at least* 0.95 according to the Law of Large Numbers.
  https://people.math.umass.edu/~lr7q/ps_files/teaching/math456/Chapter6.pdf
  The computation will terminate when the relative error is less than
  rel_accuracy or when the maximum number of samples is reached.
  @param max_num_samples the maximum number of samples to use.
  @pre `desired_rel_accuracy` is in the range [0,1].
  @return a pair the estimated volume of the set and an upper bound for the
  relative accuracy
  @throws if ambient_dimension() == 0.
  @throws if the minimum axis-aligned bounding box of the set cannot be
  computed. */
  SampledVolume CalcVolumeViaSampling(RandomGenerator* generator,
                                      const double desired_rel_accuracy = 1e-2,
                                      const int max_num_samples = 1e4) const;

  /** Computes in the L₂ norm the distance and the nearest point in this convex
   set to every column of @p points. If this set is empty, we return nullopt.
   @pre points.rows() == ambient_dimension().
   @throws if the internal convex optimization solver fails. */
  std::optional<std::pair<std::vector<double>, Eigen::MatrixXd>> Projection(
      const Eigen::Ref<const Eigen::MatrixXd>& points) const;

  /** Returns true if the exact volume can be computed for this convex set
  instance.
  @note This value reasons about to the generic case of the convex set class
  rather than the specific instance of the convex set. For example, the exact
  volume of a box is trivival to compute, but if the box is created as a
  HPolyhedron, then the exact volume cannot be computed. */
  bool has_exact_volume() const { return has_exact_volume_; }

 protected:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ConvexSet);

  /** For use by derived classes to construct a %ConvexSet.
  @param has_exact_volume Derived classes should pass `true` if they've
  implemented DoCalcVolume() to return a value (at least sometimes). */
  explicit ConvexSet(int ambient_dimension, bool has_exact_volume);

  /** Implements non-virtual base class serialization. */
  template <typename Archive>
  void Serialize(Archive* a) {
    // Visit the mutable reference inside the reset_after_move wrapper.
    int& ambient_dimension = ambient_dimension_;
    a->Visit(DRAKE_NVP(ambient_dimension));
  }

  /** Non-virtual interface implementation for Clone(). */
  virtual std::unique_ptr<ConvexSet> DoClone() const = 0;

  /** Non-virtual interface implementation for DoIsBoundedShortcut(). Trivially
  returns std::nullopt. This allows a derived class to implement its own
  boundedness checks, to potentially avoid the more expensive base class checks.
  @pre ambient_dimension() >= 0 */
  virtual std::optional<bool> DoIsBoundedShortcut() const {
    return std::nullopt;
  }

  /** Non-virtual interface implementation for DoIsBoundedShortcutParallel().
  Trivially returns std::nullopt. This allows a derived class to implement its
  own boundedness checks that leverage parallelization, to potentially avoid the
  more expensive base class checks.
  @pre ambient_dimension() >= 0 */
  virtual std::optional<bool> DoIsBoundedShortcutParallel(Parallelism) const {
    return std::nullopt;
  }

  /** Non-virtual interface implementation for DoProjectionShortcut().

  This allows a derived class to implement a method which computes the
  projection of some, but not necessarily all, of the @p points more efficiently
  than the generic implementation.

  The default implementation checks whether each column of @p points is in the
  set using DoPointInSetShortcut. Points in the set are given a distance of 0
  and are projected to themselves.

  @param[in] points are the points which we wish to project to the convex set.
  @param[in,out] projected_points are the projection of @p points onto the
  convex set.
  @return A vector `distances` which is the same size as @p points.cols().These
  are the distances from @p points to the convex set. If distances[i] has a
  value, then projected_points->col(i) is the projection of points.col(i) onto
  the set. If distances[i] is nullopt, then the projection of points.col(i) has
  not yet been computed, and so the value at projected_points->col(i) is
  meaningless.

  @pre ambient_dimension() >= 0
  @pre distances.size() == points.cols()
   */

  virtual std::vector<std::optional<double>> DoProjectionShortcut(
      const Eigen::Ref<const Eigen::MatrixXd>& points,
      EigenPtr<Eigen::MatrixXd> projected_points) const;

  /** Non-virtual interface implementation for IsEmpty(). The default
  implementation solves a feasibility optimization problem, but derived
  classes can override with a custom (more efficient) implementation.
  Zero-dimensional sets are considered to be nonempty by default. Sets which
  can be zero-dimensional and empty must handle this behavior in their
  derived implementation of DoIsEmpty. */
  virtual bool DoIsEmpty() const;

  /** Non-virtual interface implementation for MaybeGetPoint(). The default
  implementation returns nullopt. Sets that can model a single point should
  override with a custom implementation.
  @pre ambient_dimension() >= 0. */
  virtual std::optional<Eigen::VectorXd> DoMaybeGetPoint() const;

  /** Non-virtual interface implementation for MaybeGetFeasiblePoint(). The
  default implementation solves a feasibility optimization problem, but
  derived classes can override with a custom (more efficient) implementation. */
  virtual std::optional<Eigen::VectorXd> DoMaybeGetFeasiblePoint() const;

  /** Non-virtual interface implementation for PointInSet().
  @pre x.size() == ambient_dimension()
  @pre ambient_dimension() >= 0 */
  virtual bool DoPointInSet(const Eigen::Ref<const Eigen::VectorXd>& x,
                            double tol) const;

  /** A non-virtual interface implementation for PointInSet() that should be
   used when the PointInSet() can be computed more efficiently than solving a
   convex program.

   @returns Returns true if and only if x is known to be in the set. Returns
   false if and only if x is known to not be in the set. Returns std::nullopt
   if a shortcut implementation is not provided (i.e. the method has not
   elected to decide whether the point x is in the set).

   For example, membership in a VPolytope cannot be verified without solving a
   linear program and so no shortcut implementation should be provided. On the
   other hand, membership in an HPolyhedron can be checked by checking the
   inequality Ax ≤ b and so a shortcut is possible.
   */
  virtual std::optional<bool> DoPointInSetShortcut(
      const Eigen::Ref<const Eigen::VectorXd>& x, double tol) const {
    unused(x);
    unused(tol);
    return std::nullopt;
  }

  /** Non-virtual interface implementation for AddPointInSetConstraints().
  @pre vars.size() == ambient_dimension()
  @pre ambient_dimension() > 0 */
  virtual std::pair<VectorX<symbolic::Variable>,
                    std::vector<solvers::Binding<solvers::Constraint>>>
  DoAddPointInSetConstraints(
      solvers::MathematicalProgram* prog,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& vars) const = 0;

  /** Non-virtual interface implementation for
  AddPointInNonnegativeScalingConstraints().
  @pre x.size() == ambient_dimension()
  @pre ambient_dimension() > 0 */
  virtual std::vector<solvers::Binding<solvers::Constraint>>
  DoAddPointInNonnegativeScalingConstraints(
      solvers::MathematicalProgram* prog,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& x,
      const symbolic::Variable& t) const = 0;

  /** Non-virtual interface implementation for
  AddPointInNonnegativeScalingConstraints(). Subclasses must override to add the
  constraints needed to keep the point A * x + b in the non-negative scaling of
  the set. Note that subclasses do not need to add the constraint c * t + d ≥ 0
  as it is already added.
  @pre ambient_dimension() > 0
  @pre A.rows() == ambient_dimension()
  @pre A.rows() == b.rows()
  @pre A.cols() == x.size()
  @pre c.rows() == t.size() */
  virtual std::vector<solvers::Binding<solvers::Constraint>>
  DoAddPointInNonnegativeScalingConstraints(
      solvers::MathematicalProgram* prog,
      const Eigen::Ref<const Eigen::MatrixXd>& A,
      const Eigen::Ref<const Eigen::VectorXd>& b,
      const Eigen::Ref<const Eigen::VectorXd>& c, double d,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& x,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& t) const = 0;

  /** Non-virtual interface implementation for ToShapeWithPose().
  @pre ambient_dimension() == 3 */
  virtual std::pair<std::unique_ptr<Shape>, math::RigidTransformd>
  DoToShapeWithPose() const = 0;

  /** Non-virtual interface implementation for CalcVolume(). This will *only* be
  called if has_exact_volume() returns true and ambient_dimension() > 0 */
  virtual double DoCalcVolume() const;

  /** Instances of subclasses such as CartesianProduct and MinkowskiSum can
  have constituent sets with zero ambient dimension, which much be handled in a
  special manner when calling methods such as DoAddPointInSetConstraints. If the
  set is empty, a trivially infeasible constraint must be added. We also warn
  the user when this happens, since they probably didn't intend it to occur.
  If the set is nonempty, then it's the unique zero-dimensional vector space
  {0}, and no additional variables or constraints are needed. If a new variable
  is created, return it, to optionally be stored (as in
  AddPointInSetConstraints), or not be stored (as in
  DoAddPointInNonnegativeScalingConstraints). */
  std::optional<symbolic::Variable> HandleZeroAmbientDimensionConstraints(
      solvers::MathematicalProgram* prog, const ConvexSet& set,
      std::vector<solvers::Binding<solvers::Constraint>>* constraints) const;

  /** When there is a more efficient strategy to compute the affine hull of this
  set, returns affine hull as an AffineSubspace. When no efficient conversion
  exists, returns null. The default base class implementation returns null. This
  method is used by the AffineSubspace constructor to short-circuit the generic
  iterative approach. (This function is static to allow calling it from the
  AffineSubspace constructor, but is conceptially a normal member function.)
  The return type is ConvexSet to avoid a forward declaration; any non-null
  result must always have the AffineSubspace as its runtime type. */
  static std::unique_ptr<ConvexSet> AffineHullShortcut(
      const ConvexSet& self, std::optional<double> tol);

  /** NVI implementation of DoAffineHullShortcut, which trivially returns null.
  Derived classes that have efficient algorithms should override this method. */
  virtual std::unique_ptr<ConvexSet> DoAffineHullShortcut(
      std::optional<double> tol) const;

 private:
  /** Generic implementation for IsBounded() -- applicable for all convex sets.
  @pre ambient_dimension() >= 0 */
  bool GenericDoIsBounded(Parallelism parallelism) const;

  /** Generic implementation for PointInSet() -- applicable for all convex sets.
  @pre ambient_dimension() >= 0 */
  bool GenericDoPointInSet(const Eigen::Ref<const Eigen::VectorXd>& x,
                           double tol) const;

  /** Generic implementation for Projection() -- applicable for all convex sets.
   @pre ambient_dimension() >= 0
   */
  std::optional<std::pair<std::vector<double>, Eigen::MatrixXd>>
  GenericDoProjection(const Eigen::Ref<const Eigen::MatrixXd>& point) const;

  // The reset_after_move wrapper adjusts ConvexSet's default move constructor
  // and move assignment operator to set the ambient dimension of a moved-from
  // object back to zero. This is essential to keep the ambient dimension in
  // sync with any moved-from member fields in concrete ConvexSet subclasses.
  //
  // For example, the `Eigen::VectorXd x_` member field of `Point` will be moved
  // out, ending up with `x_.size() == 0` on the moved-from Point. To maintain
  // the invariant that `x_.size() == ambient_dimension_`, we need to zero the
  // dimension when `x_` is moved-from.
  //
  // Similarly, for subclasses that are composite sets (e.g., CartesianProduct)
  // the `ConvexSets sets_` vector becomes empty when moved-from. To maintain
  // an invariant like `∑(set.size() for set in sets_) == ambient_dimension_`,
  // we need to zero the dimension when `sets_` is moved-from.
  reset_after_move<int> ambient_dimension_;

  bool has_exact_volume_{false};
};

/** Provides the recommended container for passing a collection of ConvexSet
instances. */
typedef std::vector<copyable_unique_ptr<ConvexSet>> ConvexSets;

/** Helper function that allows the ConvexSets to be initialized from arguments
containing ConvexSet references, or unique_ptr<ConvexSet> instances, or any
object that can be assigned to ConvexSets::value_type. */
template <typename... Args>
ConvexSets MakeConvexSets(Args&&... args) {
  ConvexSets sets;
  constexpr size_t N = sizeof...(args);
  sets.resize(N);
  // This is a "constexpr for" loop for 0 <= I < N.
  auto args_tuple = std::forward_as_tuple(std::forward<Args>(args)...);
  auto seq_into_sets = [&]<size_t... I>(std::integer_sequence<size_t, I...>&&) {
    ((sets[I] = std::get<I>(std::move(args_tuple))), ...);
  };  // NOLINT
  seq_into_sets(std::make_index_sequence<N>{});
  return sets;
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
