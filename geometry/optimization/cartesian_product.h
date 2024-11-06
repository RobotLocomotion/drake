#pragma once

#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "drake/geometry/optimization/convex_set.h"

namespace drake {
namespace geometry {
namespace optimization {

/** The Cartesian product of convex sets is a convex set:
S = X₁ × X₂ × ⋯ × Xₙ =
    {(x₁, x₂, ..., xₙ) | x₁ ∈ X₁, x₂ ∈ X₂, ..., xₙ ∈ Xₙ}.

This class also supports a generalization of this concept in which the
coordinates are transformed by the linear map,
  {x | y = Ax + b, y ∈ Y₁ × Y₂ × ⋯ × Yₙ},
with the default values set to the identity map.  This concept is required for
reasoning about cylinders in arbitrary poses as cartesian products, and more
generally for describing any affine transform of a CartesianProduct.

Special behavior for IsEmpty: If there are no sets in the product, returns
nonempty by convention. See:
https://en.wikipedia.org/wiki/Empty_product#Nullary_Cartesian_product
Otherwise, if any set in the cartesian product is empty, the whole product
is empty.

@ingroup geometry_optimization */
class CartesianProduct final : public ConvexSet {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CartesianProduct);

  /** Constructs a default (zero-dimensional, nonempty) set. */
  CartesianProduct();

  /** Constructs the product from a vector of convex sets. */
  explicit CartesianProduct(const ConvexSets& sets);

  /** Constructs the product from a pair of convex sets. */
  CartesianProduct(const ConvexSet& setA, const ConvexSet& setB);

  /** Constructs the product of convex sets in the transformed coordinates:
  {x | y = Ax + b, y ∈ Y₁ × Y₂ × ⋯ × Yₙ}.
  @throws std::exception when `A` is not full column rank. */
  CartesianProduct(const ConvexSets& sets,
                   const Eigen::Ref<const Eigen::MatrixXd>& A,
                   const Eigen::Ref<const Eigen::VectorXd>& b);

  /** Constructs a CartesianProduct from a SceneGraph geometry and pose in the
  `reference_frame` frame, obtained via the QueryObject. If `reference_frame`
  frame is std::nullopt, then it will be expressed in the world frame.

  Although any geometry that can be used as a ConvexSet could also be a
  (trivial) CartesianProduct, we restrict this constructor to handling Cylinder
  geometry, which constructs the (non-trivial) Cartesian product of a
  HyperEllipsoid and an HPolyhedron. Most other SceneGraph geometry types are
  supported by at least one of the ConvexSet class constructors.

  @throws std::exception if geometry_id does not correspond to a Cylinder. */
  CartesianProduct(const QueryObject<double>& query_object,
                   GeometryId geometry_id,
                   std::optional<FrameId> reference_frame = std::nullopt);

  ~CartesianProduct() final;

  /** The number of factors (or sets) used in the product. */
  int num_factors() const { return sets_.size(); }

  /** Returns a reference to the ConvexSet defining the `index` factor in the
  product. */
  const ConvexSet& factor(int i) const;

  /** Returns a copy of the matrix A if it has been set, or nullopt otherwise.
   */
  std::optional<Eigen::MatrixXd> A() const { return A_; }

  /** Returns a copy of the vector b if it has been set, or nullopt otherwise.
   */
  std::optional<Eigen::VectorXd> b() const { return b_; }

  /** Returns true if each subvector is in its corresponding set with tolerance
  `tol`.  Note: Tolerance support for this query varies in the different convex
  set implementations. */
  using ConvexSet::PointInSet;

  /** @throws  if `set.has_exact_volume() == false` for any of the sets in the
  product. */
  using ConvexSet::CalcVolume;

  /** A CartesianProduct is bounded if and only if each constituent set is
  bounded. This class honors requests for parallelism only so far as its
  constituent sets do.
  @param parallelism The maximum number of threads to use.
  @note See @ref ConvexSet::IsBounded "parent class's documentation" for more
  details. */
  using ConvexSet::IsBounded;

 private:
  std::unique_ptr<ConvexSet> DoClone() const final;

  std::optional<bool> DoIsBoundedShortcutParallel(
      Parallelism parallelism) const final;

  bool DoIsEmpty() const final;

  std::optional<Eigen::VectorXd> DoMaybeGetPoint() const final;

  std::optional<Eigen::VectorXd> DoMaybeGetFeasiblePoint() const final;

  /* Given a list of vectors, one from each constituent set of this
  CartesianProduct, this stacks them into a single vector. Then, if this
  CartesianProduct has an associated transformation (in the form of an A_
  matrix and b_ vector), it applies that transformation. */
  Eigen::VectorXd StackAndMaybeTransform(
      const std::vector<Eigen::VectorXd>& points) const;

  bool DoPointInSet(const Eigen::Ref<const Eigen::VectorXd>& x,
                    double tol) const final;

  std::pair<VectorX<symbolic::Variable>,
            std::vector<solvers::Binding<solvers::Constraint>>>
  DoAddPointInSetConstraints(
      solvers::MathematicalProgram*,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>&) const final;

  std::vector<solvers::Binding<solvers::Constraint>>
  DoAddPointInNonnegativeScalingConstraints(
      solvers::MathematicalProgram* prog,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& x,
      const symbolic::Variable& t) const final;

  std::vector<solvers::Binding<solvers::Constraint>>
  DoAddPointInNonnegativeScalingConstraints(
      solvers::MathematicalProgram* prog,
      const Eigen::Ref<const Eigen::MatrixXd>& A_x,
      const Eigen::Ref<const Eigen::VectorXd>& b_x,
      const Eigen::Ref<const Eigen::VectorXd>& c, double d,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& x,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& t) const final;

  std::pair<std::unique_ptr<Shape>, math::RigidTransformd> DoToShapeWithPose()
      const final;

  std::unique_ptr<ConvexSet> DoAffineHullShortcut(
      std::optional<double> tol) const final;

  // The member variables are not const in order to support move semantics.
  ConvexSets sets_{};

  // Note: We make these optional, instead of Identity() and Zero(), so we can
  // avoid adding additional variables and constraints to MathematicalPrograms
  // in the implementation.
  std::optional<Eigen::MatrixXd> A_{std::nullopt};
  std::optional<Eigen::VectorXd> b_{std::nullopt};

  // When an `A` is passed to the constructor, we'll compute its decomposition
  // and store it here for later use. Note that even though the constructor for
  // a scene graph cylinder sets A_, it does not set A_decomp_.
  std::optional<Eigen::ColPivHouseholderQR<Eigen::MatrixXd>> A_decomp_{
      std::nullopt};

  double DoCalcVolume() const final;
};

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
