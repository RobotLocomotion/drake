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
    {(x₁, x₂, ..., xₙ) | x₁ ∈ X₁, x₂ ∈ X₂, ..., xₙ ∈ Xₙ}
We currently require the sets X to be bounded.

This class also supports a generalization of this concept in which the
coordinates are transformed by the linear map,
  {x | y = Ax + b, y ∈ Y₁ × Y₂ × ⋯ × Yₙ},
with the default values set to the identity map.  This concept is required for
reasoning about cylinders in arbitrary poses as cartesian products, and more
generally for describing any affine transform of a CartesianProduct.

@ingroup geometry_optimization
*/
class CartesianProduct final : public ConvexSet {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CartesianProduct)

  /** Constructs the product from a vector of convex sets. All of the sets must
  be bounded. */
  explicit CartesianProduct(const ConvexSets& sets);

  /** Constructs the product from a pair of convex sets. `setA` and `setB` must
  be bounded. */
  CartesianProduct(const ConvexSet& setA, const ConvexSet& setB);

  /** Constructs the product of convex sets in the transformed coordinates:
  {x | y = Ax + b, y ∈ Y₁ × Y₂ × ⋯ × Yₙ}. `A` must be full column rank, and all
  of the sets must be bounded. */
  CartesianProduct(const ConvexSets& sets,
                   const Eigen::Ref<const Eigen::MatrixXd>& A,
                   const Eigen::Ref<const Eigen::VectorXd>& b);

  /** Constructs a CartesianProduct from a SceneGraph geometry and pose in
  the @p reference_frame frame, obtained via the QueryObject. If @p
  reference_frame frame is std::nullopt, then it will be expressed in the world
  frame.

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

  /** Returns a reference to the ConvexSet defining the @p index factor in the
  product. */
  const ConvexSet& factor(int i) const;

  /** Returns true if each subvector is in its corresponding set with tolerance
  @p tol.  Note: Tolerance support for this query varies in the different
  convex set implementations. */
  using ConvexSet::PointInSet;

 private:
  bool DoIsBounded() const final;

  bool DoPointInSet(const Eigen::Ref<const Eigen::VectorXd>& x,
                    double tol) const final;

  void DoAddPointInSetConstraints(
      solvers::MathematicalProgram*,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>&) const final;

  std::vector<solvers::Binding<solvers::Constraint>>
  DoAddPointInNonnegativeScalingConstraints(
      solvers::MathematicalProgram* prog,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& x,
      const symbolic::Variable& t) const final;

  std::pair<std::unique_ptr<Shape>, math::RigidTransformd> DoToShapeWithPose()
      const final;

  // Implement support shapes for the ShapeReifier interface.
  using ShapeReifier::ImplementGeometry;
  void ImplementGeometry(const Cylinder& cylinder, void* data) final;

  // The member variables are not const in order to support move semantics.
  ConvexSets sets_{};

  // Note: We make these optional, instead of Identity() and Zero(), so we can
  // avoid adding additional variables and constraints to MathematicalPrograms
  // in the implementation.
  std::optional<Eigen::MatrixXd> A_{std::nullopt};
  std::optional<Eigen::VectorXd> b_{std::nullopt};
};

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
