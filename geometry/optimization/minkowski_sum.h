#pragma once

#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "drake/geometry/optimization/convex_set.h"
#include "drake/geometry/optimization/point.h"

namespace drake {
namespace geometry {
namespace optimization {

/** A convex set that represents the Minkowski sum of multiple sets:
S = X₁ ⨁ X₂ ⨁ ... ⨁ Xₙ =
    {x₁ + x₂ + ... + xₙ | x₁ ∈ X₁, x₂ ∈ X₂, ..., xₙ ∈ Xₙ}

@ingroup geometry_optimization
*/
class MinkowskiSum final : public ConvexSet {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MinkowskiSum)

  /** Constructs the sum from a vector of convex sets. */
  explicit MinkowskiSum(const ConvexSets& sets);

  /** Constructs the sum from a pair of convex sets. */
  MinkowskiSum(const ConvexSet& setA, const ConvexSet& setB);

  /** Constructs a MinkowskiSum from a SceneGraph geometry and pose in
  the @p reference_frame frame, obtained via the QueryObject. If @p
  reference_frame frame is std::nullopt, then it will be expressed in the world
  frame.

  Although in principle a MinkowskiSum can represent any ConvexSet as the sum of
  a single set, here we only support Capsule geometry, which will be represented
  as the (non-trivial) Minkowski sum of a sphere with a line segment.  Most
  SceneGraph geometry types are supported by at least one of the ConvexSet class
  constructors.

  @throws std::exception if geometry_id does not correspond to a Capsule. */
  MinkowskiSum(const QueryObject<double>& query_object, GeometryId geometry_id,
               std::optional<FrameId> reference_frame = std::nullopt);

  ~MinkowskiSum() final;

  /** The number of terms (or sets) used in the sum. */
  int num_terms() const { return sets_.size(); }

  /** Returns a reference to the ConvexSet defining the @p index term in the
  sum. */
  const ConvexSet& term(int index) const;

  /** Returns true if the point is in the set.

  Note: This requires the solution of a convex program; the @p tol parameter is
  currently ignored, and the solver tolerance is used instead.
  @see ConvexSet::set_solver
  */
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
  void ImplementGeometry(const Capsule& capsule, void* data) final;

  ConvexSets sets_{};  // Not marked const to support move semantics.
};

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
