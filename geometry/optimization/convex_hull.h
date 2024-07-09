#pragma once

#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "drake/geometry/optimization/convex_set.h"

namespace drake {
namespace geometry {
namespace optimization {

/** Implements the convex hull of a set of convex sets. Given sets {X₁, X₂, ...,
 Xₙ}, the convex hull is the set of all convex combinations of points in the
 sets, i.e. {∑ᵢ λᵢ xᵢ | xᵢ ∈ Xᵢ, λᵢ ≥ 0, ∑ᵢ λᵢ = 1}.

@note If any of the participating sets, Xᵢ = ∅, is empty, then the convex hull
is also considered empty. This interpretation arises because mathematical
problems involving constraints such as xᵢ ∈ Xᵢ or variations thereof become
infeasible. It is advisable to verify the emptiness of the participating sets
before attempting to construct the convex hull.
@ingroup geometry_optimization */
class ConvexHull final : public ConvexSet, private ShapeReifier {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ConvexHull)

  /** Constructs the convex hull from a vector of convex sets. */
  explicit ConvexHull(const ConvexSets& sets);

  ~ConvexHull() final;

  /** Returns the participating convex sets. */
  const ConvexSets& sets() const { return sets_; }

  /** Returns a reference to the convex set at the given index. */
  const ConvexSet& element(int index) const;

  /** Returns the number of convex sets defining the convex hull. */
  int num_elements() const { return sets_.size(); }

  using ConvexSet::IsBounded;

  /** @throws  Not implemented. */
  using ConvexSet::CalcVolume;

 private:
  std::unique_ptr<ConvexSet> DoClone() const final;

  std::optional<bool> DoIsBoundedShortcut() const final;

  bool DoIsEmpty() const final;

  std::optional<Eigen::VectorXd> DoMaybeGetPoint() const final;

  bool DoPointInSet(const Eigen::Ref<const Eigen::VectorXd>& x,
                    double tol) const final;

  std::pair<VectorX<symbolic::Variable>,
            std::vector<solvers::Binding<solvers::Constraint>>>
  DoAddPointInSetConstraints(
      solvers::MathematicalProgram* prog,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& vars)
      const final;

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

  ConvexSets sets_{};
};

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
