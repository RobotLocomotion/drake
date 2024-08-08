#pragma once

#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "drake/geometry/optimization/convex_set.h"

namespace drake {
namespace geometry {
namespace optimization {

/** Implements the convex hull of a set of convex sets. The convex hull of multiple sets is defined as the smallest convex set that contains all the sets. 
 Given non-empty convex sets {X₁, X₂, ..., Xₙ}, the convex hull is the set of all convex combinations of points in the
 sets, i.e. {∑ᵢ λᵢ xᵢ | xᵢ ∈ Xᵢ, λᵢ ≥ 0, ∑ᵢ λᵢ = 1}.
@ingroup geometry_optimization */
class ConvexHull final : public ConvexSet, private ShapeReifier {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ConvexHull)

  /** Constructs the convex hull from a vector of convex sets. 
    @param sets A vector of convex sets that define the convex hull.
    @param check_empty_sets If true, the constructor will check if any of the sets are empty. If any of the sets are empty, they will be removed from the convex hull. If false, the constructor will not check if any of the sets are empty.
    @note If any of the participating sets, Xᵢ = ∅, is empty, then it would not participate in the convex hull. 
    Therefore, it is recommended to verify the emptiness of the participating sets before constructing the convex hull.
    Otherwise, the convex hull may be interpreted as the empty set because mathematical problems involving constraints such as xᵢ ∈ Xᵢ or variations thereof become infeasible, and checks such as IsEmpty() will return true even though other participating sets are non-empty.
    @warning  The check and removing empty participating sets in check_empty_sets is not recursive. If one of the participating sets is itself a convex hull of other sets in which one is empty (which was constructed incorrectly with check_empty_sets=false), the check_empty_sets will *not* remove it but will remove the whole convex hull itself.
    */
  explicit ConvexHull(const ConvexSets& sets, const bool check_empty_sets = true);

  ~ConvexHull() final;

  /** Returns the participating convex sets. */
  const ConvexSets& sets() const { return sets_; }

  /** Returns the participating non-empty convex sets, if the constructor was called with check_empty_sets=true. */
  const std::optional<ConvexSets>& maybe_non_empty_sets() const { return non_empty_sets_; }

  /** Returns a reference to the convex set at the given index (including empty sets). */
  const ConvexSet& element(int index) const;

  /** Returns the number of convex sets defining the convex hull (including empty sets). */
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
  std::optional<ConvexSets> non_empty_sets_{};
};

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
