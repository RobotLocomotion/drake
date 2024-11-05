#pragma once

#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "drake/geometry/optimization/convex_set.h"

namespace drake {
namespace geometry {
namespace optimization {

/** Implements the convex hull of a set of convex sets. The convex hull of
multiple sets is defined as the smallest convex set that contains all the sets.
 Given non-empty convex sets {X₁, X₂, ..., Xₙ}, the convex hull is the set of
all convex combinations of points in the sets, i.e. {∑ᵢ λᵢ xᵢ | xᵢ ∈ Xᵢ, λᵢ ≥ 0,
∑ᵢ λᵢ = 1}.
@ingroup geometry_optimization */
class ConvexHull final : public ConvexSet, private ShapeReifier {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ConvexHull)

  /** Constructs the convex hull from a vector of convex sets.
  @param sets A vector of convex sets that define the convex hull.
  @param remove_empty_sets If true, the constructor will check if any of the
  sets are empty and will not consider them. If false, the constructor will
  not check if any of the sets are empty.
  @warning If remove_empty_sets is set to false, but some of the sets are in
  fact empty, then unexpected and incorrect results may occur. Only set this
  flag to false if you are sure that your sets are non-empty and performance in
  the constructor is critical.
  */
  explicit ConvexHull(const ConvexSets& sets,
                      const bool remove_empty_sets = true);

  ~ConvexHull() final;

  /** Returns the participating convex sets. */
  const ConvexSets& sets() const { return sets_; }

  /** Returns the participating sets in the convex hull. If the constructor was
  called with remove_empty_sets=false, this function will return the original
  sets, including potentially empty sets. */
  const ConvexSets& participating_sets() const { return participating_sets_; }

  /** Returns true if `this` was constructed with remove_empty_sets=true. */
  bool empty_sets_removed() const { return empty_sets_removed_; }

  /** Returns a reference to the convex set at the given index (including empty
   * sets). */
  const ConvexSet& element(int index) const;

  /** Returns the number of convex sets defining the convex hull (including
   * empty sets). */
  int num_elements() const { return sets_.size(); }

  /** @note if called on an instance that was called with
  remove_empty_sets=false, this function will reconstruct the convex hull with
  remove_empty_sets=true. Therefore, it is recommended to call this function
  only once. */
  using ConvexSet::IsEmpty;

  /** @throws  Not implemented. */
  using ConvexSet::CalcVolume;

  /** A ConvexHull is bounded if and only if each constituent set is bounded.
  This class honors requests for parallelism only so far as its constituent sets
  do.
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
  ConvexSets participating_sets_{};
  bool empty_sets_removed_;
};

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
