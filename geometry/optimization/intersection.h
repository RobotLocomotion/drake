#pragma once

#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "drake/geometry/optimization/convex_set.h"

namespace drake {
namespace geometry {
namespace optimization {

/** A convex set that represents the intersection of multiple sets:
S = X₁ ∩ X₂ ∩ ... ∩ Xₙ =
    {x | x ∈ X₁, x ∈ X₂, ..., x ∈ Xₙ}

Special behavior for IsEmpty: The intersection of zero sets (i.e. when we
have sets_.size() == 0) is always nonempty. This includes the zero-dimensional
case, which we treat as being {0}, the unique zero-dimensional vector space.

@ingroup geometry_optimization */
class Intersection final : public ConvexSet {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Intersection);

  /** Constructs a default (zero-dimensional, nonempty) set. */
  Intersection();

  /** Constructs the intersection from a vector of convex sets. */
  explicit Intersection(const ConvexSets& sets);

  /** Constructs the intersection from a pair of convex sets. */
  Intersection(const ConvexSet& setA, const ConvexSet& setB);

  ~Intersection() final;

  /** The number of elements (or sets) used in the intersection. */
  int num_elements() const { return sets_.size(); }

  /** Returns a reference to the ConvexSet defining the `index` element in the
  intersection. */
  const ConvexSet& element(int i) const;

  /** @throws  Not implemented. */
  using ConvexSet::CalcVolume;

  /** An Intersection is bounded if all its constituent sets are bounded. If any
  are unbounded, the generic method for checking boundedness is used. This class
  honors requests for parallelism only so far as its constituent sets do.
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
      const Eigen::Ref<const Eigen::MatrixXd>& A,
      const Eigen::Ref<const Eigen::VectorXd>& b,
      const Eigen::Ref<const Eigen::VectorXd>& c, double d,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& x,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& t) const final;

  // TODO(mpetersen94): Implement DoToShapeWithPose.
  std::pair<std::unique_ptr<Shape>, math::RigidTransformd> DoToShapeWithPose()
      const final;

  ConvexSets sets_{};  // Not marked const to support move semantics.
};

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
