#pragma once

#include <memory>
#include <optional>
#include <set>
#include <utility>
#include <vector>

#include "drake/common/name_value.h"
#include "drake/geometry/optimization/convex_set.h"
#include "drake/geometry/optimization/hpolyhedron.h"

namespace drake {
namespace geometry {
namespace optimization {

/** Axis-aligned hyperrectangle in Rᵈ defined by its lower bounds and upper
 * bounds as {x| lb ≤ x ≤ ub} */
class Hyperrectangle final : public ConvexSet {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Hyperrectangle)

  Hyperrectangle();

  /** Constructs a hyperrectangle from its lower and upper bounds.
   @pre lb.size() == ub.size()
   @pre lb(i) <= ub(i) for all i */
  Hyperrectangle(const Eigen::Ref<const Eigen::VectorXd>& lb,
                 const Eigen::Ref<const Eigen::VectorXd>& ub);

  /** Get the lower bounds of the hyperrectangle. */
  const Eigen::VectorXd& lb() const { return lb_; }

  /** Get the upper bounds of the hyperrectangle. */
  const Eigen::VectorXd& ub() const { return ub_; }

  /** Draws a uniform sample from the set. */
  Eigen::VectorXd UniformSample(RandomGenerator* generator) const;

  /** Get the center of the hyperrectangle. */
  Eigen::VectorXd Center() const;

  /** Helper to convert this hyperrectangle to an HPolyhedron. */
  HPolyhedron MakeHPolyhedron() const;

  /** Returns the minimum axis-aligned bounding box of a convex set, for sets
  with finite volume. (std::nullopt otherwise). */
  static std::optional<Hyperrectangle> MaybeCalcAxisAlignedBoundingBox(
      const ConvexSet& set);

  /** Passes this object to an Archive.
    Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive* a) {
    ConvexSet::Serialize(a);
    a->Visit(MakeNameValue("lb", &lb_));
    a->Visit(MakeNameValue("ub", &ub_));
    CheckInvariants();
  }

 private:
  std::unique_ptr<ConvexSet> DoClone() const final;

  /** An Hyperrectangle can not empty as lb <= ub is already checked in the ctor
   */
  bool DoIsEmpty() const final { return false; }

  std::optional<Eigen::VectorXd> DoMaybeGetPoint() const final;

  std::optional<Eigen::VectorXd> DoMaybeGetFeasiblePoint() const final;

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

  std::pair<std::unique_ptr<Shape>, math::RigidTransformd> DoToShapeWithPose()
      const final;

  double DoCalcVolume() const final;

  void CheckInvariants();

  Eigen::VectorXd lb_;
  Eigen::VectorXd ub_;
};

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
