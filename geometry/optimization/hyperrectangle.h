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
class HyperRectangle final : public ConvexSet {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(HyperRectangle)

  HyperRectangle();

  HyperRectangle(const Eigen::Ref<const Eigen::VectorXd>& lb,
                 const Eigen::Ref<const Eigen::VectorXd>& ub);

  /** Get the lower bound of the hyperrectangle. */
  const Eigen::VectorXd lb() const { return lb_; }

  /** Get the upper bound of the hyperrectangle. */
  const Eigen::VectorXd ub() const { return ub_; }

  /** Variant of UniformSample for hyperrectangles that uses uniform sampling in
   * each axis */
  Eigen::VectorXd UniformSample(RandomGenerator* generator) const;

  /** Get the center of the hyperrectangle. */
  Eigen::VectorXd Center() const;

  // Helper to convert this hyperrectangle to an HPolyhedron.
  HPolyhedron MakeHPolyhedron() const;

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
  std::unique_ptr<ConvexSet> DoClone() const;

  /** Non-virtual interface implementation for IsBounded().
  @pre ambient_dimension() >= 0 */
  bool DoIsBounded() const { return true; }

  /** Non-virtual interface implementation for IsEmpty(). The default
  implementation solves a feasibility optimization problem, but derived
  classes can override with a custom (more efficient) implementation.
  Zero-dimensional sets are considered to be nonempty by default. Sets which
  can be zero-dimensional and empty must handle this behavior in their
  derived implementation of DoIsEmpty. */
  bool DoIsEmpty() const { return false; }

  /** Non-virtual interface implementation for MaybeGetPoint(). The default
  implementation returns nullopt. Sets that can model a single point should
  override with a custom implementation.
  @pre ambient_dimension() >= 0 */
  std::optional<Eigen::VectorXd> DoMaybeGetPoint() const;

  /** Non-virtual interface implementation for MaybeGetFeasiblePoint(). The
  default implementation solves a feasibility optimization problem, but
  derived classes can override with a custom (more efficient) implementation. */
  std::optional<Eigen::VectorXd> DoMaybeGetFeasiblePoint() const;

  /** Non-virtual interface implementation for PointInSet().
  @pre x.size() == ambient_dimension()
  @pre ambient_dimension() >= 0 */
  bool DoPointInSet(const Eigen::Ref<const Eigen::VectorXd>& x,
                    double tol) const;

  /** Non-virtual interface implementation for AddPointInSetConstraints().
  @pre vars.size() == ambient_dimension()
  @pre ambient_dimension() > 0 */
  std::pair<VectorX<symbolic::Variable>,
            std::vector<solvers::Binding<solvers::Constraint>>>
  DoAddPointInSetConstraints(
      solvers::MathematicalProgram* prog,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& vars)
      const final;

  /** Non-virtual interface implementation for
  AddPointInNonnegativeScalingConstraints().
  @pre x.size() == ambient_dimension()
  @pre ambient_dimension() > 0 */
  std::vector<solvers::Binding<solvers::Constraint>>
  DoAddPointInNonnegativeScalingConstraints(
      solvers::MathematicalProgram* prog,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& x,
      const symbolic::Variable& t) const final;

  /** Non-virtual interface implementation for
  AddPointInNonnegativeScalingConstraints(). Subclasses must override to
  add the constraints needed to keep the point A * x + b in the
  non-negative scaling of the set. Note that subclasses do not need to add
  the constraint c * t + d ≥ 0 as it is already added.
  @pre ambient_dimension() > 0
  @pre A.rows() == ambient_dimension()
  @pre A.rows() == b.rows()
  @pre A.cols() == x.size()
  @pre c.rows() == t.size() */
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

  double DoVolume() const;

  void CheckInvariants();

  Eigen::VectorXd lb_;
  Eigen::VectorXd ub_;
};

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
