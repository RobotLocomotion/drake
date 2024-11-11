#pragma once

#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "drake/geometry/optimization/convex_set.h"

namespace drake {
namespace geometry {
namespace optimization {

/** A convex set that contains exactly one element.  Also known as a
singleton or unit set.

This set is always nonempty, even in the zero-dimensional case.

@ingroup geometry_optimization */
class Point final : public ConvexSet {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Point);

  /** Constructs a default (zero-dimensional, nonempty) set. */
  Point();

  /** Constructs a Point. */
  explicit Point(const Eigen::Ref<const Eigen::VectorXd>& x);

  /** Constructs a Point from a SceneGraph geometry and pose in the
  `reference_frame` frame, obtained via the QueryObject. If `reference_frame`
  frame is std::nullopt, then it will be expressed in the world frame.
  @throws std::exception if geometry_id does not correspond to a Sphere or if
  the Sphere has radius greater than `maximum_allowable_radius`. */
  Point(const QueryObject<double>& query_object, GeometryId geometry_id,
        std::optional<FrameId> reference_frame = std::nullopt,
        double maximum_allowable_radius = 0.0);

  ~Point() final;

  /** Returns true if the point is within `tol` of x(), under the L∞-norm. */
  using ConvexSet::PointInSet;

  /** Retrieves the point. */
  const Eigen::VectorXd& x() const { return x_; }

  /** Changes the element `x` describing the set.
  @pre x must be of size ambient_dimension(). */
  void set_x(const Eigen::Ref<const Eigen::VectorXd>& x);

  /** Every Point is bounded by construction.
  @param parallelism Ignored -- no parallelization is used.
  @note See @ref ConvexSet::IsBounded "parent class's documentation" for more
  details. */
  using ConvexSet::IsBounded;

 private:
  std::unique_ptr<ConvexSet> DoClone() const final;

  std::optional<bool> DoIsBoundedShortcut() const final;

  /** A Point is always nonempty, even in the zero-dimensional case. */
  bool DoIsEmpty() const final;

  std::optional<Eigen::VectorXd> DoMaybeGetPoint() const final;

  std::optional<bool> DoPointInSetShortcut(
      const Eigen::Ref<const Eigen::VectorXd>& x, double tol) const final;

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
      const Eigen::Ref<const Eigen::MatrixXd>& A,
      const Eigen::Ref<const Eigen::VectorXd>& b,
      const Eigen::Ref<const Eigen::VectorXd>& c, double d,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& x,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& t) const final;

  std::pair<std::unique_ptr<Shape>, math::RigidTransformd> DoToShapeWithPose()
      const final;

  double DoCalcVolume() const final { return 0.0; }

  std::vector<std::optional<double>> DoProjectionShortcut(
      const Eigen::Ref<const Eigen::MatrixXd>& points,
      EigenPtr<Eigen::MatrixXd> projected_points) const final;

  Eigen::VectorXd x_;
};

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
