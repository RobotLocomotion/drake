#pragma once

#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "drake/geometry/optimization/convex_set.h"

namespace drake {
namespace geometry {
namespace optimization {

/** Implements an ellipsoidal convex set represented by the quadratic form `{x
| (x-center)ᵀAᵀA(x-center) ≤ 1}`.  Note that `A` need not be square; we require
only that the matrix AᵀA is positive semi-definite.

Compare this with an alternative (very useful) parameterization of the
ellipsoid: `{Bu + center | |u|₂ ≤ 1}`, which is an affine scaling of the unit
ball.  This is related to the quadratic form by `B = A⁻¹`, when `A` is
invertible, but the quadratic form can also represent unbounded sets.

Note: the name HyperEllipsoid was taken here to avoid conflicting with
geometry::Ellipsoid and to distinguish that this class supports N dimensions.

@ingroup geometry_optimization
*/
class HyperEllipsoid final : public ConvexSet {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(HyperEllipsoid)

  /** Constructs the ellipsoid.
  @pre A.cols() == center.size(). */
  HyperEllipsoid(const Eigen::Ref<const Eigen::MatrixXd>& A,
                 const Eigen::Ref<const Eigen::VectorXd>& center);

  /** Constructs a HyperEllipsoid from a SceneGraph geometry and pose in
  the @p expressed_in frame, obtained via the QueryObject. If @p expressed_in
  frame is std::nullopt, then it will be expressed in the world frame.

  @throws std::exception if geometry_id does not represent a shape that can be
  described as an HyperEllipsoid. */
  HyperEllipsoid(const QueryObject<double>& query_object,
                 GeometryId geometry_id,
                 std::optional<FrameId> expressed_in = std::nullopt);

  virtual ~HyperEllipsoid() {}

  /** Returns the quadratic form matrix A. */
  const Eigen::MatrixXd& A() const { return A_; }

  /** Returns the center of the ellipsoid. */
  const Eigen::VectorXd& center() const { return center_; }

  /** Constructs a Ellipsoid shape description of this set.  Note that the
  choice of ellipsoid is not unique.  This method chooses to order the
  Ellipsoid parameters a ≥ b ≥ c.

  @throws std::exception if ambient_dimension() != 3
  @throws std::exception if A is not invertible (has any eigenvalue less than
  sqrt(1e-12)). This tolerance is not carefully tuned (yet).  We use Eigen's
  SelfAdjointEigenSolver to take the eigenvalues of AᵀA; this solver is listed
  as having accuracy "Good":
  https://eigen.tuxfamily.org/dox/group__TopicLinearAlgebraDecompositions.html
  but how does that translate into a numerical precision? */
  using ConvexSet::ToShapeWithPose;

  /** Constructs the L₂-norm unit ball in @p dim dimensions, {x | |x|₂ <= 1 }.
   */
  static HyperEllipsoid MakeUnitBall(int dim);

 private:
  bool DoPointInSet(const Eigen::Ref<const Eigen::VectorXd>& x,
                    double tol) const final;

  void DoAddPointInSetConstraint(
      solvers::MathematicalProgram* prog,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& vars)
      const final;

  std::vector<solvers::Binding<solvers::Constraint>>
  DoAddPointInNonnegativeScalingConstraints(
      solvers::MathematicalProgram* prog,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& x,
      const symbolic::Variable& t) const final;

  std::pair<std::unique_ptr<Shape>, math::RigidTransformd> DoToShapeWithPose()
      const final;

  // Implement support shapes for the ShapeReifier interface.
  using ShapeReifier::ImplementGeometry;
  void ImplementGeometry(const Sphere& sphere, void* data) final;
  void ImplementGeometry(const Ellipsoid& ellipsoid, void* data) final;

  Eigen::MatrixXd A_{};
  Eigen::VectorXd center_{};
};

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
