#pragma once

#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "drake/common/name_value.h"
#include "drake/geometry/optimization/affine_ball.h"
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
invertible, but the quadratic form can also represent unbounded sets. The affine
scaling of the unit ball representation is available via the AffineBall class.

Note: the name Hyperellipsoid was taken here to avoid conflicting with
geometry::Ellipsoid and to distinguish that this class supports N dimensions.

A hyperellipsoid can never be empty -- it always contains its center. This
includes the zero-dimensional case.

@ingroup geometry_optimization */
class Hyperellipsoid final : public ConvexSet {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Hyperellipsoid);

  /** Constructs a default (zero-dimensional, nonempty) set. */
  Hyperellipsoid();

  /** Constructs the ellipsoid.
  @pre A.cols() == center.size(). */
  Hyperellipsoid(const Eigen::Ref<const Eigen::MatrixXd>& A,
                 const Eigen::Ref<const Eigen::VectorXd>& center);

  /** Constructs a Hyperellipsoid from a SceneGraph geometry and pose in the
  `reference_frame` frame, obtained via the QueryObject. If `reference_frame`
  frame is std::nullopt, then it will be expressed in the world frame.
  @throws std::exception if geometry_id does not represent a shape that can be
  described as an Hyperellipsoid. */
  Hyperellipsoid(const QueryObject<double>& query_object,
                 GeometryId geometry_id,
                 std::optional<FrameId> reference_frame = std::nullopt);

  /** Constructs a Hyperellipsoid from an AffineBall.
  @pre ellipsoid.B() is invertible. */
  explicit Hyperellipsoid(const AffineBall& ellipsoid);

  ~Hyperellipsoid() final;

  /** Returns the quadratic form matrix A. */
  const Eigen::MatrixXd& A() const { return A_; }

  /** Returns the center of the ellipsoid. */
  const Eigen::VectorXd& center() const { return center_; }

  /** Computes the smallest uniform scaling of this ellipsoid for which it still
  intersects @p other. √ minₓ (x-center)ᵀAᵀA(x-center) s.t. x ∈ other.  Note
  that if center ∈ other, then we expect scaling = 0 and x = center (up to
  precision).
  @pre `other` must have the same ambient_dimension as this.
  @returns the minimal scaling and the witness point, x, on other.
  @throws std::exception if `other` is empty.
  @throws std::exception if ambient_dimension() == 0 */
  std::pair<double, Eigen::VectorXd> MinimumUniformScalingToTouch(
      const ConvexSet& other) const;

  /** Results a new Hyperellipsoid that is a scaled version of `this` about the
  center. Any point on the boundary of the ellipsoid, x, is now translated to a
  new point, x*, such that ||x* - center|| = ||x - center|| *
  pow(scale, 1.0/ambient_dimension()). The volume of the resulting shape is
  scaled up by 'scale'.
  @pre `scale` > 0.
  */
  [[nodiscard]] Hyperellipsoid Scale(double scale) const;

  /** Constructs a Ellipsoid shape description of this set.  Note that the
  choice of ellipsoid is not unique.  This method chooses to order the Ellipsoid
  parameters a ≥ b ≥ c.

  @throws std::exception if ambient_dimension() != 3
  @throws std::exception if A is not invertible (has any eigenvalue less than
  sqrt(1e-12)). This tolerance is not carefully tuned (yet).  We use Eigen's
  SelfAdjointEigenSolver to take the eigenvalues of AᵀA; this solver is listed
  as having accuracy "Good":
  https://eigen.tuxfamily.org/dox/group__TopicLinearAlgebraDecompositions.html
  but how does that translate into a numerical precision? */
  using ConvexSet::ToShapeWithPose;

  /** Constructs the an axis-aligned Hyperellipsoid with the implicit form
  (x₀-c₀)²/r₀² + (x₁-c₁)²/r₁² + ... + (x_N - c_N)²/r_N² ≤ 1, where c is
  shorthand for `center` and r is shorthand for `radius`. */
  static Hyperellipsoid MakeAxisAligned(
      const Eigen::Ref<const Eigen::VectorXd>& radius,
      const Eigen::Ref<const Eigen::VectorXd>& center);

  /** Constructs a hypersphere with `radius` and `center`. */
  static Hyperellipsoid MakeHypersphere(
      double radius, const Eigen::Ref<const Eigen::VectorXd>& center);

  /** Constructs the L₂-norm unit ball in `dim` dimensions, {x | |x|₂ <= 1 }. */
  static Hyperellipsoid MakeUnitBall(int dim);

  /** Constructs the minimum-volume ellipsoid which contains all of the
  `points`. This is commonly referred to as the outer Löwner-John ellipsoid.

  @param points is a d-by-n matrix, where d is the ambient dimension and each
  column represents one point.
  @param rank_tol the singular values of the data matrix will be considered
  non-zero if they are strictly greater than `rank_tol` * `max_singular_value`.
  The default is 1e-6 to be compatible with common solver tolerances. This is
  used to detect if the data lies on a lower-dimensional affine space than the
  ambient dimension of the ellipsoid. If this is the case, then use
  AffineBall::MinimumVolumeCircumscribedEllipsoid instead.
  @throws std::exception if the MathematicalProgram fails to solve. If this
  were to happen (due to numerical issues), then increasing `rank_tol` should
  provide a mitigation.
  @throw std::exception if points includes NaNs or infinite values.
  @throw std::exception if the numerical data rank of points is less than d.

  */
  static Hyperellipsoid MinimumVolumeCircumscribedEllipsoid(
      const Eigen::Ref<const Eigen::MatrixXd>& points, double rank_tol = 1e-6);

  /** Passes this object to an Archive.
  Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive* a) {
    ConvexSet::Serialize(a);
    a->Visit(MakeNameValue("A", &A_));
    a->Visit(MakeNameValue("center", &center_));
    CheckInvariants();
  }

  // TODO(SeanCurtis-TRI) Deprecate this function.
  /** Computes the volume for the hyperellipsoid set.*/
  double Volume() const { return CalcVolume(); }

  /** A Hyperellipsoid is bounded if the kernel of A is rank-zero.
  @param parallelism Ignored -- no parallelization is used.
  @note See @ref ConvexSet::IsBounded "parent class's documentation" for more
  details. */
  using ConvexSet::IsBounded;

 private:
  std::unique_ptr<ConvexSet> DoClone() const final;

  std::optional<bool> DoIsBoundedShortcut() const final;

  bool DoIsEmpty() const final;

  // N.B. No need to override DoMaybeGetPoint here. This class cannot represent
  // a single point.

  /** Returns the center, which is always feasible. */
  std::optional<Eigen::VectorXd> DoMaybeGetFeasiblePoint() const final;

  std::optional<bool> DoPointInSetShortcut(
      const Eigen::Ref<const Eigen::VectorXd>& x, double tol) const final;

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
      const Eigen::Ref<const Eigen::VectorXd>& b,
      const Eigen::Ref<const Eigen::VectorXd>& c, double d,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& x,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& t) const final;

  std::pair<std::unique_ptr<Shape>, math::RigidTransformd> DoToShapeWithPose()
      const final;

  std::unique_ptr<ConvexSet> DoAffineHullShortcut(
      std::optional<double> tol) const final;

  double DoCalcVolume() const final;

  void CheckInvariants() const;

  Eigen::MatrixXd A_{};
  Eigen::VectorXd center_{};
};

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
