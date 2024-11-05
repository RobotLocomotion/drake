#pragma once

#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "drake/geometry/optimization/convex_set.h"

namespace drake {
namespace geometry {
namespace optimization {
// Forward-declare Hyperellipsoid.
class Hyperellipsoid;

/** Implements an ellipsoidal convex set represented as an affine scaling of the
unit ball {Bu + center | |u|₂ ≤ 1}. B must be a square matrix.

Compare this with an alternative parametrization of the ellipsoid: {x |
(x-center)ᵀAᵀA(x-center) ≤ 1}, which utilizes a quadratic form. The two
representations are related by B = A⁻¹ if A and B are invertible.

The quadratic form parametrization is implemented in Hyperellipsoid. It can
represent unbounded sets, but not sets along a lower-dimensional affine
subspace. The AffineBall parametrization can represent sets along a
lower-dimensional affine subspace, but not unbounded sets.

An AffineBall can never be empty -- it always contains its center. This includes
the zero-dimensional case.

@ingroup geometry_optimization */
class AffineBall final : public ConvexSet {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(AffineBall);

  /** Constructs a default (zero-dimensional, nonempty) set. */
  AffineBall();

  /** Constructs the ellipsoid from a transformation matrix B and translation
  center. B describes the linear transformation that is applied to the unit ball
  in order to produce the ellipsoid, and center describes the translation of the
  center of the ellipsoid from the origin.
  @pre B.rows() == B.cols().
  @pre B.cols() == center.size(). */
  AffineBall(const Eigen::Ref<const Eigen::MatrixXd>& B,
             const Eigen::Ref<const Eigen::VectorXd>& center);

  /** Constructs an AffineBall from a Hyperellipsoid.
  @pre ellipsoid.IsBounded(). */
  explicit AffineBall(const Hyperellipsoid& ellipsoid);

  ~AffineBall() final;

  /** Returns the affine transformation matrix B. */
  const Eigen::MatrixXd& B() const { return B_; }

  /** Returns the center of the ellipsoid. */
  const Eigen::VectorXd& center() const { return center_; }

  /** Constructs an axis-aligned AffineBall with the implicit form
  (x₀-c₀)²/r₀² + (x₁-c₁)²/r₁² + ... + (x_N - c_N)²/r_N² ≤ 1, where c is
  shorthand for `center` and r is shorthand for `radius`.
  @pre radius.size() == center.size().
  @pre radius[i] >= 0, for all i. */
  static AffineBall MakeAxisAligned(
      const Eigen::Ref<const Eigen::VectorXd>& radius,
      const Eigen::Ref<const Eigen::VectorXd>& center);

  /** Constructs a hypersphere with `radius` and `center`.
  @pre radius >= 0. */
  static AffineBall MakeHypersphere(
      double radius, const Eigen::Ref<const Eigen::VectorXd>& center);

  /** Constructs the L₂-norm unit ball in `dim` dimensions, {x | |x|₂ <= 1 }.
  @pre dim >= 0. */
  static AffineBall MakeUnitBall(int dim);

  /** Constructs the minimum-volume ellipsoid which contains all of the given
  points. This is commonly referred to as the outer Löwner-John ellipsoid.

  If all of the points lie along a proper affine subspace, this method
  instead computes the minimum-k-volume ellipsoid, where k is the affine
  dimension of the convex hull of `points`.

  @param points is a d-by-n matrix, where d is the ambient dimension and each
  column represents one point.
  @param rank_tol the tolerance used to detect if the data lies in an affine
  subspace. The affine ball is computed in the subspace spanned by the left
  singular vectors of the data matrix whose associated singular values are
  larger than `rank_tol` * `max_singular_value`. The default is 1e-6 to be
  compatible with common solver tolerances.
  @throws std::exception if the MathematicalProgram fails to solve. This can
  happen due to numerical issues caused by `rank_tol` being set too low.
  @throws std::exception if points includes NaNs or infinite values.
  @pre points.rows() >= 1.
  @pre points.cols() >= 1. */
  static AffineBall MinimumVolumeCircumscribedEllipsoid(
      const Eigen::Ref<const Eigen::MatrixXd>& points, double rank_tol = 1e-6);

  /** Constructs an affine ball such that its main diameter is the line segment
  from @p x_1 to @p x_2, and the length of all other diameters are 2 * @p
  epsilon.
  @pre x_1.size() == x_2.size().
  @pre epsilon >= 0.
  @throws std::runtime_error if ‖x_1 - x_2‖₂ is less than 1e-9. */
  static AffineBall MakeAffineBallFromLineSegment(
      const Eigen::Ref<const Eigen::VectorXd>& x_1,
      const Eigen::Ref<const Eigen::VectorXd>& x_2,
      const double epsilon = 1e-3);

  /** Passes this object to an Archive.
  Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive* a) {
    ConvexSet::Serialize(a);
    a->Visit(MakeNameValue("B", &B_));
    a->Visit(MakeNameValue("center", &center_));
    CheckInvariants();
  }

  /** Every AffineBall is bounded by construction.
  @param parallelism Ignored -- no parallelization is used.
  @note See @ref ConvexSet::IsBounded "parent class's documentation" for more
  details. */
  using ConvexSet::IsBounded;

 private:
  std::unique_ptr<ConvexSet> DoClone() const final;

  /* AffineBall can only represent bounded sets. */
  std::optional<bool> DoIsBoundedShortcut() const final;

  /* AffineBall can only represent nonempty sets. */
  bool DoIsEmpty() const final { return false; };

  /* DoMaybeGetPoint only succeeds if B is a matrix of all zeros. */
  std::optional<Eigen::VectorXd> DoMaybeGetPoint() const final;

  /* Returns the center, which is always feasible. */
  std::optional<Eigen::VectorXd> DoMaybeGetFeasiblePoint() const final {
    return center_;
  };

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

  void CheckInvariants() const;

  double DoCalcVolume() const final;

  Eigen::MatrixXd B_{};
  Eigen::VectorXd center_{};
};

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
