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
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(AffineBall)

  /** Constructs a default (zero-dimensional, nonempty) set. */
  AffineBall();

  /** Constructs the ellipsoid. Note that the IsPositiveDefinite check enforces
  that B is symmetric, and only enforces positive semi-definiteness with the
  default tolerance of 0.
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

  /** Returns the volume of the AffineBall (in Euclidean space). */
  double Volume() const;

  /** Constructs the an axis-aligned AffineBall with the implicit form
  (x₀-c₀)²/r₀² + (x₁-c₁)²/r₁² + ... + (x_N - c_N)²/r_N² ≤ 1, where c is
  shorthand for `center` and r is shorthand for `radius`. */
  static AffineBall MakeAxisAligned(
      const Eigen::Ref<const Eigen::VectorXd>& radius,
      const Eigen::Ref<const Eigen::VectorXd>& center);

  /** Constructs a hypersphere with `radius` and `center`. */
  static AffineBall MakeHypersphere(
      double radius, const Eigen::Ref<const Eigen::VectorXd>& center);

  /** Constructs the L₂-norm unit ball in `dim` dimensions, {x | |x|₂ <= 1 }. */
  static AffineBall MakeUnitBall(int dim);

  /** Passes this object to an Archive.
  Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive* a) {
    ConvexSet::Serialize(a);
    a->Visit(MakeNameValue("B", &B_));
    a->Visit(MakeNameValue("center", &center_));
    CheckInvariants();
  }

 private:
  std::unique_ptr<ConvexSet> DoClone() const final;

  /* AffineBall can only represent bounded sets. */
  std::optional<bool> DoIsBoundedShortcut() const final { return true; };

  /* AffineBall can only represent nonempty sets. */
  bool DoIsEmpty() const final { return false; };

  /* DoMaybeGetPoint only succeeds if C is a matrix of all zeros. */
  std::optional<Eigen::VectorXd> DoMaybeGetPoint() const final;

  /* Returns the center, which is always feasible. */
  std::optional<Eigen::VectorXd> DoMaybeGetFeasiblePoint() const final {
    return center_;
  };

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
      const Eigen::Ref<const Eigen::VectorXd>& b,
      const Eigen::Ref<const Eigen::VectorXd>& c, double d,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& x,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& t) const final;

  std::pair<std::unique_ptr<Shape>, math::RigidTransformd> DoToShapeWithPose()
      const final;

  void CheckInvariants() const;

  Eigen::MatrixXd B_{};
  Eigen::VectorXd center_{};
};

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
