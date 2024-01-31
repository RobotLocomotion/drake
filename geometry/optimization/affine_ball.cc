#include "drake/geometry/optimization/affine_ball.h"

#include <vector>

#include "drake/geometry/optimization/affine_subspace.h"
#include "drake/geometry/optimization/hyperellipsoid.h"
#include "drake/geometry/optimization/vpolytope.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace geometry {
namespace optimization {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using solvers::Binding;
using solvers::Constraint;
using solvers::MathematicalProgram;
using solvers::VectorXDecisionVariable;
using std::sqrt;
using symbolic::Expression;
using symbolic::Variable;

AffineBall::AffineBall() : AffineBall(MatrixXd(0, 0), VectorXd(0)) {}

AffineBall::AffineBall(const Eigen::Ref<const MatrixXd>& B,
                       const Eigen::Ref<const VectorXd>& center)
    : ConvexSet(center.size(), true), B_(B), center_(center) {
  CheckInvariants();
}

namespace {
const Hyperellipsoid& CheckBounded(const Hyperellipsoid& ellipsoid) {
  DRAKE_THROW_UNLESS(ellipsoid.IsBounded());
  return ellipsoid;
}
}  // namespace

AffineBall::AffineBall(const Hyperellipsoid& ellipsoid)
    : AffineBall(CheckBounded(ellipsoid).A().inverse(), ellipsoid.center()) {}

AffineBall::~AffineBall() = default;

namespace {

double volume_of_unit_sphere(int dim) {
  DRAKE_DEMAND(dim >= 1);
  // Formula from https://en.wikipedia.org/wiki/Volume_of_an_n-ball .
  // Note: special case n≤3 only because they are common and simple.
  switch (dim) {
    case 1:
      return 2.0;
    case 2:
      return M_PI;
    case 3:
      return 4.0 * M_PI / 3.0;
    default:
      return std::pow(M_PI, dim / 2.0) / std::tgamma(dim / 2.0 + 1);
  }
}

}  // namespace

AffineBall AffineBall::MinimumVolumeCircumscribedEllipsoid(
    const Eigen::Ref<const Eigen::MatrixXd>& points, double rank_tol) {
  DRAKE_THROW_UNLESS(!points.hasNaN());
  DRAKE_THROW_UNLESS(points.allFinite());
  DRAKE_THROW_UNLESS(points.rows() >= 1);
  DRAKE_THROW_UNLESS(points.cols() >= 1);
  const int dim = points.rows();

  AffineSubspace ah(VPolytope(points), rank_tol);
  const int rank = ah.AffineDimension();
  Eigen::MatrixXd points_local = ah.ToLocalCoordinates(points);

  // Compute circumscribed ellipsoid in local coordinates
  const Hyperellipsoid hyperellipsoid_local =
      Hyperellipsoid::MinimumVolumeCircumscribedEllipsoid(points_local,
                                                          rank_tol);
  const AffineBall affineball_local(hyperellipsoid_local);

  // Lift the ellipsoid {Bu+center| |u|₂ ≤ 1} to the original coordinate system
  // i.e. the set {ABu + (Ac+t) | |u|₂ ≤ 1}
  Eigen::MatrixXd A_full = Eigen::MatrixXd::Zero(dim, dim);
  A_full.leftCols(rank) = ah.basis() * affineball_local.B();
  Eigen::VectorXd center_full =
      ah.ToGlobalCoordinates(affineball_local.center());

  return AffineBall(A_full, center_full);
}

double AffineBall::DoCalcVolume() const {
  return volume_of_unit_sphere(ambient_dimension()) *
         std::abs(B_.determinant());
}

AffineBall AffineBall::MakeAxisAligned(
    const Eigen::Ref<const VectorXd>& radius,
    const Eigen::Ref<const VectorXd>& center) {
  DRAKE_THROW_UNLESS(radius.size() == center.size());
  DRAKE_THROW_UNLESS((radius.array() >= 0).all());
  return AffineBall(MatrixXd(radius.asDiagonal()), center);
}

AffineBall AffineBall::MakeHypersphere(
    double radius, const Eigen::Ref<const VectorXd>& center) {
  DRAKE_THROW_UNLESS(radius >= 0);
  const int dim = center.size();
  return AffineBall(MatrixXd::Identity(dim, dim) * radius, center);
}

AffineBall AffineBall::MakeUnitBall(int dim) {
  DRAKE_THROW_UNLESS(dim >= 0);
  return AffineBall(MatrixXd::Identity(dim, dim), VectorXd::Zero(dim));
}

AffineBall AffineBall::MakeAffineBallFromLineSegment(
    const Eigen::Ref<const Eigen::VectorXd>& x_1,
    const Eigen::Ref<const Eigen::VectorXd>& x_2, const double epsilon) {
  DRAKE_THROW_UNLESS(x_1.size() == x_2.size());
  DRAKE_THROW_UNLESS(epsilon >= 0.0);
  const double length = (x_1 - x_2).norm();
  const double kTolerance = 1e-9;
  if (length < kTolerance) {
    throw std::runtime_error(fmt::format(
        "AffineBall:MakeAffineBallFromLineSegment: x_1 and x_2 are the same "
        "point (distance: {} < tolerance: {}).",
        length, kTolerance));
  }
  const int dim = x_1.size();
  const Eigen::VectorXd center = (x_1 + x_2) / 2.0;
  const Eigen::VectorXd r_0 = (x_1 - x_2) / length;
  // Construct r_1, ..., r_{dim-1} such that r_0, ..., r_{dim-1} are orthonormal
  // and r_0 is parallel to x_1 - x_2.
  // This is similar to the Gram-Schmidt process
  // (see https://en.wikipedia.org/wiki/Gram%E2%80%93Schmidt_process)
  // with the small modification that we construct an orthonormal basis
  // from u_0, e_0, e_1, ..., e_{dim-1}, where e_i is the i-th standard basis
  // vector, and know that the result will have one less vector than the input.
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(dim, dim);
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(dim, dim);
  R.col(0) = r_0;
  int k = 0;
  int i = 1;
  while (k < dim) {
    Eigen::VectorXd v = I.col(k);
    for (int j = 0; j < i; ++j) {
      v -= R.col(j)(k) * R.col(j);
    }
    if (v.norm() > 1e-9) {
      R.col(i) = v.normalized();
      ++i;
    }
    ++k;
  }
  DRAKE_DEMAND(i == dim);
  Eigen::MatrixXd scale_matrix = epsilon * Eigen::MatrixXd::Identity(dim, dim);
  scale_matrix(0, 0) = length / 2.0;
  return AffineBall(R * scale_matrix, center);
}

std::unique_ptr<ConvexSet> AffineBall::DoClone() const {
  return std::make_unique<AffineBall>(*this);
}

std::optional<VectorXd> AffineBall::DoMaybeGetPoint() const {
  if (B_.isZero(0)) {
    return center_;
  }
  return std::nullopt;
}

bool AffineBall::DoPointInSet(const Eigen::Ref<const Eigen::VectorXd>& x,
                              double tol) const {
  // Check that x is in the column space of B_, then find a y such that By+d =
  // x, and see if y is in the unit ball.
  const auto B_QR = Eigen::ColPivHouseholderQR<MatrixXd>(B_);
  VectorXd y = B_QR.solve(x - center_);
  if ((B_ * y).isApprox(x - center_, tol)) {
    return y.dot(y) <= 1 + tol;
  }
  return false;
}

std::pair<std::unique_ptr<Shape>, math::RigidTransformd>
AffineBall::DoToShapeWithPose() const {
  throw std::runtime_error(
      "ToShapeWithPose is not yet supported by AffineBall.");
}

std::pair<VectorX<Variable>, std::vector<Binding<Constraint>>>
AffineBall::DoAddPointInSetConstraints(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorXDecisionVariable>& x) const {
  std::vector<Binding<Constraint>> new_constraints;
  const int n = ambient_dimension();
  VectorXDecisionVariable y = prog->NewContinuousVariables(n, "y");
  // ||y||^2 <= 1, represented as 0.5yᵀIy + 0ᵀy + (-0.5) <= 0.
  new_constraints.push_back(prog->AddQuadraticAsRotatedLorentzConeConstraint(
      MatrixXd::Identity(n, n), VectorXd::Zero(n), -0.5, y));
  // x = By + center_, represented as [I, -B_] * [x; y] = center_
  MatrixXd equality_constraint_A(n, 2 * n);
  equality_constraint_A.leftCols(n) = MatrixXd::Identity(n, n);
  equality_constraint_A.rightCols(n) = -B_;
  new_constraints.push_back(prog->AddLinearEqualityConstraint(
      equality_constraint_A, center_, {x, y}));
  return {std::move(y), std::move(new_constraints)};
}

std::vector<Binding<Constraint>>
AffineBall::DoAddPointInNonnegativeScalingConstraints(
    MathematicalProgram*, const Eigen::Ref<const VectorXDecisionVariable>&,
    const Variable&) const {
  throw std::runtime_error(
      "AffineBall::DoAddPointInNonnegativeScalingConstraints() is not "
      "implemented yet.");
}

std::vector<Binding<Constraint>>
AffineBall::DoAddPointInNonnegativeScalingConstraints(
    MathematicalProgram*, const Eigen::Ref<const MatrixXd>&,
    const Eigen::Ref<const VectorXd>&, const Eigen::Ref<const VectorXd>&,
    double, const Eigen::Ref<const VectorXDecisionVariable>&,
    const Eigen::Ref<const VectorXDecisionVariable>&) const {
  throw std::runtime_error(
      "AffineBall::DoAddPointInNonnegativeScalingConstraints() is not "
      "implemented yet.");
}

void AffineBall::CheckInvariants() const {
  DRAKE_THROW_UNLESS(this->ambient_dimension() == B_.cols());
  DRAKE_THROW_UNLESS(B_.cols() == B_.rows());
  DRAKE_THROW_UNLESS(B_.cols() == center_.size());
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
