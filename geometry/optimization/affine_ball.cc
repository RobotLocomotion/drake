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

std::optional<bool> AffineBall::DoIsBoundedShortcut() const {
  return true;
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

std::optional<bool> AffineBall::DoPointInSetShortcut(
    const Eigen::Ref<const Eigen::VectorXd>& x, double tol) const {
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

std::unique_ptr<ConvexSet> AffineBall::DoAffineHullShortcut(
    std::optional<double> tol) const {
  Eigen::FullPivHouseholderQR<MatrixXd> qr(B_);
  if (tol) {
    qr.setThreshold(tol.value());
  }
  MatrixXd basis =
      qr.matrixQ() * MatrixXd::Identity(ambient_dimension(), qr.rank());
  return std::make_unique<AffineSubspace>(std::move(basis), center_);
}

std::pair<VectorX<Variable>, std::vector<Binding<Constraint>>>
AffineBall::DoAddPointInSetConstraints(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorXDecisionVariable>& x) const {
  std::vector<Binding<Constraint>> new_constraints;
  const int n = ambient_dimension();
  VectorXDecisionVariable y = prog->NewContinuousVariables(n, "y");
  // ||y||² ≤ 1, represented as 0.5yᵀIy + 0ᵀy + (-0.5) ≤ 0.
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
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorXDecisionVariable>& x,
    const Variable& t) const {
  std::vector<Binding<Constraint>> new_constraints;
  const int n = ambient_dimension();
  // The constraint is x∈tS, which can be written x=t(By+c), or equivalently,
  // x-tc=By and ||y||² ≤ t².
  VectorXDecisionVariable y = prog->NewContinuousVariables(n, "y");

  // The first constraint is the linear equality constraint
  // [-I, B, center_] * [x; y; t] = 0.
  MatrixXd constraint_A(n, n + n + 1);
  constraint_A.leftCols(n) = -MatrixXd::Identity(n, n);
  constraint_A.block(0, n, n, n) = B_;
  constraint_A.rightCols(1) = center_;
  new_constraints.push_back(prog->AddLinearEqualityConstraint(
      constraint_A, VectorXd::Zero(n), {x, y, Vector1<Variable>(t)}));

  // The second constraint is that [t, y] be in the Lorentz cone.
  new_constraints.push_back(prog->AddLorentzConeConstraint(
      MatrixXd::Identity(n + 1, n + 1), VectorXd::Zero(n + 1),
      {Vector1<Variable>(t), y}));

  return new_constraints;
}

std::vector<Binding<Constraint>>
AffineBall::DoAddPointInNonnegativeScalingConstraints(
    MathematicalProgram* prog, const Eigen::Ref<const MatrixXd>& A,
    const Eigen::Ref<const VectorXd>& b, const Eigen::Ref<const VectorXd>& c,
    double d, const Eigen::Ref<const VectorXDecisionVariable>& x,
    const Eigen::Ref<const VectorXDecisionVariable>& t) const {
  std::vector<Binding<Constraint>> new_constraints;
  const int n = ambient_dimension();
  // The constraint is Ax+b∈(c't+d)S, which can be written Ax+b=(c't+d)(By+C),
  // where we use C to refer to the center of the AffineBall, which simplifies
  // to -Ax+By+(Cc')t=b-Cd and ||y||² ≤ (c't+d)².
  VectorXDecisionVariable y = prog->NewContinuousVariables(n, "y");

  // The first constraint is the linear equality constraint
  // [-A, B, Cc'] * [x; y; t] = b - Cd.
  const int m = x.size();
  const int k = c.size();
  MatrixXd constraint_A(n, m + n + k);
  constraint_A.leftCols(m) = -A;
  constraint_A.block(0, m, n, n) = B_;
  constraint_A.rightCols(k) = center_ * c.transpose();
  new_constraints.push_back(
      prog->AddLinearEqualityConstraint(constraint_A, b, {x, y, t}));

  // The second constraint is that ||y||² ≤ (c't+d)², which can be written as
  // [c', 0; 0, I]*[t; y]+[d; 0] is in the Lorentz cone
  MatrixXd A_lorentz = MatrixXd::Zero(1 + n, k + n);
  A_lorentz.row(0).head(k) = c.transpose();
  A_lorentz.block(1, k, n, n) = MatrixXd::Identity(n, n);
  VectorXd b_lorentz = VectorXd::Zero(1 + n);
  b_lorentz[0] = d;
  new_constraints.push_back(
      prog->AddLorentzConeConstraint(A_lorentz, b_lorentz, {t, y}));

  return new_constraints;
}

void AffineBall::CheckInvariants() const {
  DRAKE_THROW_UNLESS(this->ambient_dimension() == B_.cols());
  DRAKE_THROW_UNLESS(B_.cols() == B_.rows());
  DRAKE_THROW_UNLESS(B_.cols() == center_.size());
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
