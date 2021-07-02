#include "drake/geometry/optimization/hyperellipsoid.h"

#include <memory>

#include <Eigen/Eigenvalues>

#include "drake/math/matrix_util.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace geometry {
namespace optimization {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using math::RigidTransformd;
using math::RotationMatrixd;
using solvers::Binding;
using solvers::Constraint;
using solvers::MathematicalProgram;
using solvers::VectorXDecisionVariable;
using std::sqrt;
using symbolic::Variable;

HyperEllipsoid::HyperEllipsoid(const Eigen::Ref<const MatrixXd>& A,
                               const Eigen::Ref<const VectorXd>& center)
    : ConvexSet(&ConvexSetCloner<HyperEllipsoid>, center.size()),
      A_{A},
      center_{center} {
  DRAKE_DEMAND(A.rows() == center.size());
}

HyperEllipsoid::HyperEllipsoid(const QueryObject<double>& query_object,
                               GeometryId geometry_id,
                               std::optional<FrameId> expressed_in)
    : ConvexSet(&ConvexSetCloner<HyperEllipsoid>, 3) {
  Eigen::Matrix3d A_G;
  query_object.inspector().GetShape(geometry_id).Reify(this, &A_G);
  // p_GG_varᵀ * A_Gᵀ * A_G * p_GG_var ≤ 1

  const RigidTransformd X_WE = expressed_in
                                   ? query_object.GetPoseInWorld(*expressed_in)
                                   : RigidTransformd::Identity();
  const RigidTransformd X_WG = query_object.GetPoseInWorld(geometry_id);
  const RigidTransformd X_GE = X_WG.inverse() * X_WE;

  // (p_EE_var - p_EG)ᵀ * Aᵀ * A * (p_EE_var - p_EG) ≤ 1
  // A = A_G * R_GE, center = p_EG
  A_ = A_G * X_GE.rotation().matrix();
  center_ = X_GE.inverse().translation();
}

HyperEllipsoid::~HyperEllipsoid() = default;

bool HyperEllipsoid::DoPointInSet(const Eigen::Ref<const VectorXd>& x,
                                  double tol) const {
  DRAKE_DEMAND(A_.cols() == x.size());
  const VectorXd v = A_ * (x - center_);
  return v.dot(v) <= 1.0 + tol;
}

void HyperEllipsoid::DoAddPointInSetConstraint(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorXDecisionVariable>& x) const {
  // 1.0 ≥ |A * (x - center)|_2, written as
  // z₀ ≥ |z₁...ₘ|₂ with z = A_cone* x + b_cone.
  const int m = A_.rows();
  const int n = A_.cols();
  MatrixXd A_cone(m + 1, n);
  A_cone << Eigen::RowVectorXd::Zero(n), A_;
  VectorXd b_cone(m + 1);
  b_cone << 1.0, -A_ * center_;
  prog->AddLorentzConeConstraint(A_cone, b_cone, x);
}

std::vector<Binding<Constraint>>
HyperEllipsoid::DoAddPointInNonnegativeScalingConstraints(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorXDecisionVariable>& x,
    const Variable& t) const {
  std::vector<Binding<Constraint>> constraints;
  // t ≥ |A * (x - t*center)|_2, written as
  // z₀ ≥ |z₁...ₘ|₂ with z = A_cone* [x;t].
  const int m = A_.rows();
  const int n = A_.cols();
  MatrixXd A_cone = MatrixXd::Zero(m + 1, n + 1);
  A_cone(0, n) = 1.0;  // z₀ = t.
  // z₁...ₘ = [A, -A*center]
  A_cone.block(1, 0, m, n) = A_;
  A_cone.block(1, n, m, 1) = -A_ * center_;
  constraints.emplace_back(prog->AddLorentzConeConstraint(
      A_cone, VectorXd::Zero(m + 1), {x, Vector1<Variable>(t)}));
  return constraints;
}

std::pair<std::unique_ptr<Shape>, RigidTransformd>
HyperEllipsoid::DoToShapeWithPose() const {
  // Use {R*D*u + center | |u|₂ ≤ 1} representation, but where R is an
  // orthonormal matrix representing a pure rotation and D is a positive
  // diagonal matrix representing a pure scaling. We obtain this via the
  // eigenvector decomposition: RD⁻ᵀD⁻¹Rᵀ = AᵀA.
  DRAKE_DEMAND(A_.rows() == 3);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver;
  solver.compute(A_.transpose() * A_);

  // A must be invertible for the ellipsoid parameters to be finite.
  // The eigenvalues here are the eigenvalues of AᵀA.
  DRAKE_DEMAND((solver.eigenvalues().array() > 1e-12).all());

  // solver.eigenvectors returns V, where V D_λ V^T = AᵀA, so R = V and
  // D⁻ᵀD⁻¹ = D_λ.
  Eigen::Matrix3d R_WG = solver.eigenvectors();
  if (R_WG.determinant() < 0) {
    // Handle improper rotations.
    R_WG.row(2) = -R_WG.row(2);
  }
  RigidTransformd X_WG(RotationMatrixd(R_WG), center_);
  auto shape = std::make_unique<Ellipsoid>(1.0 / sqrt(solver.eigenvalues()[0]),
                                           1.0 / sqrt(solver.eigenvalues()[1]),
                                           1.0 / sqrt(solver.eigenvalues()[2]));
  return std::make_pair(std::move(shape), X_WG);
}

HyperEllipsoid HyperEllipsoid::MakeUnitBall(int dim) {
  return HyperEllipsoid(MatrixXd::Identity(dim, dim), VectorXd::Zero(dim));
}

void HyperEllipsoid::ImplementGeometry(const Sphere& sphere, void* data) {
  auto* A = reinterpret_cast<Eigen::Matrix3d*>(data);
  *A = Eigen::Matrix3d::Identity() / sphere.radius();
}

void HyperEllipsoid::ImplementGeometry(const Ellipsoid& ellipsoid, void* data) {
  // x²/a² + y²/b² + z²/c² = 1 in quadratic form is
  // xᵀ * diag(1/a^2, 1/b^2, 1/c^2) * x = 1 and A is the square root.
  auto* A = reinterpret_cast<Eigen::Matrix3d*>(data);
  *A = Eigen::DiagonalMatrix<double, 3>(
      1.0 / ellipsoid.a(), 1.0 / ellipsoid.b(), 1.0 / ellipsoid.c());
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
