#include "drake/geometry/optimization/convex_set.h"

#include <limits>
#include <memory>

#include <Eigen/Eigenvalues>

#include "drake/math/matrix_util.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace geometry {
namespace optimization {

using math::RigidTransformd;
using math::RotationMatrixd;
using solvers::MathematicalProgram;
using solvers::VectorXDecisionVariable;
using std::pair;
using std::sqrt;

HPolyhedron::HPolyhedron(const Eigen::Ref<const Eigen::MatrixXd>& A,
                         const Eigen::Ref<const Eigen::VectorXd>& b)
    : ConvexSet(A.cols()), A_{A}, b_{b} {
  DRAKE_DEMAND(A.rows() == b.size());
}

HPolyhedron::HPolyhedron(const QueryObject<double>& query_object,
                         GeometryId geometry_id,
                         std::optional<FrameId> expressed_in) : ConvexSet(3) {
  std::pair<Eigen::MatrixXd, Eigen::VectorXd> Ab_G;
  query_object.inspector().GetShape(geometry_id).Reify(this, &Ab_G);

  const RigidTransformd X_WE = expressed_in
                                   ? query_object.GetPoseInWorld(*expressed_in)
                                   : RigidTransformd::Identity();
  const RigidTransformd X_WG = query_object.GetPoseInWorld(geometry_id);
  const RigidTransformd X_GE = X_WG.InvertAndCompose(X_WE);
  // A_G*(p_GE + R_GE*p_EE_var) ≤ b_G
  A_ = Ab_G.first * X_GE.rotation().matrix();
  b_ = Ab_G.second - Ab_G.first * X_GE.translation();
}

bool HPolyhedron::DoPointInSet(const Eigen::Ref<const Eigen::VectorXd>& x,
                               double tol) const {
  DRAKE_DEMAND(A_.cols() == x.size());
  return ((A_ * x).array() <= b_.array() + tol).all();
}

void HPolyhedron::DoAddPointInSetConstraint(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) const {
  prog->AddLinearConstraint(
      A_,
      Eigen::VectorXd::Constant(b_.size(),
                                -std::numeric_limits<double>::infinity()),
      b_, vars);
}

void HPolyhedron::ImplementGeometry(const HalfSpace&, void* data) {
  auto* Ab =
      reinterpret_cast<std::pair<Eigen::MatrixXd, Eigen::VectorXd>*>(data);
  // z <= 0.0.
  Ab->first = Eigen::RowVector3d{0.0, 0.0, 1.0};
  Ab->second = Vector1d{0.0};
}

void HPolyhedron::ImplementGeometry(const Box& box, void* data) {
  Eigen::Matrix<double, 6, 3> A;
  A << Eigen::Matrix3d::Identity(), -Eigen::Matrix3d::Identity();
  Vector6d b;
  // clang-format off
  b << box.width()/2.0, box.depth()/2.0, box.height()/2.0,
        box.width()/2.0, box.depth()/2.0, box.height()/2.0;
  // clang-format on
  auto* Ab =
      reinterpret_cast<std::pair<Eigen::MatrixXd, Eigen::VectorXd>*>(data);
  Ab->first = A;
  Ab->second = b;
}

HyperEllipsoid::HyperEllipsoid(const Eigen::Ref<const Eigen::MatrixXd>& A,
                     const Eigen::Ref<const Eigen::VectorXd>& center)
    : ConvexSet(center.size()), A_{A}, center_{center} {
  DRAKE_DEMAND(A.rows() == center.size());
}

HyperEllipsoid::HyperEllipsoid(
    const QueryObject<double>& query_object, GeometryId geometry_id,
    std::optional<FrameId> expressed_in) : ConvexSet(3) {
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

bool HyperEllipsoid::DoPointInSet(
    const Eigen::Ref<const Eigen::VectorXd>& x, double tol) const {
  DRAKE_DEMAND(A_.cols() == x.size());
  const Eigen::VectorXd v = A_ * (x - center_);
  return v.dot(v) <= 1.0 + tol;
}

void HyperEllipsoid::DoAddPointInSetConstraint(
    solvers::MathematicalProgram* prog,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) const {
  Eigen::MatrixXd A_cone(A_.rows() + 1, A_.cols());
  A_cone << Eigen::RowVectorXd::Zero(A_.cols()), A_;
  Eigen::VectorXd b_cone(center_.size() + 1);
  b_cone << 1.0, -A_ * center_;

  // 1.0 >= |A * (vars - center)|_2
  prog->AddLorentzConeConstraint(A_cone, b_cone, vars);
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
  auto shape = std::make_unique<Ellipsoid>(
      1.0/sqrt(solver.eigenvalues()[0]), 1.0/sqrt(solver.eigenvalues()[1]),
      1.0/sqrt(solver.eigenvalues()[2]));
  return std::make_pair(std::move(shape), X_WG);
}

void HyperEllipsoid::ImplementGeometry(const Sphere& sphere,
                                       void* data) {
  auto* A = reinterpret_cast<Eigen::Matrix3d*>(data);
  *A = Eigen::Matrix3d::Identity() / sphere.radius();
}

void HyperEllipsoid::ImplementGeometry(const Ellipsoid& ellipsoid,
                                       void* data) {
  // x²/a² + y²/b² + z²/c² = 1 in quadratic form is
  // xᵀ * diag(1/a^2, 1/b^2, 1/c^2) * x = 1 and A is the square root.
  auto* A = reinterpret_cast<Eigen::Matrix3d*>(data);
  *A = Eigen::DiagonalMatrix<double, 3>(
      1.0 / ellipsoid.a(), 1.0 / ellipsoid.b(), 1.0 / ellipsoid.c());
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
