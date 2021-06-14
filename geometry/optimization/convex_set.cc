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

bool HPolyhedron::PointInSet(const Eigen::Ref<const Eigen::VectorXd>& x) const {
  DRAKE_DEMAND(A_.cols() == x.size());
  return ((A_ * x).array() <= b_.array()).all();
}

void HPolyhedron::AddPointInSetConstraint(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) const {
  prog->AddLinearConstraint(A_, -std::numeric_limits<double>::infinity() * b_,
                            b_, vars);
}

namespace {

class MakeHPolyhedronReifier : public ShapeReifier {
 public:
  void ImplementGeometry(const Sphere&, void*) override {
    ThrowUnsupportedGeometry("Sphere");
  }

  void ImplementGeometry(const Cylinder&, void*) override {
    ThrowUnsupportedGeometry("Cylinder");
  }

  void ImplementGeometry(const HalfSpace&, void* data) override {
    auto* Ab =
        reinterpret_cast<std::pair<Eigen::MatrixXd, Eigen::VectorXd>*>(data);
    // z <= 0.0.
    Ab->first = Eigen::RowVector3d{0.0, 0.0, 1.0};
    Ab->second = Vector1d{0.0};
  }

  void ImplementGeometry(const Box& box, void* data) override {
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

  void ImplementGeometry(const Capsule&, void*) override {
    ThrowUnsupportedGeometry("Capsule");
  }

  void ImplementGeometry(const geometry::Ellipsoid&, void*) override {
    ThrowUnsupportedGeometry("Ellipsoid");
  }

  void ImplementGeometry(const Mesh&, void*) override {
    ThrowUnsupportedGeometry("Mesh");
  }

  // TODO(russt): Support this, but requires e.g. digging ReadObjForConvex
  // out of proximity_engine.cc.
  void ImplementGeometry(const Convex&, void*) override {
    ThrowUnsupportedGeometry("Convex");
  }
};
}  // namespace

HPolyhedron HPolyhedron::MakeFromSceneGraph(
    const QueryObject<double>& query_object, const GeometryId& geometry_id) {
  MakeHPolyhedronReifier reifier;
  std::pair<Eigen::MatrixXd, Eigen::VectorXd> Ab_G;
  query_object.inspector().GetShape(geometry_id).Reify(&reifier, &Ab_G);

  const RigidTransformd X_GW =
      query_object.GetPoseInWorld(geometry_id).inverse();
  // A_G*(p_GW + R_GW*x_W) ≤ b_G
  return HPolyhedron(Ab_G.first * X_GW.rotation().matrix(),
                     Ab_G.second - Ab_G.first * X_GW.translation());
}

Ellipsoid::Ellipsoid(const Eigen::Ref<const Eigen::MatrixXd>& A,
                     const Eigen::Ref<const Eigen::VectorXd>& center)
    : ConvexSet(center.size()), A_{A}, center_{center} {
  DRAKE_DEMAND(A.rows() == center.size());
  DRAKE_DEMAND(A.rows() == A.cols());
}

bool Ellipsoid::PointInSet(const Eigen::Ref<const Eigen::VectorXd>& x) const {
  DRAKE_DEMAND(A_.cols() == x.size());
  const Eigen::VectorXd v = A_ * (x - center_);
  return v.dot(v) <= 1.0;
}

void Ellipsoid::AddPointInSetConstraint(
    solvers::MathematicalProgram* prog,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) const {
  Eigen::MatrixXd A_cone(A_.rows() + 1, A_.cols());
  A_cone << Eigen::RowVectorXd::Zero(A_.cols()), A_;
  Eigen::VectorXd b_cone(center_.size() + 1);
  b_cone << 1.0, -A_ * center_;

  // 1.0 >= |A * (vars - center)|_2
  prog->AddLorentzConeConstraint(A_cone, b_cone, vars);
}

std::pair<std::unique_ptr<Shape>, RigidTransformd> Ellipsoid::ToSceneGraph()
    const {
  // Use {R*D*u + center | |u|₂ ≤ 1} representation, but where R is an
  // orthonormal matrix representing a pure rotation and D is a positive
  // diagonal matrix representing a pure scaling. We obtain this via the
  // eigenvector decomposition: Rᵀ*Dᵀ*D*R = (Aᵀ*A)⁻¹.
  DRAKE_DEMAND(A_.rows() == 3);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver;
  solver.compute((A_.transpose() * A_).inverse());

  Eigen::Matrix3d R_WG = solver.eigenvectors();
  if (R_WG.determinant() < 0) {
    // Handle improper rotations.
    R_WG.row(3) = -R_WG.row(3);
  }
  RigidTransformd X_WG(RotationMatrixd(R_WG), center_);
  auto shape = std::make_unique<drake::geometry::Ellipsoid>(
      sqrt(solver.eigenvalues()[0]), sqrt(solver.eigenvalues()[1]),
      sqrt(solver.eigenvalues()[2]));
  return std::make_pair(std::move(shape), X_WG);
}

namespace {
class MakeEllipsoidReifier : public ShapeReifier {
 public:
  void ImplementGeometry(const Sphere& sphere, void* data) override {
    auto* A = reinterpret_cast<Eigen::Matrix3d*>(data);
    *A = sphere.radius() * Eigen::Matrix3d::Identity();
  }

  void ImplementGeometry(const Cylinder&, void*) override {
    ThrowUnsupportedGeometry("Cylinder");
  }

  void ImplementGeometry(const HalfSpace&, void*) override {
    ThrowUnsupportedGeometry("HalfSpace");
  }

  void ImplementGeometry(const Box&, void*) override {
    ThrowUnsupportedGeometry("Box");
  }

  void ImplementGeometry(const Capsule&, void*) override {
    ThrowUnsupportedGeometry("Capsule");
  }

  void ImplementGeometry(const geometry::Ellipsoid& ellipsoid,
                         void* data) override {
    // x²/a² + y²/b² + z²/c² = 1 in quadratic form is
    // xᵀ * diag(1/a^2, 1/b^2, 1/c^2) * x = 1 and A is the square root.
    auto* A = reinterpret_cast<Eigen::Matrix3d*>(data);
    *A = Eigen::DiagonalMatrix<double, 3>(
        1.0 / ellipsoid.a(), 1.0 / ellipsoid.b(), 1.0 / ellipsoid.c());
  }

  void ImplementGeometry(const Mesh&, void*) override {
    ThrowUnsupportedGeometry("Mesh");
  }

  void ImplementGeometry(const Convex&, void*) override {
    ThrowUnsupportedGeometry("Convex");
  }
};
}  // namespace

Ellipsoid Ellipsoid::MakeFromSceneGraph(const QueryObject<double>& query_object,
                                        const GeometryId& geometry_id) {
  MakeEllipsoidReifier reifier;
  Eigen::Matrix3d A_G;
  query_object.inspector().GetShape(geometry_id).Reify(&reifier, &A_G);
  // p_GG_varᵀ * A_Gᵀ * A_G * p_GG_var ≤ 1

  const RigidTransformd& X_WG = query_object.GetPoseInWorld(geometry_id);
  const RigidTransformd X_GW = X_WG.inverse();

  // (p_WW_var - p_WG)ᵀ * Aᵀ * A * (p_WW_var - p_WG) ≤ 1
  // A = A_G * R_GW, center = p_WG
  return Ellipsoid(A_G * X_GW.rotation().matrix(), X_WG.translation());
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
