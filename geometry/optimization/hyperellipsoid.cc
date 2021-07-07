#include "drake/geometry/optimization/hyperellipsoid.h"

#include <limits>
#include <memory>

#include <Eigen/Eigenvalues>
#include <fmt/format.h>

#include "drake/math/matrix_util.h"
#include "drake/math/rotation_matrix.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/scs_solver.h"
#include "drake/solvers/solve.h"

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

Hyperellipsoid::Hyperellipsoid(const Eigen::Ref<const MatrixXd>& A,
                               const Eigen::Ref<const VectorXd>& center)
    : ConvexSet(&ConvexSetCloner<Hyperellipsoid>, center.size()),
      A_{A},
      center_{center} {
  DRAKE_DEMAND(A.cols() == center.size());
  DRAKE_DEMAND(A.allFinite());  // to ensure the set is non-empty.
}

Hyperellipsoid::Hyperellipsoid(const QueryObject<double>& query_object,
                               GeometryId geometry_id,
                               std::optional<FrameId> reference_frame)
    : ConvexSet(&ConvexSetCloner<Hyperellipsoid>, 3) {
  Eigen::Matrix3d A_G;
  query_object.inspector().GetShape(geometry_id).Reify(this, &A_G);
  // p_GG_varᵀ * A_Gᵀ * A_G * p_GG_var ≤ 1

  const RigidTransformd X_WE =
      reference_frame ? query_object.GetPoseInWorld(*reference_frame)
                      : RigidTransformd::Identity();
  const RigidTransformd& X_WG = query_object.GetPoseInWorld(geometry_id);
  const RigidTransformd X_GE = X_WG.InvertAndCompose(X_WE);

  // (p_EE_var - p_EG)ᵀ * Aᵀ * A * (p_EE_var - p_EG) ≤ 1
  // A = A_G * R_GE, center = p_EG
  A_ = A_G * X_GE.rotation().matrix();
  center_ = X_GE.inverse().translation();
}

Hyperellipsoid::~Hyperellipsoid() = default;

namespace {

double volume_of_unit_sphere(int dim) {
  // Formula from https://en.wikipedia.org/wiki/Volume_of_an_n-ball .
  // Note: special case n≤3 only because they are common and simple.
  switch (dim) {
    case 0:
      return 1.0;
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

double Hyperellipsoid::Volume() const {
  if (A_.rows() < A_.cols()) {
    return std::numeric_limits<double>::infinity();
  }
  // Note: this will (correctly) return infinity if the determinant is zero.
  return volume_of_unit_sphere(ambient_dimension()) / A_.determinant();
}

std::pair<double, VectorXd> Hyperellipsoid::MinimumUniformScalingToTouch(
    const ConvexSet& other) const {
  DRAKE_DEMAND(other.ambient_dimension() == ambient_dimension());
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(ambient_dimension());
  other.AddPointInSetConstraints(&prog, x);

  // Specify a list of solvers by preference, to avoid IPOPT getting used for
  // conic constraints.  See discussion at #15320.
  // TODO(russt): Revisit this pending resolution of #15320.
  std::vector<solvers::SolverId> preferred_solvers{solvers::MosekSolver::id(),
                                          solvers::GurobiSolver::id()};

  // If we have only linear constraints, then add a quadratic cost and solve the
  // QP.  Otherwise add a slack variable and solve the SOCP.
  bool has_only_linear_constraints = true;
  for (const auto& attribute : prog.required_capabilities()) {
    if (attribute != solvers::ProgramAttribute::kLinearConstraint &&
        attribute != solvers::ProgramAttribute::kLinearEqualityConstraint) {
      has_only_linear_constraints = false;
    }
  }
  if (has_only_linear_constraints) {
    prog.AddQuadraticErrorCost(A_.transpose() * A_, center_, x);
    preferred_solvers.emplace_back(solvers::IpoptSolver::id());
  } else {
    auto slack = prog.NewContinuousVariables<1>("slack");
    prog.AddLinearCost(slack[0]);
    // z₀ = slack, z₁ = 1, z₂...ₙ = A_*(x-center)
    // z₀z₁ ≥ z₂² + ... + zₙ²
    MatrixXd A = MatrixXd::Zero(A_.rows()+2, A_.cols()+1);
    VectorXd b(A_.rows()+2);
    A(0, 0) = 1;
    A.bottomRightCorner(A_.rows(), A_.cols()) = A_;
    b[0] = 0;
    b[1] = 1;
    b.tail(A_.rows()) = -A_*center_;
    prog.AddRotatedLorentzConeConstraint(A, b, {slack, x});
    preferred_solvers.emplace_back(solvers::ScsSolver::id());
  }
  auto solver = solvers::MakeFirstAvailableSolver(preferred_solvers);
  solvers::MathematicalProgramResult result;
  solver->Solve(prog, std::nullopt, std::nullopt, &result);
  if (!result.is_success()) {
    throw std::runtime_error(fmt::format(
        "Solver {} failed to solve the `minimum uniform scaling to touch' "
        "problem; it terminated with SolutionResult {}). This should not "
        "happen.",
        result.get_solver_id().name(), result.get_solution_result()));
  }
  return std::pair<double, VectorXd>(std::sqrt(result.get_optimal_cost()),
                                     result.GetSolution(x));
}

Hyperellipsoid Hyperellipsoid::MakeAxisAligned(
    const Eigen::Ref<const VectorXd>& radius,
    const Eigen::Ref<const VectorXd>& center) {
  DRAKE_DEMAND(radius.size() == center.size());
  DRAKE_DEMAND((radius.array() > 0).all());
  return Hyperellipsoid(MatrixXd(radius.cwiseInverse().asDiagonal()), center);
}

Hyperellipsoid Hyperellipsoid::MakeHypersphere(
    double radius, const Eigen::Ref<const VectorXd>& center) {
  DRAKE_DEMAND(radius > 0);
  const int dim = center.size();
  return Hyperellipsoid(MatrixXd::Identity(dim, dim) / radius, center);
}

Hyperellipsoid Hyperellipsoid::MakeUnitBall(int dim) {
  DRAKE_DEMAND(dim > 0);
  return Hyperellipsoid(MatrixXd::Identity(dim, dim), VectorXd::Zero(dim));
}

bool Hyperellipsoid::DoIsBounded() const {
  if (A_.rows() < A_.cols()) {
    return false;
  }
  Eigen::ColPivHouseholderQR<MatrixXd> qr(A_);
  return qr.dimensionOfKernel() == 0;
}

bool Hyperellipsoid::DoPointInSet(const Eigen::Ref<const VectorXd>& x,
                                  double tol) const {
  DRAKE_DEMAND(A_.cols() == x.size());
  const VectorXd v = A_ * (x - center_);
  return v.dot(v) <= 1.0 + tol;
}

void Hyperellipsoid::DoAddPointInSetConstraints(
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
Hyperellipsoid::DoAddPointInNonnegativeScalingConstraints(
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
Hyperellipsoid::DoToShapeWithPose() const {
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

void Hyperellipsoid::ImplementGeometry(const Sphere& sphere, void* data) {
  auto* A = static_cast<Eigen::Matrix3d*>(data);
  *A = Eigen::Matrix3d::Identity() / sphere.radius();
}

void Hyperellipsoid::ImplementGeometry(const Ellipsoid& ellipsoid, void* data) {
  // x²/a² + y²/b² + z²/c² = 1 in quadratic form is
  // xᵀ * diag(1/a^2, 1/b^2, 1/c^2) * x = 1 and A is the square root.
  auto* A = static_cast<Eigen::Matrix3d*>(data);
  *A = Eigen::DiagonalMatrix<double, 3>(
      1.0 / ellipsoid.a(), 1.0 / ellipsoid.b(), 1.0 / ellipsoid.c());
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
