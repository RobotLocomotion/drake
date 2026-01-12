#include "drake/geometry/optimization/hyperellipsoid.h"

#include <limits>
#include <memory>

#include <Eigen/Eigenvalues>
#include <fmt/format.h>

#include "drake/common/overloaded.h"
#include "drake/geometry/optimization/affine_subspace.h"
#include "drake/math/matrix_util.h"
#include "drake/math/rotation_matrix.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/clarabel_solver.h"
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
using symbolic::Expression;
using symbolic::Variable;

Hyperellipsoid::Hyperellipsoid()
    : Hyperellipsoid(MatrixXd(0, 0), VectorXd(0)) {}

Hyperellipsoid::Hyperellipsoid(const Eigen::Ref<const MatrixXd>& A,
                               const Eigen::Ref<const VectorXd>& center)
    : ConvexSet(center.size(), true), A_(A), center_(center) {
  CheckInvariants();
}

Hyperellipsoid::Hyperellipsoid(const QueryObject<double>& query_object,
                               GeometryId geometry_id,
                               std::optional<FrameId> reference_frame)
    : ConvexSet(3, true) {
  const Shape& shape = query_object.inspector().GetShape(geometry_id);
  const Eigen::Matrix3d A_G = shape.Visit<Eigen::Matrix3d>(overloaded{
      // We only handle certain shape types.
      [](const Ellipsoid& ellipsoid) {
        // x²/a² + y²/b² + z²/c² = 1 in quadratic form is
        // xᵀ * diag(1/a^2, 1/b^2, 1/c^2) * x = 1 and A is the square root.
        return Eigen::DiagonalMatrix<double, 3>(
            1.0 / ellipsoid.a(), 1.0 / ellipsoid.b(), 1.0 / ellipsoid.c());
      },
      [](const Sphere& sphere) {
        return Eigen::Matrix3d::Identity() / sphere.radius();
      },
      [&geometry_id](const auto& unsupported) -> Eigen::Matrix3d {
        throw std::logic_error(fmt::format(
            "{} (geometry_id={}) cannot be converted to a Hyperellipsoid",
            unsupported, geometry_id));
      }});
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
namespace {
MatrixXd CalcQuadraticFormA(const AffineBall& ellipsoid) {
  const auto B_QR = Eigen::ColPivHouseholderQR<MatrixXd>(ellipsoid.B());
  DRAKE_THROW_UNLESS(B_QR.isInvertible());
  return B_QR.inverse();
}
}  // namespace

Hyperellipsoid::Hyperellipsoid(const AffineBall& ellipsoid)
    : Hyperellipsoid(CalcQuadraticFormA(ellipsoid), ellipsoid.center()) {}

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

std::pair<double, VectorXd> Hyperellipsoid::MinimumUniformScalingToTouch(
    const ConvexSet& other) const {
  DRAKE_THROW_UNLESS(ambient_dimension() > 0);
  DRAKE_THROW_UNLESS(other.ambient_dimension() == ambient_dimension());
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(ambient_dimension());
  other.AddPointInSetConstraints(&prog, x);

  // Specify a list of solvers by preference, to avoid IPOPT getting used for
  // conic constraints.  See discussion at #15320.
  // TODO(russt): Revisit this pending resolution of #15320.
  std::vector<solvers::SolverId> preferred_solvers{
      solvers::MosekSolver::id(), solvers::GurobiSolver::id(),
      solvers::ClarabelSolver::id()};

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
    MatrixXd A = MatrixXd::Zero(A_.rows() + 2, A_.cols() + 1);
    VectorXd b(A_.rows() + 2);
    A(0, 0) = 1;
    A.bottomRightCorner(A_.rows(), A_.cols()) = A_;
    b[0] = 0;
    b[1] = 1;
    b.tail(A_.rows()) = -A_ * center_;
    prog.AddRotatedLorentzConeConstraint(A, b, {slack, x});
    preferred_solvers.emplace_back(solvers::ScsSolver::id());
  }
  auto solver = solvers::MakeFirstAvailableSolver(preferred_solvers);
  solvers::MathematicalProgramResult result;
  solver->Solve(prog, std::nullopt, std::nullopt, &result);
  if (!result.is_success()) {
    // Check if `other` is empty.
    MathematicalProgram prog2;
    auto x2 = prog2.NewContinuousVariables(ambient_dimension());
    other.AddPointInSetConstraints(&prog2, x2);
    auto result2 = Solve(prog2);
    if (!result2.is_success()) {
      throw std::runtime_error("The set `other` is empty.");
    } else {
      throw std::runtime_error(fmt::format(
          "Solver {} failed to solve the `minimum uniform scaling to touch' "
          "problem; it terminated with SolutionResult {}). The solver likely "
          "ran into numerical issues.",
          result.get_solver_id().name(), result.get_solution_result()));
    }
  }
  return std::pair<double, VectorXd>(std::sqrt(result.get_optimal_cost()),
                                     result.GetSolution(x));
}

Hyperellipsoid Hyperellipsoid::Scale(double scale) const {
  DRAKE_THROW_UNLESS(scale > 0);
  return Hyperellipsoid(A_ / std::pow(scale, 1.0 / ambient_dimension()),
                        center_);
}

Hyperellipsoid Hyperellipsoid::MakeAxisAligned(
    const Eigen::Ref<const VectorXd>& radius,
    const Eigen::Ref<const VectorXd>& center) {
  DRAKE_THROW_UNLESS(radius.size() == center.size());
  DRAKE_THROW_UNLESS((radius.array() > 0).all());
  return Hyperellipsoid(MatrixXd(radius.cwiseInverse().asDiagonal()), center);
}

Hyperellipsoid Hyperellipsoid::MakeHypersphere(
    double radius, const Eigen::Ref<const VectorXd>& center) {
  DRAKE_THROW_UNLESS(radius > 0);
  const int dim = center.size();
  return Hyperellipsoid(MatrixXd::Identity(dim, dim) / radius, center);
}

Hyperellipsoid Hyperellipsoid::MakeUnitBall(int dim) {
  DRAKE_THROW_UNLESS(dim > 0);
  return Hyperellipsoid(MatrixXd::Identity(dim, dim), VectorXd::Zero(dim));
}

Hyperellipsoid Hyperellipsoid::MinimumVolumeCircumscribedEllipsoid(
    const Eigen::Ref<const MatrixXd>& points, double rank_tol) {
  DRAKE_THROW_UNLESS(points.allFinite());
  const int dim = points.rows();
  int rank;
  const int n = points.cols();

  // Check the numerical rank of the data matrix.
  std::optional<Eigen::MatrixXd> U;
  Eigen::VectorXd mean = points.rowwise().mean();
  {
#if EIGEN_VERSION_AT_LEAST(5, 0, 0)
    auto svd = (points.colwise() - mean).template bdcSvd<Eigen::ComputeThinU>();
#else
    auto svd = (points.colwise() - mean).bdcSvd(Eigen::ComputeThinU);
#endif
    // Eigen's SVD rank never returns zero, and their singular values are
    // returned in decreasing order.
    if (svd.singularValues()[0] < rank_tol) {
      throw std::runtime_error(fmt::format(
          "The numerical rank of the points appears to be zero. (The largest "
          "singular value is {}, which is below rank_tol = {})",
          svd.singularValues()[0], rank_tol));
    }
    svd.setThreshold(rank_tol);
    rank = svd.rank();
    if (rank < dim) {
      throw std::runtime_error(
          "The numerical rank of the points appears to be less than the "
          "ambient dimension. The smallest singular value is {}, which is "
          "below rank_tol = {}. Decrease rank_tol or consider using "
          "AffineBall::MinimumVolumeCircumscribedEllipsoid instead.");
    }
  }

  MathematicalProgram prog;
  solvers::MatrixXDecisionVariable A =
      prog.NewSymmetricContinuousVariables(rank, "A");
  prog.AddMaximizeLogDeterminantCost(A.cast<Expression>());
  solvers::VectorXDecisionVariable b = prog.NewContinuousVariables(rank, "b");
  // TODO(russt): Avoid the symbolic computation here and write A_lorentz
  // directly, s.t. v = A_lorentz * vars + b_lorentz, where A=Aᵀ and b are the
  // vars.
  VectorX<Expression> v(dim + 1);
  v[0] = 1;
  for (int i = 0; i < n; ++i) {
    // |Ax + b|₂ <= 1, written as a Lorentz cone with v = [1; A * x + b].
    v.tail(dim) = A * points.col(i) + b;
    prog.AddLorentzConeConstraint(v);
  }

  solvers::MathematicalProgramResult result = Solve(prog);
  if (!result.is_success()) {
    throw std::runtime_error(
        fmt::format("The MathematicalProgram was not solved successfully. The "
                    "ambient dimension is {} and the data rank is {}, computed "
                    "using rank_tol={}. Consider adjusting rank_tol.",
                    dim, rank, rank_tol));
  }

  // Ax + b => A(x-c) = Ax - Ac => c = -A^{-1}b
  const MatrixXd A_sol = result.GetSolution(A);
  const VectorXd b_sol = result.GetSolution(b);
  // Note: We can use llt() because know that A will be positive definite;
  // there is a PSD constraint, but we are maximizing the eigenvalues of A and
  // the convex hull of the points is guaranteed to be bounded.
  const VectorXd c = A_sol.llt().solve(-b_sol);

  return Hyperellipsoid(A_sol, c);
}

std::unique_ptr<ConvexSet> Hyperellipsoid::DoClone() const {
  return std::make_unique<Hyperellipsoid>(*this);
}

std::optional<bool> Hyperellipsoid::DoIsBoundedShortcut() const {
  if (A_.rows() < A_.cols()) {
    return false;
  }
  Eigen::ColPivHouseholderQR<MatrixXd> qr(A_);
  return qr.dimensionOfKernel() == 0;
}

bool Hyperellipsoid::DoIsEmpty() const {
  return false;
}

std::optional<Eigen::VectorXd> Hyperellipsoid::DoMaybeGetFeasiblePoint() const {
  return center_;
}

std::optional<bool> Hyperellipsoid::DoPointInSetShortcut(
    const Eigen::Ref<const VectorXd>& x, double tol) const {
  DRAKE_DEMAND(A_.cols() == x.size());
  const VectorXd v = A_ * (x - center_);
  return v.dot(v) <= 1.0 + tol;
}

std::pair<VectorX<Variable>, std::vector<Binding<Constraint>>>
Hyperellipsoid::DoAddPointInSetConstraints(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorXDecisionVariable>& x) const {
  VectorX<Variable> new_vars;
  std::vector<Binding<Constraint>> new_constraints;
  // 1.0 ≥ |A * (x - center)|_2, written as
  // z₀ ≥ |z₁...ₘ|₂ with z = A_cone* x + b_cone.
  const int m = A_.rows();
  const int n = A_.cols();
  MatrixXd A_cone(m + 1, n);
  A_cone << Eigen::RowVectorXd::Zero(n), A_;
  VectorXd b_cone(m + 1);
  b_cone << 1.0, -A_ * center_;
  new_constraints.push_back(prog->AddLorentzConeConstraint(A_cone, b_cone, x));
  return {std::move(new_vars), std::move(new_constraints)};
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

std::vector<Binding<Constraint>>
Hyperellipsoid::DoAddPointInNonnegativeScalingConstraints(
    MathematicalProgram* prog, const Eigen::Ref<const MatrixXd>& A_x,
    const Eigen::Ref<const VectorXd>& b, const Eigen::Ref<const VectorXd>& c,
    double d, const Eigen::Ref<const VectorXDecisionVariable>& x,
    const Eigen::Ref<const VectorXDecisionVariable>& t) const {
  std::vector<Binding<Constraint>> constraints;
  // c'*t+d ≥ |A * (A_x * x + b - (c'*t+d)*center)|_2, written as
  // z₀ ≥ |z₁...ₘ|₂ with z = A_cone * [x;t] + b_cone.
  const int m = A_.rows();
  const int n_x = x.size();
  const int n_t = t.size();
  MatrixXd A_cone = MatrixXd::Zero(m + 1, n_x + n_t);
  VectorXd b_cone(m + 1);
  // z₀ = c' * t + d.
  A_cone.block(0, n_x, 1, n_t) = c.transpose();
  b_cone(0) = d;
  // z₁...ₘ = [A*A_x, -A*center*c'] * [x;t] + A*(b-d*center)
  A_cone.block(1, 0, m, n_x) = A_ * A_x;
  A_cone.block(1, n_x, m, n_t) = -A_ * center_ * c.transpose();
  b_cone.tail(m) = A_ * (b - d * center_);
  constraints.emplace_back(
      prog->AddLorentzConeConstraint(A_cone, b_cone, {x, t}));
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
  DRAKE_THROW_UNLESS((solver.eigenvalues().array() > 1e-12).all());

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

double Hyperellipsoid::DoCalcVolume() const {
  if (ambient_dimension() == 0) {
    return 0.0;
  }
  if (A_.rows() < A_.cols()) {
    return std::numeric_limits<double>::infinity();
  }
  // Note: this will (correctly) return infinity if the determinant is zero.
  return volume_of_unit_sphere(ambient_dimension()) /
         std::abs(A_.determinant());
}

void Hyperellipsoid::CheckInvariants() const {
  DRAKE_THROW_UNLESS(this->ambient_dimension() == A_.cols());
  DRAKE_THROW_UNLESS(A_.cols() == center_.size());
  DRAKE_THROW_UNLESS(A_.allFinite());  // to ensure the set is non-empty.
}

std::unique_ptr<ConvexSet> Hyperellipsoid::DoAffineHullShortcut(
    std::optional<double> tol) const {
  // Hyperellipsoids are always positive volume, so we can trivially construct
  // their affine hull as the whole vector space.
  unused(tol);
  const int n = ambient_dimension();
  return std::make_unique<AffineSubspace>(Eigen::MatrixXd::Identity(n, n),
                                          Eigen::VectorXd::Zero(n));
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
