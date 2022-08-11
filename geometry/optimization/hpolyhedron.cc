#include "drake/geometry/optimization/hpolyhedron.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <stdexcept>

#include <Eigen/Eigenvalues>
#include <fmt/format.h>

#include "drake/math/matrix_util.h"
#include "drake/math/rotation_matrix.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace geometry {
namespace optimization {

using Eigen::MatrixXd;
using Eigen::RowVectorXd;
using Eigen::VectorXd;
using math::RigidTransformd;
using solvers::Binding;
using solvers::Constraint;
using solvers::MathematicalProgram;
using solvers::MatrixXDecisionVariable;
using solvers::VectorXDecisionVariable;
using symbolic::Expression;
using symbolic::Variable;

HPolyhedron::HPolyhedron(const Eigen::Ref<const MatrixXd>& A,
                         const Eigen::Ref<const VectorXd>& b)
    : ConvexSet(&ConvexSetCloner<HPolyhedron>, A.cols()), A_{A}, b_{b} {
  DRAKE_DEMAND(A.rows() == b.size());
  // Note: If necessary, we could support infinite b, either by removing the
  // corresponding rows of A (since the constraint is vacuous), or checking
  // this explicitly in all relevant computations (like IsBounded).
  DRAKE_DEMAND(b.array().isFinite().all());
}

HPolyhedron::HPolyhedron(const QueryObject<double>& query_object,
                         GeometryId geometry_id,
                         std::optional<FrameId> reference_frame)
    : ConvexSet(&ConvexSetCloner<HPolyhedron>, 3) {
  std::pair<MatrixXd, VectorXd> Ab_G;
  query_object.inspector().GetShape(geometry_id).Reify(this, &Ab_G);

  const RigidTransformd X_WE =
      reference_frame ? query_object.GetPoseInWorld(*reference_frame)
                      : RigidTransformd::Identity();
  const RigidTransformd& X_WG = query_object.GetPoseInWorld(geometry_id);
  const RigidTransformd X_GE = X_WG.InvertAndCompose(X_WE);
  // A_G*(p_GE + R_GE*p_EE_var) ≤ b_G
  A_ = Ab_G.first * X_GE.rotation().matrix();
  b_ = Ab_G.second - Ab_G.first * X_GE.translation();
}

HPolyhedron::~HPolyhedron() = default;

Hyperellipsoid HPolyhedron::MaximumVolumeInscribedEllipsoid() const {
  MathematicalProgram prog;
  const int N = this->ambient_dimension();
  MatrixXDecisionVariable C = prog.NewSymmetricContinuousVariables(N, "C");
  VectorXDecisionVariable d = prog.NewContinuousVariables(N, "d");

  // max log det (C).  This method also imposes C ≽ 0.
  prog.AddMaximizeLogDeterminantCost(C.cast<Expression>());
  // |aᵢC|₂ ≤ bᵢ - aᵢd, ∀i
  // TODO(russt): We could potentially avoid Expression parsing here by using
  // AddLorentzConeConstraint(A,b,vars), but it's nontrivial because of the
  // duplicate entries in the symmetric matrix C.  E.g. the Lorentz cone A would
  // not be simply block_diagonal(-A_.row(i), A_.row(i), ..., A_.row(i)).
  VectorX<Expression> z(N + 1);
  for (int i = 0; i < b_.size(); ++i) {
    z[0] = b_(i) - A_.row(i).dot(d);
    z.tail(N) = C * A_.row(i).transpose();
    prog.AddLorentzConeConstraint(z);
  }
  auto result = solvers::Solve(prog);
  if (!result.is_success()) {
    throw std::runtime_error(fmt::format(
        "Solver {} failed to solve the maximum inscribed ellipse problem; it "
        "terminated with SolutionResult {}). Make sure that your polyhedron is "
        "bounded and has an interior.",
        result.get_solver_id().name(), result.get_solution_result()));
  }
  return Hyperellipsoid(result.GetSolution(C).inverse(), result.GetSolution(d));
}

VectorXd HPolyhedron::ChebyshevCenter() const {
  MathematicalProgram prog;
  VectorXDecisionVariable x = prog.NewContinuousVariables(ambient_dimension());
  VectorXDecisionVariable r = prog.NewContinuousVariables<1>("r");

  const double inf = std::numeric_limits<double>::infinity();
  // max r
  prog.AddLinearCost(Vector1d(-1.0), 0, r);

  // r ≥ 0.
  prog.AddBoundingBoxConstraint(0, inf, r);

  // aᵢᵀ x + |aᵢ| r ≤ bᵢ.
  RowVectorXd a(A_.cols() + 1);
  for (int i = 0; i < A_.rows(); i++) {
    a[0] = A_.row(i).norm();
    a.tail(A_.cols()) = A_.row(i);
    prog.AddLinearConstraint(a, -inf, b_[i], {r, x});
  }

  auto result = solvers::Solve(prog);
  if (!result.is_success()) {
    throw std::runtime_error(fmt::format(
        "Solver {} failed to solve the Chebyshev center problem; it terminated "
        "with SolutionResult {}). Make sure that your polyhedron is bounded "
        "and has an interior.",
        result.get_solver_id().name(), result.get_solution_result()));
  }
  return result.GetSolution(x);
}

HPolyhedron HPolyhedron::CartesianProduct(const HPolyhedron& other) const {
  MatrixXd A_product = MatrixXd::Zero(A_.rows() + other.A().rows(),
                                      A_.cols() + other.A().cols());
  A_product.topLeftCorner(A_.rows(), A_.cols()) = A_;
  A_product.bottomRightCorner(other.A().rows(), other.A().cols()) = other.A();
  VectorXd b_product(b_.size() + other.b().size());
  b_product << b_, other.b();
  return {A_product, b_product};
}

HPolyhedron HPolyhedron::CartesianPower(int n) const {
  MatrixXd A_power = MatrixXd::Zero(n * A_.rows(), n * A_.cols());
  for (int i{0}; i < n; ++i) {
    A_power.block(i * A_.rows(), i * A_.cols(), A_.rows(), A_.cols()) = A_;
  }
  VectorXd b_power = b_.replicate(n, 1);
  return {A_power, b_power};
}

HPolyhedron HPolyhedron::Intersection(const HPolyhedron& other) const {
  DRAKE_DEMAND(ambient_dimension() == other.ambient_dimension());
  MatrixXd A_intersect =
      MatrixXd::Zero(A_.rows() + other.A().rows(), A_.cols());
  A_intersect.topRows(A_.rows()) = A_;
  A_intersect.bottomRows(other.A().rows()) = other.A();
  VectorXd b_intersect(b_.size() + other.b().size());
  b_intersect << b_, other.b();
  return {A_intersect, b_intersect};
}

VectorXd HPolyhedron::UniformSample(
    RandomGenerator* generator,
    const Eigen::Ref<Eigen::VectorXd>& previous_sample) const {
  std::normal_distribution<double> gaussian;
  // Choose a random direction.
  VectorXd direction(ambient_dimension());
  for (int i = 0; i < direction.size(); ++i) {
    direction[i] = gaussian(*generator);
  }
  // Find max and min θ subject to
  //   A(previous_sample + θ*direction) ≤ b.
  VectorXd line_b = b_ - A_ * previous_sample;
  VectorXd line_a = A_ * direction;
  double theta_max = std::numeric_limits<double>::infinity();
  double theta_min = -theta_max;
  for (int i = 0; i < line_a.size(); ++i) {
    if (line_a[i] < 0.0) {
      theta_min = std::max(theta_min, line_b[i] / line_a[i]);
    } else if (line_a[i] > 0.0) {
      theta_max = std::min(theta_max, line_b[i] / line_a[i]);
    }
  }
  if (std::isinf(theta_max) || std::isinf(theta_min)) {
    throw std::invalid_argument(
        "The Hit and Run algorithm failed to find a feasible point in the set. "
        "The `previous_sample` must be in the set.");
  }
  // Now pick θ uniformly from [θ_min, θ_max].
  std::uniform_real_distribution<double> uniform_theta(theta_min, theta_max);
  const double theta = uniform_theta(*generator);
  // The new sample is previous_sample + θ * direction.
  return previous_sample + theta * direction;
}

// Note: This method only exists to effectively provide ChebyshevCenter(),
// which is a non-static class method, as a default argument for
// previous_sample in the UniformSample method above.
VectorXd HPolyhedron::UniformSample(
    RandomGenerator* generator) const {
  VectorXd center = ChebyshevCenter();
  return UniformSample(generator, center);
}

HPolyhedron HPolyhedron::MakeBox(const Eigen::Ref<const VectorXd>& lb,
                                 const Eigen::Ref<const VectorXd>& ub) {
  DRAKE_DEMAND(lb.size() == ub.size());
  DRAKE_DEMAND((lb.array() <= ub.array()).all());
  const int N = lb.size();
  MatrixXd A(2 * N, N);
  A << MatrixXd::Identity(N, N), -MatrixXd::Identity(N, N);
  VectorXd b(2 * N);
  b << ub, -lb;
  return HPolyhedron(A, b);
}

HPolyhedron HPolyhedron::MakeUnitBox(int dim) {
  return MakeBox(VectorXd::Constant(dim, -1.0), VectorXd::Constant(dim, 1.0));
}

bool HPolyhedron::DoIsBounded() const {
  if (A_.rows() < A_.cols()) {
    return false;
  }
  Eigen::ColPivHouseholderQR<MatrixXd> qr(A_);
  if (qr.dimensionOfKernel() > 0) {
    return false;
  }
  // Stiemke's theorem of alternatives says that, given A with ker(A) = {0}, we
  // either have existence of x ≠ 0 such that Ax ≥ 0 or we have existence of y
  // > 0 such that y^T A = 0.  Since any y that verifies the second condition
  // can be arbitrarily scaled, and would still pass the second condition,
  // instead of asking y > 0, we can equivalently ask y ≥ 1.  So boundedness
  // corresponds to the following LP being feasible: find y s.t. y ≥ 1, y^T A =
  // 0.
  MathematicalProgram prog;
  auto y = prog.NewContinuousVariables(A_.rows(), "y");
  prog.AddBoundingBoxConstraint(1.0, std::numeric_limits<double>::infinity(),
                                y);
  prog.AddLinearEqualityConstraint(A_.transpose(), VectorXd::Zero(A_.cols()),
                                   y);
  auto result = solvers::Solve(prog);
  return result.is_success();
}

bool HPolyhedron::DoPointInSet(const Eigen::Ref<const VectorXd>& x,
                               double tol) const {
  DRAKE_DEMAND(A_.cols() == x.size());
  return ((A_ * x).array() <= b_.array() + tol).all();
}

void HPolyhedron::DoAddPointInSetConstraints(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) const {
  prog->AddLinearConstraint(
      A_,
      VectorXd::Constant(b_.size(), -std::numeric_limits<double>::infinity()),
      b_, vars);
}

std::vector<Binding<Constraint>>
HPolyhedron::DoAddPointInNonnegativeScalingConstraints(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorXDecisionVariable>& x,
    const Variable& t) const {
  std::vector<Binding<Constraint>> constraints;
  // A x ≤ t b, written as [A,-b][x;t] ≤ 0
  const int m = A_.rows();
  const int n = ambient_dimension();
  MatrixXd Abar(m, n + 1);
  Abar.leftCols(n) = A_;
  Abar.col(n) = -b_;
  constraints.emplace_back(prog->AddLinearConstraint(
      Abar, VectorXd::Constant(m, -std::numeric_limits<double>::infinity()),
      VectorXd::Zero(m), {x, Vector1<Variable>(t)}));
  return constraints;
}

std::vector<Binding<Constraint>>
HPolyhedron::DoAddPointInNonnegativeScalingConstraints(
    MathematicalProgram* prog, const Eigen::Ref<const MatrixXd>& A_x,
    const Eigen::Ref<const VectorXd>& b_x, const Eigen::Ref<const VectorXd>& c,
    double d, const Eigen::Ref<const VectorXDecisionVariable>& x,
    const Eigen::Ref<const VectorXDecisionVariable>& t) const {
  std::vector<Binding<Constraint>> constraints;
  // A (A_x x + b_x) ≤ (c' t + d) b, written as [A * A_x, -b * c'][x;t] ≤ d * b
  // - A * b_x
  const int m = A_.rows();
  MatrixXd A_bar(m, x.size() + t.size());
  A_bar.leftCols(x.size()) = A_ * A_x;
  A_bar.rightCols(t.size()) = -b_ * c.transpose();
  VectorXd b_bar = d * b_ - A_ * b_x;
  constraints.emplace_back(prog->AddLinearConstraint(
      A_bar, VectorXd::Constant(m, -std::numeric_limits<double>::infinity()),
      b_bar, {x, t}));
  return constraints;
}

std::pair<std::unique_ptr<Shape>, math::RigidTransformd>
HPolyhedron::DoToShapeWithPose() const {
  throw std::runtime_error(
      "ToShapeWithPose is not implemented yet for HPolyhedron.  Implementing "
      "this will likely require additional support from the Convex shape "
      "class (to support in-memory mesh data, or file I/O).");
}

void HPolyhedron::ImplementGeometry(const HalfSpace&, void* data) {
  auto* Ab = static_cast<std::pair<MatrixXd, VectorXd>*>(data);
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
  auto* Ab = static_cast<std::pair<MatrixXd, VectorXd>*>(data);
  Ab->first = A;
  Ab->second = b;
}

HPolyhedron HPolyhedron::PontryaginDifference(const HPolyhedron& other) const {
  /**
   * The Pontryagin set difference of Polytope P = {x | Ax <= b} and
   * Q = {x | Cx <= d} can be computed P - Q = {x | Ax <= b - h}
   * where hᵢ = max aᵢᵀx subject to x ∈ Q
   */

  DRAKE_DEMAND(this->ambient_dimension() == other.ambient_dimension());
  DRAKE_DEMAND(this->IsBounded());
  DRAKE_DEMAND(other.IsBounded());
  const double kInf = std::numeric_limits<double>::infinity();

  Eigen::VectorXd b_diff(b_.rows());
  MathematicalProgram prog;
  solvers::VectorXDecisionVariable x =
      prog.NewContinuousVariables(ambient_dimension_, "x");
  // -inf <= Ax <= b
  prog.AddLinearConstraint(other.A(),
                           Eigen::VectorXd::Constant(other.b().rows(), -kInf),
                           other.b(), x);

  auto result = solvers::Solve(prog);
  // other is an empty polyhedron and so Pontryagin difference does nothing
  if (result.get_solution_result() ==
      solvers::SolutionResult::kInfeasibleConstraints) {
    return {A_, b_};
  }

  Binding<solvers::LinearCost> program_cost_binding =
      prog.AddLinearCost(A_.row(0), 0, x);
  for (int i = 0; i < b_.rows(); ++i) {
    program_cost_binding.evaluator()->UpdateCoefficients(-A_.row(i), 0);
    result = solvers::Solve(prog);
    // since constraint set is bounded and non-empty then the program should
    // always have an optimal solution
    if (!result.is_success()) {
      throw std::runtime_error(fmt::format(
          "Solver {} failed to compute the set difference; it "
          "terminated with SolutionResult {}). This should only happen"
          "if the problem is ill-conditioned",
          result.get_solver_id().name(), result.get_solution_result()));
    }
    b_diff(i) = b_(i) + result.get_optimal_cost();
  }
  return {A_, b_diff};
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
