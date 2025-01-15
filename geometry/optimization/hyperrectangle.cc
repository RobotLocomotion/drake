#include "drake/geometry/optimization/hyperrectangle.h"

#include <algorithm>
#include <array>
#include <bitset>
#include <limits>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>

#include <Eigen/Eigenvalues>
#include <fmt/format.h>

#include "drake/common/text_logging.h"
#include "drake/geometry/optimization/affine_subspace.h"
#include "drake/geometry/optimization/convex_set.h"
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

Hyperrectangle::Hyperrectangle() : ConvexSet(0, true) {}

Hyperrectangle::Hyperrectangle(const Eigen::Ref<const Eigen::VectorXd>& lb,
                               const Eigen::Ref<const Eigen::VectorXd>& ub)
    : ConvexSet(lb.size(), true), lb_(lb), ub_(ub) {
  CheckInvariants();
}

Hyperrectangle::~Hyperrectangle() = default;

std::optional<Hyperrectangle> Hyperrectangle::MaybeCalcAxisAlignedBoundingBox(
    const ConvexSet& set) {
  if (!set.IsBounded()) {
    return std::nullopt;
  }
  solvers::MathematicalProgram prog;
  int n = set.ambient_dimension();
  auto point = prog.NewContinuousVariables(n);
  set.AddPointInSetConstraints(&prog, point);
  std::array<int, 2> directions{-1, 1};
  Eigen::VectorXd cost_vector = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd lb = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd ub = Eigen::VectorXd::Zero(n);
  auto cost = prog.AddLinearCost(cost_vector.transpose(), 0.0, point);
  for (int i = 0; i < n; i++) {
    for (const auto direction : directions) {
      cost_vector(i) = static_cast<double>(direction);
      cost.evaluator()->UpdateCoefficients(cost_vector);
      auto result = solvers::Solve(prog);
      if (result.is_success()) {
        if (direction == 1) {
          lb(i) = result.get_optimal_cost();
        } else {
          ub(i) = -result.get_optimal_cost();
        }
      } else {
        drake::log()->warn(
            "Hyperrectangle::MaybeCalcAxisAlignedBoundingBox: Failed to solve "
            "the bounding box optimization problem. Maybe the set is unbounded "
            "in {} direction at dimension {}.",
            direction == 1 ? "negative" : "positive", i);
        return std::nullopt;
      }
      // reset the cost vector
      cost_vector(i) = 0.0;
    }
  }
  return Hyperrectangle(lb, ub);
}

std::unique_ptr<ConvexSet> Hyperrectangle::DoClone() const {
  return std::make_unique<Hyperrectangle>(*this);
}

std::optional<Eigen::VectorXd> Hyperrectangle::DoMaybeGetPoint() const {
  if (lb_ == ub_) {
    return lb_;
  }
  return std::nullopt;
}

std::optional<Eigen::VectorXd> Hyperrectangle::DoMaybeGetFeasiblePoint() const {
  return (ub_ + lb_) / 2.0;
}

std::optional<bool> Hyperrectangle::DoIsBoundedShortcut() const {
  return true;
}

std::optional<bool> Hyperrectangle::DoPointInSetShortcut(
    const Eigen::Ref<const Eigen::VectorXd>& x, double tol) const {
  return (x.array() >= lb_.array() - tol).all() &&
         (x.array() <= ub_.array() + tol).all();
}

Eigen::VectorXd Hyperrectangle::UniformSample(
    RandomGenerator* generator) const {
  Eigen::VectorXd sample(ambient_dimension());
  for (int i = 0; i < lb_.size(); ++i) {
    std::uniform_real_distribution<double> distribution(lb_(i), ub_(i));
    sample(i) = distribution(*generator);
  }
  return sample;
}

std::vector<solvers::Binding<solvers::Constraint>>
Hyperrectangle::DoAddPointInNonnegativeScalingConstraints(
    solvers::MathematicalProgram* prog,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& x,
    const symbolic::Variable& t) const {
  std::vector<solvers::Binding<solvers::Constraint>> constraints;
  const int n_d = x.rows();
  // Add constraints of the form x \in t * [lb, ub].
  // Can be written as:
  // [I, -lb][x,t] >= 0 and [I, -ub][x,t] <= 0
  Eigen::MatrixXd A_con_lb = Eigen::MatrixXd::Identity(n_d, n_d + 1);
  Eigen::MatrixXd A_con_ub = Eigen::MatrixXd::Identity(n_d, n_d + 1);
  A_con_lb.col(n_d) = -lb_;
  A_con_ub.col(n_d) = -ub_;
  const auto infinity_vector =
      VectorXd::Constant(n_d, std::numeric_limits<double>::infinity());
  constraints.push_back(prog->AddLinearConstraint(A_con_lb, VectorXd::Zero(n_d),
                                                  infinity_vector,
                                                  {x, Vector1<Variable>(t)}));
  constraints.push_back(prog->AddLinearConstraint(A_con_ub, -infinity_vector,
                                                  VectorXd::Zero(n_d),
                                                  {x, Vector1<Variable>(t)}));
  return constraints;
}

std::vector<solvers::Binding<solvers::Constraint>>
Hyperrectangle::DoAddPointInNonnegativeScalingConstraints(
    solvers::MathematicalProgram* prog,
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::VectorXd>& b,
    const Eigen::Ref<const Eigen::VectorXd>& c, double d,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& x,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& t) const {
  std::vector<solvers::Binding<solvers::Constraint>> constraints;
  // (c' * t + d) lb <= A * x + b <= (c' * t + d) ub
  // Can be written as:
  // A * x - c' * t * lb >= -b + d * lb and
  // A * x - c' * t * ub <= -b + d * ub
  // We need to compute A_con_lb = [A, - lb ⊗ c] and A_con_ub = [A, - ub ⊗ c]
  // where ⊗ is the outer product.
  const int n_rows = A.rows();
  const int n_cols = A.cols() + c.rows();
  Eigen::MatrixXd A_con_lb = Eigen::MatrixXd::Zero(n_rows, n_cols);
  Eigen::MatrixXd A_con_ub = Eigen::MatrixXd::Zero(n_rows, n_cols);
  A_con_lb.leftCols(A.cols()) = A;
  A_con_ub.leftCols(A.cols()) = A;
  A_con_lb.rightCols(c.rows()) = -lb_ * c.transpose();
  A_con_ub.rightCols(c.rows()) = -ub_ * c.transpose();
  const auto infinity_vector =
      VectorXd::Constant(n_rows, std::numeric_limits<double>::infinity());
  constraints.push_back(prog->AddLinearConstraint(A_con_ub, -infinity_vector,
                                                  -b + d * ub_, {x, t}));
  constraints.push_back(prog->AddLinearConstraint(A_con_lb, -b + d * lb_,
                                                  infinity_vector, {x, t}));
  return constraints;
}

std::pair<VectorX<symbolic::Variable>,
          std::vector<solvers::Binding<solvers::Constraint>>>
Hyperrectangle::DoAddPointInSetConstraints(
    solvers::MathematicalProgram* prog,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& vars) const {
  std::vector<solvers::Binding<solvers::Constraint>> constraints;
  VectorX<symbolic::Variable> vars_sym;
  constraints.push_back(prog->AddBoundingBoxConstraint(lb_, ub_, vars));
  return {std::move(vars_sym), std::move(constraints)};
}

Eigen::VectorXd Hyperrectangle::Center() const {
  return (ub_ + lb_) / 2.0;
}

HPolyhedron Hyperrectangle::MakeHPolyhedron() const {
  return HPolyhedron::MakeBox(lb_, ub_);
}

std::optional<Hyperrectangle> Hyperrectangle::MaybeGetIntersection(
    const Hyperrectangle& other) const {
  DRAKE_THROW_UNLESS(this->ambient_dimension() == other.ambient_dimension());
  if ((lb_.array() > other.ub_.array()).any() ||
      (ub_.array() < other.lb_.array()).any()) {
    return std::nullopt;
  }
  return Hyperrectangle(lb_.cwiseMax(other.lb_), ub_.cwiseMin(other.ub_));
}

std::pair<std::unique_ptr<Shape>, math::RigidTransformd>
Hyperrectangle::DoToShapeWithPose() const {
  if (ambient_dimension() != 3) {
    throw std::runtime_error(
        "Hyperrectangle::DoToShapeWithPose() is only implemented for "
        "ambient_dimension() == 3");
  }
  return std::make_pair(std::make_unique<geometry::Box>(ub_ - lb_),
                        math::RigidTransformd(Center()));
}

std::unique_ptr<ConvexSet> Hyperrectangle::DoAffineHullShortcut(
    std::optional<double> tol) const {
  MatrixXd basis = MatrixXd::Zero(ambient_dimension(), ambient_dimension());
  int current_dimension = 0;
  int num_basis_vectors = 0;
  for (int i = 0; i < ambient_dimension(); ++i) {
    // If the numerical tolerance was not specified, we use a reasonable
    // default.
    if (ub_[i] - lb_[i] > (tol ? tol.value() : 1e-12)) {
      basis(current_dimension, num_basis_vectors) = 1;
      ++num_basis_vectors;
    }
    ++current_dimension;
  }
  return std::make_unique<AffineSubspace>(basis.leftCols(num_basis_vectors),
                                          lb_);
}

double Hyperrectangle::DoCalcVolume() const {
  return (ub_ - lb_).prod();
}

void Hyperrectangle::CheckInvariants() {
  // only bounded hyperrectangles are supported.
  DRAKE_THROW_UNLESS(lb_.array().allFinite());
  DRAKE_THROW_UNLESS(ub_.array().allFinite());
  DRAKE_THROW_UNLESS(lb_.size() == ub_.size());
  DRAKE_THROW_UNLESS((lb_.array() <= ub_.array()).all());
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
