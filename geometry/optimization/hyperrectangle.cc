#include "drake/geometry/optimization/hyperrectangle.h"

#include <algorithm>
#include <bitset>
#include <limits>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>

#include <Eigen/Eigenvalues>
#include <fmt/format.h>

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

HyperRectangle::HyperRectangle() : ConvexSet(0) {}

HyperRectangle::HyperRectangle(const Eigen::Ref<const Eigen::VectorXd>& lb,
                               const Eigen::Ref<const Eigen::VectorXd>& ub)
    : ConvexSet(lb.size()), lb_(lb), ub_(ub) {}

std::optional<Eigen::VectorXd> HyperRectangle::DoMaybeGetPoint() const {
  return lb_;
}

std::optional<Eigen::VectorXd> HyperRectangle::DoMaybeGetFeasiblePoint() const {
  return (ub_ + lb_) / 2.0;
}

bool HyperRectangle::DoPointInSet(const Eigen::Ref<const Eigen::VectorXd>& x,
                                  double tol) const {
  return (x.array() >= lb_.array() - tol).all() &&
         (x.array() <= ub_.array() + tol).all();
}

Eigen::VectorXd HyperRectangle::UniformSample(
    RandomGenerator* generator) const {
  Eigen::VectorXd sample(ambient_dimension());
  for (int i = 0; i < lb_.size(); ++i) {
    std::uniform_real_distribution<double> distribution(lb_(i), ub_(i));
    sample(i) = distribution(*generator);
  }
  return sample;
}

std::vector<solvers::Binding<solvers::Constraint>>
HyperRectangle::DoAddPointInNonnegativeScalingConstraints(
    solvers::MathematicalProgram* prog,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& x,
    const symbolic::Variable& t) const {
  std::vector<solvers::Binding<solvers::Constraint>> constraints;
  const int n_d = x.rows();
  DRAKE_THROW_UNLESS(x.size() == ambient_dimension());
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
HyperRectangle::DoAddPointInNonnegativeScalingConstraints(
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
  A_con_lb.block(0, 0, n_rows, A.cols()) = A;
  A_con_ub.block(0, 0, n_rows, A.cols()) = A;
  for (int i = 0; i < c.rows(); ++i) {
    A_con_lb.col(A.cols() + i) = -lb_ * c(i);
    A_con_ub.col(A.cols() + i) = -ub_ * c(i);
  }
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
HyperRectangle::DoAddPointInSetConstraints(
    solvers::MathematicalProgram* prog,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& vars) const {
  std::vector<solvers::Binding<solvers::Constraint>> constraints;
  VectorX<symbolic::Variable> vars_sym;
  constraints.push_back(prog->AddBoundingBoxConstraint(lb_, ub_, vars));
  return {std::move(vars_sym), std::move(constraints)};
}

std::optional<HyperRectangle> ConvexSet::MaybeCalcAxisAlignedBoundingBox()
    const {
  solvers::MathematicalProgram prog;
  auto point = prog.NewContinuousVariables(ambient_dimension());
  AddPointInSetConstraints(&prog, point);
  std::vector<int> directions{-1, 1};
  Eigen::VectorXd cost_vector = Eigen::VectorXd::Zero(ambient_dimension());
  Eigen::VectorXd lb = Eigen::VectorXd::Zero(ambient_dimension());
  Eigen::VectorXd ub = Eigen::VectorXd::Zero(ambient_dimension());
  auto cost = prog.AddLinearCost(cost_vector.transpose(), 0.0, point);
  for (int i = 0; i < ambient_dimension(); i++) {
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
            "ConvexSet::MaybeCalcAxisAlignedBoundingBox(): Failed to solve the "
            "optimization problem. Maybe the set is unbounded?");
        return std::nullopt;
      }
      // reset the cost vector
      cost_vector(i) = 0.0;
    }
  }
  return HyperRectangle(lb, ub);
}

std::unique_ptr<ConvexSet> HyperRectangle::DoClone() const {
  return std::make_unique<HyperRectangle>(*this);
}

double HyperRectangle::DoVolume() const {
  return (ub_ - lb_).prod();
}

Eigen::VectorXd HyperRectangle::Center() const {
  return (ub_ + lb_) / 2.0;
}

HPolyhedron HyperRectangle::MakeHPolyhedron() const {
  return HPolyhedron::MakeBox(lb_, ub_);
}

std::pair<std::unique_ptr<Shape>, math::RigidTransformd>
HyperRectangle::DoToShapeWithPose() const {
  if (ambient_dimension() != 3) {
    throw std::runtime_error(
        "HyperRectangle::DoToShapeWithPose() is only implemented for "
        "ambient_dimension() == 3");
  }
  return std::make_pair(
      std::make_unique<geometry::Box>(ub_ - lb_),
      math::RigidTransformd(math::RotationMatrixd::Identity(), Center()));
}

void HyperRectangle::CheckInvariants() {
  DRAKE_THROW_UNLESS(lb_.size() == ub_.size());
  DRAKE_THROW_UNLESS((lb_.array() <= ub_.array()).all());
}

double ConvexSet::DoCalcVolumeViaSampling(RandomGenerator* generator,
                                          const double desired_rel_accuracy,
                                          const size_t min_num_samples,
                                          const size_t max_num_samples) const {
  DRAKE_THROW_UNLESS(desired_rel_accuracy <= 1.0);
  DRAKE_THROW_UNLESS(min_num_samples <= max_num_samples);
  const auto aabb_opt = this->MaybeCalcAxisAlignedBoundingBox();
  if (!aabb_opt.has_value()) {
    throw std::runtime_error("Cannot calculate volume of unbounded set.");
  }
  const HyperRectangle aabb = aabb_opt.value();
  size_t num_samples = 0;
  size_t num_hits = 0;
  double relative_accuracy = 1.0;
  while ((num_samples < min_num_samples ||
          relative_accuracy > desired_rel_accuracy) &&
         num_samples < max_num_samples) {
    auto point = aabb.UniformSample(generator);
    num_samples++;
    if (this->PointInSet(point)) {
      num_hits++;
    }
    const double mean = static_cast<double>(num_hits) / num_samples;
    if (mean > 0) {
      // The standard deviation of the MonteCarlo estimate is sigma/sqrt(n),
      // where sigma is the standard deviation of the Bernoulli distribution.
      // The standard deviation of the Bernoulli distribution is sqrt(p(1-p)),
      // where p is the probability of success.  Therefore, the standard
      // deviation of the estimate is sqrt((1-p)/p/n).  The relative accuracy is
      // the standard deviation divided by the mean, so the relative accuracy is
      // sqrt((1-p)/p/n)/p = sqrt((1-p)/p^2/n) = sqrt((1/p-1)/n).
      relative_accuracy = std::sqrt((1 - mean) / mean / num_samples);
    }
  }
  if (relative_accuracy > desired_rel_accuracy) {
    drake::log()->warn(
        "Volume calculation did not converge to desired accuracy {}.  "
        "Relative accuracy achieved: {}",
        desired_rel_accuracy, relative_accuracy);
  }
  return aabb.Volume() * num_hits / num_samples;
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
