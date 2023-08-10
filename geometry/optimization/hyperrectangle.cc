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

HyperRectangle::HyperRectangle() : HPolyhedron() {}

HyperRectangle::HyperRectangle(const Eigen::Ref<const Eigen::VectorXd>& lb,
                               const Eigen::Ref<const Eigen::VectorXd>& ub)
    : HPolyhedron(HPolyhedron::MakeBox(lb, ub)), lb_(lb), ub_(ub) {}

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

// std::vector<solvers::Binding<solvers::Constraint>>
// HyperRectangle::DoAddPointInNonnegativeScalingConstraints(
//     solvers::MathematicalProgram* prog,
//     const Eigen::Ref<const solvers::VectorXDecisionVariable>& x,
//     const symbolic::Variable& t) const {
//   std::vector<solvers::Binding<solvers::Constraint>> constraints;
//   const int n_rows = x.rows();
//   DRAKE_THROW_UNLESS(x.size() == lb_.size());
//   Eigen::MatrixXd A_con_lower = Eigen::MatrixXd::Identity(n_rows, n_rows +
//   1); Eigen::MatrixXd A_con_upper = Eigen::MatrixXd::Identity(n_rows, n_rows
//   + 1); A_con_lower.col(n_rows) = -lb_; A_con_upper.col(n_rows) = -ub_;
//   constraints.push_back(prog->AddLinearConstraint(
//       A_con_lower,
//       VectorXd::Constant(n_rows, -std::numeric_limits<double>::infinity()),
//       VectorXd::Zero(n_rows), {x, Vector1<Variable>(t)}));
//   constraints.push_back(prog->AddLinearConstraint(
//       A_con_upper,
//       VectorXd::Constant(n_rows, std::numeric_limits<double>::infinity()),
//       VectorXd::Zero(n_rows), {x, Vector1<Variable>(t)}));
//   return constraints;
// }

// std::pair<VectorX<symbolic::Variable>,
//           std::vector<solvers::Binding<solvers::Constraint>>>
// HyperRectangle::DoAddPointInSetConstraints(
//     solvers::MathematicalProgram* prog,
//     const Eigen::Ref<const solvers::VectorXDecisionVariable>& vars) const {
//   std::vector<solvers::Binding<solvers::Constraint>> constraints;
//   VectorX<symbolic::Variable> vars_sym(vars.size());
//   constraints.push_back(
//       prog->AddBoundingBoxConstraint(lb_, ub_, vars));
//   return {std::move(vars_sym), std::move(constraints)};
// }

std::optional<HyperRectangle> ConvexSet::MaybeCalcAxisAlignedBoundingBox()
    const {
  solvers::MathematicalProgram prog;
  auto point = prog.NewContinuousVariables(ambient_dimension());
  AddPointInSetConstraints(&prog, point);
  std::vector<int> directions{-1, 1};
  Eigen::VectorXd cost_vector = Eigen::VectorXd::Zero(ambient_dimension());
  Eigen::VectorXd lb{cost_vector};
  Eigen::VectorXd ub{cost_vector};
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
            "optimization problem. Maybe an unbounded set?");
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
      relative_accuracy = std::sqrt((1 - mean) / mean / num_samples);
    }
  }
  if (relative_accuracy > desired_rel_accuracy) {
    spdlog::warn(
        "Volume calculation did not converge to desired accuracy {}.  "
        "Relative accuracy achieved: {}",
        desired_rel_accuracy, relative_accuracy);
  }
  return aabb.Volume() * num_hits / num_samples;
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
