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

Hyperrectangle::Hyperrectangle() : ConvexSet(0) {}

Hyperrectangle::Hyperrectangle(const Eigen::Ref<const Eigen::VectorXd>& lb,
                               const Eigen::Ref<const Eigen::VectorXd>& ub)
    : ConvexSet(lb.size()), lb_(lb), ub_(ub) {
  CheckInvariants();
  set_has_exact_volume(true);
}

std::unique_ptr<ConvexSet> Hyperrectangle::DoClone() const {
  return std::make_unique<Hyperrectangle>(*this);
}

bool Hyperrectangle::DoIsBounded() const {
  // the finiteness of lb_ and ub_ is checked in the ctor.
  return true;
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

bool Hyperrectangle::DoPointInSet(const Eigen::Ref<const Eigen::VectorXd>& x,
                                  double tol) const {
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
