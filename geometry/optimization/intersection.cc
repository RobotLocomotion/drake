#include "drake/geometry/optimization/intersection.h"

#include <memory>

#include <fmt/format.h>

namespace drake {
namespace geometry {
namespace optimization {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using solvers::Binding;
using solvers::Constraint;
using solvers::MathematicalProgram;
using solvers::VectorXDecisionVariable;
using symbolic::Variable;

Intersection::Intersection(const ConvexSets& sets)
    : ConvexSet(sets[0]->ambient_dimension()), sets_{sets} {
  for (int i = 1; i < ssize(sets_); ++i) {
    DRAKE_THROW_UNLESS(sets_[i]->ambient_dimension() ==
                       sets_[0]->ambient_dimension());
  }
}

Intersection::Intersection(const ConvexSet& setA, const ConvexSet& setB)
    : ConvexSet(setA.ambient_dimension()) {
  DRAKE_THROW_UNLESS(setB.ambient_dimension() == setA.ambient_dimension());
  sets_.emplace_back(setA.Clone());
  sets_.emplace_back(setB.Clone());
}

Intersection::~Intersection() = default;

const ConvexSet& Intersection::element(int index) const {
  DRAKE_THROW_UNLESS(0 <= index && index < ssize(sets_));
  return *sets_[index];
}

std::unique_ptr<ConvexSet> Intersection::DoClone() const {
  return std::make_unique<Intersection>(*this);
}

bool Intersection::DoIsBounded() const {
  for (const auto& s : sets_) {
    if (s->IsBounded()) {
      return true;
    }
  }
  // TODO(mpetersen94): Cover case where sets are unbounded but intersection is
  // bounded.
  throw std::runtime_error(
      "Determining the boundedness of an Intersection made up of unbounded "
      "elements is not currently supported.");
}

bool Intersection::DoPointInSet(const Eigen::Ref<const VectorXd>& x,
                                double tol) const {
  for (const auto& s : sets_) {
    if (!s->PointInSet(x, tol)) {
      return false;
    }
  }
  return true;
}

void Intersection::DoAddPointInSetConstraints(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorXDecisionVariable>& x) const {
  for (const auto& s : sets_) {
    s->AddPointInSetConstraints(prog, x);
  }
}

std::vector<Binding<Constraint>>
Intersection::DoAddPointInNonnegativeScalingConstraints(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorXDecisionVariable>& x,
    const Variable& t) const {
  std::vector<Binding<Constraint>> constraints;
  for (const auto& s : sets_) {
    auto new_constraints =
        s->AddPointInNonnegativeScalingConstraints(prog, x, t);
    constraints.insert(constraints.end(),
                       std::make_move_iterator(new_constraints.begin()),
                       std::make_move_iterator(new_constraints.end()));
  }
  return constraints;
}

std::vector<Binding<Constraint>>
Intersection::DoAddPointInNonnegativeScalingConstraints(
    MathematicalProgram* prog, const Eigen::Ref<const MatrixXd>& A,
    const Eigen::Ref<const VectorXd>& b, const Eigen::Ref<const VectorXd>& c,
    double d, const Eigen::Ref<const VectorXDecisionVariable>& x,
    const Eigen::Ref<const VectorXDecisionVariable>& t) const {
  std::vector<Binding<Constraint>> constraints;
  for (const auto& s : sets_) {
    auto new_constraints =
        s->AddPointInNonnegativeScalingConstraints(prog, A, b, c, d, x, t);
    constraints.insert(constraints.end(),
                       std::make_move_iterator(new_constraints.begin()),
                       std::make_move_iterator(new_constraints.end()));
  }
  return constraints;
}

std::pair<std::unique_ptr<Shape>, math::RigidTransformd>
Intersection::DoToShapeWithPose() const {
  throw std::runtime_error(
      "ToShapeWithPose is not implemented yet for Intersection.");
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
