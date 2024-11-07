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

namespace {

int GetAmbientDimension(const ConvexSets& sets) {
  if (sets.empty()) {
    return 0;
  }
  const int ambient_dimension = sets[0]->ambient_dimension();
  for (const copyable_unique_ptr<ConvexSet>& set : sets) {
    DRAKE_THROW_UNLESS(set != nullptr);
    DRAKE_THROW_UNLESS(set->ambient_dimension() == ambient_dimension);
  }
  return ambient_dimension;
}

}  // namespace

Intersection::Intersection() : Intersection(ConvexSets{}) {}

Intersection::Intersection(const ConvexSets& sets)
    : ConvexSet(GetAmbientDimension(sets), false), sets_(sets) {}

Intersection::Intersection(const ConvexSet& setA, const ConvexSet& setB)
    : ConvexSet(setA.ambient_dimension(), false) {
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

std::optional<bool> Intersection::DoIsBoundedShortcutParallel(
    Parallelism parallelism) const {
  for (const auto& s : sets_) {
    if (s->IsBounded(parallelism)) {
      return true;
    }
  }
  return std::nullopt;
}

bool Intersection::DoIsEmpty() const {
  if (sets_.size() == 0) {
    return false;
  }
  // The empty set is annihilatory in intersection.
  for (const auto& s : sets_) {
    if (s->IsEmpty()) {
      return true;
    }
  }
  // Now actually see if the intersection is nonempty.
  return ConvexSet::DoIsEmpty();
}

std::optional<VectorXd> Intersection::DoMaybeGetPoint() const {
  std::optional<VectorXd> result;
  for (const auto& s : sets_) {
    if (std::optional<VectorXd> point = s->MaybeGetPoint()) {
      if (result.has_value() && !(*point == *result)) {
        return std::nullopt;
      }
      result = std::move(point);
    } else {
      return std::nullopt;
    }
  }
  return result;
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

std::pair<VectorX<Variable>, std::vector<Binding<Constraint>>>
Intersection::DoAddPointInSetConstraints(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorXDecisionVariable>& x) const {
  std::vector<Variable> new_vars;
  std::vector<Binding<Constraint>> new_constraints;
  for (const auto& s : sets_) {
    const auto [new_vars_in_s, new_constraints_in_s] =
        s->AddPointInSetConstraints(prog, x);
    for (int i = 0; i < new_vars_in_s.rows(); ++i) {
      new_vars.push_back(new_vars_in_s(i));
    }
    new_constraints.insert(new_constraints.end(), new_constraints_in_s.begin(),
                           new_constraints_in_s.end());
  }
  VectorX<Variable> new_vars_vec =
      Eigen::Map<VectorX<Variable>>(new_vars.data(), new_vars.size());
  return {std::move(new_vars_vec), std::move(new_constraints)};
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
