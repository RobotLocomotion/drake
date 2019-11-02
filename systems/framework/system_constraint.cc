#include "drake/systems/framework/system_constraint.h"

namespace drake {
namespace systems {

namespace {
SystemConstraintType BoundsToType(
    const Eigen::Ref<const Eigen::VectorXd>& lower,
    const Eigen::Ref<const Eigen::VectorXd>& upper) {
  DRAKE_THROW_UNLESS(lower.size() == upper.size());
  DRAKE_THROW_UNLESS((lower.array() <= upper.array()).all());

  // If any single index in f(x)[i] has a different lower and upper bound, then
  // this is an inequality constraint.
  if ((lower.array() != upper.array()).any()) {
    return SystemConstraintType::kInequality;
  }

  // Otherwise, it's an equality constraint.  For now, we only allow b = 0.0.
  DRAKE_THROW_UNLESS((lower.array() == 0.0).all());
  DRAKE_THROW_UNLESS((upper.array() == 0.0).all());
  return SystemConstraintType::kEquality;
}
}  // namespace

SystemConstraintBounds::SystemConstraintBounds(
    const Eigen::Ref<const Eigen::VectorXd>& lower,
    const Eigen::Ref<const Eigen::VectorXd>& upper)
    : size_(lower.size()),
      type_(BoundsToType(lower, upper)),
      lower_(lower),
      upper_(upper) {}

SystemConstraintBounds::SystemConstraintBounds(
    const Eigen::Ref<const Eigen::VectorXd>& lower,
    std::nullopt_t)
    : SystemConstraintBounds(
          lower,
          Eigen::VectorXd::Constant(
              lower.size(), std::numeric_limits<double>::infinity())) {}

SystemConstraintBounds::SystemConstraintBounds(
    std::nullopt_t,
    const Eigen::Ref<const Eigen::VectorXd>& upper)
    : SystemConstraintBounds(
          Eigen::VectorXd::Constant(
              upper.size(), -std::numeric_limits<double>::infinity()),
          upper) {}

SystemConstraintBounds::SystemConstraintBounds(int size)
    : size_(size),
      type_(SystemConstraintType::kEquality),
      lower_(Eigen::VectorXd::Zero(size_)),
      upper_(Eigen::VectorXd::Zero(size_)) {}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::SystemConstraint)
