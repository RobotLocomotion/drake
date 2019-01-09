#include "drake/systems/framework/system_constraint.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace systems {

SystemConstraintBounds::SystemConstraintBounds(
    const Eigen::Ref<const Eigen::VectorXd>& lower,
    const Eigen::Ref<const Eigen::VectorXd>& upper)
    : size_(lower.size()),
      type_((lower.isZero(0.0) && upper.isZero(0.0)) ?
            SystemConstraintType::kEquality :
            SystemConstraintType::kInequality),
      lower_(lower),
      upper_(upper) {
  DRAKE_THROW_UNLESS(lower_.size() == upper_.size());
  DRAKE_THROW_UNLESS((lower_.array() <= upper_.array()).all());
}

SystemConstraintBounds::SystemConstraintBounds(
    const Eigen::Ref<const Eigen::VectorXd>& lower,
    stx::nullopt_t)
    : SystemConstraintBounds(
          lower,
          Eigen::VectorXd::Constant(
              lower.size(), std::numeric_limits<double>::infinity())) {}

SystemConstraintBounds::SystemConstraintBounds(
    stx::nullopt_t,
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
