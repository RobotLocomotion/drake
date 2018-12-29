#include "drake/systems/framework/vector_base.h"

#include <limits>

#include "drake/common/default_scalars.h"

namespace drake {
namespace systems {
template <typename T>
void VectorBase<T>::SetInequalityConstraintBounds(
    const Eigen::Ref<const Eigen::VectorXd>& lower_bounds,
    const Eigen::Ref<const Eigen::VectorXd>& upper_bounds) {
  if (lower_bounds.size() != upper_bounds.size()) {
    throw std::invalid_argument(
        "VectorBase: lower_bounds and upper bounds don't have the same size.");
  }
  DRAKE_ASSERT((lower_bounds.array() < upper_bounds.array()).all());
  inequality_constraint_lower_bound_ = lower_bounds;
  inequality_constraint_upper_bound_ = upper_bounds;
}

template <typename T>
void VectorBase<T>::AppendInequalityConstraintBound(double lower_bound,
                                                    double upper_bound) {
  DRAKE_ASSERT(lower_bound < upper_bound);
  inequality_constraint_lower_bound_.conservativeResize(
      inequality_constraint_lower_bound_.size() + 1);
  inequality_constraint_lower_bound_(inequality_constraint_lower_bound_.size() -
                                     1) = lower_bound;
  inequality_constraint_upper_bound_.conservativeResize(
      inequality_constraint_upper_bound_.size() + 1);
  inequality_constraint_upper_bound_(inequality_constraint_upper_bound_.size() -
                                     1) = upper_bound;
}

template <typename T>
void VectorBase<T>::AppendInequalityConstraintLowerBound(double lower_bound) {
  AppendInequalityConstraintBound(lower_bound,
                                  std::numeric_limits<double>::infinity());
}

template <typename T>
void VectorBase<T>::AppendInequalityConstraintUpperBound(double upper_bound) {
  AppendInequalityConstraintBound(-std::numeric_limits<double>::infinity(),
                                  upper_bound);
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::VectorBase)
