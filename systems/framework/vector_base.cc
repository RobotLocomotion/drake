#include "drake/systems/framework/vector_base.h"

#include <limits>

#include "drake/common/default_scalars.h"

namespace drake {
namespace systems {
template <typename T>
bool IsLowerBoundSmaller(const T& lower_bound, const T& upper_bound) {
  return lower_bound <= upper_bound;
}

template <>
bool IsLowerBoundSmaller<symbolic::Expression>(
    const symbolic::Expression& lower_bound,
    const symbolic::Expression& upper_bound) {
  return !is_false(lower_bound <= upper_bound);
}

template <typename T>
void VectorBase<T>::AppendInequalityConstraintBounds(const T& lower_bound,
                                                     const T& upper_bound) {
  DRAKE_ASSERT(IsLowerBoundSmaller(lower_bound, upper_bound));
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
void VectorBase<T>::AppendInequalityConstraintLowerBound(const T& lower_bound) {
  AppendInequalityConstraintBounds(
      lower_bound, static_cast<T>(std::numeric_limits<double>::infinity()));
}

template <typename T>
void VectorBase<T>::AppendInequalityConstraintUpperBound(const T& upper_bound) {
  AppendInequalityConstraintBounds(
      static_cast<T>(-std::numeric_limits<double>::infinity()), upper_bound);
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::VectorBase)
