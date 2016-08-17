#include "drake/systems/framework/primitives/constant_vector_source.h"

#include "drake/common/drake_assert.h"
#include "drake/drakeSystemFramework_export.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace systems {

template <typename T>
ConstantVectorSource<T>::ConstantVectorSource(
    const Eigen::Ref<const VectorX<T>>& source_value) :
    source_value_(source_value) {
  this->DeclareOutputPort(
      kVectorValued, source_value.rows(), kContinuousSampling);
}

template <typename T>
void ConstantVectorSource<T>::EvalOutput(const ContextBase<T>& context,
                                         SystemOutput<T>* output) const {
  // Checks on the output structure are assertions, not exceptions, since
  // failures would reflect a bug in the ConstantVectorSource implementation,
  // not user error setting up the system graph. They do not require unit test
  // coverage, and should not run in release builds.

  DRAKE_ASSERT(System<T>::IsValidOutput(*output));
  DRAKE_ASSERT(System<T>::IsValidContext(context));

  System<T>::GetMutableOutputVector(output, 0) = source_value_;
}

// Explicitly instantiates on the most common scalar types.
template class DRAKESYSTEMFRAMEWORK_EXPORT ConstantVectorSource<double>;

}  // namespace systems
}  // namespace drake
