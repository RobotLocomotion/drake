#include "drake/systems/framework/primitives/constant_vector_source.h"

#include "drake/common/drake_assert.h"
#include "drake/drakeSystemFramework_export.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_context.h"

namespace drake {
namespace systems {

template <typename T>
ConstantVectorSource<T>::ConstantVectorSource(
    const Eigen::Ref<const VectorX<T>>& source_value)
    : source_value_(source_value) {
  const int n = static_cast<int>(source_value.rows());
  this->DeclareOutputPort(kVectorValued, n, kContinuousSampling);
}

template <typename T>
void ConstantVectorSource<T>::EvalOutput(const Context<T>& context,
                                         SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));
  System<T>::GetMutableOutputVector(output, 0) = source_value_;
}

// Explicitly instantiates on the most common scalar types.
template class DRAKESYSTEMFRAMEWORK_EXPORT ConstantVectorSource<double>;

}  // namespace systems
}  // namespace drake
