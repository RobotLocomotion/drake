#include "drake/systems/primitives/wrap_to_system.h"

#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/math/wrap_to.h"

namespace drake {
namespace systems {

template <typename T>
WrapTo<T>::WrapTo(int input_size) : input_size_(input_size) {
  DRAKE_DEMAND(input_size_ > 0);

  this->DeclareInputPort(kVectorValued, input_size_);
  this->DeclareVectorOutputPort(BasicVector<T>(input_size_),
                                &WrapTo::CalcWrappedOutput);
}

template <typename T>
void WrapTo<T>::set_interval(int index, const T& low, const T& high) {
  DRAKE_DEMAND(index >= 0 && index < input_size_);
  DRAKE_DEMAND(high > low);
  intervals_[index] = std::make_pair(low, high);
}

template <typename T>
void WrapTo<T>::CalcWrappedOutput(const Context<T>& context,
                                  BasicVector<T>* output) const {
  const VectorX<T> input = this->EvalVectorInput(context, 0)->CopyToVector();
  output->SetFromVector(input);

  // Loop through and set the saturation values.
  for (const auto& interval : intervals_) {
    const int index = interval.first;
    const T& low = interval.second.first;
    const T& high = interval.second.second;
    output->SetAtIndex(index, math::wrap_to(input[index], low, high));
  }
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::WrapTo)
