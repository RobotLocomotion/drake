#include "drake/systems/primitives/wrap_to_system.h"

#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/math/wrap_to.h"

namespace drake {
namespace systems {

template <typename T>
WrapToSystem<T>::WrapToSystem(int input_size) : input_size_(input_size) {
  DRAKE_DEMAND(input_size_ > 0);

  this->DeclareInputPort(kUseDefaultName, kVectorValued, input_size_);
  this->DeclareVectorOutputPort(kUseDefaultName, input_size_,
                                &WrapToSystem::CalcWrappedOutput);
}

template <typename T>
void WrapToSystem<T>::set_interval(int index, const T& low, const T& high) {
  DRAKE_DEMAND(index >= 0 && index < input_size_);
  DRAKE_DEMAND(high > low);
  intervals_[index] = Interval{low, high};
}

template <typename T>
void WrapToSystem<T>::CalcWrappedOutput(const Context<T>& context,
                                  BasicVector<T>* output) const {
  const auto& input = this->get_input_port(0).Eval(context);
  output->SetFromVector(input);

  // Loop through and set the saturation values.
  for (const auto& index_interval_pair : intervals_) {
    const int index = index_interval_pair.first;
    const Interval& v = index_interval_pair.second;
    output->SetAtIndex(index, math::wrap_to(input[index], v.low, v.high));
  }
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::WrapToSystem)
