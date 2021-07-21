#include "drake/systems/framework/output_port.h"

namespace drake::systems {

template<typename T>
void OutputPort<T>::CheckValidAllocation(const AbstractValue& proposed) const {
  if (this->get_data_type() != kVectorValued)
    return;  // Nothing we can check for an abstract port.

  const auto* const proposed_vec = proposed.maybe_get_value<BasicVector<T>>();
  if (proposed_vec == nullptr) {
    throw std::logic_error(
        fmt::format("OutputPort::Allocate(): expected BasicVector output type "
                    "but got {} for {}.",
                    proposed.GetNiceTypeName(), GetFullDescription()));
  }

  const int proposed_size = proposed_vec->get_value().size();
  if (proposed_size != this->size()) {
    throw std::logic_error(
        fmt::format("OutputPort::Allocate(): expected vector output type of "
                    "size {} but got a vector of size {} for {}.",
                    this->size(), proposed_size, GetFullDescription()));
  }
}

}  // namespace drake::systems

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::OutputPort)
