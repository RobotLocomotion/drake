#include "drake/systems/primitives/bus_creator.h"

#include <utility>

#include "drake/common/default_scalars.h"

namespace drake {
namespace systems {

template <typename T>
BusCreator<T>::BusCreator(
    std::variant<std::string, UseDefaultName> output_port_name)
    : LeafSystem<T>(SystemTypeTag<BusCreator>{}) {
  LeafSystem<T>::DeclareAbstractOutputPort(
      std::move(output_port_name), BusValue{}, &BusCreator<T>::CalcOutput,
      {this->all_input_ports_ticket()});
}

template <typename T>
template <typename U>
BusCreator<T>::BusCreator(const BusCreator<U>& other)
    : BusCreator(other.get_output_port().get_name()) {
  for (InputPortIndex i{0}; i < other.num_input_ports(); ++i) {
    const auto& port = other.get_input_port(i, /* warn_deprecated = */ false);
    switch (port.get_data_type()) {
      case kAbstractValued: {
        BusCreator<T>::DeclareAbstractInputPort(port.get_name(),
                                                *port.Allocate());
        break;
      }
      case kVectorValued: {
        BusCreator<T>::DeclareVectorInputPort(port.get_name(), port.size());
        break;
      }
    }
  }
}

template <typename T>
BusCreator<T>::~BusCreator() = default;

template <typename T>
InputPort<T>& BusCreator<T>::DeclareVectorInputPort(
    std::variant<std::string, UseDefaultName> name, int size) {
  return LeafSystem<T>::DeclareVectorInputPort(std::move(name), size);
}

template <typename T>
InputPort<T>& BusCreator<T>::DeclareAbstractInputPort(
    std::variant<std::string, UseDefaultName> name,
    const AbstractValue& model_value) {
  return LeafSystem<T>::DeclareAbstractInputPort(std::move(name), model_value);
}

template <typename T>
void BusCreator<T>::CalcOutput(const Context<T>& context,
                               BusValue* output) const {
  output->Clear();
  for (InputPortIndex i{0}; i < this->num_input_ports(); ++i) {
    const auto& port = this->get_input_port(i, /* warn_deprecated = */ false);
    if (port.HasValue(context)) {
      const AbstractValue& value = port.template Eval<AbstractValue>(context);
      output->Set(port.get_name(), value);
    }
  }
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::BusCreator);
