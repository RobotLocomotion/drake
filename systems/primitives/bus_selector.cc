#include "drake/systems/primitives/bus_selector.h"

#include <utility>

#include "drake/common/default_scalars.h"

namespace drake {
namespace systems {

template <typename T>
BusSelector<T>::BusSelector(
    std::variant<std::string, UseDefaultName> input_port_name)
    : LeafSystem<T>(SystemTypeTag<BusSelector>{}) {
  LeafSystem<T>::DeclareAbstractInputPort(std::move(input_port_name),
                                          Value<BusValue>{});
}

template <typename T>
template <typename U>
BusSelector<T>::BusSelector(const BusSelector<U>& other)
    : BusSelector(other.get_input_port().get_name()) {
  for (OutputPortIndex i{0}; i < other.num_output_ports(); ++i) {
    const auto& port = other.get_output_port(i, /* warn_deprecated = */ false);
    switch (port.get_data_type()) {
      case kAbstractValued: {
        BusSelector<T>::DeclareAbstractOutputPort(port.get_name(),
                                                  *port.Allocate());
        break;
      }
      case kVectorValued: {
        BusSelector<T>::DeclareVectorOutputPort(port.get_name(), port.size());
        break;
      }
    }
  }
}

template <typename T>
BusSelector<T>::~BusSelector() = default;

template <typename T>
OutputPort<T>& BusSelector<T>::DeclareVectorOutputPort(
    std::variant<std::string, UseDefaultName> name, int size) {
  const OutputPortIndex index{this->num_output_ports()};
  OutputPort<T>& result = LeafSystem<T>::DeclareVectorOutputPort(
      std::move(name), size,
      [this, index](const Context<T>& context, BasicVector<T>* output) {
        return this->CalcVectorOutput(context, index, output);
      },
      {this->all_input_ports_ticket()});
  DRAKE_DEMAND(result.get_index() == index);
  return result;
}

template <typename T>
OutputPort<T>& BusSelector<T>::DeclareAbstractOutputPort(
    std::variant<std::string, UseDefaultName> name,
    const AbstractValue& model_value) {
  const OutputPortIndex index{this->num_output_ports()};
  OutputPort<T>& result = LeafSystem<T>::DeclareAbstractOutputPort(
      std::move(name),
      [model = copyable_unique_ptr<AbstractValue>(model_value.Clone())]() {
        return model->Clone();
      },
      [this, index](const Context<T>& context, AbstractValue* output) {
        this->CalcAbstractOutput(context, index, output);
      },
      {this->all_input_ports_ticket()});
  DRAKE_DEMAND(result.get_index() == index);
  return result;
}

template <typename T>
void BusSelector<T>::CalcVectorOutput(const Context<T>& context,
                                      OutputPortIndex index,
                                      BasicVector<T>* output) const {
  const BusValue& input =
      this->get_input_port().template Eval<BusValue>(context);
  const std::string& name = this->get_output_port(index).get_name();
  const AbstractValue* value = input.Find(name);
  if (value == nullptr) {
    throw std::logic_error(
        fmt::format("Missing value for input signal {} on {}", name,
                    this->GetSystemPathname()));
  }
  const BasicVector<T>* vector = value->maybe_get_value<BasicVector<T>>();
  if (vector == nullptr) {
    throw std::logic_error(
        fmt::format("Wrong type (non-vector) for input signal {} on {}", name,
                    this->GetSystemPathname()));
  }
  output->SetFrom(*vector);
}

template <typename T>
void BusSelector<T>::CalcAbstractOutput(const Context<T>& context,
                                        OutputPortIndex index,
                                        AbstractValue* output) const {
  const BusValue& input =
      this->get_input_port().template Eval<BusValue>(context);
  const std::string& name = this->get_output_port(index).get_name();
  const AbstractValue* value = input.Find(name);
  if (value == nullptr) {
    throw std::logic_error(
        fmt::format("Missing value for input signal {} on {}", name,
                    this->GetSystemPathname()));
  }
  output->SetFrom(*value);
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::BusSelector);
