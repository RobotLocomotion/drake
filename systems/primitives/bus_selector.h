#pragma once

#include <string>
#include <variant>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/bus_value.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/** This system unpacks values from a single input port of type BusValue onto
heterogeneous output ports, where each output port's value comes from the same-
named signal on the bus.

The value on input port may contain additional bus value entries which are not
selected (because there is no output port with that name); this is not an error.

@system
name: BusSelector
input_ports:
- u0
output_ports:
- y0
- ...
- y(N-1)
@endsystem

The port names shown in the figure above are the defaults. Custom names may be
specified when setting up the %BusSelector.

When an output port is evaluated but the input port's bus doesn't contain that
signal name, it is an error.

Because of the all-encompassing nature of AbstractValue, a vector-valued bus
signal can be output on an abstract-valued port. Of course, the vector value can
still be retrieved from the %AbstractValue. However, we recommend declaring
output ports as vector-valued when the corresponding input signal is known to be
vector-valued to maximize utility and minimize surprise.

@sa BusCreator, BusValue
@tparam_default_scalar
@ingroup primitive_systems */
template <typename T>
class BusSelector final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BusSelector);

  /** Constructs a %BusSelector with the given input port name, and no outputs.
  Use DeclareVectorOutputPort() and DeclareAbstractOutputPort() to add ports. */
  explicit BusSelector(std::variant<std::string, UseDefaultName>
                           input_port_name = kUseDefaultName);

  /** Scalar-converting copy constructor. See @ref system_scalar_conversion. */
  template <typename U>
  explicit BusSelector(const BusSelector<U>&);

  ~BusSelector() final;

  /** Declares a vector output port with the given attributes. The port name
  will also be used as the name of the signal to find in the BusValue input
  port.  The type of the signal on the input bus must be `BasicVector<T>`. */
  OutputPort<T>& DeclareVectorOutputPort(
      std::variant<std::string, UseDefaultName> name, int size);

  /** Declares an abstract output port with the given attributes. The port name
  will also be used as the name of the signal to find in the BusValue input
  port. */
  OutputPort<T>& DeclareAbstractOutputPort(
      std::variant<std::string, UseDefaultName> name,
      const AbstractValue& model_value);

 private:
  template <typename>
  friend class BusSelector;

  void CalcVectorOutput(const Context<T>& context, OutputPortIndex index,
                        BasicVector<T>* output) const;

  void CalcAbstractOutput(const Context<T>& context, OutputPortIndex index,
                          AbstractValue* output) const;
};

}  // namespace systems
}  // namespace drake
