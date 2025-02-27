#pragma once

#include <string>
#include <variant>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/bus_value.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/** This system unpacks values from a single input port of type BusValue onto
heterogeneous output ports, based on the names of the signals on the bus.

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
specified when setting up the %BusCreator.

When an output port is evaluated but the input port's bus doesn't contain that
signal name, it is an error.

@sa BusCreator, BusValue
@tparam_default_scalar
@ingroup primitive_systems */
template <typename T>
class BusSelector final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BusSelector);

  /** Constructs a %BusSelector with ... */
  explicit BusSelector(std::variant<std::string, UseDefaultName>
                           input_port_name = kUseDefaultName);

  /** Scalar-converting copy constructor. See @ref system_scalar_conversion. */
  template <typename U>
  explicit BusSelector(const BusSelector<U>&);

  ~BusSelector() final;

  /** Declares a vector output port with the given attributes. The port name
  will be also used as the name of the signal to find in the BusValue input
  port. */
  OutputPort<T>& DeclareVectorOutputPort(
      std::variant<std::string, UseDefaultName> name, int size);

  /** Declares an abstract output port with the given attributes. The port name
  will be also used as the name of the signal to find in the BusValue input
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
