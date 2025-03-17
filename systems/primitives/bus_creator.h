#pragma once

#include <string>
#include <variant>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/bus_value.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/** This system packs values from heterogeneous input ports into a single output
port of type BusValue.

@system
name: BusCreator
input_ports:
- u0
- ...
- u(N-1)
output_ports:
- y0
@endsystem

The port names shown in the figure above are the defaults. Custom names may be
specified when setting up the %BusCreator.

When an input port is not connected, it is not an error; its value will simply
not appear as part of the BusValue on the output port.

@sa BusSelector, BusValue
@tparam_default_scalar
@ingroup primitive_systems */
template <typename T>
class BusCreator final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BusCreator);

  /** Constructs a %BusCreator with no inputs, and the given output port name.
  Use DeclareAbstractInputPort() and DeclareVectorInputPort() to add ports. */
  explicit BusCreator(std::variant<std::string, UseDefaultName>
                          output_port_name = kUseDefaultName);

  /** Scalar-converting copy constructor. See @ref system_scalar_conversion. */
  template <typename U>
  explicit BusCreator(const BusCreator<U>&);

  ~BusCreator() final;

  /** Declares a vector input port with the given attributes. The port name
  will also be used as the name of this signal in the BusValue output port.
  The type of the signal on the output bus will be `BasicVector<T>`. */
  InputPort<T>& DeclareVectorInputPort(
      std::variant<std::string, UseDefaultName> name, int size);

  /** Declares an abstract input port with the given attributes. The port name
  will also be used as the name of this signal in the BusValue output port. */
  InputPort<T>& DeclareAbstractInputPort(
      std::variant<std::string, UseDefaultName> name,
      const AbstractValue& model_value);

 private:
  template <typename>
  friend class BusCreator;

  void CalcOutput(const Context<T>& context, BusValue* output) const;
};

}  // namespace systems
}  // namespace drake
