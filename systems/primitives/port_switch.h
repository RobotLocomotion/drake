#pragma once

#include <memory>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/**
A simple system that passes through the value from just one of its input
ports to the output.  All inputs (except for the port_selector) must have
the same data type as the output.

Because of the System frameworks' "pull architecture", this system only
triggers the evaluation of the port_selector port and the output port that is
selected; so entire sub-diagrams can potentially be added with minimal
runtime cost.  Just remember that their state dynamics *will* still be
evaluated, and their output ports could be evaluated via other connections.

@system{PortSwitch,
 @input_port{port_selector}
 @input_port{value0 (with assigned port name)}
 @inputport{...}
 @inputport{valueN (with assigned port name)},
 @output_port{value}
}

Instantiated templates for the following scalar types @p T are provided:

 - double
 - AutoDiffXd
 - symbolic::Expression
  */
template <typename T>
class PortSwitch final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PortSwitch)

  /** Constructs the system with the output port, and all input ports (declared
  via DeclareInputPort) except the port_selector declared as vector-valued
  ports of size `vector_size`. */
  explicit PortSwitch(int vector_size);

  /** Constructs the system using `model_value` as the model for the output
  port, and all input ports (declared via DeclareInputPort) except the
  port_selector. `OutputType` must be such that `Value<OutputType>` is
  permitted. */
  template <typename OutputType>
  explicit PortSwitch(const OutputType& model_value)
      : PortSwitch(-1, AbstractValue::Make<OutputType>(model_value)) {}

  /** Scalar-type converting copy constructor. See @ref
  system_scalar_conversion. */
  template <typename U>
  explicit PortSwitch(const PortSwitch<U>& other);

  virtual ~PortSwitch() = default;

  const InputPort<T>& get_port_selector_input_port() const {
    return this->get_input_port(0);
  }

  const OutputPort<T>& get_output_port() const {
    return System<T>::get_output_port(0);
  }

  /** Declares a new input port to the switch with port name `name`.  The
  type of this port is already defined by the type of the output port. This
  must be called before any Context is allocated. */
  const InputPort<T>& DeclareInputPort(std::string name);

 private:
  PortSwitch(int vector_size, std::unique_ptr<const AbstractValue> model_value);

  // Copies the vector from the selected input port to the output.
  void CopyVectorOut(const Context<T>& context, BasicVector<T>* vector) const;

  // Copies the abstract value from the selected input port to the output.
  void CopyValueOut(const Context<T>& context, AbstractValue* value) const;

  // Allow different specializations to access each other's private data.
  template <typename U>
  friend class PortSwitch;

  const int vector_size_{-1};
  const std::unique_ptr<const AbstractValue> model_value_;
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::PortSwitch)
