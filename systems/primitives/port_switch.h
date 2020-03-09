#pragma once

#include <memory>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/unused.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/**
A simple system that passes through the value from just one of its input
ports to the output.  All inputs (except for the port_selector) must have
the same data type as the output.

This system only evaluates the port_selector port and the input port that is
indicated by port_selector at runtime. Because of the System framework's "pull
architecture", this means that entire sub-diagrams can potentially be added
with minimal runtime cost (their outputs will not be evaluated until they are
selected). Just remember that their state dynamics *will* still be evaluated
when the diagram's dynamics are evaluated (e.g. during simulation), and their
output ports could be evaluated via other connections.

@system{PortSwitch,
 @input_port{port_selector}
 @input_port{value0 (with assigned port name)}
 @inputport{...}
 @inputport{valueN (with assigned port name)},
 @output_port{value}
}

@tparam_default_scalar
*/
template <typename T>
class PortSwitch final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PortSwitch)

  /** Constructs a vector-valued %PortSwitch. All input ports declared via
  DeclareInputPort() will be vector-valued ports of size `vector_size`,
  which must be greater than zero. */
  explicit PortSwitch(int vector_size);

  /** Constructs a %PortSwitch using the type of `model_value` as the model for
  the output port.  All input ports declared via DeclareInputPort() will be
  abstract-valued ports using the same `model_value`. */
  template <typename OutputType>
  explicit PortSwitch(const OutputType& model_value)
      : PortSwitch(-1, AbstractValue::Make<OutputType>(model_value), nullptr,
                   nullptr) {}

  /** Constructs a %PortSwitch using the type of `dummy_value` as the model for
  the output port.  This version provides support for input/output values
  that are templated on scalar type; the scalar type on the port is kept
  consistent with the scalar type of the System (even through scalar
  conversion). The type of `dummy_value` must be default constructible.  */
  template <template <typename> class OutputType>
  explicit PortSwitch(const OutputType<T>& dummy_value)
      : PortSwitch(
            -1, std::unique_ptr<AbstractValue>(new Value<OutputType<double>>()),
            std::unique_ptr<AbstractValue>(new Value<OutputType<AutoDiffXd>>()),
            std::unique_ptr<AbstractValue>(
                new Value<OutputType<symbolic::Expression>>())) {
    unused(dummy_value);
  }

  /** Scalar-type converting copy constructor. See @ref
  system_scalar_conversion. */
  template <typename U>
  explicit PortSwitch(const PortSwitch<U>& other);

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
  // Shared constructor.  Either vector_size > 0 or model_value_double !=
  // nullptr.  If model_value_autodiff or model_value_symbolic are nullptr,
  // then they are set to model_value_double.
  PortSwitch(int vector_size,
             const std::shared_ptr<const AbstractValue> model_value_double,
             const std::shared_ptr<const AbstractValue> model_value_autodiff,
             const std::shared_ptr<const AbstractValue> model_value_symbolic);

  const AbstractValue& get_model_value() const;

  // Copies the vector from the selected input port to the output.
  void CopyVectorOut(const Context<T>& context, BasicVector<T>* vector) const;

  // Copies the abstract value from the selected input port to the output.
  void CopyValueOut(const Context<T>& context, AbstractValue* value) const;

  // Allow different specializations to access each other's private data.
  template <typename U>
  friend class PortSwitch;

  const int vector_size_{-1};

  // In order to support scalar conversion of templated abstract values, we
  // keep around copies of the model value for each potential scalar type. But
  // scalar conversion should also work when the abstract values are not
  // templated on scalar type; in this case all of the model_values below will
  // point to the same value (justifying the use of shared_ptr).  Make sure to
  // keep these const because it is also easy/convenient to share these values
  // even across scalar-converted copies of the system.
  const std::shared_ptr<const AbstractValue> model_value_double_;
  const std::shared_ptr<const AbstractValue> model_value_autodiff_;
  const std::shared_ptr<const AbstractValue> model_value_symbolic_;
};

// Declare explicit specializations.
template <>
const AbstractValue& PortSwitch<double>::get_model_value() const;
template <>
const AbstractValue& PortSwitch<AutoDiffXd>::get_model_value() const;
template <>
const AbstractValue& PortSwitch<symbolic::Expression>::get_model_value() const;

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::PortSwitch)
