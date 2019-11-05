#pragma once

#include <optional>
#include <string>

#include "drake/common/random.h"
#include "drake/systems/framework/context_base.h"
#include "drake/systems/framework/framework_common.h"
#include "drake/systems/framework/port_base.h"

namespace drake {
namespace systems {

/** An InputPort is a System resource that describes the kind of input a
System accepts, on a given port. It does not directly contain any runtime
input port data; that is always contained in a Context. The actual value will
be either the value of an OutputPort to which this is connected, or a fixed
value set in a Context.

%InputPortBase is the scalar type-independent part of an InputPort. */
class InputPortBase : public PortBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(InputPortBase)

  ~InputPortBase() override;

  /** Returns the index of this input port within the owning System. For a
  Diagram, this will be the index within the Diagram, _not_ the index within
  a LeafSystem whose input port was exported. */
  InputPortIndex get_index() const { return InputPortIndex(get_int_index()); }

  /** Returns true if this is a random port. */
  bool is_random() const { return static_cast<bool>(random_type_); }

  /** Returns the RandomDistribution if this is a random port. */
  std::optional<RandomDistribution> get_random_type() const {
    return random_type_;
  }

  // A using-declaration adds these methods into our class's Doxygen.
  // (Placed in an order that makes sense for the class's table of contents.)
  using PortBase::get_name;
  using PortBase::GetFullDescription;
  using PortBase::get_data_type;
  using PortBase::size;
  using PortBase::ticket;

 protected:
  /** Signature of a function suitable for returning the cached value of a
  particular input port. Will return nullptr if the port is not connected. */
  using EvalAbstractCallback =
      std::function<const AbstractValue*(const ContextBase&)>;

  /** Provides derived classes the ability to set the base class members at
  construction.

  @param owning_system
    The System that owns this input port.
  @param name
    A name for the port. Input port names should be non-empty and unique
    within a single System.
  @param index
    The index to be assigned to this InputPort.
  @param ticket
    The DependencyTicket to be assigned to this InputPort.
  @param data_type
    Whether the port described is vector- or abstract-valued.
  @param size
    If the port described is vector-valued, the number of elements, or kAutoSize
    if determined by connections. Ignored for abstract-valued ports.
  @param random_type
    Input ports may optionally be labeled as random, if the port is intended to
    model a random-source "noise" or "disturbance" input. */
  InputPortBase(
      internal::SystemMessageInterface* owning_system, std::string name,
      InputPortIndex index, DependencyTicket ticket, PortDataType data_type,
      int size, const std::optional<RandomDistribution>& random_type,
      EvalAbstractCallback eval);

  /** Evaluate this port; throws an exception if the port is not connected. */
  const AbstractValue& DoEvalRequired(const ContextBase& context) const {
    const AbstractValue* const result = eval_(context);
    if (!result) { ThrowRequiredMissing(); }
    return *result;
  }

  /** Evaluate this port; returns nullptr if the port is not connected. */
  const AbstractValue* DoEvalOptional(const ContextBase& context) const {
    return eval_(context);
  }

  /** Throws an exception that this port is not connected, but was expected to
  be connected (i.e., an Eval caller expected that it was always connected). */
  [[noreturn]] void ThrowRequiredMissing() const;

 private:
  const EvalAbstractCallback eval_;
  const std::optional<RandomDistribution> random_type_;
};

}  // namespace systems
}  // namespace drake
