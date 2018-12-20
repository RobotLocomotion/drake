#pragma once

#include <string>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_optional.h"
#include "drake/common/random.h"
#include "drake/systems/framework/framework_common.h"

namespace drake {
namespace systems {

class SystemBase;

/** An InputPort is a System resource that describes the kind of input a
System accepts, on a given port. It does not directly contain any runtime
input port data; that is always contained in a Context. The actual value will
be either the value of an OutputPort to which this is connected, or a fixed
value set in a Context.

%InputPortBase is the scalar type-independent part of an InputPort. */
class InputPortBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(InputPortBase)

  virtual ~InputPortBase();

  /** Returns the index of this input port within the owning System. For a
  Diagram, this will be the index within the Diagram, _not_ the index within
  a LeafSystem whose input port was exported. */
  InputPortIndex get_index() const { return index_; }

  /** Returns the DependencyTicket for this input port within the owning
  System. */
  DependencyTicket ticket() const {
    return ticket_;
  }

  /** Returns a reference to the SystemBase that owns this input port. Note that
  for a diagram input port this will be the diagram, not the leaf system whose
  input port was exported. */
  const SystemBase& get_system_base() const { return owning_system_; }

  /** Returns the port data type. */
  PortDataType get_data_type() const { return data_type_; }

  /** Returns the fixed size expected for a vector-valued input port. Not
  meaningful for abstract-valued input ports. */
  int size() const { return size_; }

  /** Returns true if this is a random port. */
  bool is_random() const { return static_cast<bool>(random_type_); }

  /** Returns the RandomDistribution if this is a random port. */
  optional<RandomDistribution> get_random_type() const { return random_type_; }

  /** Get port name. */
  const std::string& get_name() const { return name_; }

 protected:
  /** Provides derived classes the ability to set the base
  class members at construction.

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
  InputPortBase(SystemBase* owning_system, std::string name,
                InputPortIndex index, DependencyTicket ticket,
                PortDataType data_type, int size,
                const optional<RandomDistribution>& random_type);

 private:
  // Associated System and System resources.
  const SystemBase& owning_system_;
  const InputPortIndex index_;
  const DependencyTicket ticket_;

  // Port details.
  const PortDataType data_type_;
  const int size_;
  const std::string name_;
  const optional<RandomDistribution> random_type_;
};

}  // namespace systems
}  // namespace drake
