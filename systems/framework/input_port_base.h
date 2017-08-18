#pragma once

#include "drake/common/constants.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_optional.h"
#include "drake/systems/framework/framework_common.h"

namespace drake {
namespace systems {

class SystemBase;

/** An InputPort is a System resource describes the kind of input a
System accepts, on a given port. It is not a mechanism for handling any
actual input data; that is handled by a corresponding runtime object in
the Context for that System.

%InputPortBase is the scalar type-independent part of an InputPort. */
class InputPortBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(InputPortBase)

  /** (Internal use only) Provides derived classes the ability to set the base
  class members at construction. Assigns a DependencyTicket.

  @param index
    The index to be assigned to this InputPort.
  @param data_type
    Whether the port described is vector or abstract valued.
  @param size
    If the port described is vector-valued, the number of elements, or kAutoSize
    if determined by connections.
  @param random_type Input ports may optionally be labeled as random, if the
                     port is intended to model a random-source "noise" or
                     "disturbance" input.
  @param system
    The System that will own this new input port. This port will be assigned the
    next available input port index in this system, and the next available
    dependency ticket. */
  InputPortBase(InputPortIndex index, PortDataType data_type, int size,
                const optional<RandomDistribution>& random_type,
                SystemBase* system);

  virtual ~InputPortBase() {}

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
  const SystemBase& get_system_base() const { return system_; }

  /** Returns the port data type specified at port construction. */
  PortDataType get_data_type() const { return data_type_; }

  /** Returns the fixed size expected for a vector-valued input port. Not
  meaningful for abstract input ports. */
  int size() const { return size_; }

  /** Returns true if this is a random port. */
  bool is_random() const { return static_cast<bool>(random_type_); }

  /** Returns the RandomDistribution if this is a random port. */
  optional<RandomDistribution> get_random_type() const { return random_type_; }

 private:
  // Associated System and System resources.
  const SystemBase& system_;
  const InputPortIndex index_;
  const DependencyTicket ticket_;

  // Port details.
  const PortDataType data_type_;
  const int size_;
  const optional<RandomDistribution> random_type_;
};

}  // namespace systems
}  // namespace drake
