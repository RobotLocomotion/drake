#pragma once

#include <string>

#include "drake/systems/framework/framework_common.h"
#include "drake/systems/framework/port_base.h"

namespace drake {
namespace systems {

/** %OutputPortBase handles the scalar type-independent aspects of an
OutputPort. An OutputPort belongs to a System and represents the properties of
one of that System's output ports. */
class OutputPortBase : public PortBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(OutputPortBase)

  ~OutputPortBase() override;

  /** Returns the index of this output port within the owning System. For a
  Diagram, this will be the index within the Diagram, _not_ the index within
  the LeafSystem whose output port was forwarded. */
  OutputPortIndex get_index() const {
    return OutputPortIndex(get_int_index());
  }

  // A using-declaration adds these methods into our class's Doxygen.
  // (Placed in an order that makes sense for the class's table of contents.)
  using PortBase::get_name;
  using PortBase::GetFullDescription;
  using PortBase::get_data_type;
  using PortBase::size;
  using PortBase::ticket;

#ifndef DRAKE_DOXYGEN_CXX
  // Internal use only. Returns the prerequisite for this output port -- either
  // a cache entry in this System, or an output port of a child System.
  internal::OutputPortPrerequisite GetPrerequisite() const {
    return DoGetPrerequisite();
  }
#endif

 protected:
  /** Provides derived classes the ability to set the base class members at
  construction.

  @param owning_system
    The System that owns this output port.
  @param owning_system_id
    The ID of owning_system.
  @param name
    A name for the port. Must not be empty. Output port names should be unique
    within a single System.
  @param index
    The index to be assigned to this OutputPort.
  @param ticket
    The DependencyTicket to be assigned to this OutputPort.
  @param data_type
    Whether the port described is vector or abstract valued.
  @param size
    If the port described is vector-valued, the number of elements expected,
    otherwise ignored. */
  OutputPortBase(
      internal::SystemMessageInterface* owning_system,
      internal::SystemId owning_system_id, std::string name,
      OutputPortIndex index, DependencyTicket ticket, PortDataType data_type,
      int size);

  /** Concrete output ports must implement this to return the prerequisite
  dependency ticket for this port, which may be in the current System or one
  of its immediate child subsystems. */
  virtual internal::OutputPortPrerequisite DoGetPrerequisite() const = 0;
};

}  // namespace systems
}  // namespace drake
