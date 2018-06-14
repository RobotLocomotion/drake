#pragma once

#include "drake/systems/framework/framework_common.h"

namespace drake {
namespace systems {

// Break the SystemBase <=> OutputPortBase physical dependency cycle. An
// OutputPortBase contains a back-pointer to its owning SystemBase, but that
// pointer is forward-declared here and never dereferenced within this file.
class SystemBase;

/** %OutputPortBase handles the scalar type-independent aspects of an
OutputPort. An OutputPort belongs to a System and represents the properties of
one of that System's output ports. */
class OutputPortBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(OutputPortBase)

  virtual ~OutputPortBase();

  /** Returns the index of this output port within the owning System. For a
  Diagram, this will be the index within the Diagram, _not_ the index within
  the LeafSystem whose output port was forwarded. */
  OutputPortIndex get_index() const {
    return index_;
  }

  /** Returns the DependencyTicket for this output port within the owning
  System. */
  DependencyTicket ticket() const {
    return ticket_;
  }

  /** Gets the port data type specified at port construction. */
  PortDataType get_data_type() const { return data_type_; }

  /** Returns the fixed size expected for a vector-valued output port. Not
  meaningful for abstract output ports. */
  int size() const { return size_; }

  /** Returns a reference to the System that owns this output port. Note that
  for a diagram output port this will be the diagram, not the leaf system whose
  output port was forwarded. */
  const SystemBase& get_system_base() const {
    return owning_system_;
  }

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
  @param index
    The index to be assigned to this OutputPort.
  @param ticket
    The DependencyTicket to be assigned to this OutputPort.
  @param data_type
    Whether the port described is vector or abstract valued.
  @param size
    If the port described is vector-valued, the number of elements expected,
    otherwise ignored. */
  OutputPortBase(SystemBase* owning_system,
                 OutputPortIndex index, DependencyTicket ticket,
                 PortDataType data_type, int size);

  SystemBase& get_mutable_system_base() {
    return owning_system_;
  }

  /** Concrete output ports must implement this to return the prerequisite
  dependency ticket for this port, which may be in the current System or one
  of its immediate child subsystems. */
  virtual internal::OutputPortPrerequisite DoGetPrerequisite() const = 0;

 private:
  // Associated System and System resources.
  SystemBase& owning_system_;
  const OutputPortIndex index_;
  const DependencyTicket ticket_;

  // Port details.
  const PortDataType data_type_;
  const int size_;
};

}  // namespace systems
}  // namespace drake
