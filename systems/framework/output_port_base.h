#pragma once

#include <string>
#include <utility>

#include "drake/common/drake_optional.h"
#include "drake/systems/framework/framework_common.h"

namespace drake {
namespace systems {

class SystemBase;

/** An OutputPort belongs to a System and represents the properties of one of
that System's output ports. OutputPort objects are assigned OutputPortIndex
values in the order they are declared; these are unique within a single System.
A DependencyTicket is assigned so that downstream dependents can subscribe to
the value of this port. Also, each output port has a single prerequisite to
which it must be subscribed. For leaf systems, that prerequisite is a cache
entry that holds the most-recently-calculated port value. For diagrams, it is
the output port of a child subsystem which has been exported to be an output
of its parent.

%OutputPortBase handles the scalar type-independent aspects of an OutputPort. */
class OutputPortBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(OutputPortBase)

  virtual ~OutputPortBase() = default;

  /** Returns the index of this output port within the owning System. For a
  Diagram, this will be the index within the Diagram, _not_ the index within
  the LeafSystem whose output port was forwarded. */
  OutputPortIndex get_index() const {
    return oport_index_;
  }

  /** Returns the DependencyTicket for this output port within the owning
  System. */
  DependencyTicket ticket() const {
    return ticket_;
  }

  /** Returns a reference to the SystemBase that owns this output port. Note
  that for a diagram output port this will be the diagram, not the leaf system
  whose output port was exported. */
  const SystemBase& get_system_base() const { return system_; }

  /** Gets the port data type specified at port construction. */
  PortDataType get_data_type() const { return data_type_; }

  /** Returns the fixed size expected for a vector-valued output port. Not
  meaningful for abstract output ports. */
  int size() const { return size_; }

  /** Returns the prerequisite for this output port -- either a cache entry
  in this System, or an output port of a child System. */
  std::pair<optional<SubsystemIndex>, DependencyTicket> GetPrerequisite()
      const {
    return DoGetPrerequisite();
  };

 protected:
  /** Provides derived classes the ability to set the base class members at
  construction.
  @param data_type
    Whether the port described is vector or abstract valued.
  @param size
    If the port described is vector-valued, the number of elements expected,
    otherwise ignored.
  @param system
    The System that will own this new output port. This port will
    be assigned the next available output port index in this system, and
    the next available dependency ticket. */
  explicit OutputPortBase(PortDataType data_type, int size, SystemBase* system);

  /** This is useful for error messages and produces `"output port <#> of
  GetSystemIdString()"` with whatever System identification string is produced
  by that method. */
  std::string GetPortIdString() const;

  /** Concrete output port must implement this to return the prerequisite
  dependency ticket for this port, which may be in the current System or one
  of its immediate child subsystems. */
  virtual std::pair<optional<SubsystemIndex>, DependencyTicket>
  DoGetPrerequisite() const = 0;

 private:
  // Associated System and System resources.
  const SystemBase& system_;
  const OutputPortIndex oport_index_;
  const DependencyTicket ticket_;

  // Port details.
  const PortDataType data_type_;
  const int size_;
};

}  // namespace systems
}  // namespace drake
