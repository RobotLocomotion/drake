#pragma once

#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/value.h"
#include "drake/systems/framework/diagram_context.h"
#include "drake/systems/framework/framework_common.h"
#include "drake/systems/framework/output_port.h"

namespace drake {
namespace systems {

/** (Advanced.) Holds information about a subsystem output port that has been
exported to become one of this Diagram's output ports. The actual methods for
determining the port's value are supplied by the LeafOutputPort that ultimately
underlies the source port, although that may be any number of levels down.
This is intended for internal use in implementing Diagram.

@tparam_default_scalar
*/
template <typename T>
class DiagramOutputPort final : public OutputPort<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiagramOutputPort)

  ~DiagramOutputPort() final = default;

  /** Obtains a reference to the subsystem output port that was exported to
  create this diagram port. Note that the source may itself be a diagram
  output port. */
  const OutputPort<T>& get_source_output_port() const {
    return *source_output_port_;
  }

 private:
  friend class internal::FrameworkFactory;

  // Constructs a %DiagramOutputPort that exports an output port of one of the
  // diagram's child subsystems.
  //
  // @param diagram The Diagram that will own this port.
  // @param system_interface The same Diagram cast to its base class.
  // @param system_id The ID of the same Diagram.
  // @param name A name for the port.  Output ports names must be unique
  //     within the `diagram` System.
  // @param index The output port index to be assigned to the new port.
  // @param ticket The DependencyTicket to be assigned to the new port.
  // @param source_output_port An output port of one of this diagram's child
  //     subsystems that is to be forwarded to the new port.
  // @param source_subsystem_index The index of the child subsystem that owns
  //     `source_output_port`.
  //
  // @pre The `diagram` System must actually be a Diagram.
  // @pre `diagram` lifetime must exceed the port's; we retain the pointer here.
  // @pre `diagram` and `system_interface` must be the same object.
  // @pre `source_output_port` must be owned by a child of `diagram`.
  // @pre `source_subsystem_index` must be the index of that child in `diagram`.
  //
  // This is intended only for use by Diagram and we depend on the caller to
  // meet the above preconditions, not all of which can be independently
  // verified here.
  //
  // To avoid a Diagram <=> DiagramOutputPort physical dependency, this class
  // doesn't have access to a declaration of Diagram<T> so would not be able
  // to static_cast up to System<T> as is required by OutputPort. We expect
  // the caller to do that cast for us so take a System<T> here.
  DiagramOutputPort(const System<T>* diagram,
                    internal::SystemMessageInterface* system_interface,
                    internal::SystemId system_id,
                    std::string name,
                    OutputPortIndex index,
                    DependencyTicket ticket,
                    const OutputPort<T>* source_output_port,
                    SubsystemIndex source_subsystem_index)
      : OutputPort<T>(diagram, system_interface, system_id, std::move(name),
                      index, ticket, source_output_port->get_data_type(),
                      source_output_port->size()),
        source_output_port_(source_output_port),
        source_subsystem_index_(source_subsystem_index) {
    DRAKE_DEMAND(index.is_valid() && ticket.is_valid());
    DRAKE_DEMAND(source_subsystem_index.is_valid());
    DRAKE_DEMAND(source_output_port != nullptr);
  }

  // Asks the source system output port to allocate an appropriate object.
  std::unique_ptr<AbstractValue> DoAllocate() const final {
    return source_output_port_->Allocate();
  }

  // Given the whole Diagram context, extracts the appropriate subcontext and
  // delegates to the source output port.
  void DoCalc(const Context<T>& diagram_context,
              AbstractValue* value) const final {
    const Context<T>& subcontext = get_subcontext(diagram_context);
    return source_output_port_->Calc(subcontext, value);
  }

  // Given the whole Diagram context, extracts the appropriate subcontext and
  // delegates to the source output port.
  const AbstractValue& DoEval(const Context<T>& diagram_context) const final {
    const Context<T>& subcontext = get_subcontext(diagram_context);
    return source_output_port_->template Eval<AbstractValue>(subcontext);
  }

  // Returns the source output port's subsystem, and the ticket for that
  // port's tracker.
  internal::OutputPortPrerequisite DoGetPrerequisite() const final {
    return {source_subsystem_index_, source_output_port_->ticket()};
  };

  // Check that an AbstractValue provided to Calc() is suitable for this port
  // by asking the source port to do it.
  void ThrowIfInvalidPortValueType(
      const Context<T>& context,
      const AbstractValue& proposed_value) const final {
    OutputPort<T>::ThrowIfInvalidPortValueType(
        *source_output_port_, get_subcontext(context), proposed_value);
  }

  // Digs out the right subcontext for delegation.
  const Context<T>& get_subcontext(const Context<T>& diagram_context) const {
    // Profiling revealed that it is too expensive to do a dynamic_cast here.
    // A static_cast is safe as long as this is invoked only by methods that
    // validate the SystemId, so that we know this Context is ours. As of this
    // writing, only OutputPort::Eval and OutputPort::Calc invoke this and
    // they do so safely.
    const DiagramContext<T>* diagram_context_downcast =
        static_cast<const DiagramContext<T>*>(&diagram_context);
    DRAKE_DEMAND(diagram_context_downcast != nullptr);
    return diagram_context_downcast->GetSubsystemContext(
        source_subsystem_index_);
  }

  const OutputPort<T>* const source_output_port_;
  const SubsystemIndex source_subsystem_index_;
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::DiagramOutputPort)
