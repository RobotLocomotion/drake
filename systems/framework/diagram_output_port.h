#pragma once

#include <memory>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/diagram_context.h"
#include "drake/systems/framework/framework_common.h"
#include "drake/systems/framework/output_port.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

template <typename T>
class Diagram;

/// Holds information about the subsystem output port that has been exported to
/// become one of this Diagram's output ports. The actual methods for
/// determining the port's value are supplied by the LeafSystem that ultimately
/// underlies the source port, although that may be any number of levels down.
template <typename T>
class DiagramOutputPort final : public OutputPort<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiagramOutputPort)

  /// Construct a %DiagramOutputPort for the given `diagram` that exports the
  /// indicated port. That port's owning system must be a subsystem of the
  /// diagram.
  DiagramOutputPort(const System<T>& diagram,
                    const internal::SystemMessageInterface& system_base,
                    OutputPortIndex index,
                    const OutputPort<T>* source_output_port,
                    SubsystemIndex source_subsystem_index)
      : OutputPort<T>(diagram, system_base, index,
                      source_output_port->get_data_type(),
                      source_output_port->size()),
        source_output_port_(source_output_port),
        source_subsystem_index_(source_subsystem_index) {}

  ~DiagramOutputPort() final = default;

  /// Obtain a reference to the subsystem output port that was exported to
  /// create this diagram port. Note that the source may itself be a diagram
  /// output port.
  const OutputPort<T>& get_source_output_port() const {
    return *source_output_port_;
  }

 private:
  // These forward to the source system output port.
  std::unique_ptr<AbstractValue> DoAllocate() const final {
    return source_output_port_->Allocate();
  }

  // Pass in just the source System's Context, not the whole Diagram context
  // we're given.
  void DoCalc(
      const Context<T>& context, AbstractValue* value) const final {
    const Context<T>& subcontext = get_subcontext(context);
    return source_output_port_->Calc(subcontext, value);
  }

  const AbstractValue& DoEval(const Context<T>& context) const final {
    const Context<T>& subcontext = get_subcontext(context);
    return source_output_port_->Eval(subcontext);
  }

  // Dig out the right subcontext for delegation.
  const Context<T>& get_subcontext(const Context<T>& context) const {
    const DiagramContext<T>* diagram_context =
        dynamic_cast<const DiagramContext<T>*>(&context);
    DRAKE_DEMAND(diagram_context != nullptr);
    return diagram_context->GetSubsystemContext(source_subsystem_index_);
  }

  const OutputPort<T>* const source_output_port_;
  const SubsystemIndex source_subsystem_index_;
};

}  // namespace systems
}  // namespace drake
