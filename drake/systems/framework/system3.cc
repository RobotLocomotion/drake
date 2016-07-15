#include "drake/systems/framework/system3.h"

#include <memory>

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/context3.h"

namespace drake {
namespace systems {

std::unique_ptr<AbstractContext3> AbstractSystem3::CreateDefaultContext() const {
  std::unique_ptr<AbstractContext3> context = CreateEmptyContext();

  AllocateOutputPorts(context.get());
  AllocateAndConnectInputPorts(context.get());

  // Allow the concrete System to allocate context resources for its
  // internal use, and set up computation dependencies.
  DoAcquireContextResources(context.get());
  return context;
}

// To evaluate, first find the subsystem that owns the port and the
// corresponding subcontext that owns the cache entry. Then delegate 
// computation to that cache entry, using that subsystem.
const AbstractValue& AbstractSystem3::EvalOutputPort(
    const AbstractContext3& context, int port_num) const {
  const int n = get_num_output_ports();
  if (!(0 <= port_num && port_num < n))
    throw std::out_of_range(
        "AbstractSystem::EvalOutputPort(): port " + std::to_string(port_num) +
        " is out of range. There are " + std::to_string(n) + " output ports.");
  // TODO(sherm1) Validate context, at least in Debug?

  const OutputPort3& port = get_output_port(port_num);
  std::pair<const AbstractSystem3*, int> owner = port.get_owner_system();

  // TODO(sherm1) This should use a prebuilt index rather than searching.
  std::vector<int> path = owner.first->get_path_from_root_system();
  const AbstractContext3& subcontext =
    context.get_root_context().find_subcontext(path);

  CacheEntry* entry = context.get_mutable_output_entry(port_num);
  entry->RealizeCacheEntry(*owner.first, subcontext);
  return entry->get_abstract_value();
}

// To evaluate, first find the output port that is providing the value for
// this input port, then use EvalOutputPort() to get the value.
const AbstractValue& AbstractSystem3::EvalInputPort(
    const AbstractContext3& context, int port_num) const {
  const int n = get_num_input_ports();
  if (!(0 <= port_num && port_num < n))
    throw std::out_of_range(
        "AbstractSystem3::EvalInputPort(): port " + std::to_string(port_num) +
        " is out of range. There are " + std::to_string(n) + " input ports.");

  const InputPort3& port = get_input_port(port_num);
  const OutputPort3* connection = port.get_connection();

  if (connection == nullptr)
    throw std::logic_error("AbstractSystem3::EvalInputPort(): input port " +
                           std::to_string(port_num) +
                           " is not connected so can't provide a value.");

  std::pair<const AbstractSystem3*, int> owner = connection->get_owner_system();

  // TODO(sherm1) This should use a prebuilt index rather than searching.
  std::vector<int> path = owner.first->get_path_from_root_system();
  const AbstractContext3& subcontext =
    context.get_root_context().find_subcontext(path);

  CacheEntry* entry = subcontext.get_mutable_output_entry(owner.second);
  entry->RealizeCacheEntry(*owner.first, subcontext);
  return entry->get_abstract_value();
}

std::unique_ptr<AbstractContext3> AbstractSystem3::CreateEmptyContext()
    const {
  // Obtain a Context of whatever concrete subtype is preferred by this
  // concrete System.
  std::unique_ptr<AbstractContext3> context = DoCreateEmptyContext();
  context->SetNumInputPorts(get_num_input_ports());
  context->SetNumOutputPorts(get_num_output_ports());

  // Recursively add a subcontext for each of this System's subsystems. All the
  // input and output slots are allocated but no value resources are created
  // and no connections are made.
  for (int i = 0; i < get_num_subsystems(); ++i) {
    const AbstractSystem3& subsystem = get_subsystem(i);
    std::unique_ptr<AbstractContext3> subcontext = subsystem.CreateEmptyContext();
    const int subcontext_num = context->AddSubcontext(std::move(subcontext));
    DRAKE_ABORT_UNLESS(subcontext_num == i);
  }

  return context;
}

// Allocate output ports bottom up so higher systems can point to already-
// existing cache resources.
void AbstractSystem3::AllocateOutputPorts(
    AbstractContext3* context) const {

  for (int i = 0; i < get_num_subsystems(); ++i) {
    AbstractContext3* subcontext = context->get_mutable_subcontext(i);
    DRAKE_ABORT_UNLESS(subcontext->get_parent_context() == context &&
                       subcontext->get_subcontext_num() == i);
    get_subsystem(i).AllocateOutputPorts(subcontext);
  }

  for (int i = 0; i < get_num_output_ports(); ++i)
    context->SetOutputPort(*this, i);
}

// Allocate input ports bottom up so higher systems can point to already-
// existing cache resources.
void AbstractSystem3::AllocateAndConnectInputPorts(
    AbstractContext3* context) const {

  for (int i = 0; i < get_num_subsystems(); ++i) {
    AbstractContext3* subcontext = context->get_mutable_subcontext(i);
    DRAKE_ABORT_UNLESS(subcontext->get_parent_context() == context &&
                       subcontext->get_subcontext_num() == i);
    get_subsystem(i).AllocateAndConnectInputPorts(subcontext);
  }

  for (int i = 0; i < get_num_input_ports(); ++i)
    context->SetInputPort(*this, i);
}

}  // namespace systems
}  // namespace drake