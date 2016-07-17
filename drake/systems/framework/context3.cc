// For now, this is an empty .cc file that only serves to confirm
// context.h is a stand-alone header.

#include "drake/systems/framework/context3.h"

#include <memory>

#include "drake/systems/framework/system3.h"

namespace drake {
namespace systems {

void AbstractContext3::SetOutputPort(const AbstractSystem3& system,
                                    int port_num) {
  const OutputPortFinder& port_finder = system.get_output_port_finder(port_num);
  OutputEntryFinder& entry_finder = outputs_[port_num];

printf("SetOutputPort(%s,%d): ", system.GetSubsystemPathName().c_str(), port_num);

  if (port_finder.subsystem_num >= 0) {
    // This is an inherited port.
    printf("inherited from subsys %d(%s), output port %d\n",
           port_finder.subsystem_num,
           system.get_subsystem(port_finder.subsystem_num).GetSubsystemPathName().c_str(),
           port_finder.port_num);

    entry_finder.subcontext_num = port_finder.subsystem_num;
    entry_finder.location = port_finder.port_num;
    const AbstractContext3& subcontext =
        get_subcontext(entry_finder.subcontext_num);
    const OutputEntryFinder& subfinder =
        subcontext.get_output_entry_finder(entry_finder.location);
    entry_finder.entry = subfinder.entry;
    return;
  }

  // This port is owned by system, so the context needs to make room for its
  // value.
  const OutputPort3& port = system.get_output_port(port_num);
  const int entry_num = (int)cache_.size();
  cache_.emplace_back(port.get_model_value());
  CacheEntry& entry = cache_.back();
  entry.set_calculator(system.get_output_port_calculator(port_num));
  entry_finder = OutputEntryFinder(-1, entry_num, &entry);

  printf("owned. Assigned cache entry %d (%p).\n", entry_num, &entry);
}

void AbstractContext3::SetInputPort(const AbstractSystem3& system, int port_num) {
  const InputPortFinder& port_finder = system.get_input_port_finder(port_num);
  InputEntryFinder& entry_finder = inputs_[port_num];

printf("SetInputPort(%s,%d): ", system.GetSubsystemPathName().c_str(), port_num);
  // An input port doesn't require its own cache resources so we don't care
  // if it is inherited. If it is connected to an output port, we'll just
  // locate that port's entry in the context and point to that. That entry
  // could be higher in the context diagram, so it is critical that all output
  // entries are allocated before any input entries.

  entry_finder.subcontext_num = port_finder.subsystem_num;
  entry_finder.port_num = port_finder.port_num;
  entry_finder.entry = nullptr;  // Will point to output cache if connected.

  const InputPort3& port = system.get_input_port(port_num);
  const OutputPort3* connection = port.get_connection();
  if (!connection) {
    printf("not connected.\n");
    return;  // This port is not connected.
  }

  // The input port is connected; find corresponding output entry. We need to
  // locate the subcontext that owns this OutputPort.
  std::pair<const AbstractSystem3*, int> owner = connection->get_owner_system();
  DRAKE_ABORT_UNLESS(owner.first && owner.second >= 0);

  AbstractContext3* owner_subcontext =
      owner.first->find_my_mutable_subcontext(this);

  OutputEntryFinder* output_entry_finder =
      owner_subcontext->get_mutable_output_entry_finder(owner.second);

  entry_finder.entry = output_entry_finder->entry;
  printf("connected to subsys %s oport %d, cache=%p.\n", owner.first->GetSubsystemPathName().c_str(),
         owner.second, entry_finder.entry);
}

const AbstractSystem3& AbstractContext3::find_my_subsystem(
    const AbstractSystem3& some_subsystem) const {
  // TODO(sherm1) This should use a prebuilt index rather than searching.
  std::vector<int> path = get_path_from_root_context();
  const AbstractSystem3& my_subsystem =
      some_subsystem.get_root_system().find_subsystem(path);
  return my_subsystem;
}

}  // namespace systems
}  // namespace drake

template drake::systems::Context3<double>;