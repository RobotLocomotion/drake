#include "drake/systems/framework/output_port_base.h"

#include <sstream>

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/system_base.h"

namespace drake {
namespace systems {

OutputPortBase::OutputPortBase(PortDataType data_type, int size,
                               SystemBase* system)
    : system_(*system),
      oport_index_(system->get_num_output_ports()),
      ticket_(system->assign_next_dependency_ticket()),
      data_type_(data_type),
      size_(size) {
  DRAKE_DEMAND(system != nullptr);
  if (size_ == kAutoSize)
    DRAKE_ABORT_MSG("Auto-size ports are not yet implemented.");
}

std::string OutputPortBase::GetPortIdString() const {
  const std::string port_id =
      "OutputPort[" + std::to_string(this->get_index())
          + "] of subsystem " + this->get_system_base().GetSystemPathname()
          + "(" + NiceTypeName::RemoveNamespaces
          (this->get_system_base().GetSystemType()) +
          ").";
  return port_id;
}

}  // namespace systems
}  // namespace drake
