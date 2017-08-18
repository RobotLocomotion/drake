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
  std::ostringstream oss;
  oss << "output port " << this->get_index() << " of "
      << this->get_system_base().GetSystemIdString();
  return oss.str();
}

}  // namespace systems
}  // namespace drake
