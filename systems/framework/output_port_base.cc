#include "drake/systems/framework/output_port_base.h"

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/framework_common.h"

namespace drake {
namespace systems {

OutputPortBase::~OutputPortBase() = default;

OutputPortBase::OutputPortBase(
    SystemBase* owning_system,
    OutputPortIndex index, DependencyTicket ticket, PortDataType data_type,
    int size)
    : owning_system_(*owning_system),
      index_(index),
      ticket_(ticket),
      data_type_(data_type),
      size_(size) {
  DRAKE_DEMAND(owning_system != nullptr);
  if (size_ == kAutoSize)
    DRAKE_ABORT_MSG("Auto-size ports are not yet implemented.");
}

}  // namespace systems
}  // namespace drake
