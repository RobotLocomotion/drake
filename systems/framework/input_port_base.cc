#include "drake/systems/framework/input_port_base.h"

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/system_base.h"

namespace drake {
namespace systems {

InputPortBase::InputPortBase(InputPortIndex index, PortDataType data_type,
                             int size,
                             const optional<RandomDistribution>& random_type,
                             SystemBase* system)
    : system_(*system),
      index_(index),
      ticket_(system->assign_next_dependency_ticket()),
      data_type_(data_type),
      size_(size),
      random_type_(random_type) {
  DRAKE_DEMAND(system != nullptr);
  if (size_ == kAutoSize)
    DRAKE_ABORT_MSG("Auto-size ports are not yet implemented.");
  if (is_random() && data_type_ != kVectorValued) {
    DRAKE_ABORT_MSG("Random input ports must be vector valued.");
  }
}

}  // namespace systems
}  // namespace drake
