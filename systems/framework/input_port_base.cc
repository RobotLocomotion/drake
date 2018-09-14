#include "drake/systems/framework/input_port_base.h"

#include <utility>

#include "drake/common/drake_assert.h"

namespace drake {
namespace systems {

InputPortBase::InputPortBase(SystemBase* owning_system, std::string name,
                             InputPortIndex index, DependencyTicket ticket,
                             PortDataType data_type, int size,
                             const optional<RandomDistribution>& random_type)
    : owning_system_(*owning_system),
      index_(index),
      ticket_(ticket),
      data_type_(data_type),
      size_(size),
      name_(std::move(name)),
      random_type_(random_type) {
  DRAKE_DEMAND(owning_system != nullptr);
  DRAKE_DEMAND(!name_.empty());
  if (size_ == kAutoSize) {
    DRAKE_ABORT_MSG("Auto-size ports are not yet implemented.");
  }
  if (is_random() && data_type_ != kVectorValued) {
    DRAKE_ABORT_MSG("Random input ports must be vector valued.");
  }
}

InputPortBase::~InputPortBase() = default;

}  // namespace systems
}  // namespace drake
