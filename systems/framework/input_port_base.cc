#include "drake/systems/framework/input_port_base.h"

#include <stdexcept>
#include <utility>

#include <fmt/format.h>

#include "drake/common/drake_assert.h"

namespace drake {
namespace systems {

InputPortBase::InputPortBase(
    internal::SystemMessageInterface* owning_system,
    internal::SystemId owning_system_id, std::string name, InputPortIndex index,
    DependencyTicket ticket, PortDataType data_type, int size,
    const std::optional<RandomDistribution>& random_type,
    EvalAbstractCallback eval, ValueProducer::AllocateCallback alloc)
    : PortBase("Input", owning_system, owning_system_id, std::move(name), index,
               ticket, data_type, size),
      eval_(std::move(eval)),
      alloc_(std::move(alloc)),
      random_type_(random_type) {
  if (is_random() && data_type != kVectorValued) {
    throw std::logic_error("Random input ports must be vector valued.");
  }
  DRAKE_DEMAND(eval_ != nullptr);
  DRAKE_DEMAND(alloc_ != nullptr);
}

InputPortBase::~InputPortBase() = default;

std::unique_ptr<AbstractValue> InputPortBase::Allocate() const {
  std::unique_ptr<AbstractValue> value = alloc_();
  if (value == nullptr) {
    throw std::logic_error(fmt::format(
        "InputPort::Allocate(): allocator returned a nullptr for {}.",
        GetFullDescription()));
  }
  return value;
}

void InputPortBase::ThrowRequiredMissing() const {
  throw std::logic_error(fmt::format(
      "InputPort::Eval(): required {} is not connected", GetFullDescription()));
}

}  // namespace systems
}  // namespace drake
