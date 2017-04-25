#include "drake/systems/framework/trigger.h"

#include <utility>

namespace drake {
namespace systems {

namespace internal {
const ForcedTrigger kGlobalForcedTrigger;
const std::vector<const Trigger*> kGlobalForcedTriggerVec(1, &kGlobalForcedTrigger);
}  // namespace internal

const std::vector<const Trigger*>& ForcedTrigger::OneForcedTrigger() {
  return internal::kGlobalForcedTriggerVec;
}

std::unique_ptr<Trigger> AbstractTrigger::Clone() const {
  std::unique_ptr<AbstractValue> cloned_data = data_->Clone();
  return std::unique_ptr<Trigger>(
      new AbstractTrigger(std::move(cloned_data)));
}

}  // namespace systems
}  // namespace drake
