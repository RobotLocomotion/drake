#include "drake/systems/framework/trigger.h"

#include <utility>

namespace drake {
namespace systems {

//////////////////////////////////////////////////////////////
// This is probably a really bad idea???
namespace {
const ForcedTrigger kGlobalForcedTrigger;
const std::vector<const Trigger*> kGlobalForcedTriggerVec(
    1, &kGlobalForcedTrigger);
}  // namespace

const std::vector<const Trigger*>& ForcedTrigger::OneForcedTrigger() {
  return kGlobalForcedTriggerVec;
}
//////////////////////////////////////////////////////////////

std::unique_ptr<Trigger> AbstractTrigger::DoClone() const {
  std::unique_ptr<AbstractValue> cloned_data = data_->Clone();
  return std::unique_ptr<Trigger>(new AbstractTrigger(std::move(cloned_data)));
}

}  // namespace systems
}  // namespace drake
