#include "drake/systems/framework/trigger.h"

#include <utility>

namespace drake {
namespace systems {

std::ostream& operator<<(std::ostream& out, const Trigger::TriggerType& type) {
  switch (type) {
    case Trigger::TriggerType::kUnknown:
      out << "UnknownTrigger";
      break;

    case Trigger::TriggerType::kForced:
      out << "ForcedTrigger";
      break;

    case Trigger::TriggerType::kPeriodic:
      out << "PeriodicTrigger";
      break;

    case Trigger::TriggerType::kPerStep:
      out << "PerStepTrigger";
      break;
  }
  return out;
}

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

}  // namespace systems
}  // namespace drake
