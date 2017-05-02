#include "drake/systems/framework/trigger.h"

#include <utility>

namespace drake {
namespace systems {

std::ostream& operator<<(std::ostream& out, const Trigger::TriggerType& type) {
  switch (type) {
    case Trigger::TriggerType::kUnknown:
      out << "UnknownTrigger";
      break;

    case Trigger::TriggerType::kTimed:
      out << "TimedTrigger";

    case Trigger::TriggerType::kPeriodic:
      out << "PeriodicTrigger";
      break;

    case Trigger::TriggerType::kPerStep:
      out << "PerStepTrigger";
      break;

    case Trigger::TriggerType::kWitness:
      out << "WitnessTrigger";
      break;
  }
  return out;
}

}  // namespace systems
}  // namespace drake
