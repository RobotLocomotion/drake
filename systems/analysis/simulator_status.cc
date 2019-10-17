#include "drake/systems/analysis/simulator_status.h"

#include <string>
#include <utility>

#include <fmt/format.h>

namespace drake {
namespace systems {

/** Returns a human-readable message explaining the return result. */
std::string SimulatorStatus::FormatMessage() const {
  if (reason() == kReachedBoundaryTime) {
    DRAKE_DEMAND(return_time() == boundary_time());
    return fmt::format(
        "Simulator successfully reached the boundary time ({}).",
        boundary_time());
  }

  // Equality is unlikely but allowed in case a termination request happens
  // at exactly the boundary time.
  DRAKE_DEMAND(return_time() <= boundary_time());

  // Attempt to identify the relevant subsystem in human-readable terms. If no
  // subsystem was provided we just call it "System". Otherwise, we obtain its
  // type and its name and call it "type System 'name'", e.g.
  // "MultibodyPlant<double> System 'my_plant'".
  const std::string system_id =
      system() == nullptr
      ? "System"
      : fmt::format(
          "{} System '{}'",
          NiceTypeName::RemoveNamespaces(system()->GetSystemType()),
          system()->GetSystemPathname());

  if (reason() == kReachedTerminationCondition) {
    return fmt::format(
        "Simulator returned early at time {} because {} requested termination "
        "with message: \"{}\"",
        return_time(), system_id, message());
  }

  DRAKE_DEMAND(reason() == kEventHandlerFailed);
  return fmt::format(
      "Simulator stopped at time {} because {} failed "
      "with message: \"{}\"",
      return_time(), system_id, message());
}

}  // namespace systems
}  // namespace drake


