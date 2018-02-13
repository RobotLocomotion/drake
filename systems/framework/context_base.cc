#include "drake/systems/framework/context_base.h"

#include "drake/common/unused.h"

namespace drake {
namespace systems {

std::string ContextBase::GetSystemPathname() const {
  // TODO(sherm1) Replace with the real pathname.
  return "/dummy/system/pathname";
}

// Set up trackers for independent sources: time, accuracy, state, parameters,
// and input ports.
void ContextBase::CreateWellKnownTrackers() {
  DependencyGraph& trackers = graph_;
  // This is the dummy "tracker" used for constants and anything else that has
  // no dependencies on any Context source. Ignoring return value.
  trackers.CreateNewDependencyTracker(
      DependencyTicket(internal::kNothingTicket), "nothing");

  // Allocate time tracker. Ignoring return value.
  trackers.CreateNewDependencyTracker(
      DependencyTicket(internal::kTimeTicket), "t");

  // TODO(sherm1) Add the rest of the well-known tickets here.
}

}  // namespace systems
}  // namespace drake
