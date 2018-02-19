#include "drake/systems/framework/context_base.h"

#include <string>
#include <typeinfo>

namespace drake {
namespace systems {

std::unique_ptr<ContextBase> ContextBase::Clone() const {
  std::unique_ptr<ContextBase> clone_ptr(CloneWithoutPointers());

  // Verify that the most-derived Context didn't forget to override
  // CloneWithoutPointers().
  const ContextBase& source = *this;  // Deref here to avoid typeid warning.
  ContextBase& clone = *clone_ptr;
  DRAKE_ASSERT(typeid(source) == typeid(clone));

  // Create a complete mapping of tracker pointers.
  DependencyTracker::PointerMap tracker_map;
  BuildTrackerPointerMap(clone, &tracker_map);

  // Then do a pointer fixup pass.
  clone.FixTrackerPointers(source, tracker_map);
  return clone_ptr;
}

ContextBase::~ContextBase() {}

std::string ContextBase::GetSystemPathname() const {
  // TODO(sherm1) Replace with the real pathname.
  return "/dummy/system/pathname";
}

// Set up trackers for independent sources: time, accuracy, state, parameters,
// and input ports.
void ContextBase::CreateBuiltInTrackers() {
  DependencyGraph& trackers = graph_;
  // This is the dummy "tracker" used for constants and anything else that has
  // no dependencies on any Context source. Ignoring return value.
  trackers.CreateNewDependencyTracker(
      DependencyTicket(internal::kNothingTicket), "nothing");

  // Allocate time tracker. Ignoring return value.
  trackers.CreateNewDependencyTracker(
      DependencyTicket(internal::kTimeTicket), "t");

  // TODO(sherm1) Add the rest of the built-in tickets here.
}

}  // namespace systems
}  // namespace drake
