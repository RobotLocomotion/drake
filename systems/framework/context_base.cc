#include "drake/systems/framework/context_base.h"

#include "drake/common/unused.h"
#include "drake/systems/framework/system_base.h"

namespace drake {
namespace systems {

// Set up trackers for independent sources: time, accuracy, state, parameters,
// and input ports.
void ContextBase::CreateWellKnownTrackers() {
  DependencyGraph& trackers = graph_;
  // This is the dummy "tracker" used for constants and anything else that has
  // no dependencies on any Context source.
  (void)trackers.CreateNewDependencyTracker(SystemBase::nothing_ticket(),
                                            "nothing");

  // Allocate trackers for time, accuracy, q, v, z.
  auto& time_tracker =
      trackers.CreateNewDependencyTracker(SystemBase::time_ticket(), "t");
  auto& accuracy_tracker = trackers.CreateNewDependencyTracker(
      SystemBase::accuracy_ticket(), "accuracy");
  auto& q_tracker =
      trackers.CreateNewDependencyTracker(SystemBase::q_ticket(), "q");
  auto& v_tracker =
      trackers.CreateNewDependencyTracker(SystemBase::v_ticket(), "v");
  auto& z_tracker =
      trackers.CreateNewDependencyTracker(SystemBase::z_ticket(), "z");

  // Continuous state xc depends on q, v, and z.
  auto& xc_tracker =
      trackers.CreateNewDependencyTracker(SystemBase::xc_ticket(), "xc");
  xc_tracker.SubscribeToPrerequisite(&q_tracker);
  xc_tracker.SubscribeToPrerequisite(&v_tracker);
  xc_tracker.SubscribeToPrerequisite(&z_tracker);

  // Allocate the "all discrete variables" xd tracker. The associated System is
  // responsible for allocating the individual discrete variable group xdᵢ
  // trackers and subscribing this one to each of those.
  auto& xd_tracker =
      trackers.CreateNewDependencyTracker(SystemBase::xd_ticket(), "xd");

  // Allocate the "all abstract variables" xa tracker. The associated System is
  // responsible for allocating the individual abstract variable xaᵢ
  // trackers and subscribing this one to each of those.
  auto& xa_tracker =
      trackers.CreateNewDependencyTracker(SystemBase::xa_ticket(), "xa");

  // The complete state x={xc,xd,xa}.
  auto& x_tracker =
      trackers.CreateNewDependencyTracker(SystemBase::all_state_ticket(), "x");
  x_tracker.SubscribeToPrerequisite(&xc_tracker);
  x_tracker.SubscribeToPrerequisite(&xd_tracker);
  x_tracker.SubscribeToPrerequisite(&xa_tracker);

  // Allocate the "all parameters" p tracker. The associated System is
  // responsible for allocating the individual numeric parameter pnᵢ and
  // abstract paraemter paᵢ trackers and subscribing this one to each of those.
  auto& p_tracker = trackers.CreateNewDependencyTracker(
      SystemBase::all_parameters_ticket(), "p");

  // Allocate the "all input ports" u tracker. The associated System is
  // responsible for allocating the individual input port uᵢ
  // trackers and subscribing this one to each of those.
  auto& u_tracker = trackers.CreateNewDependencyTracker(
      SystemBase::all_input_ports_ticket(), "u");

  // Allocate the "all sources" tracker. The complete list of known sources
  // is t,a,x,p, and u. Cache entries are not included separately because they
  // must ultimately depend on these same sources.
  auto& everything_tracker = trackers.CreateNewDependencyTracker(
      SystemBase::all_sources_ticket(), "all sources");
  everything_tracker.SubscribeToPrerequisite(&time_tracker);
  everything_tracker.SubscribeToPrerequisite(&accuracy_tracker);
  everything_tracker.SubscribeToPrerequisite(&x_tracker);
  everything_tracker.SubscribeToPrerequisite(&p_tracker);
  everything_tracker.SubscribeToPrerequisite(&u_tracker);

  auto& configuration_tracker = trackers.CreateNewDependencyTracker(
      SystemBase::configuration_ticket(), "configuration");
  // This default subscription must be changed if configuration is not
  // represented by q in this System.
  configuration_tracker.SubscribeToPrerequisite(&q_tracker);

  auto& velocity_tracker = trackers.CreateNewDependencyTracker(
      SystemBase::velocity_ticket(), "velocity");
  // This default subscription must be changed if velocity is not
  // represented by v in this System.
  velocity_tracker.SubscribeToPrerequisite(&v_tracker);

  // This tracks configuration & velocity regardless of their source.
  auto& kinematics_tracker = trackers.CreateNewDependencyTracker(
      SystemBase::kinematics_ticket(), "kinematics");
  kinematics_tracker.SubscribeToPrerequisite(&configuration_tracker);
  kinematics_tracker.SubscribeToPrerequisite(&velocity_tracker);

  auto& xcdot_tracker = trackers.CreateNewDependencyTracker(
      SystemBase::xcdot_ticket(), "xcdot");
  // TODO(sherm1) Connect to cache entry.
  unused(xcdot_tracker);

  auto& xdhat_tracker = trackers.CreateNewDependencyTracker(
      SystemBase::xdhat_ticket(), "xdhat");
  // TODO(sherm1) Connect to cache entry.
  unused(xdhat_tracker);
}

}  // namespace systems
}  // namespace drake
