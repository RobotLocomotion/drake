#include "drake/systems/framework/system_base.h"

namespace drake {
namespace systems {

std::unique_ptr<ContextBase> SystemBase::MakeContext() const {
  // Derived class creates the concrete Context object, which already contains
  // all the well-known trackers (the ones with fixed tickets).
  std::unique_ptr<ContextBase> context_ptr = DoMakeContext();
  DRAKE_DEMAND(context_ptr != nullptr);
  ContextBase& context = *context_ptr;

  // Add the independent-source trackers and wire them up appropriately. That
  // includes input ports since their dependencies are external.
  CreateSourceTrackers(&context);

  // TODO(sherm1) Cache entry values & trackers, and output port trackers
  // are created here.

  return context_ptr;
}

void SystemBase::AcquireContextResources(ContextBase* context) const {
  DRAKE_DEMAND(context != nullptr);
  // Let the derived class acquire its needed resources and validate
  // that it can handle a System with this structure.
  DoAcquireContextResources(&*context);

  // TODO(sherm1) Cache allocation occurs here.
}

// Set up trackers for variable-numbered independent sources: discrete and
// abstract state, numerical and abstract parameters, and input ports.
// The generic trackers like "all parameters" are already present in the
// supplied Context, but we have to subscribe them to the individual
// elements now.
void SystemBase::CreateSourceTrackers(ContextBase* context_ptr) const {
  // TODO(sherm1) Ticket allocation occurs here.
  unused(context_ptr);
}

// Static member definitions.
constexpr DependencyTicket SystemBase::nothing_ticket_;
constexpr DependencyTicket SystemBase::time_ticket_;
constexpr DependencyTicket SystemBase::accuracy_ticket_;
constexpr DependencyTicket SystemBase::q_ticket_;
constexpr DependencyTicket SystemBase::v_ticket_;
constexpr DependencyTicket SystemBase::z_ticket_;
constexpr DependencyTicket SystemBase::xc_ticket_;
constexpr DependencyTicket SystemBase::xd_ticket_;
constexpr DependencyTicket SystemBase::xa_ticket_;
constexpr DependencyTicket SystemBase::x_ticket_;
constexpr DependencyTicket SystemBase::configuration_ticket_;
constexpr DependencyTicket SystemBase::velocity_ticket_;
constexpr DependencyTicket SystemBase::kinematics_ticket_;
constexpr DependencyTicket SystemBase::all_parameters_ticket_;
constexpr DependencyTicket SystemBase::all_input_ports_ticket_;
constexpr DependencyTicket SystemBase::all_sources_ticket_;
constexpr DependencyTicket SystemBase::xcdot_ticket_;
constexpr DependencyTicket SystemBase::xdhat_ticket_;

}  // namespace systems
}  // namespace drake
