#include "drake/systems/lcm/lcm_interface_system.h"

#include <limits>
#include <utility>

#include "drake/lcm/drake_lcm.h"

namespace drake {
namespace systems {
namespace lcm {

using drake::lcm::DrakeLcm;
using drake::lcm::DrakeLcmInterface;
using drake::lcm::DrakeSubscriptionInterface;

LcmInterfaceSystem::LcmInterfaceSystem(std::string lcm_url)
    : LcmInterfaceSystem(std::make_unique<DrakeLcm>(lcm_url)) {}

LcmInterfaceSystem::LcmInterfaceSystem(std::unique_ptr<DrakeLcmInterface> owned)
    : LcmInterfaceSystem(owned.get()) {
  owned_lcm_ = std::move(owned);
}

LcmInterfaceSystem::LcmInterfaceSystem(DrakeLcmInterface* lcm)
    : lcm_(lcm) {
  DRAKE_THROW_UNLESS(lcm != nullptr);
}

LcmInterfaceSystem::~LcmInterfaceSystem() = default;

void LcmInterfaceSystem::Publish(
    const std::string& channel, const void* data, int data_size,
    optional<double> time_sec) {
  lcm_->Publish(channel, data, data_size, time_sec);
}

std::shared_ptr<DrakeSubscriptionInterface> LcmInterfaceSystem::Subscribe(
    const std::string& channel, HandlerFunction handler) {
  return lcm_->Subscribe(channel, std::move(handler));
}

int LcmInterfaceSystem::HandleSubscriptions(int timeout_millis) {
  return lcm_->HandleSubscriptions(timeout_millis);
}

// The only effect of this override is that time is prevented from advancing
// while there are messages in LCM receive queue.  This System is stateless, so
// there is no Context data to update within any event handler on this System.
// This override will fully drain the LCM receive queue, and will never block
// waiting for any pending messages.  The LcmPublisherSystem outputs do not
// change as a direct result of this method.
void LcmInterfaceSystem::DoCalcNextUpdateTime(
    const Context<double>& context,
    systems::CompositeEventCollection<double>*,
    double* time) const {
  const int timeout_millis = 0;  // Do not block.
  const int num_handled = lcm_->HandleSubscriptions(timeout_millis);
  if (num_handled > 0) {
    // Schedule an update event at the current time.  We don't actually do
    // anything ourself for the update, but by pausing time we'll allow the
    // *subsequent* event interrogation at time == now to learn that some
    // LcmSubscriberSystem might have had a (meaningful) update, without
    // simulation time advancing any further.
    *time = context.get_time();
  } else {
    *time = std::numeric_limits<double>::infinity();
  }
}

}  // namespace lcm
}  // namespace systems
}  // namespace drake
