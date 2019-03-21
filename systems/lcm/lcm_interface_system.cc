#include "drake/systems/lcm/lcm_interface_system.h"

#include <limits>
#include <utility>

#include "drake/common/text_logging.h"
#include "drake/common/unused.h"
#include "drake/lcm/drake_lcm.h"

namespace drake {
namespace systems {
namespace lcm {

using drake::lcm::DrakeLcm;
using drake::lcm::DrakeLcmInterface;
using drake::lcm::DrakeSubscription;

LcmInterfaceSystem::LcmInterfaceSystem(std::string lcm_url)
    : LcmInterfaceSystem(std::make_unique<DrakeLcm>(lcm_url)) {
  url_ = lcm_url;
}

LcmInterfaceSystem::LcmInterfaceSystem(std::unique_ptr<DrakeLcmInterface> owned)
    : LcmInterfaceSystem(owned.get()) {
  owned_lcm_ = std::move(owned);
}

LcmInterfaceSystem::LcmInterfaceSystem(DrakeLcmInterface* lcm)
    : lcm_(lcm) {
  DRAKE_THROW_UNLESS(lcm != nullptr);
}

LcmInterfaceSystem::~LcmInterfaceSystem() = default;

std::string LcmInterfaceSystem::get_requested_lcm_url() const {
  return url_;
}

void LcmInterfaceSystem::Publish(
    const std::string& channel, const void* data, int data_size,
    optional<double> time_sec) {
  lcm_->Publish(channel, data, data_size, time_sec);
}

std::unique_ptr<DrakeSubscription> LcmInterfaceSystem::Subscribe(
    const std::string& channel, int queue_capacity, HandlerFunction handler) {
  return lcm_->Subscribe(channel, queue_capacity, std::move(handler));
}

int LcmInterfaceSystem::HandleSubscriptions(int timeout_millis) {
  return lcm_->HandleSubscriptions(timeout_millis);
}

void LcmInterfaceSystem::DoCalcNextUpdateTime(
    const Context<double>& context,
    systems::CompositeEventCollection<double>*,
    double* time) const {
  const int num_handled = lcm_->HandleSubscriptions(0);
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
