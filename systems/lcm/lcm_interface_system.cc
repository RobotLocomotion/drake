#include "drake/systems/lcm/lcm_interface_system.h"

#include <limits>
#include <utility>

#include <fmt/format.h>

#include "drake/lcm/drake_lcm.h"
#include "drake/systems/lcm/lcm_system_graphviz.h"

namespace drake {
namespace systems {
namespace lcm {

using drake::lcm::DrakeLcm;
using drake::lcm::DrakeLcmInterface;
using drake::lcm::DrakeLcmParams;
using drake::lcm::DrakeSubscriptionInterface;

LcmInterfaceSystem::LcmInterfaceSystem(std::string lcm_url)
    : LcmInterfaceSystem(DrakeLcmParams{.lcm_url = std::move(lcm_url)}) {}

LcmInterfaceSystem::LcmInterfaceSystem(const DrakeLcmParams& params)
    : LcmInterfaceSystem(std::make_unique<DrakeLcm>(params)) {}

LcmInterfaceSystem::LcmInterfaceSystem(std::unique_ptr<DrakeLcmInterface> owned)
    : LcmInterfaceSystem(owned.get()) {
  owned_lcm_ = std::move(owned);
}

LcmInterfaceSystem::LcmInterfaceSystem(DrakeLcmInterface* lcm) : lcm_(lcm) {
  DRAKE_THROW_UNLESS(lcm != nullptr);
}

LcmInterfaceSystem::~LcmInterfaceSystem() = default;

std::string LcmInterfaceSystem::get_lcm_url() const {
  return lcm_->get_lcm_url();
}

void LcmInterfaceSystem::Publish(const std::string& channel, const void* data,
                                 int data_size,
                                 std::optional<double> time_sec) {
  lcm_->Publish(channel, data, data_size, time_sec);
}

std::shared_ptr<DrakeSubscriptionInterface> LcmInterfaceSystem::Subscribe(
    const std::string& channel, HandlerFunction handler) {
  return lcm_->Subscribe(channel, std::move(handler));
}

std::shared_ptr<DrakeSubscriptionInterface>
LcmInterfaceSystem::SubscribeMultichannel(std::string_view regex,
                                          MultichannelHandlerFunction handler) {
  return lcm_->SubscribeMultichannel(regex, std::move(handler));
}

std::shared_ptr<DrakeSubscriptionInterface>
LcmInterfaceSystem::SubscribeAllChannels(MultichannelHandlerFunction handler) {
  return lcm_->SubscribeAllChannels(std::move(handler));
}

int LcmInterfaceSystem::HandleSubscriptions(int timeout_millis) {
  return lcm_->HandleSubscriptions(timeout_millis);
}

void LcmInterfaceSystem::OnHandleSubscriptionsError(
    const std::string& error_message) {
  drake::lcm::internal::OnHandleSubscriptionsError(lcm_, error_message);
}

// The only effect of this override is that time is prevented from advancing
// while there are messages in LCM receive queue.  This System is stateless, so
// there is no Context data to update within any event handler on this System.
// This override will fully drain the LCM receive queue, and will never block
// waiting for any pending messages.  The LcmPublisherSystem outputs do not
// change as a direct result of this method.
void LcmInterfaceSystem::DoCalcNextUpdateTime(
    const Context<double>& context,
    systems::CompositeEventCollection<double>* events, double* time) const {
  const int timeout_millis = 0;  // Do not block.
  const int num_handled = lcm_->HandleSubscriptions(timeout_millis);
  if (num_handled > 0) {
    // Schedule an update event at the current time.  We don't actually do
    // anything ourself for the update, but by pausing time we'll allow the
    // *subsequent* event interrogation at time == now to learn that some
    // LcmSubscriberSystem might have had a (meaningful) update, without
    // simulation time advancing any further.
    *time = context.get_time();

    // At least one Event object must be returned when time ≠ ∞.
    PublishEvent<double> event(TriggerType::kTimed);
    event.AddToComposite(events);
  } else {
    *time = std::numeric_limits<double>::infinity();
  }
}

LeafSystem<double>::GraphvizFragment LcmInterfaceSystem::DoGetGraphvizFragment(
    const GraphvizFragmentParams& params) const {
  const std::string node_id = internal::LcmSystemGraphviz::get_node_id(*this);

  // Set the well-known ID, enable twaining, and tack on the URL.
  GraphvizFragmentParams new_params{params};
  new_params.node_id = node_id;
  new_params.options.emplace("split", "I/O");
  new_params.header_lines.push_back(fmt::format("lcm_url={}", get_lcm_url()));
  GraphvizFragment result =
      LeafSystem<double>::DoGetGraphvizFragment(new_params);
  result.fragments.push_back(fmt::format(
      "{}in [color={}];", node_id, internal::LcmSystemGraphviz::get_color()));
  result.fragments.push_back(fmt::format(
      "{}out [color={}];", node_id, internal::LcmSystemGraphviz::get_color()));
  return result;
}

}  // namespace lcm
}  // namespace systems
}  // namespace drake
