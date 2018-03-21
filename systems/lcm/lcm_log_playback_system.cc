#include "drake/systems/lcm/lcm_log_playback_system.h"

#include <cmath>
#include <memory>

#include "drake/common/drake_assert.h"

namespace drake {
namespace systems {
namespace lcm {

LcmLogPlaybackSystem::LcmLogPlaybackSystem(drake::lcm::DrakeLcmLog* log)
    : log_(log) {
  DRAKE_DEMAND(log != nullptr);
}

LcmLogPlaybackSystem::~LcmLogPlaybackSystem() {}

void LcmLogPlaybackSystem::DoCalcNextUpdateTime(
    const Context<double>& context,
    systems::CompositeEventCollection<double>* events, double* time) const {
  // If there are no more messages in the log, do nothing.
  const double message_time = log_->GetNextMessageTime();
  if (std::isinf(message_time)) {
    LeafSystem<double>::DoCalcNextUpdateTime(context, events, time);
    return;
  }

  // Prepare a callback that dispatches message(s) to all the subscribers.
  PublishEvent<double>::PublishCallback callback = [log = log_](
      const Context<double>& callback_context, const PublishEvent<double>&) {
    // Use a loop to publish all messages that occur at the exact same time.
    while (log->GetNextMessageTime() == callback_context.get_time()) {
      log->DispatchMessageAndAdvanceLog(callback_context.get_time());
    }
  };

  // Schedule a publish event at the next message time.
  DRAKE_DEMAND(std::isfinite(message_time));
  DRAKE_DEMAND(message_time > context.get_time());
  *time = message_time;
  events->get_mutable_publish_events().add_event(
      std::make_unique<systems::PublishEvent<double>>(
          Event<double>::TriggerType::kTimed, callback));
}

}  // namespace lcm
}  // namespace systems
}  // namespace drake
