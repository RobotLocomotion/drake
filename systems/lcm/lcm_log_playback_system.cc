#include "drake/systems/lcm/lcm_log_playback_system.h"

#include <algorithm>
#include <cmath>
#include <memory>

#include "drake/common/drake_throw.h"

namespace drake {
namespace systems {
namespace lcm {

LcmLogPlaybackSystem::LcmLogPlaybackSystem(drake::lcm::DrakeLcmLog* log)
    : log_(log) {
  DRAKE_THROW_UNLESS(log != nullptr);
}

LcmLogPlaybackSystem::~LcmLogPlaybackSystem() {}

void LcmLogPlaybackSystem::DoCalcNextUpdateTime(
    const Context<double>& context,
    systems::CompositeEventCollection<double>* events, double* time) const {
  // We do not support events other than our own message timing events.
  LeafSystem<double>::DoCalcNextUpdateTime(context, events, time);
  DRAKE_THROW_UNLESS(events->HasEvents() == false);
  DRAKE_THROW_UNLESS(std::isinf(*time));

  // If there are no more messages in the log, do nothing.
  const double next_message_time = log_->GetNextMessageTime();
  if (std::isinf(next_message_time)) {
    return;
  }
  DRAKE_THROW_UNLESS(next_message_time > context.get_time());

  // Prepare a callback that dispatches message(s) to all the subscribers.
  PublishEvent<double>::PublishCallback callback =
      [](const System<double>& system, const Context<double>& callback_context,
         const PublishEvent<double>&) {
        const auto& sys = dynamic_cast<const LcmLogPlaybackSystem&>(system);
        // Use a loop to publish all messages that occur at the exact same time.
        while (sys.log_->GetNextMessageTime() == callback_context.get_time()) {
          sys.log_->DispatchMessageAndAdvanceLog(callback_context.get_time());
        }
      };

  // Schedule a publish event at the next message time.
  *time = next_message_time;
  events->get_mutable_publish_events().add_event(
      systems::PublishEvent<double>(
          TriggerType::kTimed, callback));
}

}  // namespace lcm
}  // namespace systems
}  // namespace drake
