#include "drake/systems/lcm/lcm_log_playback_system.h"

namespace drake {
namespace systems {
namespace lcm {

LcmLogPlaybackSystem::LcmLogPlaybackSystem(drake::lcm::DrakeLcmLog* log) : log_(log) {
  DRAKE_DEMAND(log_ != nullptr);

  if (log_->is_write_only()) {
    throw std::runtime_error("Log must be read only.");
  }
}

void LcmLogPlaybackSystem::DoCalcNextUpdateTime(
    const Context<double>&,
    systems::CompositeEventCollection<double>* events,
    double* time) const {

  *time = log_->GetNextMessageTime();
  EventCollection<PublishEvent<double>>& pub_events =
      events->get_mutable_publish_events();

  PublishEvent<double>::PublishCallback callback =
    [this](const Context<double>& context, const PublishEvent<double>&) {
      DRAKE_DEMAND(context.get_time() == log_->GetNextMessageTime());
      log_->DispatchMessageToAllSubscribers();
      log_->AdvanceLog();
    };

  pub_events.add_event(
      std::make_unique<systems::PublishEvent<double>>(
          Event<double>::TriggerType::kTimed,
          callback));
}

}  // namespace lcm
}  // namespace systems
}  // namespace drake
