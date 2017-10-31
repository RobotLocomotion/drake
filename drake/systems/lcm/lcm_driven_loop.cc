#include "drake/systems/lcm/lcm_driven_loop.h"

namespace drake {
namespace systems {
namespace lcm {

LcmDrivenLoop::LcmDrivenLoop(
    const System<double>& system, const LcmSubscriberSystem& driving_subscriber,
    std::unique_ptr<Context<double>> context, drake::lcm::DrakeLcm* lcm,
    std::unique_ptr<LcmMessageToTimeInterface> time_converter)
    : system_(system),
      driving_sub_(driving_subscriber),
      lcm_(lcm),
      time_converter_(std::move(time_converter)),
      stepper_(
          std::make_unique<Simulator<double>>(system_, std::move(context))) {
  DRAKE_DEMAND(lcm != nullptr);
  DRAKE_DEMAND(time_converter_ != nullptr);

  // Allocates extra context and output just for the driving subscriber, so
  // that this can explicitly query the message.
  sub_context_ = driving_sub_.CreateDefaultContext();
  sub_output_ = driving_sub_.AllocateOutput(*sub_context_);
  sub_swap_state_ = sub_context_->CloneState();
  sub_events_ = driving_sub_.AllocateCompositeEventCollection();

  // Disables simulator's publish on its internal time step.
  stepper_->set_publish_every_time_step(false);
  stepper_->set_publish_at_initialization(false);

  stepper_->Initialize();

  // Starts the subscribing thread.
  lcm_->StartReceiveThread();
}

const AbstractValue& LcmDrivenLoop::WaitForMessage() {
  driving_sub_.WaitForMessage(driving_sub_.GetMessageCount(*sub_context_));

  driving_sub_.CalcNextUpdateTime(*sub_context_, sub_events_.get());

  // If driving_sub_.WaitForMessage() returned, a message should be received
  // and an event should be queued by driving_sub_.CalcNextUpdateTime().
  if (sub_events_->HasDiscreteUpdateEvents()) {
    driving_sub_.CalcDiscreteVariableUpdates(
        *sub_context_, sub_events_->get_discrete_update_events(),
        &sub_swap_state_->get_mutable_discrete_state());
  } else if (sub_events_->HasUnrestrictedUpdateEvents()) {
    driving_sub_.CalcUnrestrictedUpdate(*sub_context_,
        sub_events_->get_unrestricted_update_events(), sub_swap_state_.get());
  } else {
    DRAKE_DEMAND(false);
  }
  sub_context_->get_mutable_state().CopyFrom(*sub_swap_state_);

  driving_sub_.CalcOutput(*sub_context_, sub_output_.get());
  return *(sub_output_->get_data(0));
}

void LcmDrivenLoop::RunToSecondsAssumingInitialized(double stop_time) {
  double msg_time;

  while (true) {
    msg_time = time_converter_->GetTimeInSeconds(WaitForMessage());
    if (msg_time >= stop_time) break;

    stepper_->StepTo(msg_time);

    // Explicitly publish after we are done with all the intermediate
    // computation.
    if (publish_on_every_received_message_) {
      system_.Publish(stepper_->get_context());
    }
  }
}

}  // namespace lcm
}  // namespace systems
}  // namespace drake
