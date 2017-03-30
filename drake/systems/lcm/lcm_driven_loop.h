#pragma once

#include <memory>
#include <limits>
#include <utility>

#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace drake {
namespace systems {
namespace lcm {

/**
 * A generic translator interface that extracts time in seconds from an
 * abstract type.
 */
class LcmMessageToTimeInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LcmMessageToTimeInterface)
  virtual ~LcmMessageToTimeInterface() {}

  virtual double GetTimeInSeconds(
      const AbstractValue& abstract_value) const = 0;

 protected:
  LcmMessageToTimeInterface() {}
};

/**
 * A translator class for Lcm message types that have a "utime" field, which
 * is in micro seconds.
 */
template <typename MessageType>
class UtimeMessageToSeconds : public LcmMessageToTimeInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(UtimeMessageToSeconds)
  UtimeMessageToSeconds() {}
  ~UtimeMessageToSeconds() {}

  double GetTimeInSeconds(const AbstractValue& abstract_value) const override {
    const MessageType& msg = abstract_value.GetValue<MessageType>();
    return static_cast<double>(msg.utime) / 1e6;
  }
};

/**
 * This class implements a loop that's driven by a Lcm message. The context time
 * is explicitly slaved to the time in the received Lcm message. Some states are
 * also likely to be slaved to the message implicitly (e.g. robot states). This
 * is intended to implement a control loop that handles a state message from
 * the network, computes some numbers and then sends some command out.
 *
 * This class uses the Simulator class internally for event handling
 * (kPublishAction, kDiscreteUpdateAction, kUnrestrictedUpdateAction) and
 * state "integration" (e.g. I term in a PID). The user needs to be mindful of
 * how
 * their system is configured, especially about event timing since time is
 * slaved to some outside source.
 *
 * The main message handling loop is roughly:
 * <pre>
 * msg = wait_for_message("channel");
 * simulator.context.Initilaize(msg);
 * while(context.time < T_max) {
 *   msg = wait_for_message("channel");
 *   simulator.StepTo(msg.time);
 *   if (publish) {
 *     system.Publish(simulator.context);
 *   }
 * }
 * </pre>
 *
 * There are a couple assumptions in this implementation.
 * 1. The loop is blocked only on one Lcm message.
 * 2. It's pointless to recompute without a new Lcm message, thus the handler
 * loop is blocking.
 * 3. The computation for the given system should be faster than the incoming
 * message rate.
 *
 * Subtle things about lcm subscribers:
 * LcmSubscriberSystem's output is not guaranteed to be the same given the same
 * context. E.g. if a Lcm messages comes in between two calls to CalcOutput, the
 * second call would return the new message while the "state" in the context
 * remains the same. The LcmSubscriberSystem will be fixed.
 */
class LcmDrivenLoop {
 public:
  /**
   * Constructor.
   * @param system Const reference to the handler system.
   * @param context Unique pointer to a context allocated for @p system.
   * @param lcm Pointer to Lcm interface. Cannot be nullptr.
   * @param driving_subscriber Pointer to the driving subscriber. Cannot be
   * nullptr.
   * @param time_converter Unique pointer to a converter that extracts time in
   * seconds from the driving message time. Cannot be nullptr.
   */
  LcmDrivenLoop(const System<double>& system,
                std::unique_ptr<Context<double>> context,
                drake::lcm::DrakeLcm* lcm,
                LcmSubscriberSystem* driving_subscriber,
                std::unique_ptr<LcmMessageToTimeInterface> time_converter)
      : system_(system),
        lcm_(lcm),
        time_converter_(std::move(time_converter)),
        semaphore_(1),
        driving_sub_(driving_subscriber),
        stepper_(
            std::make_unique<Simulator<double>>(system_, std::move(context))) {
    DRAKE_DEMAND(lcm != nullptr);
    DRAKE_DEMAND(driving_subscriber != nullptr);
    DRAKE_DEMAND(time_converter_ != nullptr);

    // Tells the driving subscriber to wake this up when it gets a new message.
    driving_sub_->set_notification(&semaphore_);

    // Allocates extra context and output just for the driving subscriber, so
    // that this can explicitly query the message.
    sub_context_ = driving_sub_->CreateDefaultContext();
    sub_output_ = driving_sub_->AllocateOutput(*sub_context_);

    // Disables simulator's publish on its internal time step.
    stepper_->set_publish_every_time_step(false);

    // Starts the subscribing thread.
    lcm_->StartReceiveThread();
  }

  /**
   * Returns a const reference of AbstractValue that contains the received Lcm
   * message.
   */
  const AbstractValue& WaitForMessage() {
    semaphore_.wait();
    driving_sub_->CalcOutput(*sub_context_, sub_output_.get());
    return *(sub_output_->get_data(0));
  }

  /**
   * Starts the main loop assuming that the context (e.g. state and time) has
   * already been properly initialized by the caller. This version is useful
   * if the underly System needs custom initialization depending on the first
   * received Lcm message.
   */
  void RunAssumingInitializedTo(
      double stop_time = std::numeric_limits<double>::infinity()) {
    double msg_time;
    stepper_->Initialize();

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

  /**
   * Waits for the first Lcm message, sets the context's time to the message's
   * time, then calls RunAssumingInitliazedTo(). This is convenient if no
   * special initialization is required by the underlying system.
   */
  void RunWithDefaultInitializationTo(
      double stop_time = std::numeric_limits<double>::infinity()) {
    const AbstractValue& first_msg = WaitForMessage();
    double msg_time = time_converter_->GetTimeInSeconds(first_msg);
    // Init our time to the msg time.
    stepper_->get_mutable_context()->set_time(msg_time);

    RunAssumingInitializedTo(stop_time);
  }

  /**
   * If @p flag is set to true, the main loop will explicitly have all
   * subsystems publish after it handles one driving message. The user should
   * either make sure ((no subsystems have DeclarePublishPeriodSec() or
   * set_publish_period() calls) and @p flag being ture), or
   * ((every subsytem have there desired publish rates set properly with
   * DeclarePublishPeriodSec() or set_publish_period() calls) and @p flag
   * being false) to avoid repeated publishing. The former corresponds to
   * "publish everything whenever a new message has been handled". The
   * latrer roughly corresponds to "All subsystems publish on their declared
   * rates." However, this strongly depends on the timing of the driving
   * message and computation in message handling, and the publish is likely
   * to be not uniform. Suppose the driving message comes at 1 hz, and the
   * message handlers take very little time to handle one message, and are
   * declared to be publishing at 200hz. What you should oberserve is that no
   * messages will be published for 1 second, and 200 messages will be
   * published almost at once. This is most likely not the intended behavior,
   * and indicates some basic assumptions are probably wrong.
   *
   * The first setup is recommended, because it fits the intuitive "one state
   * in, one control out" picture better. The second setup is also fine given
   * everything happens "fast enough".
   */
  void set_publish_on_every_received_message(bool flag) {
    publish_on_every_received_message_ = flag;
  }

  /**
   * Returns a mutable pointer to the context.
   */
  Context<double>* get_mutable_context() {
    return stepper_->get_mutable_context();
  }

  /**
   * Returns a const reference to the message to seconds converter.
   */
  const LcmMessageToTimeInterface& get_message_to_time_converter() const {
    return *time_converter_;
  }

 private:
  // The system that does stuff.
  const System<double>& system_;

  // The lcm interface for publishing and subscribing.
  drake::lcm::DrakeLcm* lcm_;

  // Extracts time in seconds from received lcm messages.
  std::unique_ptr<LcmMessageToTimeInterface> time_converter_;

  // If true, explicitly calls system_.Publish() after every step in the loop.
  bool publish_on_every_received_message_{true};

  // The semaphore that blocks the loop thread until a message triggers
  // driving_sub_'s message handler.
  Semaphore semaphore_;

  // THE message subscriber.
  LcmSubscriberSystem* driving_sub_;

  // Reusing the simulator to manage event handling and state progression.
  std::unique_ptr<Simulator<double>> stepper_;

  // Separate context and output port for the driving subscriber.
  std::unique_ptr<Context<double>> sub_context_;
  std::unique_ptr<SystemOutput<double>> sub_output_;
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake
