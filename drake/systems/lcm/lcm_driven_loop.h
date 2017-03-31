#pragma once

#include <limits>
#include <memory>
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
 * their system's configuration especially about event timing, since time is
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
 * TODO(siyuan): Fix this:
 * LcmSubscriberSystem's output is not guaranteed to be the same given the same
 * context. E.g. if a Lcm messages comes in between two calls to CalcOutput, the
 * second call would return the new message while the "state" in the context
 * remains the same. The LcmSubscriberSystem will be fixed.
 */
class LcmDrivenLoop {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LcmDrivenLoop)

  /**
   * Constructor.
   * @param system Const reference to the handler system. Its life span must be
   * longer than this.
   * @param driving_subscriber Const reference to the driving subscriber. Its
   * life span must be longer than this.
   * @param context Unique pointer to a context allocated for @p system.
   * @param lcm Pointer to Lcm interface. Cannot be nullptr. Its life span must
   * be longer than this.
   * @param time_converter Unique pointer to a converter that extracts time in
   * seconds from the driving message time. Cannot be nullptr.
   */
  LcmDrivenLoop(const System<double>& system,
                const LcmSubscriberSystem& driving_subscriber,
                std::unique_ptr<Context<double>> context,
                drake::lcm::DrakeLcm* lcm,
                std::unique_ptr<LcmMessageToTimeInterface> time_converter);

  /**
   * Returns a const reference of AbstractValue that contains the received Lcm
   * message.
   */
  const AbstractValue& WaitForMessage();

  /**
   * Starts the main loop assuming that the context (e.g. state and time) has
   * already been properly initialized by the caller. This version is useful
   * if the underlying System needs custom initialization depending on the first
   * received Lcm message.
   */
  void RunAssumingInitializedTo(
      double stop_time = std::numeric_limits<double>::infinity());

  /**
   * Waits for the first Lcm message, sets the context's time to the message's
   * time, then calls RunAssumingInitliazedTo(). This is convenient if no
   * special initialization is required by the underlying system.
   */
  void RunWithDefaultInitializationTo(
      double stop_time = std::numeric_limits<double>::infinity());

  /**
   * If @p flag is set to true, the main loop will explicitly call Publish()
   * on the given system after it handles one driving message.
   *
   * To correctly implement "publish everything whenever a new message has
   * been handled", the user needs to make sure that no subsystems in the
   * given system have declared period publish, and @p flag is true.
   * This is the recommended publish setup because of its simplicity, and
   * suites typical "one state message in, one control message out " use case.
   *
   * Alternatively, the user can set @p flag to false, and declare periodic
   * publish events for all the appropriate subsystems. However, the actual
   * publishing behavior strongly depends on the timing of the driving message
   * and computation in message handling, and the publish is unlikely to be
   * uniform. Suppose the driving message comes at 1 hz, and the message
   * handlers take very little time to handle one message, and are declared to
   * be publishing at 200hz. What you should observe is that no messages will
   * be published for 1 second, and 200 messages will be published almost at
   * once. This is most likely not the intended behavior, and indicates some
   * basic assumptions might be incorrect.
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

  // THE message subscriber.
  const LcmSubscriberSystem& driving_sub_;

  // The lcm interface for publishing and subscribing.
  drake::lcm::DrakeLcm* lcm_;

  // Extracts time in seconds from received lcm messages.
  std::unique_ptr<LcmMessageToTimeInterface> time_converter_;

  // If true, explicitly calls system_.Publish() after every step in the loop.
  bool publish_on_every_received_message_{true};

  // Reusing the simulator to manage event handling and state progression.
  std::unique_ptr<Simulator<double>> stepper_;

  // Separate context and output port for the driving subscriber.
  std::unique_ptr<Context<double>> sub_context_;
  std::unique_ptr<SystemOutput<double>> sub_output_;

  int message_count_{0};
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake
