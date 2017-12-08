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
 * is explicitly slaved to the time in the received Lcm message. This class is
 * intended to provide a generalized way to implement a message handling loop:
 * an input message arrives, from which a response is computed and published.
 * A common use case is to implement a distributed controller for a physical
 * robot, where low level communication with the hardware is handled in the
 * device driver (a separate process than the controller). The device driver
 * sends a message containing the estimated state. The controller processes
 * that message and sends back a command in response. The device driver finally
 * receives and executes the command.
 *
 * This class is designed to be agnostic to different types of the driving Lcm
 * message to provide a generic API. The Lcm message is internally encapsulated
 * in AbstractValue, which erases its type. In addition, the message time stamp
 * is the only required information by this class, which can be extracted by
 * an instance of LcmMessageToTimeInterface. It is assumed that the caller knows
 * the concrete type of the message, and is able to supply a time converter.
 *
 * This class uses the Simulator class internally for event handling
 * (kPublishAction, kDiscreteUpdateAction, kUnrestrictedUpdateAction) and
 * continuous state integration (e.g. the I term in a PID). The main message
 * handling loop conceptually is:
 * <pre>
 * while(context.time < stop_time) {
 *   msg = wait_for_message("channel");
 *   simulator.StepTo(msg.time);
 *   if (publish) {
 *     system.Publish(simulator.context);
 *   }
 * }
 * </pre>
 *
 * Since time is slaved to some outside source, the user needs to be mindful of
 * `system`'s configuration especially about event timing. For example, let us
 * assume that `system` is configured to perform a discrete time action every
 * 5ms (at 200Hz), and the necessary computation for `system` to step forward
 * in time is very small. Now, suppose the driving message arrives at 1 Hz in
 * real time. One would observe 200 such actions occur in rapid succession
 * followed by nearly one second of silence. This is because
 * `msg = wait_for_message("channel")` takes about one second in real time,
 * and `simulator.StepTo(msg.time)`, which forwards the simulator's clock by
 * one second and performs 200 actions takes about 0 seconds in real time.
 * The root cause is that the 200Hz rate of the handler system is tied to the
 * internal virtual clock rather than real time. This problem is less
 * significant when the computation time for handling one message is roughly
 * the same as the interval between consecutive driving messages.
 *
 * This implementation relies on several assumptions:
 * 1. The loop is blocked only on one Lcm message.
 * 2. It's pointless to for the handler system to perform any computation
 * without a new Lcm message, thus the handler loop is blocking.
 * 3. The computation for the given system should be faster than the incoming
 * message rate.
 */
class LcmDrivenLoop {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LcmDrivenLoop)

  /**
   * Constructor.
   * @param system Const reference to the handler system. Its life span must be
   * longer than `this`.
   * @param driving_subscriber Const reference to the driving subscriber. Its
   * life span must be longer than `this`.
   * @param context Unique pointer to a context allocated for @p system. Can be
   * nullptr, in which case a context will be allocated internally.
   * @param lcm Pointer to Lcm interface. Its life span must be longer than
   * `this`. @p lcm cannot be nullptr, otherwise aborts.
   * @param time_converter Unique pointer to a converter that extracts time in
   * seconds from the driving message time. Cannot be nullptr, otherwise `this`
   * aborts. @p time_converter is necessary because of two reasons. 1: The Lcm
   * message type agnostic design of this class. 2: Lcm messages lack a uniform
   * time stamp field that has consistent units. So extracting the time stamp
   * depends on the concrete message content.
   */
  LcmDrivenLoop(const System<double>& system,
                const LcmSubscriberSystem& driving_subscriber,
                std::unique_ptr<Context<double>> context,
                drake::lcm::DrakeLcm* lcm,
                std::unique_ptr<LcmMessageToTimeInterface> time_converter);

  /**
   * Blocks the caller until a driving Lcm message is received, then returns
   * a const reference of AbstractValue to that message. The call is assumed
   * to know the type of the actual message and have means to inspect the
   * message.
   */
  // TODO(siyuan): add a time out version.
  const AbstractValue& WaitForMessage();

  /**
   * Starts the message handling loop assuming the context (e.g. state and
   * time) has already been properly initialized by the caller if necessary.
   * @param stop_time End time in seconds relative to the time stamp in the
   * driving Lcm message.
   */
  void RunToSecondsAssumingInitialized(
      double stop_time = std::numeric_limits<double>::infinity());

  /**
   * Sets a flag that forces Publish() at the very beginning of the message
   * handling loop as well as inside the loop. To achieve "publish whenever
   * a new message has been handled", the user needs to make sure that no
   * subsystems have declared period publish, and @p flag is true.
   */
  void set_publish_on_every_received_message(bool flag) {
    publish_on_every_received_message_ = flag;
  }

  /**
   * Returns a mutable reference to the context.
   */
  Context<double>& get_mutable_context() {
    return stepper_->get_mutable_context();
  }

  /**
   * Returns a const reference to the message to seconds converter.
   */
  const LcmMessageToTimeInterface& get_message_to_time_converter() const {
    return *time_converter_;
  }

 private:
  // The handler system.
  const System<double>& system_;

  // The message subscriber.
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
  std::unique_ptr<State<double>> sub_swap_state_;
  std::unique_ptr<CompositeEventCollection<double>> sub_events_;
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake
