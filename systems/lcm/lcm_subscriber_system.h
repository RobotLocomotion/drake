#pragma once

#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/drake_throw.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/serializer.h"

namespace drake {
namespace systems {
namespace lcm {

/**
 * Receives LCM messages from a given channel and outputs them to a
 * System<double>'s port. This class stores the most recently processed LCM
 * message in the State. When a LCM message arrives asynchronously, an update
 * event is scheduled to process the message and store it in the State at the
 * earliest possible simulation time. The output is always consistent with the
 * State.
 *
 * To process a LCM message, CalcNextUpdateTime() needs to be called first to
 * check for new messages and schedule a callback event if a new LCM message
 * has arrived. The message is then processed and stored in the Context by
 * CalcUnrestrictedUpdate(). When this system is evaluated by the Simulator,
 * all these operations are taken care of by the Simulator. On the other hand,
 * the user needs to manually replicate this process without the Simulator.
 *
 * If LCM service in use is a drake::lcm::DrakeLcmLog (not live operation),
 * then see drake::systems::lcm::LcmLogPlaybackSystem for a helper to advance
 * the log cursor in concert with the simulation.
 *
 * @system
 * name: LcmSubscriberSystem
 * output_ports:
 * - y0
 * @endsystem
 *
 * @ingroup message_passing
 */
class LcmSubscriberSystem : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LcmSubscriberSystem)

  /**
   * Factory method that returns a subscriber System that provides
   * Value<LcmMessage> message objects on its sole abstract-valued output port.
   *
   * @tparam LcmMessage message type to deserialize, e.g., lcmt_drake_signal.
   *
   * @param[in] channel The LCM channel on which to subscribe.
   *
   * @param lcm A non-null pointer to the LCM subsystem to subscribe on. If
   * `wait_for_message_on_initialization_timeout > 0`, then the pointer must
   * remain valid for the lifetime of the returned system.
   *
   * @param wait_for_message_on_initialization_timeout Configures the behavior
   * of initialization events (see System::ExecuteInitializationEvents() and
   * Simulator::Initialize()) by specifying the number of seconds (wall-clock
   * elapsed time) to wait for a new message. If this timeout is <= 0,
   * initialization will copy any already-received messages into the Context but
   * will not process any new messages. If this timeout is > 0, initialization
   * will call lcm->HandleSubscriptions() until at least one message is received
   * or until the timeout. Pass ∞ to wait indefinitely.
   */
  template <typename LcmMessage>
  static std::unique_ptr<LcmSubscriberSystem> Make(
      const std::string& channel, drake::lcm::DrakeLcmInterface* lcm,
      double wait_for_message_on_initialization_timeout = 0.0) {
    return std::make_unique<LcmSubscriberSystem>(
        channel, std::make_unique<Serializer<LcmMessage>>(), lcm,
        wait_for_message_on_initialization_timeout);
  }

  /**
   * Constructor that returns a subscriber System that provides message objects
   * on its sole abstract-valued output port.  The type of the message object
   * is determined by the @p serializer.
   *
   * @param[in] channel The LCM channel on which to subscribe.
   *
   * @param[in] serializer The serializer that converts between byte vectors
   * and LCM message objects. Cannot be null.
   *
   * @param lcm A non-null pointer to the LCM subsystem to subscribe on. If
   * `wait_for_message_on_initialization_timeout > 0`, then the pointer must
   * remain valid for the lifetime of the returned system.
   *
   * @param wait_for_message_on_initialization_timeout Configures the behavior
   * of initialization events (see System::ExecuteInitializationEvents() and
   * Simulator::Initialize()) by specifying the number of seconds (wall-clock
   * elapsed time) to wait for a new message. If this timeout is <= 0,
   * initialization will copy any already-received messages into the Context but
   * will not process any new messages. If this timeout is > 0, initialization
   * will call lcm->HandleSubscriptions() until at least one message is received
   * or until the timeout. Pass ∞ to wait indefinitely.
   */
  LcmSubscriberSystem(const std::string& channel,
                      std::shared_ptr<const SerializerInterface> serializer,
                      drake::lcm::DrakeLcmInterface* lcm,
                      double wait_for_message_on_initialization_timeout = 0.0);

  ~LcmSubscriberSystem() override;

  /// Returns the default name for a system that subscribes to @p channel.
  static std::string make_name(const std::string& channel);

  const std::string& get_channel_name() const;

  /**
   * Blocks the caller until its internal message count exceeds
   * `old_message_count` with an optional timeout.
   * @param old_message_count Internal message counter.
   *
   * @param message If non-null, will return the received message.
   *
   * @param timeout The duration (in seconds) to wait before returning; a
   * non-positive duration will not time out.
   *
   * @return Returns the new count of received messages. If a timeout occurred,
   * this will be less than or equal to old_message_count.
   *
   * @pre If `message` is specified, this system must be abstract-valued.
   */
  int WaitForMessage(int old_message_count, AbstractValue* message = nullptr,
                     double timeout = -1.) const;

  /**
   * Returns the internal message counter. Meant to be used with
   * `WaitForMessage`.
   */
  int GetInternalMessageCount() const;

  /**
   * Returns the message counter stored in @p context.
   */
  int GetMessageCount(const Context<double>& context) const;

 private:
  // Callback entry point from LCM into this class.
  void HandleMessage(const void*, int);

  void DoCalcNextUpdateTime(const Context<double>& context,
                            systems::CompositeEventCollection<double>* events,
                            double* time) const final;

  EventStatus ProcessMessageAndStoreToAbstractState(const Context<double>&,
                                                    State<double>* state) const;

  EventStatus Initialize(const Context<double>&, State<double>* state) const;

  // The channel on which to receive LCM messages.
  const std::string channel_;

  // Converts LCM message bytes to Value<LcmMessage> objects.
  // Will be non-null iff our output port is abstract-valued.
  const std::shared_ptr<const SerializerInterface> serializer_;

  // The mutex that guards received_message_ and received_message_count_.
  mutable std::mutex received_message_mutex_;

  // A condition variable that's signaled every time the handler is called.
  mutable std::condition_variable received_message_condition_variable_;

  // The bytes of the most recently received LCM message.
  std::vector<uint8_t> received_message_;

  // A message counter that's incremented every time the handler is called.
  int received_message_count_{0};

  // When we are destroyed, our subscription will be automatically removed
  // (if the DrakeLcmInterface supports removal).
  std::shared_ptr<drake::lcm::DrakeSubscriptionInterface> subscription_;

  // A little hint to help catch use-after-free.
  int magic_number_{};

  // The lcm interface is (maybe) used to handle subscriptions during
  // Initialization.
  drake::lcm::DrakeLcmInterface* const lcm_;

  // A timeout in seconds.
  const double wait_for_message_on_initialization_timeout_;
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake
