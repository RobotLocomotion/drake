#pragma once

#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/lcm/drake_lcm_message_handler_interface.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_and_vector_base_translator.h"
#include "drake/systems/lcm/lcm_translator_dictionary.h"
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
 * CalcDiscreteVariableUpdates() or CalcUnrestrictedUpdate() depending on the
 * output type. When this system is evaluated by the Simulator, all these
 * operations are taken care of by the Simulator. On the other hand, the user
 * needs to manually replicate this process without the Simulator.
 *
 * @ingroup message_passing
 */
class LcmSubscriberSystem : public LeafSystem<double>,
                            public drake::lcm::DrakeLcmMessageHandlerInterface {
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
   * @param lcm A non-null pointer to the LCM subsystem to subscribe on.
   */
  template <typename LcmMessage>
  static std::unique_ptr<LcmSubscriberSystem> Make(
      const std::string& channel, drake::lcm::DrakeLcmInterface* lcm) {
    return std::make_unique<LcmSubscriberSystem>(
        channel, std::make_unique<Serializer<LcmMessage>>(), lcm);
  }

  /**
   * Constructor that returns a subscriber System that provides message objects
   * on its sole abstract-valued output port.  The type of the message object is
   * determined by the @p serializer.
   *
   * @param[in] channel The LCM channel on which to subscribe.
   *
   * @param[in] serializer The serializer that converts between byte vectors
   * and LCM message objects.
   *
   * @param lcm A non-null pointer to the LCM subsystem to subscribe on.
   */
  LcmSubscriberSystem(const std::string& channel,
                      std::unique_ptr<SerializerInterface> serializer,
                      drake::lcm::DrakeLcmInterface* lcm);

  /**
   * Constructor that returns a subscriber System that provides vector data on
   * its sole vector-valued output port.  The message contents are mapped to
   * vector data by the @p translator.
   *
   * @param[in] channel The LCM channel on which to subscribe.
   *
   * @param[in] translator A reference to the translator that converts between
   * LCM message objects and `drake::systems::VectorBase` objects. The
   * reference must remain valid for the lifetime of this `LcmSubscriberSystem`
   * object.
   *
   * @param lcm A non-null pointer to the LCM subsystem to subscribe on.
   */
  LcmSubscriberSystem(const std::string& channel,
                      const LcmAndVectorBaseTranslator& translator,
                      drake::lcm::DrakeLcmInterface* lcm);

  /**
   * Constructor that returns a subscriber System that provides vector data on
   * its sole vector-valued output port.  The message contents are mapped to
   * vector data by the a translator found in the @p translator_dictionary.
   *
   * @param[in] channel The LCM channel on which to subscribe.
   *
   * @param[in] translator_dictionary A dictionary for obtaining the appropriate
   * translator for a particular LCM channel. The reference must remain valid
   * for the lifetime of this `LcmSubscriberSystem` object.
   *
   * @param lcm A non-null pointer to the LCM subsystem to subscribe on.
   */
  LcmSubscriberSystem(const std::string& channel,
                      const LcmTranslatorDictionary& translator_dictionary,
                      drake::lcm::DrakeLcmInterface* lcm);

  ~LcmSubscriberSystem() override;

  /// Returns the default name for a system that subscribes to @p channel.
  static std::string make_name(const std::string& channel);

  const std::string& get_channel_name() const;

  /**
   * Returns the translator used by this subscriber. This translator can be used
   * to translate a BasicVector into a serialized LCM message, which is then
   * passed to DrakeMockLcm::InduceSubscriberCallback(). This mimics a message
   * reception by an LCM subscriber and is useful for unit testing.
   * @pre this system is using a vector-valued port (not abstract-valued).
   */
  const LcmAndVectorBaseTranslator& get_translator() const;

  /**
   * Blocks the caller until @p old_message_count is different from the
   * internal message counter, and the internal message counter is returned.
   */
  int WaitForMessage(int old_message_count) const;

  /**
   * Returns the message counter stored in @p context.
   */
  int GetMessageCount(const Context<double>& context) const;

 protected:
  void DoCalcNextUpdateTime(const Context<double>& context,
                            systems::CompositeEventCollection<double>* events,
                            double* time) const override;

  void DoCalcUnrestrictedUpdate(
      const Context<double>&,
      const std::vector<const systems::UnrestrictedUpdateEvent<double>*>&,
      State<double>* state) const override {
    ProcessMessageAndStoreToAbstractState(&state->get_mutable_abstract_state());
  }

  std::unique_ptr<AbstractValues> AllocateAbstractState() const override;

  void DoCalcDiscreteVariableUpdates(
      const Context<double>&,
      const std::vector<const systems::DiscreteUpdateEvent<double>*>&,
      DiscreteValues<double>* discrete_state) const override {
    ProcessMessageAndStoreToDiscreteState(discrete_state);
  }

  std::unique_ptr<DiscreteValues<double>> AllocateDiscreteState()
      const override;

  void SetDefaultState(const Context<double>& context,
                       State<double>* state) const override;

 private:
  // All constructors delegate to here.
  LcmSubscriberSystem(const std::string& channel,
                      const LcmAndVectorBaseTranslator* translator,
                      std::unique_ptr<SerializerInterface> serializer,
                      drake::lcm::DrakeLcmInterface* lcm);

  void ProcessMessageAndStoreToDiscreteState(
      DiscreteValues<double>* discrete_state) const;

  void ProcessMessageAndStoreToAbstractState(
      AbstractValues* abstract_state) const;

  // Callback entry point from LCM into this class. Also wakes up one thread
  // block on notification_ if it's not nullptr.
  void HandleMessage(const std::string& channel, const void* message_buffer,
                     int message_size) override;

  // This pair of methods is used for the output port when we're using a
  // translator.
  std::unique_ptr<BasicVector<double>> AllocateTranslatorOutputValue() const;
  void CalcTranslatorOutputValue(const Context<double>& context,
                                 BasicVector<double>* output_vector) const;

  // This pair of methods is used for the output port when we're using a
  // serializer.
  std::unique_ptr<systems::AbstractValue> AllocateSerializerOutputValue() const;
  void CalcSerializerOutputValue(const Context<double>& context,
                                 AbstractValue* output_value) const;

  // The channel on which to receive LCM messages.
  const std::string channel_;

  // Converts LCM message bytes to VectorBase objects.
  // Will be non-null iff our output port is vector-valued.
  const LcmAndVectorBaseTranslator* const translator_{};

  // Converts LCM message bytes to Value<LcmMessage> objects.
  // Will be non-null iff our output port is abstract-valued.
  const std::unique_ptr<SerializerInterface> serializer_;

  // The mutex that guards received_message_ and received_message_count_.
  mutable std::mutex received_message_mutex_;

  // A condition variable that's signaled every time the handler is called.
  mutable std::condition_variable received_message_condition_variable_;

  // The bytes of the most recently received LCM message.
  std::vector<uint8_t> received_message_;

  // A message counter that's incremented every time the handler is called.
  int received_message_count_{0};

  drake::lcm::DrakeLcmInterface* lcm_interface_;
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake
