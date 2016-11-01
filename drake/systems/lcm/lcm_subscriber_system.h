#pragma once

#include <mutex>
#include <string>
#include <vector>

#include "drake/common/drake_export.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/lcm/drake_lcm_message_handler_interface.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_and_vector_base_translator.h"
#include "drake/systems/lcm/lcm_translator_dictionary.h"
#include "drake/systems/lcm/serializer.h"

namespace drake {
namespace systems {
namespace lcm {

/**
 * Receives LCM messages from a given channel and outputs them to a
 * System<double>'s port. The output port value is the most recently
 * decoded message, modulo any network or threading delays.
 */
class DRAKE_EXPORT LcmSubscriberSystem : public LeafSystem<double>,
    public drake::lcm::DrakeLcmMessageHandlerInterface  {
 public:
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
      const std::string& channel,
      drake::lcm::DrakeLcmInterface* lcm) {
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
   * LCM message objects and `drake::systems::VectorBase` objects. This
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
   * translator for a particular LCM channel.
   *
   * @param lcm A non-null pointer to the LCM subsystem to subscribe on.
   */
  LcmSubscriberSystem(const std::string& channel,
                      const LcmTranslatorDictionary& translator_dictionary,
                      drake::lcm::DrakeLcmInterface* lcm);

  ~LcmSubscriberSystem() override;

  std::string get_name() const override;

  /// Returns the default name for a system that subscribes to @p channel.
  static std::string get_name(const std::string& channel);

  const std::string& get_channel_name() const;

  std::unique_ptr<SystemOutput<double>> AllocateOutput(
      const Context<double>& context) const override;

  void EvalOutput(const Context<double>& context,
                  SystemOutput<double>* output) const override;

  /**
   * Returns the translator used by this subscriber. This translator can be used
   * to translate a BasicVector into a serialized LCM message, which is then
   * passed to DrakeMockLcm::InduceSubscriberCallback(). This mimics a message
   * reception by an LCM subscriber and is useful for unit testing.
   * @pre this system is using a vector-valued port (not abstract-valued).
   */
  const LcmAndVectorBaseTranslator& get_translator() const;

  // Disable copy and assign.
  LcmSubscriberSystem(const LcmSubscriberSystem&) = delete;
  LcmSubscriberSystem& operator=(const LcmSubscriberSystem&) = delete;

 protected:
  std::unique_ptr<BasicVector<double>> AllocateOutputVector(
      const SystemPortDescriptor<double>& descriptor) const override;

 private:
  // All constructors delegate to here.
  LcmSubscriberSystem(const std::string& channel,
                      const LcmAndVectorBaseTranslator* translator,
                      std::unique_ptr<SerializerInterface> serializer,
                      drake::lcm::DrakeLcmInterface* lcm);

  // Callback entry point from LCM into this class.
  void HandleMessage(const std::string& channel, const void* message_buffer,
      int message_size) override;

  // The channel on which to receive LCM messages.
  const std::string channel_;

  // Converts LCM message bytes to VectorBase objects.
  // Will be non-null iff our output port is vector-valued.
  const LcmAndVectorBaseTranslator* const translator_{};

  // Converts LCM message bytes to Value<LcmMessage> objects.
  // Will be non-null iff our output port is abstract-valued.
  const std::unique_ptr<SerializerInterface> serializer_;

  // The mutex that guards received_message_.
  mutable std::mutex received_message_mutex_;

  // The bytes of the most recently received LCM message.
  std::vector<uint8_t> received_message_;
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake
