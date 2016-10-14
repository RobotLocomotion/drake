#pragma once

#include <mutex>

#include "drake/common/drake_export.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/lcm/drake_lcm_message_handler_interface.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_and_vector_base_translator.h"
#include "drake/systems/lcm/lcm_translator_dictionary.h"

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
   * A constructor.
   *
   * @param[in] channel The LCM channel on which to subscribe.
   *
   * @param[in] translator A reference to the translator that converts between
   * LCM message objects and `drake::systems::VectorBase` objects. This
   * reference must remain valid for the lifetime of this `LcmSubscriberSystem`
   * object.
   *
   * @param[in] lcm A pointer to the LCM subsystem. This pointer must not be
   * null and must be valid during the construction of this
   * `LcmSubscriberSystem`.
   */
  LcmSubscriberSystem(const std::string& channel,
                      const LcmAndVectorBaseTranslator& translator,
                      drake::lcm::DrakeLcmInterface* lcm);

  /**
   * A constructor.
   *
   * @param[in] channel The LCM channel on which to subscribe.
   *
   * @param[in] translator_dictionary A dictionary for obtaining the appropriate
   * translator for a particular LCM channel.
   *
   * @param[in] lcm A pointer to the LCM subsystem. This pointer must not be
   * null.
   */
  LcmSubscriberSystem(const std::string& channel,
                      const LcmTranslatorDictionary& translator_dictionary,
                      drake::lcm::DrakeLcmInterface* lcm);

  ~LcmSubscriberSystem() override;

  std::string get_name() const override;

  /// Returns the default name for a system that subscribes to @p channel.
  static std::string get_name(const std::string& channel);

  const std::string& get_channel_name() const;

  void EvalOutput(const Context<double>& context,
                  SystemOutput<double>* output) const override;

  /**
   * Returns the translator used by this subscriber. This translator can be used
   * to translate a BasicVector into a serialized LCM message, which is then
   * passed to DrakeMockLcm::InduceSubscriberCallback(). This mimics a message
   * reception by an LCM subscriber and is useful for unit testing.
   */
  const LcmAndVectorBaseTranslator& get_translator() const;

 protected:
  std::unique_ptr<BasicVector<double>> AllocateOutputVector(
      const SystemPortDescriptor<double>& descriptor) const override;

 private:
  // Callback entry point from LCM into this class.
  void HandleMessage(const std::string& channel, const void* message_buffer,
      int message_size) override;

  // The channel on which to receive LCM messages.
  const std::string channel_;

  // The translator that converts between LCM messages and
  // drake::systems::VectorBase objects.
  const LcmAndVectorBaseTranslator& translator_;

  // The mutex that guards received_message_.
  mutable std::mutex received_message_mutex_;

  // The bytes of the most recently received LCM message.
  std::vector<uint8_t> received_message_;

  // Disable copy and assign.
  LcmSubscriberSystem(const LcmSubscriberSystem&) = delete;
  LcmSubscriberSystem& operator=(const LcmSubscriberSystem&) = delete;
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake
