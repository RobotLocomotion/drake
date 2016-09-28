#pragma once

#include <mutex>

#include <lcm/lcm-cpp.hpp>

#include "drake/drakeLCMSystem2_export.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_and_vector_base_translator.h"
#include "drake/systems/lcm/lcm_receive_thread.h"
#include "drake/systems/lcm/lcm_translator_dictionary.h"

namespace drake {
namespace systems {
namespace lcm {

/**
 * Receives LCM messages from a given channel and outputs them to a
 * System<double>'s port. The output port value is the most recently
 * decoded message, modulo any network or threading delays.
 */
class DRAKELCMSYSTEM2_EXPORT LcmSubscriberSystem : public LeafSystem<double> {
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
                      ::lcm::LCM* lcm);

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
                      ::lcm::LCM* lcm);

  ~LcmSubscriberSystem() override;

  std::string get_name() const override;

  /// Returns the default name for a system that subscribes to @p channel.
  static std::string get_name(const std::string& channel);

  void EvalOutput(const Context<double>& context,
                  SystemOutput<double>* output) const override;

  /**
   * Sets the `message_bytes` that will provide the value for `EvalOutput`;
   * typically only used for unit testing.
   *
   * This class's constructors subscribe to an `LCM` channel that provides the
   * values for `EvalOutput`.  However, if `LCM` is not providing any message
   * data (e.g., in a unit test, or if the channel is not being published
   * during a simulation), this method can be used to provide a value.
   *
   * When both `LCM` and `SetMessage` are updating the output value, the most
   * recent update wins.
   */
  void SetMessage(std::vector<uint8_t> message_bytes);

  /**
   * Sets the message vector that will provide the value for `EvalOutput`;
   * typically only used for unit testing.  The value will come translating the
   * given @p time and @p message_vector to bytes, which are then stored and
   * for decoding.
   *
   * This class's constructors subscribe to an `LCM` channel that provides the
   * values for `EvalOutput`.  However, if `LCM` is not providing any message
   * data (e.g., in a unit test, or if the channel is not being published
   * during a simulation), this method can be used to provide a value.
   *
   * When both `LCM` and `SetMessage` are updating the output value, the most
   * recent update wins.
   */
  void SetMessage(double time, const BasicVector<double>& message_vector);

 protected:
  std::unique_ptr<BasicVector<double>> AllocateOutputVector(
      const SystemPortDescriptor<double>& descriptor) const override;

 private:
  // Callback entry point from LCM into this class.
  void HandleMessage(const ::lcm::ReceiveBuffer* rbuf,
                     const std::string& channel);

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
