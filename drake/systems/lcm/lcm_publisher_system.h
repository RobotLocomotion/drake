#pragma once

#include <string>

#include "drake/common/drake_export.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_and_vector_base_translator.h"
#include "drake/systems/lcm/lcm_translator_dictionary.h"
#include "drake/systems/lcm/serializer.h"

namespace drake {
namespace systems {
namespace lcm {

/**
 * Publishes an LCM message containing information from its input port.
 */
class DRAKE_EXPORT LcmPublisherSystem : public LeafSystem<double> {
 public:
  /**
   * Factory method that returns a publisher System that takes
   * Value<LcmMessage> message objects on its sole abstract-valued input port.
   *
   * @tparam LcmMessage message type to serialize, e.g., lcmt_drake_signal.
   *
   * @param[in] channel The LCM channel on which to publish.
   *
   * @param lcm A non-null pointer to the LCM subsystem to publish on.
   * The pointer must remain valid for the lifetime of this object.
   */
  template <typename LcmMessage>
  static std::unique_ptr<LcmPublisherSystem> Make(
      const std::string& channel,
      drake::lcm::DrakeLcmInterface* lcm) {
    return std::make_unique<LcmPublisherSystem>(
        channel, std::make_unique<Serializer<LcmMessage>>(), lcm);
  }

  /**
   * Constructor that returns a publisher System that takes message objects
   * on its sole abstract-valued input port.  The type of the message object is
   * determined by the @p serializer.
   *
   * @param[in] channel The LCM channel on which to publish.
   *
   * @param[in] serializer The serializer that converts between byte vectors
   * and LCM message objects.
   *
   * @param lcm A non-null pointer to the LCM subsystem to publish on.
   * The pointer must remain valid for the lifetime of this object.
   */
  LcmPublisherSystem(const std::string& channel,
                     std::unique_ptr<SerializerInterface> serializer,
                     drake::lcm::DrakeLcmInterface* lcm);

  /**
   * Constructor that returns a publisher System that takes vector data on
   * its sole vector-valued input port.  The vector data are mapped to
   * message contents by the @p translator.
   *
   * @param[in] channel The LCM channel on which to publish.
   *
   * @param[in] translator The translator that converts between LCM message
   * objects and `drake::systems::VectorBase` objects. This reference
   * is aliased by this constructor and thus must remain valid for the lifetime
   * of this object.
   *
   * @param lcm A non-null pointer to the LCM subsystem to publish on.
   * The pointer must remain valid for the lifetime of this object.
   */
  LcmPublisherSystem(const std::string& channel,
                     const LcmAndVectorBaseTranslator& translator,
                     drake::lcm::DrakeLcmInterface* lcm);

  /**
   * Constructor that returns a publisher System that takes vector data on
   * its sole vector-valued input port.  The vector data are mapped to
   * message contents by the translator found in the @p translator_dictionary.
   *
   * @param[in] channel The LCM channel on which to publish.
   *
   * @param[in] translator_dictionary A dictionary for obtaining the appropriate
   * translator for a particular LCM channel.
   *
   * @param lcm A non-null pointer to the LCM subsystem to publish on.
   * The pointer must remain valid for the lifetime of this object.
   */
  LcmPublisherSystem(const std::string& channel,
                     const LcmTranslatorDictionary& translator_dictionary,
                     drake::lcm::DrakeLcmInterface* lcm);

  ~LcmPublisherSystem() override;

  // Disable copy and assign.
  LcmPublisherSystem(const LcmPublisherSystem&) = delete;
  LcmPublisherSystem& operator=(const LcmPublisherSystem&) = delete;

  std::string get_name() const override;

  const std::string& get_channel_name() const;

  /// Returns the default name for a system that publishes @p channel.
  static std::string get_name(const std::string& channel);

  /**
   * Takes the VectorBase from the input port of the context and publishes
   * it onto an LCM channel.
   */
  void DoPublish(const Context<double>& context) const override;

  /**
   * This System has no output ports so EvalOutput() does nothing.
   */
  void EvalOutput(const Context<double>& context,
                  SystemOutput<double>* output) const override {}

  /**
   * Returns the translator used by this publisher. This can be used to convert
   * a serialized LCM message provided by
   * DrakeMockLcm::get_last_published_message() into a BasicVector. It is useful
   * in unit tests for verifying that a BasicVector was correctly published as
   * an LCM message.
   * @pre this system is using a vector-valued port (not abstract-valued).
   */
  const LcmAndVectorBaseTranslator& get_translator() const;

 private:
  // All constructors delegate to here.
  LcmPublisherSystem(
      const std::string& channel,
      const LcmAndVectorBaseTranslator* translator,
      std::unique_ptr<SerializerInterface> serializer,
      drake::lcm::DrakeLcmInterface* lcm);

  // The channel on which to publish LCM messages.
  const std::string channel_;

  // Converts VectorBase objects into LCM message bytes.
  // Will be non-null iff our input port is vector-valued.
  const LcmAndVectorBaseTranslator* const translator_{};

  // Converts Value<LcmMessage> objects into LCM message bytes.
  // Will be non-null iff our input port is abstract-valued.
  std::unique_ptr<SerializerInterface> serializer_;

  // A pointer to the LCM subsystem.
  drake::lcm::DrakeLcmInterface* const lcm_{};
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake
