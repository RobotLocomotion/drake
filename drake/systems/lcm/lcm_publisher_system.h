#pragma once

#include <memory>
#include <string>

#include "drake/common/drake_copyable.h"
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
class LcmPublisherSystem : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LcmPublisherSystem)

  /**
   * A factory method that returns an %LcmPublisherSystem that takes
   * Value<LcmMessage> message objects on its sole abstract-valued input port.
   *
   * @tparam LcmMessage message type to serialize, e.g., lcmt_drake_signal.
   *
   * @param[in] channel The LCM channel on which to publish.
   *
   * @param lcm A non-null pointer to the LCM subsystem. The pointer must remain
   * valid for the lifetime of this object.
   */
  template <typename LcmMessage>
  static std::unique_ptr<LcmPublisherSystem> Make(
      const std::string& channel, drake::lcm::DrakeLcmInterface* lcm) {
    return std::make_unique<LcmPublisherSystem>(
        channel, std::make_unique<Serializer<LcmMessage>>(), lcm);
  }

  /**
   * A constructor for an %LcmPublisherSystem that takes LCM message objects on
   * its sole abstract-valued input port. The LCM message type is determined by
   * the provided `serializer`.
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
   * A constructor for an %LcmPublisherSystem that takes vector data on its sole
   * vector-valued input port. The vector data is mapped to message content by
   * the provided `translator`.
   *
   * @param[in] channel The LCM channel on which to publish.
   *
   * @param[in] translator The translator that converts between LCM message
   * objects and drake::systems::VectorBase objects. This reference must remain
   * valid for the lifetime of this object.
   *
   * @param lcm A non-null pointer to the LCM subsystem to publish on.
   * The pointer must remain valid for the lifetime of this object.
   */
  LcmPublisherSystem(const std::string& channel,
                     const LcmAndVectorBaseTranslator& translator,
                     drake::lcm::DrakeLcmInterface* lcm);

  /**
   * Constructor that returns a publisher System that takes vector data on
   * its sole vector-valued input port. The vector data are mapped to message
   * contents by the `translator` found in the provided `translator_dictionary`.
   *
   * @param[in] channel The LCM channel on which to publish.
   *
   * @param[in] translator_dictionary A dictionary for obtaining the appropriate
   * translator for a particular LCM channel. This reference must remain
   * valid for the lifetime of this object.
   *
   * @param lcm A non-null pointer to the LCM subsystem to publish on. The
   * pointer must remain valid for the lifetime of this object.
   */
  LcmPublisherSystem(const std::string& channel,
                     const LcmTranslatorDictionary& translator_dictionary,
                     drake::lcm::DrakeLcmInterface* lcm);

  ~LcmPublisherSystem() override;

  const std::string& get_channel_name() const;

  /// Returns the default name for a system that publishes @p channel.
  static std::string make_name(const std::string& channel);

  /**
   * Sets the publishing period of this system. See
   * LeafSystem::DeclarePublishPeriodSec() for details about the semantics of
   * parameter `period`.
   */
  void set_publish_period(double period);

  /**
   * Takes the VectorBase from the input port of the context and publishes
   * it onto an LCM channel.
   */
  void DoPublish(const Context<double>& context) const override;

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
  LcmPublisherSystem(const std::string& channel,
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

  // A const pointer to an LCM subsystem. Note that while the pointer is const,
  // the LCM subsystem is not const.
  drake::lcm::DrakeLcmInterface* const lcm_{};
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake
