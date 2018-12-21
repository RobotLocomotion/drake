#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/drake_throw.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_and_vector_base_translator.h"
#include "drake/systems/lcm/lcm_translator_dictionary.h"
#include "drake/systems/lcm/serializer.h"

namespace drake {

// Forward-declare so we can keep a pointer to a DrakeLcm if needed.
namespace lcm {
class DrakeLcm;
}  // namespace lcm

namespace systems {
namespace lcm {

/**
 * Publishes an LCM message containing information from its input port.
 * Optionally sends a one-time initialization message.
 *
 * @note You should generally provide an LCM interface yourself, since there
 * should normally be just one of these typically-heavyweight objects per
 * program. However, if you're sure there isn't any other need for an LCM
 * interface in your program, you can let %LcmPublisherSystem allocate and
 * maintain a drake::lcm::DrakeLcm object internally.
 *
 * @system{ LcmPublisherSystem,
 *  @input_port{lcm_message}, }
 *
 * @ingroup message_passing
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
   * @param lcm A pointer to the LCM subsystem to use, which must
   * remain valid for the lifetime of this object. If null, a
   * drake::lcm::DrakeLcm object is allocated and maintained internally, but
   * see the note in the class comments.
   */
  template <typename LcmMessage>
  static std::unique_ptr<LcmPublisherSystem> Make(
      const std::string& channel,
      drake::lcm::DrakeLcmInterface* lcm) {
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
   * @param lcm A pointer to the LCM subsystem to use, which must
   * remain valid for the lifetime of this object. If null, a
   * drake::lcm::DrakeLcm object is allocated and maintained internally, but
   * see the note in the class comments.
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
   * @param lcm A pointer to the LCM subsystem to use, which must
   * remain valid for the lifetime of this object. If null, a
   * drake::lcm::DrakeLcm object is allocated and maintained internally, but
   * see the note in the class comments.
   *
   * @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
   */
  LcmPublisherSystem(const std::string& channel,
                     const LcmAndVectorBaseTranslator& translator,
                     drake::lcm::DrakeLcmInterface* lcm);

  /**
   * Constructor that passes a unique_ptr of the LcmAndVectorBaseTranslator,
   * for the LcmPublisherSystem to own.
   *
   * @param[in] channel The LCM channel on which to publish.
   *
   * @param[in] translator The translator that converts between LCM message
   * objects and drake::systems::VectorBase objects.
   *
   * @param lcm A pointer to the LCM subsystem to use, which must
   * remain valid for the lifetime of this object. If null, a
   * drake::lcm::DrakeLcm object is allocated and maintained internally, but
   * see the note in the class comments.
   *
   * @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
   */
  LcmPublisherSystem(
      const std::string& channel,
      std::unique_ptr<const LcmAndVectorBaseTranslator> translator,
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
   * @param lcm A pointer to the LCM subsystem to use, which must
   * remain valid for the lifetime of this object. If null, a
   * drake::lcm::DrakeLcm object is allocated and maintained internally, but
   * see the note in the class comments.
   *
   * @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
   */
  LcmPublisherSystem(const std::string& channel,
                     const LcmTranslatorDictionary& translator_dictionary,
                     drake::lcm::DrakeLcmInterface* lcm);

  ~LcmPublisherSystem() override;

  /**
   * This is the type of an initialization message publisher that can be
   * provided via AddInitializationMessage().
   */
  using InitializationPublisher = std::function<void(
      const Context<double>& context, drake::lcm::DrakeLcmInterface* lcm)>;

  /**
   * Specifies a message-publishing function to be invoked once from an
   * initialization event. If this method is not called, no initialization event
   * will be created.
   *
   * You can only call this method once.
   * @throws std::logic_error if called a second time.
   *
   * @pre The publisher function may not be null.
   */
  void AddInitializationMessage(
      InitializationPublisher initialization_publisher);

  /**
   * Returns the channel name supplied during construction.
   */
  const std::string& get_channel_name() const;

  /**
   * Returns the default name for a system that publishes @p channel.
   */
  static std::string make_name(const std::string& channel);

  /**
   * Sets the publishing period of this system. See
   * LeafSystem::DeclarePeriodicPublish() for details about the semantics of
   * parameter `period`.
   */
  void set_publish_period(double period);

  /**
   * Returns the translator used by this publisher. This can be used to convert
   * a serialized LCM message provided by
   * DrakeMockLcm::get_last_published_message() into a BasicVector. It is useful
   * in unit tests for verifying that a BasicVector was correctly published as
   * an LCM message.
   * @pre this system is using a vector-valued port (not abstract-valued).
   */
  const LcmAndVectorBaseTranslator& get_translator() const;

  /**
   * Returns a mutable reference to the LCM object in use by this publisher.
   * This may have been supplied in the constructor or may be an
   * internally-maintained object of type drake::lcm::DrakeLcm.
   */
  drake::lcm::DrakeLcmInterface& lcm() {
    DRAKE_DEMAND(lcm_ != nullptr);
    return *lcm_;
  }

  /**
   * Returns the sole input port.
   */
  const InputPort<double>& get_input_port() const {
    DRAKE_THROW_UNLESS(this->get_num_input_ports() == 1);
    return LeafSystem<double>::get_input_port(0);
  }

  DRAKE_DEPRECATED("Don't use the indexed overload; use the no-arg overload.")
  const InputPort<double>& get_input_port(int index) const {
    DRAKE_THROW_UNLESS(index == 0);
    return get_input_port();
  }

  // This system has no output ports.
  void get_output_port(int) = delete;

 private:
  // All constructors delegate to here. If the lcm pointer is null, we'll
  // allocate and maintain a DrakeLcm object internally.
  LcmPublisherSystem(const std::string& channel,
                     const LcmAndVectorBaseTranslator* translator,
                     std::unique_ptr<const LcmAndVectorBaseTranslator>
                         owned_translator,
                     std::unique_ptr<SerializerInterface> serializer,
                     drake::lcm::DrakeLcmInterface* lcm);

  // Takes the VectorBase from the input port of the context and publishes
  // it onto an LCM channel.
  void DoPublish(
      const Context<double>& context,
      const std::vector<const systems::PublishEvent<double>*>&) const override;

  // The channel on which to publish LCM messages.
  const std::string channel_;

  // Optionally, the method to call during initialization (empty if none).
  InitializationPublisher initialization_publisher_;

  // Converts VectorBase objects into LCM message bytes.
  // Will be non-null iff our input port is vector-valued.
  const LcmAndVectorBaseTranslator* const translator_{};

  // This will be non-null iff the unique_ptr constructor was called.
  // Internal users should only use translator_.
  std::unique_ptr<const LcmAndVectorBaseTranslator> const owned_translator_;

  // Converts Value<LcmMessage> objects into LCM message bytes.
  // Will be non-null iff our input port is abstract-valued.
  std::unique_ptr<SerializerInterface> serializer_;

  // If we're not given a DrakeLcm object, we allocate one and keep it here.
  // The unique_ptr is const, not the held object.
  std::unique_ptr<drake::lcm::DrakeLcm> const owned_lcm_;

  // A const pointer to an LCM subsystem. Note that while the pointer is const,
  // the LCM subsystem is not const. This may refer to an externally-supplied
  // object or the owned_lcm_ object above.
  drake::lcm::DrakeLcmInterface* const lcm_;
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake
