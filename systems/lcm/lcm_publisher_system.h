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
 * Optionally sends a one-time initialization message. Publishing can be set up
 * to happen on a per-step or periodic basis. Publishing "by force", through
 * `LcmPublisherSystem::Publish(const Context&)`, is also enabled.
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
   *
   * @param publish_period Period that messages will be published (optional).
   * If the publish period is zero, LcmPublisherSystem will use per-step
   * publishing instead; see LeafSystem::DeclarePerStepPublishEvent().
   *
   * @pre publish_period is non-negative.
   */
  template <typename LcmMessage>
  static std::unique_ptr<LcmPublisherSystem> Make(
      const std::string& channel,
      drake::lcm::DrakeLcmInterface* lcm,
      double publish_period = 0.0) {
    return std::make_unique<LcmPublisherSystem>(
        channel, std::make_unique<Serializer<LcmMessage>>(), lcm,
        publish_period);
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
   *
   * @param publish_period Period that messages will be published (optional).
   * If the publish period is zero, LcmPublisherSystem will use per-step
   * publishing instead; see LeafSystem::DeclarePerStepPublishEvent().
   *
   * @pre publish_period is non-negative.
   */
  LcmPublisherSystem(const std::string& channel,
                     std::unique_ptr<SerializerInterface> serializer,
                     drake::lcm::DrakeLcmInterface* lcm,
                     double publish_period = 0.0);

  DRAKE_DEPRECATED(
      "The LcmAndVectorBaseTranslator and its related code are deprecated, "
      "and will be removed on 2019-05-01.")
  LcmPublisherSystem(
      const std::string&, const LcmAndVectorBaseTranslator&,
      drake::lcm::DrakeLcmInterface*, double publish_period = 0.0);

  DRAKE_DEPRECATED(
      "The LcmAndVectorBaseTranslator and its related code are deprecated, "
      "and will be removed on 2019-05-01.")
  LcmPublisherSystem(
      const std::string&, std::unique_ptr<const LcmAndVectorBaseTranslator>,
      drake::lcm::DrakeLcmInterface*, double publish_period = 0.0);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  DRAKE_DEPRECATED(
      "The LcmAndVectorBaseTranslator and its related code are deprecated, "
      "and will be removed on 2019-05-01.")
  LcmPublisherSystem(
      const std::string&, const LcmTranslatorDictionary&,
      drake::lcm::DrakeLcmInterface*, double publish_period = 0.0);
#pragma GCC diagnostic pop

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

  DRAKE_DEPRECATED("Pass publish period to constructor instead. This method "
                   "will be removed after 4/14/19.")
  void set_publish_period(double period) {
    if (disable_internal_per_step_publish_events_) {
      drake::log()->info("LcmPublisherSystem publish period set "
          "multiple times. Multiple publish periods now registered "
          "(did you mean to do this?)");
    }
    disable_internal_per_step_publish_events_ = true;
    const double offset = 0.0;
    this->DeclarePeriodicPublishEvent(period, offset,
        &LcmPublisherSystem::PublishInputAsLcmMessage);
  }

  DRAKE_DEPRECATED(
      "The LcmAndVectorBaseTranslator and its related code are deprecated, "
      "and will be removed on 2019-05-01.")
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

  // Don't use the indexed overload; use the no-arg overload.
  void get_input_port(int index) = delete;

  // This system has no output ports.
  void get_output_port(int) = delete;

 private:
  EventStatus PublishInputAsLcmMessage(const Context<double>& context) const;

  // All constructors delegate to here. If the lcm pointer is null, we'll
  // allocate and maintain a DrakeLcm object internally.
  LcmPublisherSystem(const std::string& channel,
                     const LcmAndVectorBaseTranslator* translator,
                     std::unique_ptr<const LcmAndVectorBaseTranslator>
                         owned_translator,
                     std::unique_ptr<SerializerInterface> serializer,
                     drake::lcm::DrakeLcmInterface* lcm,
                     double publish_period);

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

  // TODO(edrumwri) Remove this when set_publish_period() is removed.
  bool disable_internal_per_step_publish_events_{false};
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake
