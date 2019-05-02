#pragma once

#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/drake_throw.h"
#include "drake/common/hash.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/systems/framework/event.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/serializer.h"

namespace drake {

// Forward-declare so we can keep a pointer to a DrakeLcm if needed.
namespace lcm {
class DrakeLcm;
}  // namespace lcm

namespace systems {
namespace lcm {

using TriggerTypeSet = std::unordered_set<TriggerType, DefaultHash>;

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
   * Sets the default set of publish triggers:
   *   if publish_period = 0, publishes on forced events and per step,
   *   if publish_period > 0, publishes on forced events and periodically.
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
   * A factory method for an %LcmPublisherSystem that takes LCM message objects
   * on its sole abstract-valued input port. The LCM message type is determined
   * by the provided `serializer`.
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
   * @param publish_triggers Set of triggers that determine when messages will
   * be published. Supported TriggerTypes are {kForced, kPeriodic, kPerStep}.
   * Will throw an error if empty or if unsupported types are provided.
   *
   * @param publish_period Period that messages will be published (optional).
   * publish_period should only be non-zero if one of the publish_triggers is
   * kPeriodic.
   *
   * @pre publish_period is non-negative.
   * @pre trigger_types contains a subset of {kForced, kPeriodic, kPerStep}.
   * @pre publish_period > 0 if and only if trigger_types contains kPeriodic.
   */
  template <typename LcmMessage>
  static std::unique_ptr<LcmPublisherSystem> Make(
      const std::string& channel,
      drake::lcm::DrakeLcmInterface* lcm,
      const TriggerTypeSet& publish_triggers,
      double publish_period = 0.0) {
    return std::make_unique<LcmPublisherSystem>(
        channel, std::make_unique<Serializer<LcmMessage>>(), lcm,
        publish_triggers, publish_period);
  }

  /**
   * A constructor for an %LcmPublisherSystem that takes LCM message objects on
   * its sole abstract-valued input port. The LCM message type is determined by
   * the provided `serializer`. Will publish on forced events and either
   * periodic or per-step events, as determined by publish_period.
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
   * @param publish_triggers Set of triggers that determine when messages will
   * be published. Supported TriggerTypes are {kForced, kPeriodic, kPerStep}.
   * Will throw an exception if empty or if unsupported types are provided.
   *
   * @param publish_period Period that messages will be published (optional).
   * publish_period should only be non-zero if one of the publish_triggers is
   * kPerStep.
   *
   * @pre publish_period is non-negative.
   * @pre publish_period > 0 iff trigger_types contains kPeriodic.
   * @pre trigger_types contains a subset of {kForced, kPeriodic, kPerStep}.
   */
  LcmPublisherSystem(const std::string& channel,
      std::unique_ptr<SerializerInterface> serializer,
      drake::lcm::DrakeLcmInterface* lcm,
      const TriggerTypeSet& publish_triggers,
      double publish_period = 0.0);

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
    DRAKE_THROW_UNLESS(this->num_input_ports() == 1);
    return LeafSystem<double>::get_input_port(0);
  }

  // Don't use the indexed overload; use the no-arg overload.
  void get_input_port(int index) = delete;

  // This system has no output ports.
  void get_output_port(int) = delete;

 private:
  EventStatus PublishInputAsLcmMessage(const Context<double>& context) const;

  // The channel on which to publish LCM messages.
  const std::string channel_;

  // Optionally, the method to call during initialization (empty if none).
  InitializationPublisher initialization_publisher_;

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
