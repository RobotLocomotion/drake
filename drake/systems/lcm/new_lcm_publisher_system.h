#pragma once

#include <memory>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/lcm/translator.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/translator_internal.h"

namespace drake {
namespace systems {
namespace lcm {

/**
 * Publishes an LCM message containing information from its input port.
 */
template <typename DataType, typename MsgType>
class NewLcmPublisherSystem : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(NewLcmPublisherSystem)

  using TranslatorType = typename drake::lcm::TranslatorBase<DataType, MsgType>;

  // For VectorBase and it's derived.
  template <typename DataType1 = DataType,
            typename = typename std::enable_if<
                std::is_base_of<systems::VectorBase<double>, DataType1>::value>::type>
  NewLcmPublisherSystem(const std::string& channel,
      std::unique_ptr<TranslatorType> translator,
      int vector_size, drake::lcm::DrakeLcmInterface* lcm)
      : channel_(channel), translator_(std::move(translator)), lcm_(lcm) {
    DeclareInputPort(kVectorValued, vector_size);
    set_name(make_name(channel_));
  }

  // For every thing else.
  template <typename DataType1 = DataType,
            typename = typename std::enable_if<
                !std::is_base_of<systems::VectorBase<double>, DataType1>::value
            >::type,
            typename = void>
  NewLcmPublisherSystem(const std::string& channel,
      std::unique_ptr<TranslatorType> translator,
      drake::lcm::DrakeLcmInterface* lcm)
      : channel_(channel), translator_(std::move(translator)), lcm_(lcm) {
    DeclareAbstractInputPort();
    set_name(make_name(channel_));
  }

  const std::string& get_channel_name() const { return channel_; }

  /// Returns the default name for a system that publishes @p channel.
  static std::string make_name(const std::string& channel) {
    return "LcmPublisherSystem(" + channel + ")";
  }

  /**
   * Sets the publishing period of this system. See
   * LeafSystem::DeclarePublishPeriodSec() for details about the semantics of
   * parameter `period`.
   */
  void set_publish_period(double period) {
    LeafSystem<double>::DeclarePublishPeriodSec(period);
  }

  const TranslatorType& get_translator() const { return *translator_; };

 private:
  /**
   * Takes the VectorBase from the input port of the context and publishes
   * it onto an LCM channel.
   */
  void DoPublish(const Context<double>& context) const override {
    MsgType msg;
    // Encode<DataType>(context, &msg);
    const DataType& data =
        translator_internal::DataTypeTraits<DataType, std::is_base_of<VectorBase<double>, DataType>::value>::get_data(*this, context);
    translator_->Encode(context.get_time(), data, &msg);

    const int msg_size = msg.getEncodedSize();
    message_bytes_.resize(msg_size);
    int encoded_size = msg.encode(message_bytes_.data(), 0, msg_size);
    DRAKE_THROW_UNLESS(encoded_size == msg_size);

    lcm_->Publish(channel_, message_bytes_.data(), message_bytes_.size());
  }

  /**
   * This System has no output ports so CalcOutput() does nothing.
   */
  void DoCalcOutput(const Context<double>&,
                    SystemOutput<double>*) const override {}

  // The channel on which to publish LCM messages.
  const std::string channel_;

  std::unique_ptr<TranslatorType> translator_;

  // A const pointer to an LCM subsystem. Note that while the pointer is const,
  // the LCM subsystem is not const.
  drake::lcm::DrakeLcmInterface* const lcm_{};

  mutable std::vector<uint8_t> message_bytes_;
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake pragma once
