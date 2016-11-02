#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_export.h"
#include "drake/common/drake_throw.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {
namespace lcm {

/**
 * %SerializerInterface translates between LCM message bytes and
 * drake::systems::AbstractValue objects that contain LCM messages, e.g., a
 * Value<lcmt_drake_signal>.  See Serializer for a message-specific concrete
 * subclass.
 */
class DRAKE_EXPORT SerializerInterface {
 public:
  virtual ~SerializerInterface();

  /**
   * Creates a value-initialized (zeroed) instance of the message object.
   * The result can be used as the output object filled in by Deserialize.
   */
  virtual std::unique_ptr<AbstractValue> CreateDefaultValue() const = 0;

  /**
   * Translates LCM message bytes into a drake::systems::AbstractValue object.
   */
  virtual void Deserialize(
      const void* message_bytes, int message_length,
      AbstractValue* abstract_value) const = 0;

  /**
   * Translates a drake::systems::AbstractValue object into LCM message bytes.
   */
  virtual void Serialize(const AbstractValue& abstract_value,
                         std::vector<uint8_t>* message_bytes) const = 0;

  // Disable copy and assign.
  SerializerInterface(const SerializerInterface&) = delete;
  SerializerInterface& operator=(const SerializerInterface&) = delete;

 protected:
  SerializerInterface() {}
};

/**
 * %Serializer is specific to a single LcmMessage type, and translates between
 * LCM message bytes and drake::systems::Value<LcmMessage> objects.
 *
 * @tparam LcmMessage message type to serialize, e.g., lcmt_drake_signal.
 */
template <typename LcmMessage>
class Serializer : public SerializerInterface {
 public:
  Serializer() {}
  ~Serializer() override {}

  std::unique_ptr<AbstractValue> CreateDefaultValue() const override {
    // NOTE: We create the message using value-initialization ("{}") to ensure
    // the POD fields are zeroed (instead of using default construction ("()"),
    // which would leave the POD data uninitialized.)
    return std::make_unique<Value<LcmMessage>>(LcmMessage{});
  }

  void Deserialize(
      const void* message_bytes, int message_length,
      AbstractValue* abstract_value) const override {
    DRAKE_DEMAND(abstract_value != nullptr);
    LcmMessage& message = abstract_value->GetMutableValue<LcmMessage>();
    int consumed = message.decode(message_bytes, 0, message_length);
    DRAKE_THROW_UNLESS(consumed == message_length);
  }

  void Serialize(const AbstractValue& abstract_value,
                 std::vector<uint8_t>* message_bytes) const override {
    DRAKE_DEMAND(message_bytes != nullptr);
    const LcmMessage& message = abstract_value.GetValue<LcmMessage>();
    const int message_length = message.getEncodedSize();
    message_bytes->resize(message_length);
    int consumed = message.encode(message_bytes->data(), 0, message_length);
    DRAKE_THROW_UNLESS(consumed == message_length);
  }

  // Disable copy and assign.
  Serializer(const Serializer&) = delete;
  Serializer& operator=(const Serializer&) = delete;
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake
