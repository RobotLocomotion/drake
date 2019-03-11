#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/value.h"

namespace drake {
namespace systems {
namespace lcm {

/**
 * %SerializerInterface translates between LCM message bytes and
 * drake::AbstractValue objects that contain LCM messages, e.g., a
 * Value<lcmt_drake_signal>.  See Serializer for a message-specific concrete
 * subclass.
 */
class SerializerInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SerializerInterface)

  virtual ~SerializerInterface();

  /**
   * Creates a value-initialized (zeroed) instance of the message object.
   * The result can be used as the output object filled in by Deserialize.
   */
  virtual std::unique_ptr<AbstractValue> CreateDefaultValue() const = 0;

  /**
   * Translates LCM message bytes into a drake::AbstractValue object.
   */
  virtual void Deserialize(
      const void* message_bytes, int message_length,
      AbstractValue* abstract_value) const = 0;

  /**
   * Translates a drake::AbstractValue object into LCM message bytes.
   */
  virtual void Serialize(const AbstractValue& abstract_value,
                         std::vector<uint8_t>* message_bytes) const = 0;

 protected:
  SerializerInterface() {}
};

/**
 * %Serializer is specific to a single LcmMessage type, and translates between
 * LCM message bytes and drake::Value<LcmMessage> objects.
 *
 * @tparam LcmMessage message type to serialize, e.g., lcmt_drake_signal.
 */
template <typename LcmMessage>
class Serializer : public SerializerInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Serializer)

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
    LcmMessage& message = abstract_value->get_mutable_value<LcmMessage>();
    int consumed = message.decode(message_bytes, 0, message_length);
    DRAKE_THROW_UNLESS(consumed == message_length);
  }

  void Serialize(const AbstractValue& abstract_value,
                 std::vector<uint8_t>* message_bytes) const override {
    DRAKE_DEMAND(message_bytes != nullptr);
    const LcmMessage& message = abstract_value.get_value<LcmMessage>();
    const int message_length = message.getEncodedSize();
    message_bytes->resize(message_length);
    int consumed = message.encode(message_bytes->data(), 0, message_length);
    DRAKE_THROW_UNLESS(consumed == message_length);
  }
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake
