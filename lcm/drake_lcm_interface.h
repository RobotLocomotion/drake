#pragma once

#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/common/drake_throw.h"

namespace drake {
namespace lcm {

/**
 * A pure virtual interface that enables LCM to be mocked.
 */
class DrakeLcmInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DrakeLcmInterface)
  virtual ~DrakeLcmInterface() = default;

  /**
   * A callback used by DrakeLcmInterface::Subscribe(), with arguments:
   * - `message_buffer` A pointer to the byte vector that is the serial
   *   representation of the LCM message.
   * - `message_size` The size of `message_buffer`.
   */
  using HandlerFunction = std::function<void(const void*, int)>;

  /**
   * Publishes an LCM message on channel @p channel.
   *
   * @param[in] channel The channel on which to publish the message.
   * Must not be the empty string.
   *
   * @param[in] data A buffer containing the serialized bytes of the message to
   * publish.
   *
   * @param[in] data_size The length of @data in bytes.
   *
   * @param[in] time_sec Time in seconds when the publish event occurred.
   * If unknown, use drake::nullopt or a default-constructed optional.
   */
  virtual void Publish(const std::string& channel, const void* data,
                       int data_size, optional<double> time_sec) = 0;

  /**
   * Subscribes to an LCM channel without automatic message decoding. The
   * handler will be invoked when a message arrives on channel @p channel.
   *
   * @param channel The channel to subscribe to.
   * Must not be the empty string.
   */
  virtual void Subscribe(const std::string& channel, HandlerFunction) = 0;

 protected:
  DrakeLcmInterface() = default;
};

/**
 * Publishes an LCM message on channel @p channel.
 *
 * @param lcm The LCM service on which to publish the message.
 * Must not be null.
 *
 * @param channel The channel on which to publish the message.
 * Must not be the empty string.
 *
 * @param message The message to publish.
 *
 * @param time_sec Time in seconds when the publish event occurred.
 * If unknown, use the default value of drake::nullopt.
 */
template <typename Message>
void Publish(DrakeLcmInterface* lcm, const std::string& channel,
             const Message& message, optional<double> time_sec = {}) {
  DRAKE_THROW_UNLESS(lcm != nullptr);
  const int num_bytes = message.getEncodedSize();
  DRAKE_THROW_UNLESS(num_bytes >= 0);
  const size_t size_bytes = static_cast<size_t>(num_bytes);
  std::vector<uint8_t> bytes(size_bytes);
  message.encode(bytes.data(), 0, num_bytes);
  lcm->Publish(channel, bytes.data(), num_bytes, time_sec);
}

/**
 * Subscribes to an LCM channel named @p channel and decodes messages of type
 * @p Message.
 *
 * @param lcm The LCM service on which to subscribe.
 * Must not be null.
 *
 * @param channel The channel on which to subscribe.
 * Must not be the empty string.
 *
 * @param handler The callback when a message is received and decoded without
 * error.
 *
 * @param on_error The callback when a message is received and cannot be
 * decoded; if no error handler is given, an exception is thrown instead.
 *
 * @note Depending on the specific DrakeLcmInterface implementation, the
 * handler might be invoked on a different thread than this function.
 */
template <typename Message>
void Subscribe(DrakeLcmInterface* lcm, const std::string& channel,
               std::function<void(const Message&)> handler,
               std::function<void()> on_error = {}) {
  DRAKE_THROW_UNLESS(lcm != nullptr);
  lcm->Subscribe(channel, [=](const void* bytes, int size) {
    Message received{};
    const int size_decoded = received.decode(bytes, 0, size);
    if (size_decoded == size) {
      handler(received);
    } else if (on_error) {
      on_error();
    } else {
      throw std::runtime_error("Error decoding message on " + channel);
    }
  });
}

}  // namespace lcm
}  // namespace drake
