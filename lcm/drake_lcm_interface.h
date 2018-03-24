#pragma once

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/common/drake_throw.h"
#include "drake/lcm/drake_lcm_message_handler_interface.h"

namespace drake {
namespace lcm {

/**
 * A pure virtual interface that enables LCM to be mocked.
 */
class DrakeLcmInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DrakeLcmInterface)
  DrakeLcmInterface() = default;
  virtual ~DrakeLcmInterface() = default;

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
   * @param[in] time_sec Time in seconds when the publish event occurred. Note
   * that this argument is only used when generating a Lcm log.
   */
  virtual void Publish(const std::string& channel, const void* data,
                       int data_size, double time_sec = 0) = 0;

  /**
   * Subscribes to an LCM channel without automatic message decoding. The
   * callback method within @p handler will be invoked when a message arrives on
   * channel @p channel.
   *
   * @param[in] channel The channel to subscribe to.
   * Must not be the empty string.
   *
   * @param[in] handler A class instance whose callback method will be invoked.
   */
  virtual void Subscribe(const std::string& channel,
                         DrakeLcmMessageHandlerInterface* handler) = 0;
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
  lcm->Publish(channel, bytes.data(), num_bytes, time_sec.value_or(0.0));
}

}  // namespace lcm
}  // namespace drake
