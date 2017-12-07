#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/lcm/drake_lcm_message_handler_interface.h"

namespace drake {
namespace lcm {

/**
 * A *mock* LCM instance. This does not actually publish or subscribe to LCM
 * messages. It contains additional methods for accessing the most recent
 * message that was "published," and faking a callback.
 */
class DrakeMockLcm : public DrakeLcmInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DrakeMockLcm);

  /**
   * A constructor that creates a DrakeMockLcm with loopback disabled, i.e., a
   * call to Publish() will not result in subscriber callback functions being
   * executed. To enable loopback behavior, call EnableLoopBack().
   */
  DrakeMockLcm();

  /**
   * Enables loopback behavior. With loopback enabled, a call to Publish() will
   * result in subscriber callback functions being called. Without loopback
   * enabled, the only way to induce a call to a subscriber's callback function
   * is through InduceSubscriberCallback().
   */
  void EnableLoopBack() { enable_loop_back_ = true; }

  void StartReceiveThread() override;

  void StopReceiveThread() override;

  void Publish(const std::string& channel, const void* data,
               int data_size, double time_sec = 0) override;

  /**
   * Obtains the most recently "published" message on a particular channel.
   * This method automatically decodes the message into an LCM message whose
   * type is specified by the template type. Throws a std::runtime_error
   * exception if no LCM message was published on the provided channel or if the
   * message failed to be decoded by the provided LCM message type.
   *
   * @tparam T The LCM message type.
   *
   * @param[in] channel the LCM channel for which the last published message is
   * returned.
   *
   * @return the decoded most recently transmitted LCM message on the provided
   * channel.
   */
  template<typename T>
  T DecodeLastPublishedMessageAs(const std::string& channel) const {
      const std::vector<uint8_t>& message_bytes =
          get_last_published_message(channel);

    T transmitted_message{};
    const int num_bytes = transmitted_message.decode(message_bytes.data(), 0,
                                                     message_bytes.size());

    if (num_bytes != static_cast<int>(message_bytes.size())) {
      throw std::runtime_error("DrakeMockLcm::DecodeLastPublishedMessageAs(): "
        "ERROR: Failed to decode the LCM message. Number of bytes decoded: " +
        std::to_string(num_bytes) + ". Number of message bytes that should "
        "have been decoded: " + std::to_string(message_bytes.size()) + ".");
    }

    return transmitted_message;
  }

  /**
   * Obtains the most recently "published" message on a particular channel. A
   * std::runtime_error will be thrown if no message was published on the
   * provide channel.
   *
   * @param[in] channel The channel on which the LCM message was published.
   *
   * @return A reference to a vector containing the serialized bytes of the
   * LCM message that was previously published on channel @p channel.
   *
   * @pre A message was previously published on channel @p channel.
   */
  const std::vector<uint8_t>& get_last_published_message(
      const std::string& channel) const;

  /**
   * Creates a subscription. Only one subscription per channel name is
   * permitted. A std::runtime_error is thrown if more than one subscription to
   * the same channel name is attempted.
   */
  void Subscribe(const std::string& channel,
                 DrakeLcmMessageHandlerInterface* handler) override;

  /**
   * Fakes a callback. This will only work if StartReceivedThread() was already
   * called, otherwise this method will do nothing. The callback is executed by
   * the same thread as the one calling this method.
   *
   * @param[in] channel The channel on which to publish the message.
   *
   * @param[in] data A buffer containing the serialized bytes of the message to
   * publish.
   *
   * @param[in] data_size The length of @data in bytes.
   */
  void InduceSubscriberCallback(const std::string& channel, const void* data,
                               int data_size);

 private:
  bool enable_loop_back_{false};
  bool receive_thread_started_{false};

  struct LastPublishedMessage {
    std::vector<uint8_t> data{};
  };

  std::map<std::string, LastPublishedMessage> last_published_messages_;

  // Maps the channel name to the subscriber.
  std::map<std::string, DrakeLcmMessageHandlerInterface*> subscriptions_;
};

}  // namespace lcm
}  // namespace drake
