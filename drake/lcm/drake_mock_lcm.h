#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "drake/common/drake_export.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/lcm/drake_lcm_message_handler_interface.h"

namespace drake {
namespace lcm {

/**
 * A *mock* LCM instance. This does not actually publish or subscribe to LCM
 * messages. It contains additional methods for accessing the most recent
 * message that was "published," and faking a callback.
 */
class DRAKE_EXPORT DrakeMockLcm : public DrakeLcmInterface {
 public:
  DrakeMockLcm();

  // Disable copy and assign.
  DrakeMockLcm(const DrakeMockLcm&) = delete;
  DrakeMockLcm& operator=(const DrakeMockLcm&) = delete;

  void StartReceiveThread() override;

  void StopReceiveThread() override;

  void Publish(const std::string& channel, const void* data,
               int data_size) override;

  /**
   * Obtains the most recently "published" message on a particular channel.
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
