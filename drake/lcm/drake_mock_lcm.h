#pragma once

#include <map>
#include <memory>
#include <vector>

#include "drake/drakeLcm_export.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/lcm/drake_lcm_message_handler_interface.h"

namespace drake {
namespace lcm {

using std::make_unique;
using std::unique_ptr;

/// @cond

// This is a mock subscriber to an LCM channel. It simply passes the
// serialized LCM message to the `DrakeLcmMessageHandlerInterface`
// object.
class DRAKELCM_EXPORT DrakeMockLcmSubscriber {
 public:
  explicit DrakeMockLcmSubscriber(DrakeLcmMessageHandlerInterface*
      drake_handler);

  // Disables copy and assign.
  DrakeMockLcmSubscriber(const DrakeMockLcmSubscriber&) = delete;
  DrakeMockLcmSubscriber& operator=(const DrakeMockLcmSubscriber&) = delete;

  DrakeLcmMessageHandlerInterface* get_subscriber();

 private:
  DrakeLcmMessageHandlerInterface* drake_handler_;
};

/// @endcond

/**
 * A *mock* LCM instance. This does not actually publish or subscribe to LCM
 * messages. It contains additional methods for accessing the most recent
 * message that was "published," and faking a callback.
 */
class DRAKELCM_EXPORT DrakeMockLcm : public DrakeLcmInterface {
 public:
  DrakeMockLcm();

  // Disable copy and assign.
  DrakeMockLcm(const DrakeMockLcm&) = delete;
  DrakeMockLcm& operator=(const DrakeMockLcm&) = delete;

  void StartReceiveThread() override;

  void StopReceiveThread() override;

  void Publish(const std::string& channel, const void *data, int data_size)
      override;

  /**
   * Obtains the most recently "published" message on a particular channel.
   *
   * @param[in] channel The channel on which the LCM message was published.
   *
   * @param[out] data A pointer to where the pointer to the message's serialized
   * byte array should be saved.
   *
   * @param[out] data_size A pointer to where the size of the byte array should
   * be saved.
   *
   * @return `true` if a message was published, `false` otherwise.
   */
  bool get_last_published_message(const std::string& channel, void** data,
      int* data_size);

  /**
   * Creates a subscription. Only one subscription per channel name is
   * permitted. A std::runtime_error is thrown if more than one subscription to
   * the same channel name is attempted.
   */
  void Subscribe(const std::string& channel,
      DrakeLcmMessageHandlerInterface* handler) override;

  /**
   * Fakes a callback. This will only work if StartReceivedThread() was already
   * called. The callback is executed by the same thread as the one calling this
   * method.
   *
   * @param[in] channel The channel on which to publish the message.
   *
   * @param[in] data A buffer containing the serialized bytes of the message to
   * publish.
   *
   * @param[in] data_size The length of @data in bytes.
   */
  void InduceSubsciberCallback(const std::string& channel, const void *data,
      int data_size);

 private:
  bool received_thread_started_{false};

  struct LastPublishedMessage {
    std::string channel{};
    std::vector<uint8_t> data{};
    int data_size{};
  };

  std::map<std::string, std::unique_ptr<LastPublishedMessage>>
    last_published_messages_;

  // Maps the channel name to the subscriber.
  std::map<std::string, std::unique_ptr<DrakeMockLcmSubscriber>> subscriptions_;
};

}  // namespace lcm
}  // namespace drake
