#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/drake_optional.h"
#include "drake/lcm/drake_lcm_interface.h"

namespace drake {
namespace lcm {

/**
 * A *mock* LCM instance. This only manipulates LCM messages in memory, not on
 * the wire.  It is similar to a DrakeLcm object with a "memq://" URL, but is
 * guaranteed to behave deterministically (without a hidden background thread).
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
  DRAKE_DEPRECATED("2019-06-01",
      "Call HandleSubscriptions(0) to propagate messages instead.")
  void EnableLoopBack() { enable_loop_back_ = true; }

  /**
   * Obtains the most recently "published" message on a particular channel.
   * This method automatically decodes the message into an LCM message whose
   * type is specified by the template type.
   * @throws std::runtime_error if no LCM message was published on the provided
   * channel or if the message failed to be decoded by the provided LCM message
   * type.
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
  DRAKE_DEPRECATED("2019-06-01", "Use a drake::lcm::Subscriber instead.")
  T DecodeLastPublishedMessageAs(const std::string& channel) const {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
      const std::vector<uint8_t>& message_bytes =
          get_last_published_message(channel);
#pragma GCC diagnostic pop

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
  DRAKE_DEPRECATED("2019-06-01", "Use a drake::lcm::Subscriber instead.")
  const std::vector<uint8_t>& get_last_published_message(
      const std::string& channel) const;

  /**
   * Returns the time of the most recent publication on a particular channel.
   * Returns nullopt iff a message has never been published on this channel or
   * the most recent Publish call had no time_sec.
   */
  DRAKE_DEPRECATED("2019-06-01",
      "There is no easy replacement.  One option is to use a DrakeLcmLog "
      "instead of a DrakeMockLcm; then, the logged lcm::LogEvent.timestamp "
      "will show the publication time")
  optional<double> get_last_publication_time(const std::string& channel) const;

  /**
   * Fakes a callback. The callback is executed by the same thread as the one
   * calling this method.
   *
   * @param[in] channel The channel on which to publish the message.
   *
   * @param[in] data A buffer containing the serialized bytes of the message to
   * publish.
   *
   * @param[in] data_size The length of @data in bytes.
   */
  DRAKE_DEPRECATED("2019-06-01", "Call drake::lcm::Publish() instead.")
  void InduceSubscriberCallback(const std::string& channel, const void* data,
                               int data_size);

  void Publish(const std::string&, const void*, int, optional<double>) override;
  std::shared_ptr<DrakeSubscriptionInterface> Subscribe(
      const std::string&, HandlerFunction) override;
  int HandleSubscriptions(int) override;

 private:
  bool enable_loop_back_{false};

  struct LastPublishedMessage {
    std::vector<uint8_t> data{};
    optional<double> time_sec{};
    bool handled{};
  };

  // Use an ordered collection so that HandleSubscriptions is deterministic.
  std::map<std::string, LastPublishedMessage> last_published_messages_;

  // Maps the channel name to the subscriber.
  std::multimap<std::string, HandlerFunction> subscriptions_;
};

}  // namespace lcm
}  // namespace drake
