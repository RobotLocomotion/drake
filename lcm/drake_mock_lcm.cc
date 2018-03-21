#include "drake/lcm/drake_mock_lcm.h"

#include <cstring>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
#include "drake/lcm/drake_lcm_message_handler_interface.h"

namespace drake {
namespace lcm {

DrakeMockLcm::DrakeMockLcm() {}


void DrakeMockLcm::Publish(const std::string& channel, const void* data,
                           int data_size, double) {
  DRAKE_THROW_UNLESS(!channel.empty());
  if (last_published_messages_.find(channel) ==
      last_published_messages_.end()) {
    last_published_messages_[channel] = LastPublishedMessage();
  }
  LastPublishedMessage* saved_message{nullptr};
  saved_message = &last_published_messages_[channel];

  DRAKE_DEMAND(saved_message);

  const uint8_t* bytes = static_cast<const uint8_t*>(data);
  saved_message->data = std::vector<uint8_t>(&bytes[0], &bytes[data_size]);

  if (enable_loop_back_) {
    InduceSubscriberCallback(channel, data, data_size);
  }
}

const std::vector<uint8_t>& DrakeMockLcm::get_last_published_message(
    const std::string& channel) const {
  if (last_published_messages_.find(channel) ==
      last_published_messages_.end()) {
    throw std::runtime_error(
        "DrakeMockLcm::get_last_published_message: ERROR: "
        "No message was previous published on channel \"" +
        channel + "\".");
  }

  const LastPublishedMessage* message = &last_published_messages_.at(channel);
  DRAKE_DEMAND(message);

  return message->data;
}

void DrakeMockLcm::Subscribe(const std::string& channel,
                             DrakeLcmMessageHandlerInterface* handler) {
  DRAKE_THROW_UNLESS(!channel.empty());
  if (subscriptions_.find(channel) == subscriptions_.end()) {
    subscriptions_[channel] = handler;
  } else {
    throw std::runtime_error(
        "DrakeMockLcm::Subscribe: Subscription to "
        "channel \"" +
        channel + "\" already exists.");
  }
}

void DrakeMockLcm::InduceSubscriberCallback(const std::string& channel,
                                            const void* data, int data_size) {
  if (subscriptions_.find(channel) == subscriptions_.end()) {
    throw std::runtime_error(
        "DrakeMockLcm::InduceSubscriberCallback: No "
        "subscription to channel \"" +
        channel + "\".");
  } else {
    subscriptions_[channel]->HandleMessage(channel, data, data_size);
  }
}

}  // namespace lcm
}  // namespace drake
