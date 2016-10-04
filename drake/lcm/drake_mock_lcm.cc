#include "drake/lcm/drake_mock_lcm.h"

#include <cstring>

#include "drake/common/drake_assert.h"
#include "drake/lcm/drake_lcm_message_handler_interface.h"

namespace drake {
namespace lcm {

DrakeMockLcmSubscriber::DrakeMockLcmSubscriber(DrakeLcmMessageHandlerInterface*
  drake_handler) : drake_handler_(drake_handler) { }


DrakeLcmMessageHandlerInterface* DrakeMockLcmSubscriber::get_subscriber() {
  return drake_handler_;
}

DrakeMockLcm::DrakeMockLcm() {
}

void DrakeMockLcm::StartReceiveThread() {
  DRAKE_DEMAND(!receive_thread_started_);
  receive_thread_started_ = true;
}

void DrakeMockLcm::StopReceiveThread() {
  DRAKE_DEMAND(receive_thread_started_);
  receive_thread_started_ = false;
}

void DrakeMockLcm::Publish(const std::string& channel, const void* data,
                           int data_size) {
  LastPublishedMessage* saved_message{nullptr};
  if (last_published_messages_.find(channel) ==
      last_published_messages_.end()) {
    last_published_messages_[channel] =
        std::make_unique<LastPublishedMessage>();
  }
  saved_message = last_published_messages_[channel].get();

  DRAKE_DEMAND(saved_message);

  saved_message->channel = channel;
  saved_message->data.resize(data_size);
  std::memcpy(&saved_message->data[0], data, data_size);
  saved_message->data_size = data_size;
}

bool DrakeMockLcm::get_last_published_message(const std::string& channel,
    void** data, int* data_size) {
  bool result{true};
  if (last_published_messages_.find(channel) ==
      last_published_messages_.end()) {
    result = false;
  } else {
    LastPublishedMessage* message = last_published_messages_[channel].get();
    DRAKE_DEMAND(message);
    *data = &message->data[0];
    *data_size = message->data_size;
  }
  return result;
}

void DrakeMockLcm::Subscribe(const std::string& channel,
    DrakeLcmMessageHandlerInterface* handler) {
  if (subscriptions_.find(channel) == subscriptions_.end()) {
    auto subscriber = std::make_unique<DrakeMockLcmSubscriber>(handler);
    subscriptions_[channel] = std::move(subscriber);
  } else {
    throw std::runtime_error("DrakeMockLcm::Subscribe: Subscription to "
        "channel \"" + channel + "\" already exists.");
  }
}

void DrakeMockLcm::InduceSubsciberCallback(const std::string& channel,
    const void* data, int data_size) {
  if (receive_thread_started_) {
    if (subscriptions_.find(channel) == subscriptions_.end()) {
      throw std::runtime_error("DrakeMockLcm::InduceSubsciberCallback: No "
          "subscription to channel \"" + channel + "\".");
    } else {
      subscriptions_[channel]->get_subscriber()->HandleMessage(channel, data,
          data_size);
    }
  }
}

}  // namespace lcm
}  // namespace drake
