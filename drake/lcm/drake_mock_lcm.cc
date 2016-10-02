#include "drake/lcm/drake_mock_lcm.h"

#include <cstring>

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
  // Do nothing.
}

void DrakeMockLcm::Publish(const std::string& channel, const void *data,
                       unsigned int data_size) {
  last_published_message_.set = true;
  last_published_message_.channel = channel;
  last_published_message_.data.resize(data_size);
  std::memcpy(&last_published_message_.data[0], data, data_size);
  last_published_message_.data_size = data_size;
}

bool DrakeMockLcm::get_last_published_message(std::string* channel, void** data,
                                              unsigned int* data_size) {
  bool result = last_published_message_.set;
  if (result) {
    *channel = last_published_message_.channel;
    *data = &last_published_message_.data[0];
    *data_size = last_published_message_.data_size;
  }
  return result;
}

void DrakeMockLcm::Subscribe(const std::string& channel,
    void (DrakeLcmMessageHandlerInterface::*handlerMethod)(
        const std::string& channel, const void* message_buffer,
        uint32_t message_length),
    DrakeLcmMessageHandlerInterface* handler) {
  if (subscriptions_.find(channel) == subscriptions_.end()) {
    auto subscriber = make_unique<DrakeMockLcmSubscriber>(handler);
    subscriptions_[channel] = std::move(subscriber);
  } else {
    throw std::runtime_error("DrakeMockLcm::Subscribe: Subscription to "
        "channel \"" + channel + "\" already exists.");
  }
}

void DrakeMockLcm::InduceSubsciberCallback(const std::string& channel,
    const void *data, unsigned int data_size) {
  if (subscriptions_.find(channel) == subscriptions_.end()) {
    throw std::runtime_error("DrakeMockLcm::InduceSubsciberCallback: No "
        "subscription to channel \"" + channel + "\".");
  } else {
    subscriptions_[channel]->get_subscriber()->HandleMessage(channel, data,
        data_size);
  }
}

}  // namespace lcm
}  // namespace drake
