#include "drake/lcm/drake_lcm_mock.h"

#include <cstring>

#include "drake/lcm/drake_lcm_message_handler_interface.h"

namespace drake {
namespace lcm {

DrakeLcmMockSubscriber::DrakeLcmMockSubscriber(DrakeLcmMessageHandlerInterface*
  drake_handler) : drake_handler_(drake_handler) { }


DrakeLcmMessageHandlerInterface* DrakeLcmMockSubscriber::get_subscriber() {
  return drake_handler_;
}

DrakeLcmMock::DrakeLcmMock() {
}

void DrakeLcmMock::StartReceiveThread() {
  // Do nothing. A thread is created each time InduceSubscriberCallback() is
  // called.
}

void DrakeLcmMock::Publish(const std::string& channel, const void *data,
                       unsigned int data_size) {
  last_published_message_.set = true;
  last_published_message_.channel = channel;
  last_published_message_.data.resize(data_size);
  std::memcpy(&last_published_message_.data[0], data, data_size);
  last_published_message_.data_size = data_size;
}

bool DrakeLcmMock::get_last_published_message(std::string* channel, void** data,
                                              unsigned int* data_size) {
  bool result = last_published_message_.set;
  if (result) {
    *channel = last_published_message_.channel;
    *data = &last_published_message_.data[0];
    *data_size = last_published_message_.data_size;
  }
  return result;
}

void DrakeLcmMock::Subscribe(const std::string& channel,
    void (DrakeLcmMessageHandlerInterface::*handlerMethod)(
        const void* message_buffer, uint32_t message_length),
    DrakeLcmMessageHandlerInterface* handler) {
  if (subscriptions_.find(channel) == subscriptions_.end() ) {
    auto subscriber = make_unique<DrakeLcmMockSubscriber>(handler);
    subscriptions_[channel] = std::move(subscriber);
  } else {
    throw std::runtime_error("DrakeLcmMock::Subscribe: Subscription to "
        "channel \"" + channel + "\" already exists.");
  }
}

void DrakeLcmMock::InduceSubsciberCallback(const std::string& channel,
    const void *data, unsigned int data_size) {
  // TODO
}

}  // namespace lcm
}  // namespace drake
