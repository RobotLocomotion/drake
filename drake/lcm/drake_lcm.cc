#include "drake/lcm/drake_lcm.h"

namespace drake {
namespace lcm {

DrakeLcmSubscriber::DrakeLcmSubscriber(DrakeLcmMessageHandlerInterface*
  drake_handler) : drake_handler_(drake_handler) { }

void DrakeLcmSubscriber::LcmCallback(const ::lcm::ReceiveBuffer* rbuf,
    const std::string& channel) {
  drake_handler_->HandleMessage(rbuf->data, rbuf->data_size);
}

DrakeLcm::DrakeLcm() {
}

void DrakeLcm::StartReceiveThread() {
  receive_thread_ = make_unique<LcmReceiveThread>(&lcm_);
}

::lcm::LCM* DrakeLcm::get_lcm_instance() {
  return &lcm_;
}

void DrakeLcm::Publish(const std::string& channel, const void *data,
                       unsigned int data_size) {
  lcm_.publish(channel, data, data_size);
}

void DrakeLcm::Subscribe(const std::string& channel,
    void (DrakeLcmMessageHandlerInterface::*handlerMethod)(
        const void* message_buffer, uint32_t message_length),
    DrakeLcmMessageHandlerInterface* handler) {
  auto subscriber = make_unique<DrakeLcmSubscriber>(handler);
  lcm_.subscribe(channel, &DrakeLcmSubscriber::LcmCallback, subscriber.get());
  subscriptions_.push_back(std::move(subscriber));
}

}  // namespace lcm
}  // namespace drake
