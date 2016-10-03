#include "drake/lcm/drake_lcm.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace lcm {

DrakeLcmSubscriber::DrakeLcmSubscriber(DrakeLcmMessageHandlerInterface*
  drake_handler) : drake_handler_(drake_handler) { }

void DrakeLcmSubscriber::LcmCallback(const ::lcm::ReceiveBuffer* rbuf,
    const std::string& channel) {
  drake_handler_->HandleMessage(channel, rbuf->data, rbuf->data_size);
}

DrakeLcm::DrakeLcm() {
}

void DrakeLcm::StartReceiveThread() {
  DRAKE_DEMAND(receive_thread_ == nullptr);
  receive_thread_ = make_unique<LcmReceiveThread>(&lcm_);
}

void DrakeLcm::StopReceiveThread() {
  DRAKE_DEMAND(receive_thread_ != nullptr);
  receive_thread_->Stop();
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
        const std::string& channel, const void* message_buffer,
        uint32_t message_length),
    DrakeLcmMessageHandlerInterface* handler) {
  auto subscriber = make_unique<DrakeLcmSubscriber>(handler);
  auto sub = lcm_.subscribe(channel, &DrakeLcmSubscriber::LcmCallback,
      subscriber.get());
  sub->setQueueCapacity(1);
  subscriptions_.push_back(std::move(subscriber));
}

}  // namespace lcm
}  // namespace drake
