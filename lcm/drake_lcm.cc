#include "drake/lcm/drake_lcm.h"

#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"

namespace drake {
namespace lcm {
namespace {

void Callback(const ::lcm::ReceiveBuffer* buffer,
              const std::string& /* channel */ ,
              DrakeLcm::HandlerFunction* context) {
  DRAKE_DEMAND(buffer != nullptr);
  DRAKE_DEMAND(context != nullptr);
  DrakeLcm::HandlerFunction& handler = *context;
  handler(buffer->data, buffer->data_size);
}

}  // namespace

DrakeLcm::DrakeLcm() {}

DrakeLcm::~DrakeLcm() { receive_thread_.reset(); }

void DrakeLcm::StartReceiveThread() {
  DRAKE_DEMAND(receive_thread_ == nullptr);
  receive_thread_ = std::make_unique<LcmReceiveThread>(&lcm_);
}

void DrakeLcm::StopReceiveThread() {
  if (receive_thread_ != nullptr) {
    receive_thread_->Stop();
    receive_thread_.reset();
  }
}

::lcm::LCM* DrakeLcm::get_lcm_instance() { return &lcm_; }

void DrakeLcm::Publish(const std::string& channel, const void* data,
                       int data_size, optional<double>) {
  DRAKE_THROW_UNLESS(!channel.empty());
  lcm_.publish(channel, data, data_size);
}

void DrakeLcm::Subscribe(const std::string& channel, HandlerFunction handler) {
  DRAKE_THROW_UNLESS(!channel.empty());
  handlers_.emplace_back(std::move(handler));
  // The handlers_ is a std::list so that the context pointers remain stable.
  HandlerFunction* const context = &handlers_.back();
  lcm_.subscribeFunction(channel, &Callback, context)->setQueueCapacity(1);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
void DrakeLcm::Subscribe(const std::string& channel,
                         DrakeLcmMessageHandlerInterface* handler) {
  Subscribe(channel, std::bind(
      std::mem_fn(&DrakeLcmMessageHandlerInterface::HandleMessage), handler,
      channel, std::placeholders::_1, std::placeholders::_2));
}
#pragma GCC diagnostic pop  // pop -Wdeprecated-declarations

}  // namespace lcm
}  // namespace drake
