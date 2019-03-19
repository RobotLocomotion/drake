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

DrakeLcm::DrakeLcm() : DrakeLcm(std::string{}) {}

DrakeLcm::DrakeLcm(std::string lcm_url)
    : requested_lcm_url_(std::move(lcm_url)),
      lcm_(requested_lcm_url_) {
  // Ensure that LCM's self-test happens deterministically (here, in our ctor),
  // and NOT in the receive thread or first HandleSubscriptions call.  Without
  // this, ThreadSanitizer builds may report false positives related to the
  // self-test happening concurrently with the LCM publishing.
  lcm_.getFileno();
}

DrakeLcm::~DrakeLcm() { receive_thread_.reset(); }

void DrakeLcm::StartReceiveThread() {
  DRAKE_DEMAND(receive_thread_ == nullptr);

  // Now launch the thread.
  receive_thread_ = std::make_unique<LcmReceiveThread>(&lcm_);
}

void DrakeLcm::StopReceiveThread() {
  if (receive_thread_ != nullptr) {
    receive_thread_->Stop();
    receive_thread_.reset();
  }
}

::lcm::LCM* DrakeLcm::get_lcm_instance() { return &lcm_; }

std::string DrakeLcm::get_requested_lcm_url() const {
  return requested_lcm_url_;
}

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

int DrakeLcm::HandleSubscriptions(int timeout_millis) {
  DRAKE_THROW_UNLESS(!IsReceiveThreadRunning());
  // Keep pumping handleTimeout until its empty, but only wait for the timeout
  // on the first attempt.
  int total_messages = 0;
  for (int at_most_one = lcm_.handleTimeout(timeout_millis);
       at_most_one > 0; at_most_one = lcm_.handleTimeout(0)) {
    total_messages += at_most_one;
  }
  return total_messages;
}

}  // namespace lcm
}  // namespace drake
