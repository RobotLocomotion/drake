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
  // Ensure that LCM's self-test happens before our thread starts running.
  // Without this, ThreadSanitizer builds may report false positives related to
  // the self-test happening concurrently with the LCM publishing.
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
  int result = 0;

  int single_handled = lcm_.handleTimeout(timeout_millis);
  while (true) {
    // Check for LCM error.
    if (single_handled < 0) { return -1; }

    // Check for completion.
    if (single_handled == 0) { break; }

    // Accumulate the message tally.
    result += single_handled;

    // Keep pumping but do NOT block again.
    single_handled = lcm_.handleTimeout(0);
  }

  return result;
}

}  // namespace lcm
}  // namespace drake
