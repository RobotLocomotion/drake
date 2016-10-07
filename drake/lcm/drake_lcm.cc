#include "drake/lcm/drake_lcm.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace lcm {

// This is the actual subscriber to an LCM channel. It simply extracts the
// serialized LCM message and passes it to the `DrakeLcmMessageHandlerInterface`
// object. A single type of subscriber is used to avoid DrakeLcm from being
// templated on the subscriber type.
class DRAKE_EXPORT DrakeLcm::Subscriber {
 public:
  explicit Subscriber(DrakeLcmMessageHandlerInterface* drake_handler)
      : drake_handler_(drake_handler) {}

  // Disables copy, assign, and move.
  Subscriber(const Subscriber&) = delete;
  Subscriber& operator=(const Subscriber&) = delete;
  Subscriber(Subscriber&&) = delete;
  Subscriber& operator=(Subscriber&&) = delete;

  void LcmCallback(const ::lcm::ReceiveBuffer* rbuf,
                   const std::string& channel) {
    drake_handler_->HandleMessage(channel, rbuf->data, rbuf->data_size);
  }

 private:
  DrakeLcmMessageHandlerInterface* const drake_handler_{};
};

DrakeLcm::DrakeLcm() {}

DrakeLcm::~DrakeLcm() { receive_thread_.reset(); }

void DrakeLcm::StartReceiveThread() {
  DRAKE_DEMAND(receive_thread_ == nullptr);
  receive_thread_ = std::make_unique<LcmReceiveThread>(&lcm_);
}

void DrakeLcm::StopReceiveThread() {
  if (receive_thread_ != nullptr) receive_thread_->Stop();
}

::lcm::LCM* DrakeLcm::get_lcm_instance() { return &lcm_; }

void DrakeLcm::Publish(const std::string& channel, const void* data,
                       int data_size) {
  lcm_.publish(channel, data, data_size);
}

void DrakeLcm::Subscribe(const std::string& channel,
                         DrakeLcmMessageHandlerInterface* handler) {
  auto subscriber = std::make_unique<Subscriber>(handler);
  auto sub =
      lcm_.subscribe(channel, &Subscriber::LcmCallback, subscriber.get());
  sub->setQueueCapacity(1);
  subscriptions_.push_back(std::move(subscriber));
}

}  // namespace lcm
}  // namespace drake
