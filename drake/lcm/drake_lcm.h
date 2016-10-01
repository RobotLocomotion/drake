#pragma once

#include <memory>

#include "drake/drakeLcm_export.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/lcm/drake_lcm_message_handler_interface.h"
#include "drake/lcm/lcm_receive_thread.h"

namespace drake {
namespace lcm {

using std::make_unique;
using std::unique_ptr;

/// @cond

// This is the actual subscriber to the LCM channel. It simply extracts the
// serialized LCM message and passes it to the `DrakeLcmMessageHandlerInterface`
// object. A single type of subscriber is used to avoid DrakeLcm from being
// templated on the subscriber type.
class DRAKELCM_EXPORT DrakeLcmSubscriber {
 public:
  DrakeLcmSubscriber(DrakeLcmMessageHandlerInterface* drake_handler) :
      drake_handler_(drake_handler) {

  }

  void LcmCallback(const ::lcm::ReceiveBuffer* rbuf,
      const std::string& channel) {
    drake_handler_->HandleMessage(rbuf->data, rbuf->data_size);
  }

 private:
  DrakeLcmMessageHandlerInterface* drake_handler_;
};

/// @endcond

/**
 * A wrapper around a *real* LCM instance.
 */
class DRAKELCM_EXPORT DrakeLcm : public DrakeLcmInterface {
 public:
  /**
   * A constructor that initializes an LCM instance and LCM receive thread.
   */
  explicit DrakeLcm() {
  }

  /**
   * Starts the receive thread. This should be called *after* all of the
   * subscribers are instantiated. Otherwise, the subscribers may be destroyed
   * while the receive thread is still running resulting a segmentation fault.
   */
  void StartReceiveThread() {
    receive_thread_ = make_unique<LcmReceiveThread>(&lcm_);
  }

  /**
   * An accessor to the real LCM instance encapsulated by this object. The
   * returned pointer is guaranteed to be valid for the duration of this
   * object's lifetime.
   */
  ::lcm::LCM* get_lcm_instance() {
    return &lcm_;
  }

  void Publish(const std::string& channel, const void *data,
               unsigned int data_size) override {
    lcm_.publish(channel, data, data_size);
  }

  void Subscribe(const std::string& channel,
      void (DrakeLcmMessageHandlerInterface::*handlerMethod)(
          const void* message_buffer, uint32_t message_length),
      DrakeLcmMessageHandlerInterface* handler) override {
    auto subscriber = make_unique<DrakeLcmSubscriber>(handler);
    lcm_.subscribe(channel, &DrakeLcmSubscriber::LcmCallback, subscriber.get());
    subscriptions_.push_back(std::move(subscriber));
  }

 private:
  ::lcm::LCM lcm_;
  std::unique_ptr<LcmReceiveThread> receive_thread_;
  std::vector<unique_ptr<DrakeLcmSubscriber>> subscriptions_;
};

}  // namespace lcm
}  // namespace drake
