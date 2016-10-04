#pragma once

#include <memory>

#include "lcm/lcm-cpp.hpp"

#include "drake/drakeLcm_export.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/lcm/drake_lcm_message_handler_interface.h"
#include "drake/lcm/lcm_receive_thread.h"

namespace drake {
namespace lcm {

using std::make_unique;
using std::unique_ptr;

/// @cond

// This is the actual subscriber to an LCM channel. It simply extracts the
// serialized LCM message and passes it to the `DrakeLcmMessageHandlerInterface`
// object. A single type of subscriber is used to avoid DrakeLcm from being
// templated on the subscriber type.
class DRAKELCM_EXPORT DrakeLcmSubscriber {
 public:
  explicit DrakeLcmSubscriber(DrakeLcmMessageHandlerInterface* drake_handler);

  // Disables copy and assign.
  DrakeLcmSubscriber(const DrakeLcmSubscriber&) = delete;
  DrakeLcmSubscriber& operator=(const DrakeLcmSubscriber&) = delete;

  void LcmCallback(const ::lcm::ReceiveBuffer* rbuf,
      const std::string& channel);

 private:
  DrakeLcmMessageHandlerInterface* drake_handler_;
};

/// @endcond

/**
 * A wrapper around a *real* LCM instance.
 */
class DRAKELCM_EXPORT DrakeLcm : public DrakeLcmInterface {
 public:
  DrakeLcm();

  // Disable copy and assign.
  DrakeLcm(const DrakeLcm&) = delete;
  DrakeLcm& operator=(const DrakeLcm&) = delete;

  /**
   * A destructor that forces the receive thread to be stopped.
   */
  ~DrakeLcm();

  void StartReceiveThread() override;

  void StopReceiveThread() override;

  /**
   * An accessor to the real LCM instance encapsulated by this object. The
   * returned pointer is guaranteed to be valid for the duration of this
   * object's lifetime.
   */
  ::lcm::LCM* get_lcm_instance();

  void Publish(const std::string& channel, const void *data,
               int data_size) override;

  void Subscribe(const std::string& channel,
      void (DrakeLcmMessageHandlerInterface::*handlerMethod)(
          const std::string& channel, const void* message_buffer,
          int message_length),
      DrakeLcmMessageHandlerInterface* handler) override;

 private:
  ::lcm::LCM lcm_;
  std::unique_ptr<LcmReceiveThread> receive_thread_{nullptr};
  std::vector<unique_ptr<DrakeLcmSubscriber>> subscriptions_{};
};

}  // namespace lcm
}  // namespace drake
