#pragma once

#include <memory>
#include <string>
#include <vector>

#include "lcm/lcm-cpp.hpp"

#include "drake/common/drake_export.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/lcm/drake_lcm_message_handler_interface.h"
#include "drake/lcm/lcm_receive_thread.h"

namespace drake {
namespace lcm {

/**
 * A wrapper around a *real* LCM instance.
 */
class DRAKE_EXPORT DrakeLcm : public DrakeLcmInterface {
 public:
  DrakeLcm();

  // Disable copy and assign.
  DrakeLcm(const DrakeLcm&) = delete;
  DrakeLcm& operator=(const DrakeLcm&) = delete;

  /**
   * A destructor that forces the receive thread to be stopped.
   */
  ~DrakeLcm() override;

  void StartReceiveThread() override;

  void StopReceiveThread() override;

  /**
   * An accessor to the real LCM instance encapsulated by this object. The
   * returned pointer is guaranteed to be valid for the duration of this
   * object's lifetime.
   */
  ::lcm::LCM* get_lcm_instance();

  void Publish(const std::string& channel, const void* data,
               int data_size) override;

  void Subscribe(const std::string& channel,
                 DrakeLcmMessageHandlerInterface* handler) override;

 private:
  class Subscriber;
  ::lcm::LCM lcm_;
  std::unique_ptr<LcmReceiveThread> receive_thread_{nullptr};
  std::vector<std::unique_ptr<Subscriber>> subscriptions_;
};

}  // namespace lcm
}  // namespace drake
