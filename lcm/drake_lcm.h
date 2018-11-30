#pragma once

#include <list>
#include <memory>
#include <string>

#include "lcm/lcm-cpp.hpp"

#include "drake/common/drake_copyable.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/lcm/lcm_receive_thread.h"

namespace drake {
namespace lcm {

/**
 * A wrapper around a *real* LCM instance.
 */
class DrakeLcm : public DrakeLcmInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DrakeLcm);

  /**
   * Constructs using LCM's default URL (either the default hard-coded URL, or
   * else LCM_DEFAULT_URL environment variable if it is set).
   */
  DrakeLcm();

  /**
   * Constructs using the given URL.  If empty, it will use the default URL as
   * per the no-argument constructor.
   */
  explicit DrakeLcm(std::string lcm_url);

  /**
   * A destructor that forces the receive thread to be stopped.
   */
  ~DrakeLcm() override;

   /**
   * Starts the receive thread. This must be called for subscribers to receive
   * any messages.
   *
   * @pre StartReceiveThread() was not called.
   */
  void StartReceiveThread();

  /**
   * Stops the receive thread. This must be called prior to any subscribers
   * being destroyed. Note that the receive thread will be automatically stopped
   * by this class's destructor, so usage of this method will be extremely rare.
   * It will only be needed if this class's instance and the subscribers to LCM
   * channels are owned by different classes. In such a scenario, this method
   * can be used to ensure the receive thread is destroyed before the
   * subscribers are destroyed.
   *
   * @pre StartReceiveThread() was called.
   */
  void StopReceiveThread();

  /**
   * Indicates that the receiving thread is running.
   */
  bool IsReceiveThreadRunning() const {
    return receive_thread_ != nullptr;
  }

  /**
   * An accessor to the real LCM instance encapsulated by this object. The
   * returned pointer is guaranteed to be valid for the duration of this
   * object's lifetime.
   */
  ::lcm::LCM* get_lcm_instance();

  /**
   * Returns the LCM URL passed into the constructor; this can be empty.
   */
  std::string get_requested_lcm_url() const;

  void Publish(const std::string& channel, const void* data,
               int data_size, optional<double> time_sec) override;

  void Subscribe(const std::string&, HandlerFunction) override;

 private:
  std::string requested_lcm_url_;
  ::lcm::LCM lcm_;
  std::unique_ptr<LcmReceiveThread> receive_thread_{nullptr};
  std::list<HandlerFunction> handlers_;
};

}  // namespace lcm
}  // namespace drake
