#pragma once

#include <atomic>
#include <thread>

#include <lcm/lcm-cpp.hpp>

#include "drake/common/drake_export.h"

namespace drake {
namespace lcm {

/**
 * Maintains a thread that receives LCM messages and dispatches the messages to
 * the appropriate message handlers.
 */
class DRAKE_EXPORT LcmReceiveThread {
 public:
  /**
   * A constructor that instantiates the thread.
   *
   * @param[in] lcm A pointer to the LCM instance through which to access the
   * LCM network. This parameter cannot be `nullptr` and must remain valid for
   * the lifetime of this object.
   */
  explicit LcmReceiveThread(::lcm::LCM* lcm);

  /**
   * The destructor that ensures the thread that receives LCM message is
   * stopped.
   */
  ~LcmReceiveThread();

  // Disable copy and assign.
  LcmReceiveThread(const LcmReceiveThread&) = delete;
  LcmReceiveThread& operator=(const LcmReceiveThread&) = delete;

  /**
   * Stops the LCM receive thread. This stops the reception of LCM messages.
   */
  void Stop();

 private:
  // Loops waiting for LCM messages and dispatching them to the appropriate
  // subscriber message handlers when they arrive.
  void LoopWithSelect();

  // Whether to stop lcm_thread_.
  std::atomic<bool> stop_{false};

  // A pointer to the LCM instance.
  ::lcm::LCM* const lcm_{nullptr};

  // The thread responsible for receiving LCM messages.
  std::thread lcm_thread_;
};

}  // namespace lcm
}  // namespace drake
