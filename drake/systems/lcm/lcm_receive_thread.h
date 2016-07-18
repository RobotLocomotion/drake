#pragma once

#include <atomic>
#include <thread>
#include <iostream>

#include <lcm/lcm-cpp.hpp>

#include "drake/drakeLCMSystem2_export.h"

namespace drake {
namespace systems {
namespace lcm {

/**
 * Implements a thread that is responsible for reciving LCM messages.
 */
class DRAKELCMSYSTEM2_EXPORT LcmReceiveThread {
 public:
  /**
   * The constructor.
   *
   * @param[in] lcm A pointer to the LCM subsystem through which to loop.
   * This parameter cannot be nullptr. This parameter must remain valid for the
   * lifetime of this `LcmReceiveThread`.
   */
  explicit LcmReceiveThread(::lcm::LCM* lcm);

  /**
   * The destructor. Ensures that the thread used for receiving LCM message is
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
  /**
   * Loops waiting for LCM messages and dispatches them to the appropriate
   * subscriber message handlers when they arrive.
   */
  void LoopWithSelect();

  // Whether to stop the lcm_thread_.
  std::atomic<bool> stop_{false};

  // A pointer to the LCM subsystem.
  ::lcm::LCM* const lcm_{nullptr};

  // The thread responsible for receiving LCM messages.
  std::thread lcm_thread_;
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake
