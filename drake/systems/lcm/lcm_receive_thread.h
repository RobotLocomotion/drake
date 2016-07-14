#pragma once

// TODO(liang.fok) Move this class into a directory that is dedicated to
// LCM-based systems after it is mature and proven useful.

#include <mutex>
#include <stdexcept>
#include <thread>
#include <iostream>

#include <lcm/lcm-cpp.hpp>

#include "drake/drakeLCMSystem2_export.h"
#include "drake/systems/lcm/lcm_basic_vector_translator.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/basic_state_vector.h"
#include "drake/systems/framework/system_interface.h"
#include "drake/systems/framework/vector_interface.h"

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
   *
   * @throws runtime_error if @p lcm is nullptr.
   */
  explicit LcmReceiveThread(::lcm::LCM* lcm);

  /**
   * The destructor. Ensures that the thread used for receiving LCM message is
   * stopped.
   */
  ~LcmReceiveThread();

  /**
   * Returns a pointer to the LCM subsystem.
   */
  ::lcm::LCM* get_lcm() const;

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
  bool stop_{false};

  // A pointer to the LCM subsystem.
  ::lcm::LCM* lcm_{nullptr};

  // The thread responsible for receiving LCM messages.
  std::thread lcm_thread_;
};

}  // namespace lcm
}  // namespace systems
}  // namesapce drake
