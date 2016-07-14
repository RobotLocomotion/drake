#include "drake/systems/lcm/lcm_receive_thread.h"

#include <iostream>

#if defined(_WIN32)
#include <Winsock2.h>
#else
#include <sys/select.h>
#endif

namespace drake {
namespace systems {
namespace lcm {

LcmReceiveThread::LcmReceiveThread(::lcm::LCM* lcm) : stop_(false), lcm_(lcm) {
  // Checks if the supplied lcm parameter is nullptr. Throws an exception if it
  // is.
  if (lcm_ == nullptr) {
    throw std::runtime_error(
      "LcmReceiveThread: ERROR: lcm pointer is nullptr.");
  }

  // Spawns a thread that calls this->LoopWithSelect().
  lcm_thread_ = std::thread(&LcmReceiveThread::LoopWithSelect, this);
}

LcmReceiveThread::~LcmReceiveThread() {
  stop_ = true;
  lcm_thread_.join();
}

::lcm::LCM* LcmReceiveThread::get_lcm() const {
  return lcm_;
}

// Waits for an LCM message to arrive.
bool WaitForLcm(::lcm::LCM* lcm, double timeout) {
  int lcmFd = lcm->getFileno();

  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = timeout * 1e6;

  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(lcmFd, &fds);

  int status = select(lcmFd + 1, &fds, 0, 0, &tv);
  if (status == -1 && errno != EINTR) {
    // throw std::runtime_error("WaitForLcm: select() returned error: " +
    //   std::to_string(errno));
    std::cout << "WaitForLcm: select() returned error: " << errno << std::endl;
  } else if (status == -1 && errno == EINTR) {
    // throw std::runtime_error("WaitForLcm: select() interrupted.");
    std::cout << "WaitForLcm: select() interrupted." << std::endl;
  }
  return (status > 0 && FD_ISSET(lcmFd, &fds));
}

void LcmReceiveThread::LoopWithSelect() {
  while (!stop_) {
    const double timeoutInSeconds = 0.3;
    bool lcmReady = WaitForLcm(lcm_, timeoutInSeconds);

    if (stop_) break;

    if (lcmReady) {
      if (lcm_->handle() != 0) {
        std::cout << "LcmReceiverThread: LoopWithSelect: lcm->handle() "
                  << "returned non-zero value."
                  << std::endl;
        break;
      }
    }
  }
}

void LcmReceiveThread::Stop() {
  stop_ = true;
}

}  // namespace lcm
}  // namespace systems
}  // namespace drake
