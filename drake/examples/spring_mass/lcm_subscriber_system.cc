#include "drake/examples/spring_mass/lcm_subscriber_system.h"

#include <iostream>

#if defined(_WIN32)
#include <Winsock2.h>
#else
#include <sys/select.h>
#endif

namespace drake {
namespace systems {
namespace lcm {
namespace internal {

// Waits for an LCM message to arrive.
bool WaitForLcm(::lcm::LCM& lcm, double timeout) {
  int lcmFd = lcm.getFileno();

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

void LcmLoop::LoopWithSelect() {
  while (!stop_) {
    const double timeoutInSeconds = 0.3;
    bool lcmReady = WaitForLcm(lcm_, timeoutInSeconds);

    if (stop_) break;

    if (lcmReady) {
      if (lcm_.handle() != 0) {
        std::cout << "LoopWithSelect: lcm->handle() returned non-zero"
                  << std::endl;
        break;
      }
    }
  }
}

void LcmLoop::Stop() {
  stop_ = true;
}

}  // namespace internal
}  // namespace lcm
}  // namespace systems
}  // namesapce drake