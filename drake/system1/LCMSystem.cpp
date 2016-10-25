
#include <iostream>
#include "drake/system1/LCMSystem.h"

#if defined(_WIN32)
#include <Winsock2.h>
#else
#include <sys/select.h>
#endif

// NOLINTNEXTLINE(build/namespaces) This code will be deleted soon.
using namespace drake;

bool waitForLCM(lcm::LCM& lcm, double timeout) {
  int lcmFd = lcm.getFileno();

  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = timeout * 1e6;

  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(lcmFd, &fds);

  int status = select(lcmFd + 1, &fds, 0, 0, &tv);
  if (status == -1 && errno != EINTR) {
    std::cout << "select() returned error: " << errno << std::endl;
  } else if (status == -1 && errno == EINTR) {
    std::cout << "select() interrupted" << std::endl;
  }
  return (status > 0 && FD_ISSET(lcmFd, &fds));
}

void drake::internal::LCMLoop::loopWithSelect() {
  //    cout << "starting lcm handler thread " << this_thread::get_id() << endl;

  while (!this->stop) {
    const double timeoutInSeconds = 0.3;
    bool lcmReady = waitForLCM(lcm, timeoutInSeconds);

    if (this->stop) break;

    if (lcmReady) {
      if (lcm.handle() != 0) {
        std::cout << "lcm->handle() returned non-zero" << std::endl;
        break;
      }
    }
  }
  //    cout << "stopping lcm handler thread " << this_thread::get_id() << endl;
}
