#include "drake/systems/lcm/lcm_receive_thread.h"

#include <iostream>

#if defined(_WIN32)
#include <Winsock2.h>
#else
#include <sys/select.h>
#endif

#include "drake/common/drake_assert.h"

namespace drake {
namespace systems {
namespace lcm {

LcmReceiveThread::LcmReceiveThread(::lcm::LCM* lcm) : lcm_(lcm) {
  DRAKE_DEMAND(lcm);
  // Spawns a thread that calls this->LoopWithSelect().
  lcm_thread_ = std::thread(&LcmReceiveThread::LoopWithSelect, this);
}

LcmReceiveThread::~LcmReceiveThread() {
  // TODO(liang.fok) Refactor to not employ a blocking destructor. Prefer to use
  // detach() after removing use of cross-thread bare pointers. Before this is
  // done, do NOT use Drake in a safety critical system!
  Stop();
}

namespace {

// Waits for an LCM message to arrive.
bool WaitForLcm(::lcm::LCM* lcm, double timeout) {
  int lcm_file_descriptor = lcm->getFileno();

  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = timeout * 1e6;

  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(lcm_file_descriptor, &fds);

  int status = select(lcm_file_descriptor + 1, &fds, 0, 0, &tv);
  int error_code = errno;
  if (status == -1 && error_code != EINTR) {
    std::cout << "WaitForLcm: select() returned error: " << error_code
              << std::endl;
  } else if (status == -1 && error_code == EINTR) {
    std::cout << "WaitForLcm: select() interrupted." << std::endl;
  }
  return (status > 0 && FD_ISSET(lcm_file_descriptor, &fds));
}

}  // namespace

void LcmReceiveThread::LoopWithSelect() {
  while (!stop_) {
    const double timeout_in_seconds = 0.3;
    bool lcm_ready = WaitForLcm(lcm_, timeout_in_seconds);

    if (stop_) break;

    if (lcm_ready) {
      if (lcm_->handle() != 0) {
        std::cout << "LcmReceiverThread: LoopWithSelect: lcm->handle() "
                  << "returned non-zero value." << std::endl;
        return;
      }
    }
  }
}

void LcmReceiveThread::Stop() {
  stop_ = true;
  lcm_thread_.join();
}

}  // namespace lcm
}  // namespace systems
}  // namespace drake
