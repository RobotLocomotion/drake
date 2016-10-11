#include "drake/lcm/lcm_receive_thread.h"

#include <iostream>

#include <sys/select.h>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace lcm {

LcmReceiveThread::LcmReceiveThread(::lcm::LCM* lcm) : lcm_(lcm) {
  DRAKE_DEMAND(lcm);
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
    drake::log()->trace("WaitForLcm: select() returned error: {}", error_code);
  } else if (status == -1 && error_code == EINTR) {
    drake::log()->trace("WaitForLcm: select() interrupted.");
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
        drake::log()->trace(
            "LcmReceiverThread::LoopWithSelect: lcm->handle() "
            "returned non-zero value.");
        return;
      }
    }
  }
}

void LcmReceiveThread::Stop() {
  if (!stop_) {
    stop_ = true;
    lcm_thread_.join();
  }
}

}  // namespace lcm
}  // namespace drake
