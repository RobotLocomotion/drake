#include "drake/lcm/lcm_receive_thread.h"

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace lcm {

namespace {
// Keep this in sync with drake_lcm.cc.
const int kMagicTimeoutMillis = 247;
}

LcmReceiveThread::LcmReceiveThread(::lcm::LCM* lcm) : lcm_(lcm) {
  DRAKE_DEMAND(lcm != nullptr);
  lcm_thread_ = std::thread(&LcmReceiveThread::Looper, this);
}

LcmReceiveThread::LcmReceiveThread(DrakeLcm* lcm)
    : LcmReceiveThread(lcm->get_lcm_instance()) {}

LcmReceiveThread::~LcmReceiveThread() {
  Stop();
}

void LcmReceiveThread::Looper() {
  while (!stop_) {
    const int count = lcm_->handleTimeout(kMagicTimeoutMillis);
    if (count < 0) {
      drake::log()->error("lcm::handleTimeout() error");
      return;
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
