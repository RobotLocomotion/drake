#include <thread>

#include "drake/common/autodiff.h"
#include "drake/common/test/autodiffxd_test.h"

namespace drake {
namespace test {
namespace {

TEST_F(AutoDiffXdTest, Thread) {
  const int kSleepMs = 10;
  const int kAdSize = 4;
  const int kBlockSize = 4;
  const int kThreads = 1000;

  // In each thread, ask for some AutoDiffXd and then sleep.
  auto thread_action = []() {
    std::vector<AutoDiffXd> held;
    for (int k = 0; k < kBlockSize; k++) {
      Eigen::VectorXd v(kAdSize);
      v.setZero();
      held.emplace_back(1, v);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(kSleepMs));
  };

  // Launch many threads as quickly as possible, each doing thread_action.
  std::vector<std::thread> threads;
  for (int i = 0; i < kThreads; ++i) {
    threads.emplace_back(thread_action);
  }

  // Wait for them all to finish.
  for (auto& item : threads) {
    item.join();
  }

}

}  // namespace
}  // namespace test
}  // namespace drake
