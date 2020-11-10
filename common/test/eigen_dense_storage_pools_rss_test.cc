/// @file
/// This test is built and run separately because its use of python conflicts
/// with use of valgrind tools.

#include <cmath>
#include <thread>
#include <vector>

#include "pybind11/embed.h"
#include <gtest/gtest.h>

// Disable the include checking intended for clients; this module is internal
// to the autodiff system.
#define DRAKE_COMMON_AUTODIFF_HEADER
#include "drake/common/eigen_dense_storage_pools.h"
#undef DRAKE_COMMON_AUTODIFF_HEADER

namespace Eigen {

using DenseStoragePools = DenseStorage<
  /* T = */ double,
  /* Size = */ drake::internal::kMaxRowsAtCompileTimeThatTriggersPools,
  /* _Rows = */ Dynamic,
  /* _Cols = */ 1,
  /* _Options = */ 0>;


namespace {

// This bit of python code is the most convenient and platform-independent way
// to measure memory Resident Set Size (RSS).
const char* py_rss_code = R"""(
import psutil
p = psutil.Process()
rss = p.memory_info().rss
)""";

// Get this process' current resident set size in bytes.
int get_rss() {
  namespace py = pybind11;
  py::scoped_interpreter guard{};
  auto locals = py::dict();
  py::exec(py_rss_code, py::globals(), locals);
  int rss = locals["rss"].cast<int>();
  return rss;
}

// Ensure that memory held in a per-thread pool is released when the thread
// terminates.
GTEST_TEST(EigenDenseStoragePoolsRssTest, ThreadReaper) {
  int rss_before = get_rss();

  int rss_in_thread{};
  std::thread thread1([&rss_in_thread]() {
      // Allocate a lot of memory in the form of DenseStoragePools instances.
      const int kBlockSize = (1 << 14);
      const int kBlockCount = (1 << 12);

      std::vector<DenseStoragePools> claimed;
      for (int k = 0; k < kBlockCount; k++) {
        claimed.emplace_back(DenseStoragePools(kBlockSize, kBlockSize, 1));
      }

      rss_in_thread = get_rss();
      // Scope end releases the claimed storage back to pool.
      // Thread end destroys the per-thread pool.
    });

  thread1.join();

  int rss_after = get_rss();

  // RSS after is at least as much as before the thread exercise.
  EXPECT_TRUE(rss_after >= rss_before);
  // RSS during the thread is greater than after it terminated.
  EXPECT_TRUE(rss_in_thread > rss_after);
  // RSS after is closer to RSS before than to RSS during the thread.
  EXPECT_TRUE(std::abs(rss_after - rss_before) <
              std::abs(rss_after - rss_in_thread));
}

}  // namespace
}  // namespace Eigen
