#include "drake/common/test_utilities/limit_malloc.h"

#include <atomic>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace drake {
namespace test {
namespace {

// A set of limits and current tallies.
class Monitor {
 public:
  Monitor(const LimitMalloc* owner, LimitMallocParams args)
      : owner_(owner), args_(std::move(args)) {}

  // Returns true iff our owner is x.
  bool has_owner(const LimitMalloc* x) const { return owner_ == x; }

  // To be called by our hooks when an allocation attempt occurs,
  void malloc(size_t) { ObserveAllocation(); }
  void calloc(size_t, size_t) { ObserveAllocation(); }
  void realloc(void*, size_t, bool is_noop) {
    if (!(is_noop && args_.ignore_realloc_noops)) {
      ObserveAllocation();
    }
  }

 private:
  void ObserveAllocation();

  // Do not de-reference owner_, it may be dangling; use for operator== only.
  const LimitMalloc* const owner_{};

  // The in-effect limits for this Monitor.
  const LimitMallocParams args_{};

  // The current tallies for this Monitor.
  std::atomic_int observed_num_allocations_{0};
};

// A cut-down version of drake/common/never_destroyed.
template <typename T>
class never_destroyed {
 public:
  never_destroyed() { new (&storage_) T(); }
  T& access() { return *reinterpret_cast<T*>(&storage_); }
 private:
  typename std::aligned_storage<sizeof(T), alignof(T)>::type storage_;
};

class ActiveMonitor {
 public:
  // Returns the currently-active monitor (or nullptr if none).  The result
  // remains valid even if the LimitMalloc guard is destroyed in the meantime.
  static std::shared_ptr<Monitor> load() {
    auto& self = singleton();
    std::shared_ptr<Monitor> result;
    if (self.active_.get() != nullptr) {
      std::lock_guard<std::mutex> guard(self.mutex_);
      result = self.active_;
    }
    return result;
  }

  // Swaps the currently-active monitor, returning the prior one.
  static std::shared_ptr<Monitor> exchange(std::shared_ptr<Monitor> next) {
    auto& self = singleton();
    std::lock_guard<std::mutex> guard(self.mutex_);
    self.active_.swap(next);
    return next;
  }

  // Removes and returns currently-active monitor.
  static std::shared_ptr<Monitor> reset() {
    std::shared_ptr<Monitor> no_monitor;
    return exchange(no_monitor);
  }

  // To be called by our hooks when an allocation attempt occurs,
  static void malloc(size_t size) {
    auto monitor = singleton().load();
    if (monitor) {
      monitor->malloc(size);
    }
  }
  static void calloc(size_t nmemb, size_t size) {
    auto monitor = singleton().load();
    if (monitor) {
      monitor->calloc(nmemb, size);
    }
  }
  static void realloc(void* ptr, size_t size, bool is_noop) {
    auto monitor = singleton().load();
    if (monitor) {
      monitor->realloc(ptr, size, is_noop);
    }
  }

 private:
  // Singleton mutex guarding active_;
  std::mutex mutex_;
  // Singleton shared_ptr denoting the active monitor.
  std::shared_ptr<Monitor> active_;

  static ActiveMonitor& singleton() {
    static never_destroyed<ActiveMonitor> instance;
    return instance.access();
  }
};

void Monitor::ObserveAllocation() {
  bool failure = false;

  // Check the allocation-call limit.
  const int observed = ++observed_num_allocations_;
  if ((args_.max_num_allocations >= 0) &&
      (observed > args_.max_num_allocations)) {
    failure = true;
  }

  // TODO(jwnimmer-tri) Add more limits (requested bytes?) here.

  if (!failure) { return; }

  // Report an error (but re-enable malloc before doing so!).
  ActiveMonitor::reset();
  std::cerr << "abort due to malloc while LimitMalloc is in effect";
  std::cerr << std::endl;
  // TODO(jwnimmer-tri) It would be nice to print a backtrace here.
  std::abort();
}

}  // namespace

LimitMalloc::LimitMalloc() : LimitMalloc({ .max_num_allocations = 0 }) {}

LimitMalloc::LimitMalloc(LimitMallocParams args) {
  // Prepare a monitor with our requested limits.
  auto monitor = std::make_shared<Monitor>(this, std::move(args));

  // Activate our Monitor.
  auto prior = ActiveMonitor::exchange(monitor);
  if (prior) {
    throw std::logic_error("Cannot nest LimitMalloc guards");
  }
}

LimitMalloc::~LimitMalloc() {
  // De-activate our Monitor.
  auto prior = ActiveMonitor::reset();
  if (!(prior && prior->has_owner(this))) {
    std::cerr << "LimitMalloc dtor invariant failure\n";
  }
}

}  // namespace test
}  // namespace drake

#ifndef __APPLE__
// https://www.gnu.org/software/libc/manual/html_node/Replacing-malloc.html#Replacing-malloc
extern "C" void* __libc_malloc(size_t);
extern "C" void* __libc_free(void*);
extern "C" void* __libc_calloc(size_t, size_t);
extern "C" void* __libc_realloc(void*, size_t);
void* malloc(size_t size) {
  drake::test::ActiveMonitor::malloc(size);
  return __libc_malloc(size);
}
void free(void* ptr) {
  __libc_free(ptr);
}
void* calloc(size_t nmemb, size_t size) {
  drake::test::ActiveMonitor::calloc(nmemb, size);
  return __libc_calloc(nmemb, size);
}
void* realloc(void* ptr, size_t size) {
  void* result = __libc_realloc(ptr, size);
  bool is_noop = (result == ptr);
  drake::test::ActiveMonitor::realloc(ptr, size, is_noop);
  return result;
}
#endif
