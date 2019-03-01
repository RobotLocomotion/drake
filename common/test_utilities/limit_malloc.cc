#include "drake/common/test_utilities/limit_malloc.h"

#include <atomic>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <new>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace drake {
namespace test {
namespace {

// A cut-down version of drake/common/never_destroyed.
template <typename T>
class never_destroyed {
 public:
  never_destroyed() { new (&storage_) T(); }
  T& access() { return *reinterpret_cast<T*>(&storage_); }
 private:
  typename std::aligned_storage<sizeof(T), alignof(T)>::type storage_;
};

// A set of limits and current tallies.
// Intentionally copyable etc., for simplicity.
class Monitor {
 public:
  Monitor(const LimitMalloc* owner, LimitMallocParams args)
      : owner_(owner), args_(std::move(args)) {}

  // Returns the currently-active monitor.  The result will continue to be
  // valid even if the LimitMalloc guard is destroyed in the meantime.
  static std::shared_ptr<Monitor> load() noexcept {
    return std::atomic_load(active());
  }

  // Swaps the currently-active monitor, returning the prior one.
  static std::shared_ptr<Monitor> exchange(std::shared_ptr<Monitor> next) {
    return std::atomic_exchange(active(), std::move(next));
  }

  // Returns true iff our owner is x.
  bool has_owner(const LimitMalloc* x) const { return owner_ == x; }

  // To be called by our hooks when an allocation attempt occurs,
  void ObserveAllocation() noexcept;

 private:
  // Singleton shared_ptr denoting the active monitor.
  static std::shared_ptr<Monitor>* active() noexcept {
    static never_destroyed<std::shared_ptr<Monitor>> result;
    return &result.access();
  }

  // Do not de-reference owner_, it may be dangling; use for operator== only.
  const LimitMalloc* const owner_{};

  // The in-effect limits for this Monitor.
  const LimitMallocParams args_{};

  // The current tallies for this Monitor.
  std::atomic_int observed_num_allocations_{0};
};

void Monitor::ObserveAllocation() noexcept {
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
  std::shared_ptr<Monitor> no_monitor;
  std::atomic_store(active(), no_monitor);
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
  auto prior = Monitor::exchange(monitor);
  if (prior) {
    throw std::logic_error("Cannot nest LimitMalloc guards");
  }
}

LimitMalloc::~LimitMalloc() {
  // De-activate our Monitor.
  std::shared_ptr<Monitor> no_monitor;
  auto prior = Monitor::exchange(no_monitor);
  if (!(prior && prior->has_owner(this))) {
    throw std::underflow_error("LimitMalloc dtor invariant failure");
  }
}

}  // namespace test
}  // namespace drake

// https://www.gnu.org/software/libc/manual/html_node/Replacing-malloc.html#Replacing-malloc
extern "C" void* __libc_malloc(size_t);
extern "C" void* __libc_free(void*);
extern "C" void* __libc_calloc(size_t, size_t);
extern "C" void* __libc_realloc(void*, size_t);
void* malloc(size_t size) {
  auto monitor = drake::test::Monitor::load();
  if (monitor) { monitor->ObserveAllocation(); }
  return __libc_malloc(size);
}
void free(void* ptr) {
  __libc_free(ptr);
}
void* calloc(size_t nmemb, size_t size) {
  auto monitor = drake::test::Monitor::load();
  if (monitor) { monitor->ObserveAllocation(); }
  return __libc_calloc(nmemb, size);
}
void* realloc(void* ptr, size_t size) {
  auto monitor = drake::test::Monitor::load();
  if (monitor) { monitor->ObserveAllocation(); }
  return __libc_realloc(ptr, size);
}
