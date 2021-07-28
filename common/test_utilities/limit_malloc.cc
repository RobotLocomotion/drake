#include "drake/common/test_utilities/limit_malloc.h"

#include <signal.h>

#include <atomic>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <stdexcept>
#include <type_traits>
#include <utility>

// Analyze the cases where heap hooks should be compiled and linked. Avoid
// Apple platforms; also avoid ASan builds (spelled various ways for different
// compilers). For ASan background, see #14901.
#define DRAKE_INSTALL_HEAP_HOOKS 1

#ifdef __APPLE__
#  undef DRAKE_INSTALL_HEAP_HOOKS
#  define DRAKE_INSTALL_HEAP_HOOKS 0
#elif defined(__has_feature)
// Detect ASan the clang way.
#  if __has_feature(address_sanitizer)
#    undef DRAKE_INSTALL_HEAP_HOOKS
#    define DRAKE_INSTALL_HEAP_HOOKS 0
#  endif
#elif defined(__SANITIZE_ADDRESS__)
// Detect ASan the gcc way.
#  if __SANITIZE_ADDRESS__
#    undef DRAKE_INSTALL_HEAP_HOOKS
#    define DRAKE_INSTALL_HEAP_HOOKS 0
#  endif
#endif

// Functions that are called during dl_init must not use the sanitizer runtime.
#ifdef __clang__
#define DRAKE_NO_SANITIZE __attribute__((no_sanitize("address")))
#else
#define DRAKE_NO_SANITIZE
#endif

// Declare this function here so its definition can follow the
// platform-specific enabling of observations.
static void EvaluateMinNumAllocations(int observed, int min_num_allocations);

namespace drake {
namespace test {
namespace {

// LimitMalloc does not work properly with some configurations. Check this
// predicate and disarm to avoid erroneous results.
bool IsSupportedConfiguration() {
  static const bool is_supported{DRAKE_INSTALL_HEAP_HOOKS &&
                                 !std::getenv("LSAN_OPTIONS") &&
                                 !std::getenv("VALGRIND_OPTS")};
  return is_supported;
}

// This variable is used as an early short-circuit for our malloc hooks.  When
// false, we execute as minimal code footprint as possible.  This keeps dl_init
// happy when its invoke malloc and we can't yet execute our C++ code.
volatile bool g_any_monitor_has_ever_been_installed = false;

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

  int num_allocations() const { return observed_num_allocations_.load(); }
  const LimitMallocParams& params() const { return args_; }

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
    g_any_monitor_has_ever_been_installed = true;
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
  DRAKE_NO_SANITIZE
  static void malloc(size_t size) {
    if (g_any_monitor_has_ever_been_installed) {
      auto monitor = load();
      if (monitor) {
        monitor->malloc(size);
      }
    }
  }
  DRAKE_NO_SANITIZE
  static void calloc(size_t nmemb, size_t size) {
    if (g_any_monitor_has_ever_been_installed) {
      auto monitor = load();
      if (monitor) {
        monitor->calloc(nmemb, size);
      }
    }
  }
  DRAKE_NO_SANITIZE
  static void realloc(void* ptr, size_t size, bool is_noop) {
    if (g_any_monitor_has_ever_been_installed) {
      auto monitor = load();
      if (monitor) {
        monitor->realloc(ptr, size, is_noop);
      }
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
  if (!IsSupportedConfiguration()) { return; }

  bool failure = false;

  // Check the allocation-call limit.
  const int observed = ++observed_num_allocations_;
  if ((args_.max_num_allocations >= 0) &&
      (observed > args_.max_num_allocations)) {
    failure = true;
  }

  // TODO(jwnimmer-tri) Add more limits (requested bytes?) here.

  if (!failure) { return; }

  // Non-fatal breakpoint action; use with helper script:
  // tools/dynamic_analysis/dump_limit_malloc_stacks
  const auto break_env = std::getenv("DRAKE_LIMIT_MALLOC_NONFATAL_BREAKPOINT");
  // Use the action if the environment variable is defined and not empty.
  if ((break_env != nullptr) && (*break_env != '\0')) {
    ::raise(SIGTRAP);
    return;
  }

  // Report an error (but re-enable malloc before doing so!).
  ActiveMonitor::reset();
  std::cerr << "abort due to malloc #" << observed
            << " while max_num_allocations = "
            << args_.max_num_allocations << " in effect";
  std::cerr << std::endl;
  // TODO(jwnimmer-tri) It would be nice to print a backtrace here.
  std::abort();
}

}  // namespace

LimitMalloc::LimitMalloc() : LimitMalloc({ .max_num_allocations = 0 }) {}

LimitMalloc::LimitMalloc(LimitMallocParams args) {
  // Make sure the configuration check is warm before trying it within a malloc
  // call stack.
  IsSupportedConfiguration();

  // Prepare a monitor with our requested limits.
  auto monitor = std::make_shared<Monitor>(this, std::move(args));

  // Activate our Monitor.
  auto prior = ActiveMonitor::exchange(monitor);
  if (prior) {
    throw std::logic_error("Cannot nest LimitMalloc guards");
  }
}

int LimitMalloc::num_allocations() const {
  return ActiveMonitor::load()->num_allocations();
}

const LimitMallocParams& LimitMalloc::params() const {
  return ActiveMonitor::load()->params();
}

LimitMalloc::~LimitMalloc() {
  // Copy out the monitor's data before we delete it.
  const int observed = num_allocations();
  const LimitMallocParams args = params();

  // De-activate our Monitor.
  auto prior = ActiveMonitor::reset();

  if (!(prior && prior->has_owner(this))) {
    std::cerr << "LimitMalloc dtor invariant failure\n";
  }

  ::EvaluateMinNumAllocations(observed, args.min_num_allocations);
}

}  // namespace test
}  // namespace drake


// Optionally compile the code that places the heap hooks.
#if DRAKE_INSTALL_HEAP_HOOKS
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

static void EvaluateMinNumAllocations(int observed, int min_num_allocations) {
  if (!drake::test::IsSupportedConfiguration()) { return; }

  if ((min_num_allocations >= 0) && (observed < min_num_allocations)) {
    std::cerr << "abort due to scope end with "
              << observed << " mallocs while min_num_allocations = "
              << min_num_allocations << " in effect";
    std::cerr << std::endl;
    std::abort();
  }
}
#else
// Evaluating the minimum is a no-op when no counts are observed.
static void EvaluateMinNumAllocations(int observed, int min_num_allocations) {}
#endif

#undef DRAKE_NO_SANITIZE
#undef DRAKE_INSTALL_HEAP_HOOKS
