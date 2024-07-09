#include "drake/common/hwy_dynamic.h"

#include <mutex>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/never_destroyed.h"

namespace drake {
namespace internal {
namespace {

struct Globals {
  mutable std::mutex mutex;
  std::vector<void (*)()> resets;
};

Globals& get_singleton() {
  static never_destroyed<Globals> storage;
  return storage.access();
}

}  // namespace

void HwyDynamicRegisterResetFunction(void (*reset)()) {
  DRAKE_DEMAND(reset != nullptr);
  auto& singleton = get_singleton();
  std::lock_guard<std::mutex> guard(singleton.mutex);
  // Check for duplicates.
  for (const auto& existing : singleton.resets) {
    if (reset == existing) {
      return;
    }
  }
  // Not yet added; add it now.
  singleton.resets.push_back(reset);
}

void HwyDynamicReset() {
  const auto& singleton = get_singleton();
  // No mutex guard here; this function is documented as not thread-safe.
  for (const auto& reset : singleton.resets) {
    reset();
  }
}

}  // namespace internal
}  // namespace drake
