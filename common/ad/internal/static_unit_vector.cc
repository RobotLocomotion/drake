#include "drake/common/ad/internal/static_unit_vector.h"

#include <array>

#include "drake/common/drake_assert.h"

namespace drake {
namespace ad {
namespace internal {
namespace {

// Returns the base pointer to a global data array that looks like this:
//
//   [0 ... 0 1 0 ... 0]
//            ^ There's a 1.0 in the middle at kMaxVectorSize.
//
// By offsetting into this array, we can provide a unit vector or zero vector.
const double* GetGlobalData() {
  struct StorageInitializer {
    StorageInitializer() {
      // Zero out all of the memory; set the single unit value.
      storage = {};
      storage[kMaxStaticVectorSize] = 1.0;
    }
    std::array<double, 2 * kMaxStaticVectorSize> storage;
  };
  // This is global is initialized upon first use and never destroyed.
  static const auto* const global = new StorageInitializer;
  return global->storage.data();
}

}  // namespace

const double* GetStaticUnitVector(int offset) {
  DRAKE_DEMAND(offset >= 0);
  DRAKE_DEMAND(offset < kMaxStaticVectorSize);
  return GetGlobalData() + kMaxStaticVectorSize - offset;
}

}  // namespace internal
}  // namespace ad
}  // namespace drake
