#include "drake/common/hash.h"

namespace drake {
namespace internal {

#if  __cplusplus < 201703L
constexpr size_t FNV1aHasher::kFnvPrime;
#endif

}  // namespace internal
}  // namespace drake
