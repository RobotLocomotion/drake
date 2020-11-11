#include "drake/common/pool_ladder_allocator.h"

namespace drake {
namespace internal {
namespace pool_ladder_allocator {

std::atomic_int Ladder::s_instance_count_{0};

}  // namespace pool_ladder_allocator
}  // namespace internal
}  // namespace drake
