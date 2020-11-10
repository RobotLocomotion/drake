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
// Initialize the pointer to the thread-local ladder to nullptr, in an attempt
// to avoid compiler-supplied wrapper functions. See here for some discussion:
// https://stackoverflow.com/questions/13106049/what-is-the-performance-penalty-of-c11-thread-local-variables-in-gcc-4-8
__thread drake::internal::pool_ladder_allocator::Ladder*
DenseStoragePools::s_ladder_{};

thread_local DenseStoragePools::Reaper DenseStoragePools::s_reaper_;

}  // namespace Eigen
