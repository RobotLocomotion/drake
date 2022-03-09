#pragma once

#include <csetjmp>

namespace drake {
namespace solvers {
namespace internal {

// Returns a reference to a thread-local setjmp buffer.
std::jmp_buf& get_per_thread_csdp_jmp_buf();

}  // namespace internal
}  // namespace solvers
}  // namespace drake
