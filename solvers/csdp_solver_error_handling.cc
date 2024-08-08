#include "drake/solvers/csdp_solver_error_handling.h"

namespace drake {
namespace solvers {
namespace internal {

std::jmp_buf& get_per_thread_csdp_jmp_buf() {
  static thread_local std::jmp_buf per_thread_buffer;
  return per_thread_buffer;
}

}  // namespace internal
}  // namespace solvers
}  // namespace drake

extern "C" {

// The library code in CSDP calls the C exit() function. Since no library
// should do that, our CSDP package.BUILD.bazel rule uses the preprocessor
// to re-route that call into this function instead. We put the declaration
// here (instead of a header file) because CSDP doesn't need such a header.
void drake_csdp_cpp_wrapper_exit(int);

// Define the function (separate from declaration) to avoid compiler warnings.
void drake_csdp_cpp_wrapper_exit(int) {
  std::jmp_buf& env = drake::solvers::internal::get_per_thread_csdp_jmp_buf();
  std::longjmp(env, 1);
}

}  // extern C
