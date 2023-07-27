#pragma once

namespace drake {
namespace pydrake {
namespace internal {

/* If possible, redirects Drake's C++ logs to Python's `logging` module. This
function is not thread-safe; no other threads should be running when it is
called. */
void MaybeRedirectPythonLogging();

/* Implements pydrake.common.use_native_cpp_logging(). This function is not
thread-safe; no other threads should be running when it is called. */
void UseNativeCppLogging();

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
