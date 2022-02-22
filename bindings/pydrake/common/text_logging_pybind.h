#pragma once

namespace drake {
namespace pydrake {
namespace internal {

/* If possible, redirects Drake's C++ logs to Python's `logging` module. */
void MaybeRedirectPythonLogging();

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
