#pragma once

namespace drake {
namespace pydrake {
namespace internal {

/* If possible, redirects Drake's C++ logs to Python's `logging` module. */
bool MaybeRedirectPythonLogging();

bool MaybeUndoRedirectPythonLogging();

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
