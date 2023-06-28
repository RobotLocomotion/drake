#pragma once

namespace drake {
namespace pydrake {
namespace internal {

/*
If possible, redirects Drake's C++ logs to Python's `logging` module.
Returns true if successfully redirected, false otherwise.
*/
bool MaybeRedirectPythonLogging();

/*
If possible, reverts the redirect done by `MaybeRedirectPythonLogging()`.
Returns true if successfully reverted, false otherwise.
*/
bool MaybeUndoRedirectPythonLogging();

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
