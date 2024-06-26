#pragma once

// XXX overview

namespace drake {
namespace internal {

/* */
void HwyDynamicRegisterResetFunction(void (*)());

/* (For testing only) Clears the latched target detection. This allows for
testing multiple different targets from the same test program. This function
is NOT thread-safe; only use this in single-threaded tests. */
void HwyDynamicReset();

}  // namespace internal
}  // namespace drake
