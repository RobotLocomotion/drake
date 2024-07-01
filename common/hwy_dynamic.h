#pragma once

namespace drake {
namespace internal {

/* This file provides two singleton-like functions whose purpose is to assist
with unit tests that want to probe code that uses "hwy/highway.h" for SIMD. */

/* Anywhere in Drake that uses Highway for dynamic CPU dispatch should call this
helper function to register a reset handler to clear the latched CPU detection.
(Most developers never need to worry about this, because it's automatic when you
use the tools in `hwy_dynamic_impl.h`.) This function is safe to call from
multiple threads currently. */
void HwyDynamicRegisterResetFunction(void (*)());

/* (For testing only) Clears the latched CPU target detection by calling all of
the reset handlers registered using the HwyDynamicRegisterResetFunction() above.
This allows for testing multiple different CPU targets from within the same test
program. This function is NOT thread-safe; only use this in single-threaded
tests. */
void HwyDynamicReset();

}  // namespace internal
}  // namespace drake
