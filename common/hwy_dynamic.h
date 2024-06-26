#pragma once

namespace drake {
namespace internal {

/* This file provides two singleton-like functions whose purpose is to assist
 with unit test that want to probe that that uses "hwy/highway.h" for SIMD. */

/* Anywhere in Drake that uses Highway for dynamic CPU dispatch should call this
helper function to register a reset handler to clear the detction. (This happens
automatically when you use `hwy_dynamic_impl.h`.) This function is safe to call
from multiple threads currently. */
void HwyDynamicRegisterResetFunction(void (*)());

/* (For testing only) Clears the latched CPU target detection by calling all of
the reset handlers registered using the HwyDynamicRegisterResetFunction() above.
This allows for testing multiple different CPU targets from within the same test
program. This function is NOT thread-safe; only use this in single-threaded
tests. */
void HwyDynamicReset();

}  // namespace internal
}  // namespace drake
