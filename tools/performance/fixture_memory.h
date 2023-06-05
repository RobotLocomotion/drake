#pragma once

namespace drake {
namespace tools {
namespace performance {

/** Globally enables memory reporting (for all suites within this program).
This is not thread-safe, so should only be called on main thread.

Note that the allocation counting implementation is not thread-safe. (To reduce
the instrumentation cost, it does not use buslocked additions.) Therefore, the
allocation statistics might under-count on multi-threaded programs. */
void EnableMemoryManager();

/** Zeros out all memory stats. This useful when you don't want allocations
from SetUp amortized over the state iterations. This is not thread-safe, so
should only be called on main thread. */
void TareMemoryManager();

}  // namespace performance
}  // namespace tools
}  // namespace drake
