#pragma once

namespace drake {
namespace tools {
namespace performance {

/** Globally enables memory reporting (for all suites within this program). */
void EnableMemoryManager();

/** Zeros out all memory stats. This useful when you don't want allocations
from SetUp amortized over the state iterations. */
void TareMemoryManager();

}  // namespace performance
}  // namespace tools
}  // namespace drake
