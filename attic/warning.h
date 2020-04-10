#pragma once

namespace drake {
namespace internal {

/** Prints a deprecation warning the first time it is called.  Subsequent calls
print no warning. */
void WarnOnceAboutAtticCode();

}  // namespace internal
}  // namespace drake
