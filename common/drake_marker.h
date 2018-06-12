#pragma once

/// @file
/// This is an internal (not installed) header. Do not use this outside of
/// `find_resource.cc`.

namespace drake {
namespace internal {

// Provides a concrete object to ensure that this marker library is linked.
// This returns a simple magic constant to ensure the library was loaded
// correctly.
int drake_marker_lib_check();

}  // namespace internal
}  // namespace drake
