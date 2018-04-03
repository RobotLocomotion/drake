#pragma once

#include <string>

namespace drake {

/// Returns a directory location suitable for temporary files.
/// @return The value of the environment variable TEST_TMPDIR if defined or
/// otherwise ${TMPDIR:-/tmp}/robotlocomotion_drake_XXXXXX where each X is
/// replaced by a character from the portable filename character set. Any
/// trailing / will be stripped from the output.
/// @throws std::runtime_error If the path referred to by TEST_TMPDIR or
/// ${TMPDIR:-/tmp}/robotlocomotion_drake_XXXXXX cannot be created, does not
/// exist, or is not a directory.
std::string temp_directory();

}  // namespace drake
