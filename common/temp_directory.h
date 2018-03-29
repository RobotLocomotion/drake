#pragma once

#include <string>

namespace drake {

/// Returns a directory location suitable for temporary files.
/// @return In order of preference, the value of a defined environment variable
/// TEST_TMPDIR, TMPDIR, TMP, TEMP, TEMPDIR, or the path /tmp if none of them
/// are defined. Any trailing / will be stripped from the output.
/// @throws std::runtime_error If the path referred to by a defined environment
/// variable or /tmp does not exist or is not a directory.
std::string temp_directory();

}  // namespace drake
