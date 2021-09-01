#pragma once

#include <string>

namespace drake {

/// Returns a directory location suitable for temporary files.
/// The directory will be called prefix/robotlocomotion_drake_XXXXXX where each
/// X is replaced by a character from the portable filename character set. The
/// path *prefix* is defined as one of the following (in decreasing priority):
///
///    - ${TEST_TMPDIR}
///    - ${TMPDIR}
///    - /tmp
///
/// If successful, this will always create a new directory that the caller is
/// ultimately responsible for deleting.
///
/// @return The path representing an *existing* directory
///         prefix/robotlocomotion_drake_XXXXXX. There will be no trailing `/`.
/// @throws if the directory prefix/robotlocomotion_drake_XXXXXX cannot be
///         created, or is not a directory.
std::string temp_directory();

}  // namespace drake
