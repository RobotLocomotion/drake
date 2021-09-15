#pragma once

#include <string>

namespace drake {

/// Returns a directory location suitable for temporary files.
/// The directory will be called ${parent}/robotlocomotion_drake_XXXXXX where
/// each X is replaced by a character from the portable filename character set.
/// The path ${parent} is defined as one of the following (in decreasing
/// priority):
///
///    - ${TEST_TMPDIR}
///    - ${TMPDIR}
///    - /tmp
///
/// If successful, this will always create a new directory. While the caller is
/// not obliged to delete the directory, it has full power to do so based on
/// specific context and need.
///
/// @return The path representing a newly created directory There will be no
///         trailing `/`.
/// @throws if the directory ${parent}/robotlocomotion_drake_XXXXXX cannot be
///         created, or is not a directory.
std::string temp_directory();

}  // namespace drake
