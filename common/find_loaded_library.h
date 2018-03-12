#pragma once

#include <string>

#include "drake/common/drake_optional.h"

namespace drake {

/// This function returns the absolute path of the library with the name
/// `library_name` if that library was loaded in the current running
/// process. Otherwise it returns an empty optional.
optional<std::string> LoadedLibraryPath(const std::string& library_name);


}  // namespace drake
