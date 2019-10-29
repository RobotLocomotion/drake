#pragma once

#include <optional>
#include <string>

namespace drake {

/// This function returns the absolute path of the library with the name
/// `library_name` if that library was loaded in the current running
/// process. Otherwise it returns an empty optional.
std::optional<std::string> LoadedLibraryPath(const std::string& library_name);


}  // namespace drake
