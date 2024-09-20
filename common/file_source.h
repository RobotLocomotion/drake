#pragma once

#include <filesystem>
#include <variant>

#include "drake/common/memory_file.h"

namespace drake {

/** Represents a file. The file can be on-disk or in-memory. */
using FileSource = std::variant<std::filesystem::path, MemoryFile>;

/** Returns a string representation of this shape. */
std::string to_string(const FileSource& source);

}  // namespace drake
