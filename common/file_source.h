#pragma once

#include <filesystem>
#include <string>
#include <variant>

#include "drake/common/fmt.h"
#include "drake/common/memory_file.h"

namespace drake {

/** Represents a file. The file can be on-disk or in-memory. */
using FileSource = std::variant<std::filesystem::path, MemoryFile>;

/** Returns a string representation. */
std::string to_string(const FileSource& source);

}  // namespace drake

DRAKE_FORMATTER_AS(, drake, FileSource, x, drake::to_string(x))
