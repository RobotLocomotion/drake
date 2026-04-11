#pragma once

#include <filesystem>
#include <string>
#include <variant>

#include "drake/common/drake_deprecated.h"
#include "drake/common/memory_file.h"

namespace drake {

/** Represents a file. The file can be on-disk or in-memory. */
using FileSource = std::variant<std::filesystem::path, MemoryFile>;

DRAKE_DEPRECATED("2026-07-01",
                 "Use fmt::to_string instead, with `#include <fmt/std.h>`.")
std::string to_string(const FileSource& source);

}  // namespace drake
