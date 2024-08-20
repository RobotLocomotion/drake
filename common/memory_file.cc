#include "drake/common/memory_file.h"

#include <optional>
#include <utility>

#include <fmt/format.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"

namespace drake {

MemoryFile MemoryFile::Make(const std::filesystem::path& path) {
  std::optional<std::string> contents = ReadFile(path);
  if (!contents.has_value()) {
    throw std::runtime_error(fmt::format(
        "MemoryFile::Make() cannot read the file '{}'.", path.string()));
  }
  return MemoryFile(std::move(*contents), path.string());
}

MemoryFile::MemoryFile() = default;

MemoryFile::MemoryFile(std::string contents, std::string filename_hint)
    : contents_(std::move(contents)), filename_hint_(std::move(filename_hint)) {
  sha256_ = Sha256::Checksum(contents_);
  DRAKE_DEMAND(filename_hint_.find_first_of("\n") == std::string::npos);
}

}  // namespace drake
