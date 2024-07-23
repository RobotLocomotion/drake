#include "drake/common/memory_file.h"

#include <algorithm>
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

  return MemoryFile(std::move(*contents), path.extension(), path.string());
}

MemoryFile::MemoryFile() = default;

MemoryFile::MemoryFile(std::string contents, std::string extension,
                       std::string filename_hint)
    : contents_(std::move(contents)),
      extension_(std::move(extension)),
      filename_hint_(std::move(filename_hint)) {
  std::transform(extension_.begin(), extension_.end(), extension_.begin(),
                 [](unsigned char c) {
                   return std::tolower(c);
                 });
  if (!(extension_.empty() || extension_.starts_with("."))) {
    throw std::runtime_error(
        fmt::format("MemoryFile given invalid extension. Must be of the form "
                    "'.foo'; given '{}'.",
                    extension_));
  }

  sha256_ = Sha256::Checksum(contents_);
  DRAKE_DEMAND(filename_hint_.find_first_of("\n") == std::string::npos);
}

}  // namespace drake
