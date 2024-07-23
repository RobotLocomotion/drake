#include "drake/common/memory_file.h"

#include <algorithm>
#include <optional>
#include <utility>

#include <fmt/format.h>

#include "drake/common/drake_throw.h"
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
  if (!(extension_.value().empty() || extension_.value().starts_with("."))) {
    throw std::runtime_error(
        fmt::format("MemoryFile given invalid extension. Must be empty or of "
                    "the form '.foo'; given '{}'.",
                    extension_.value()));
  }
  std::transform(extension_.value().begin(), extension_.value().end(),
                 extension_.value().begin(), [](unsigned char c) {
                   return std::tolower(c);
                 });

  sha256_.value().checksum = Sha256::Checksum(contents_.value());
  DRAKE_THROW_UNLESS(filename_hint_.value().find_first_of("\n") ==
                     std::string::npos);
}

}  // namespace drake
