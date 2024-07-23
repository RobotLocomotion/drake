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
  std::string& ext = extension_;
  if (!(ext.empty() || ext.starts_with("."))) {
    throw std::runtime_error(
        fmt::format("MemoryFile given invalid extension. Must be empty or of "
                    "the form '.foo'; given '{}'.",
                    ext));
  }
  std::transform(ext.begin(), ext.end(), ext.begin(),
                 [](unsigned char c) {
                   return std::tolower(c);
                 });

  EmptySha256& wrapper = sha256_;
  const std::string& bytes = contents_;
  wrapper.value = Sha256::Checksum(bytes);
  const std::string& hint = filename_hint_;
  DRAKE_THROW_UNLESS(hint.find_first_of("\n") == std::string::npos);
}


}  // namespace drake
