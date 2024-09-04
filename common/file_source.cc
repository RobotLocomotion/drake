#include "drake/common/file_source.h"

#include <algorithm>
#include <utility>

#include <fmt/format.h>

namespace drake {
namespace {

std::string GetExtensionLower(const std::filesystem::path& file_path) {
  std::string ext = file_path.extension();
  std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) {
    return std::tolower(c);
  });
  return ext;
}

}  // namespace

FileSource::FileSource() = default;

FileSource::FileSource(std::filesystem::path path) : source_(std::move(path)) {
  extension_ = GetExtensionLower(this->path());
}

FileSource::FileSource(MemoryFile file) : source_(std::move(file)) {
  // Note: MemoryFile promises .extension() is of the desired format.
  extension_ = memory_file().extension();
}

bool FileSource::empty() const {
  // Note: is_path() is expressed in terms of *this* method.
  return std::holds_alternative<std::filesystem::path>(source_) &&
         path().empty();
}

std::string FileSource::description() const {
  return empty()     ? std::string()
         : is_path() ? path().string()
                     : memory_file().filename_hint();
}

void FileSource::clear() {
  source_ = std::filesystem::path();
}

}  // namespace drake
