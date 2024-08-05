#include "drake/common/file_contents.h"

#include <fstream>
#include <sstream>
#include <utility>

#include <fmt/format.h>

#include "drake/common/drake_assert.h"

namespace drake {
namespace common {

FileContents FileContents::Make(const std::filesystem::path& path) {
    return FileContents(Read(path), path.string());
}

std::string FileContents::Read(const std::filesystem::path& path) {
    std::ifstream f(path);
    if (!f.good()) {
      throw std::runtime_error(fmt::format(
          "FileContents::Make() cannot read the file '{}'.", path.string()));
    }
    std::stringstream contents;
    contents << f.rdbuf();
    return std::move(contents).str();
}

FileContents::FileContents() = default;

FileContents::FileContents(std::string contents, std::string filename_hint)
    : contents_(std::move(contents)), filename_hint_(std::move(filename_hint)) {
  sha256_ = Sha256::Checksum(contents_);
  DRAKE_DEMAND(filename_hint_.find_first_of("\n") == std::string::npos);
}

}  // namespace common
}  // namespace drake
