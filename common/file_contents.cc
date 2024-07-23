#include "drake/common/file_contents.h"

#include <utility>

#include "drake/common/drake_assert.h"

namespace drake {
namespace common {

FileContents::FileContents() = default;

FileContents::FileContents(std::string contents, std::string filename_hint)
    : contents_(std::move(contents)), filename_hint_(std::move(filename_hint)) {
  sha256_ = Sha256::Checksum(contents_);
  DRAKE_DEMAND(filename_hint_.find_first_of("\n") == std::string::npos);
}

}  // namespace common
}  // namespace drake
