#pragma once

#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/common/sha256.h"

namespace drake {
namespace common {

/** A record of a file's contents and related metadata. */
class FileContents {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FileContents);

  /** Default constructor with no contents, checksum, or filename hint. In this
  case, the `checksum` will contain all zeros. */
  FileContents();

  /** Constructs the record with the given `contents` and filename hint.
  @pre `filename_hint` contains no newlines. */
  FileContents(std::string contents, std::string filename_hint);

  /** The contents of a nominal file. */
  const std::string& contents() const { return contents_; }

  /** The checksum of `this` instance's `contents()`. */
  const Sha256& sha256() const { return sha256_; }

  /** Some notional filename for the `contents`. It need not be a valid path to
  an existing file, but applications can use this as an "identifier" for the
  contents. As such, the hint should be unique in the domain of usage. */
  const std::string& filename_hint() const { return filename_hint_; }

 private:
  std::string contents_;

  std::string filename_hint_;

  Sha256 sha256_;

  // TODO(jwnimmer-tri) In the future, we might want to track additional
  // metadata here, e.g., mime type.
};

}  // namespace common
}  // namespace drake