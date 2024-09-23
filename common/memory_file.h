#pragma once

#include <filesystem>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/common/fmt.h"
#include "drake/common/reset_after_move.h"
#include "drake/common/sha256.h"

namespace drake {

/** A virtual file, stored in memory. */
class MemoryFile final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MemoryFile);

  /** Creates an instance of %MemoryFile from the file located at the given
   `path`. The filename_hint() will be the stringified path. Making a
   %MemoryFile computes the hash of its contents. If all you want is the
   contents, use drake::ReadFile() or drake::ReadFileOrThrow() instead.
   @throws std::exception if the file at `path` cannot be read. */
  static MemoryFile Make(const std::filesystem::path& path);

  /** Default constructor with no contents, checksum, or filename hint. In this
  case, the `checksum` will be the checksum of the empty contents. */
  MemoryFile();

  /** Constructs a new file from the given `contents`.

   @param contents        The contents of a file.
   @param extension       The extension typically associated with the file
                          contents. The case is unimportant, but it must either
                          be empty or of the form `.foo`.
   @param filename_hint   A label for the file. The label is used for warning
                          and error messages. Otherwise, the label has no
                          other functional purpose. It need not be a valid file
                          name, but must consist of a single line (no newlines).
   @warning An empty extension may be problematic. Many consumers of %MemoryFile
            key on the extension to determine if the file is suitable for a
            purpose. Always provide an accurate, representative extension when
            possible.
   @throws std::exception if `filename_hint` contains newlines.
   @throws std::exception if `extension` is not empty and the first character
                          isn't '.'. */
  MemoryFile(std::string contents, std::string extension,
             std::string filename_hint);

  ~MemoryFile();

  /** Returns the file's contents. */
  const std::string& contents() const { return contents_; }

  /** Returns the extension (as passed to the constructor). When not empty, it
   will always be reported with a leading period and all lower case characters.
   */
  const std::string& extension() const { return extension_; }

  /** Returns the checksum of `this` instance's `contents()`. */
  const Sha256& sha256() const { return sha256_.value().checksum; }

  /** Returns the notional "filename" for this file`. */
  const std::string& filename_hint() const { return filename_hint_; }

  /** Returns a string representation. Note: the file contents will be limited
   to `contents_limit` number of characters. To include the full contents, pass
   any number less than or equal to zero. */
  std::string to_string(int contents_limit = 100) const;

 private:
  reset_after_move<std::string> contents_;

  reset_after_move<std::string> extension_;

  reset_after_move<std::string> filename_hint_;

  /* A tiny wrapper to change the default-constructed value from all zeros, to
   the hash of an empty string. */
  struct EmptySha256 {
    Sha256 checksum = Sha256::Checksum("");
  };

  reset_after_move<EmptySha256> sha256_;
};

}  // namespace drake

DRAKE_FORMATTER_AS(, drake, MemoryFile, x, x.to_string())
