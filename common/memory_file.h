#pragma once

#include <filesystem>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/common/sha256.h"

namespace drake {

/** The idea of a data file stored in memory. */
class MemoryFile {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MemoryFile);

  /** Creates an instance of %MemoryFile from the file located at the given
   `path`. The filename_hint() will be the stringified path. This includes the
   calculation of the content's hash. To get the contents without computing the
   hash, use drake::ReadFile() or drake::ReadFileOrThrow().
   @throws std::exception if the file at `path` cannot be read. */
  static MemoryFile Make(const std::filesystem::path& path);

  /** Default constructor with no contents, checksum, or filename hint. In this
  case, the `checksum` will be the checksum of the empty contents. */
  MemoryFile();

  /** Constructs the a new file from the given `contents`.

   @param contents        The ascii contents of a file.
   @param extension       The extension typically associated with the file
                          contents. The case is unimportant, but it must either
                          be empty or of the form `.foo`.
   @param filename_hint   A label for the file. The label is used for warning
                          and error messages. Otherwise, the label has no
                          other functional purpose.
   @warning An empty extension may be problematic. Many consumes of %MemoryFile
            key on the extension to determine if the file is suitable for a
            purpose. Always provide an accurate, representative extension when
            possible.
   @pre `filename_hint` contains no newlines. */
  MemoryFile(std::string contents, std::string extension,
             std::string filename_hint);

  /** The contents of a nominal file. */
  const std::string& contents() const { return contents_; }

  /** The extension passed to the constructor. When not empty, it will always be
   reported with a leading period and all lower case characters. */
  const std::string& extension() const { return extension_; }

  /** The checksum of `this` instance's `contents()`. */
  const Sha256& sha256() const { return hash_.sha256(); }

  /** Some notional filename for the `contents`. It need not be a valid path to
   an existing file, but applications can use this as an "identifier" for the
   contents. As such, the hint should be unique in the domain of usage. */
  const std::string& filename_hint() const { return filename_hint_; }

 private:
  std::string contents_;

  std::string extension_;

  std::string filename_hint_;

  /* `sha256() is documented as the hash of contents(). Sha256 doesn't natively
   support this.
     - Its default-constructed value is not the hash of empty contents,
     - Its move semantics leave the value unchanged.
   This wrapper serves a role akin to `reset_after_move`, except it defaults
   to a value *other* than the default constructed value for Sha256. */
  class ResetSha256 {
   public:
    ResetSha256() { reset(); }
    ResetSha256(const ResetSha256&) = default;
    ResetSha256& operator=(const ResetSha256&) = default;
    ResetSha256(ResetSha256&& other);
    ResetSha256& operator=(ResetSha256&& other);

    void set(const Sha256& hash) { hash_ = hash; }
    const Sha256& sha256() const { return hash_; }

   private:
    void reset() { hash_ = kEmptyHash; }

    Sha256 hash_;
    static const Sha256 kEmptyHash;
  };

  ResetSha256 hash_;
};

}  // namespace drake
