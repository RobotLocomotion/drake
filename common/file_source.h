#pragma once

#include <filesystem>
#include <string>
#include <utility>
#include <variant>

#include "drake/common/drake_copyable.h"
#include "drake/common/memory_file.h"

namespace drake {

/** Represents a file. The file can be on-disk or in-memory. This is essentially
 a sugared-up variant of a file system path and a MemoryFile, offering more
 succinct methods for determining what the variant holds and to get the value.
 */
class FileSource {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FileSource);

  /** Constructs an empty file source. Note: an empty file source will report
   as neither a path nor a memory file. */
  FileSource();

  /** Constructs from a file path. */
  // NOLINTNEXTLINE(runtime/explicit) This conversion is desirable.
  FileSource(std::filesystem::path path);

  template <typename StringLike>
  // NOLINTNEXTLINE(runtime/explicit) This conversion is desirable.
  FileSource(StringLike path_string)
      : FileSource(std::filesystem::path(std::move(path_string))) {}

  /** Constructs from an in-memory file. */
  // NOLINTNEXTLINE(runtime/explicit) This conversion is desirable.
  FileSource(MemoryFile file);

  /** Reports `true` if this source is a filesystem path. */
  bool is_path() const {
    return !empty() && std::holds_alternative<std::filesystem::path>(source_);
  }

  /** Reports `true` if this source contains an in-memory mesh definition. */
  bool is_in_memory() const {
    return !empty() && std::holds_alternative<MemoryFile>(source_);
  }

  /** Provides a source-agnostic description of the mesh. If is_path() is true,
   it is the path. If is_in_memory() is true, it is the filename_hint for the
   in-memory file. */
  std::string description() const;

  /** Returns the extension of the file type -- all lower case and including
   the dot. If `this` is constructed from a file path, the extension is
   extracted from the path. I.e., /foo/bar/some.txt and /foo/bar/some.TXT would
   both report the ".txt" extension. The "extension" portion of the filename is
   defined as in std::filesystem::path::extension().

   If `this` is constructed from an in-memory file, it is the extension defined
   by the MemoryFile. */
  const std::string& extension() const { return extension_; }

  /** Returns the source's file path.
   @pre is_path() return s`true`. */
  const std::filesystem::path& path() const {
    return std::get<std::filesystem::path>(source_);
  }

  /** Returns the source's in-memory file.
   @pre is_in_memory() return s`true`. */
  const MemoryFile& memory_file() const {
    return std::get<MemoryFile>(source_);
  }

  /** Empties the file source so that is_empty() will return `true`. */
  void clear();

  /** Reports `true` if the source contains no file information. */
  bool empty() const;

 private:
  std::variant<std::filesystem::path, MemoryFile> source_;

  // We cache the extension so that we don't have recompute it each time.
  std::string extension_;
};

}  // namespace drake
