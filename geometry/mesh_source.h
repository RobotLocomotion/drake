#pragma once

#include <filesystem>
#include <string>
#include <utility>
#include <variant>

#include "drake/common/drake_copyable.h"
#include "drake/common/reset_after_move.h"
#include "drake/geometry/in_memory_mesh.h"

namespace drake {
namespace geometry {

/** Provides a general abstraction to the definition of a mesh. A mesh
 definition can come from disk or memory. APIs that support both can take as
 specification an instance of %MeshSource to communicate that ability. */
class MeshSource final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MeshSource);

  /** Constructs from a file path.
   Note: the path will not be validated in any way (existence, availability,
   naming an actual mesh file, etc.). Validation occurs where the %MeshSource's
   path is *used*. */
  // NOLINTNEXTLINE(runtime/explicit) This conversion is desirable.
  MeshSource(std::filesystem::path path);

  /** Constructs from an in-memory mesh. */
  // NOLINTNEXTLINE(runtime/explicit) This conversion is desirable.
  MeshSource(InMemoryMesh&& mesh);

  ~MeshSource();

  /** Reports `true` if this source is a filesystem path. */
  bool is_path() const {
    return std::holds_alternative<std::filesystem::path>(source_.value());
  }

  /** Reports `true` if this source contains an in-memory mesh definition. */
  bool is_in_memory() const {
    return std::holds_alternative<InMemoryMesh>(source_.value());
  }

  /** Provides a source-agnostic description of the mesh. If is_path() is true,
   it is the path. If is_in_memory() is true, it is the filename_hint for the
   in-memory mesh file. If the in-memory mesh file has an empty filename hint,
   the description will explicitly communicate that; the empty string will
   _never_ be returned. */
  std::string description() const;

  /** Returns the extension of the mesh type -- all lower case and including
   the dot. If is_path() is `true`, the extension is extracted from the path.
   I.e., /foo/bar/mesh.obj and /foo/bar/mesh.OBJ would both report the ".obj"
   extension. The "extension" portion of the filename is defined as in
   std::filesystem::path::extension().

   If is_in_memory() is `true`, it is the extension reported by the MemoryFile.
   */
  const std::string& extension() const { return extension_.value(); }

  /** Returns the source's file path.
   @pre is_path() returns `true`. */
  const std::filesystem::path& path() const {
    return std::get<std::filesystem::path>(source_.value());
  }

  /** Returns the source's in-memory mesh data.
   @pre is_in_memory() returns `true`. */
  const InMemoryMesh& in_memory() const {
    return std::get<InMemoryMesh>(source_.value());
  }

 private:
  reset_after_move<std::variant<InMemoryMesh, std::filesystem::path>> source_;
  // We cache the extension so that we don't have recompute it each time.
  reset_after_move<std::string> extension_;
};

}  // namespace geometry
}  // namespace drake
