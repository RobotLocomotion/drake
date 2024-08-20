#pragma once

#include <filesystem>
#include <string>
#include <utility>
#include <variant>

#include "drake/common/drake_copyable.h"
#include "drake/common/file_contents.h"
#include "drake/common/string_map.h"

namespace drake {
namespace geometry {

/** Representation of a mesh file stored in memory. At a minimum it should
 include the contents of a mesh file. If that mesh file makes reference to
 additional files (e.g., a .obj file can reference a .mtl which in turn
 references a .png file), then those files _can_ be included in supporting
 files. Each supporting file is a represented by a key-value pair. The key
 is the URI as it appears in the referencing file and the value is the file's
 contents. Failure to provide the supporting files may or may not lead to
 errors; it depends on the context in which the mesh data is used. */
struct InMemoryMesh {
  // TODO(SeanCurtis-TRI): FileContents must include extension/mime type so
  // knowing how to interpret the bits is self-contained.
  /** They key for the actual mesh file data in `data`. */
  common::FileContents mesh_file;

  /** The optional collection of supporting files. */
  string_map<common::FileContents> supporting_files;
};

/** Provides an general abstraction to the definition of a mesh. A mesh
 definition can come disk or memory. APIs that support both can take as
 specification, an instance of %MeshSource to communicate that ability. */
class MeshSource {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MeshSource);

  /** Constructs from a file path. This explicitly allows for implicit
   conversion from std::filesystem::path. */
  // NOLINTNEXTLINE(runtime/explicit) This conversion is desirable.
  MeshSource(std::filesystem::path path);

  template <typename StringLike>
  // NOLINTNEXTLINE(runtime/explicit) This conversion is desirable.
  MeshSource(StringLike path_string)
      : MeshSource(std::filesystem::path(std::move(path_string))) {}

  // TODO(SeanCurtis-TRI): When FileContents tracks its own extension, remove
  // the additional parameter here and allow implicit conversion from
  // InMemoryMesh.
  /** Constructs from an in-memory mesh. */
  MeshSource(InMemoryMesh mesh, std::string extension);

  /** Reports `true` if this source is a filesystem path. */
  bool IsPath() const {
    return std::holds_alternative<std::filesystem::path>(source_);
  }

  /** Reports `true` if this source contains an in-memory mesh definition. */
  bool IsInMemory() const {
    return std::holds_alternative<InMemoryMesh>(source_);
  }

  /** Provides a source-agnostic description of the mesh. If IsPath() is true,
   it is the path. If IsInMemory() is true, it is the filename_hint for the
   in-memory mesh file. */
  std::string description() const;

  /** Returns the extension of the mesh type -- all lower case and including
   the dot. If `this` is constructed from a file path, the extension is
   extracted from the path. I.e., /foo/bar/mesh.obj and /foo/bar/mesh.OBJ would
   both report the ".obj" extension. The "extension" portion of the filename is
   defined as in std::filesystem::path::extension().

   If `this` is constructed using in-memory file contents, it is the extension
   passed to the constructor. */
  const std::string& extension() const { return extension_;  }

  /** Returns the source's file path.
   @pre IsPath() return s`true`. */
  const std::filesystem::path& path() const {
    return std::get<std::filesystem::path>(source_);
  }

  /** Returns the source's in-memory mesh data.
   @pre IsInMemory() return s`true`. */
  const InMemoryMesh& mesh_data() const {
    return std::get<InMemoryMesh>(source_);
  }

 private:
  std::variant<std::filesystem::path, InMemoryMesh> source_;
  std::string extension_;
};

}  // namespace geometry
}  // namespace drake
