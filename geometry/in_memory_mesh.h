#pragma once

#include <filesystem>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/file_source.h"
#include "drake/common/memory_file.h"
#include "drake/common/string_map.h"

namespace drake {
namespace geometry {

/** Representation of a mesh file stored in memory. At a minimum it should
 include the contents of a mesh file. If that mesh file makes reference to
 additional files (e.g., a .obj file can reference a .mtl which in turn
 references a .png file), then those files _can_ be included in `this`
 %InMemoryMesh's supporting files. Each supporting file is a represented by a
 key-value pair. The key is the file URI as it appears in the referencing file
 and the value is a FileSource. All supporting files can be located on disk;
 only the main mesh file is strictly required to be in memory. Failure to
 provide the supporting files may or may not lead to errors; it depends on the
 context in which the mesh data is used. */
class InMemoryMesh {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(InMemoryMesh);

  /** Constructs an empty file. */
  InMemoryMesh();

  /** Constructs a file from the given mesh file and a set of optional
   supporting files. */
  explicit InMemoryMesh(MemoryFile mesh_file,
                        string_map<FileSource> supporting_files = {});

  /** Returns the base mesh file. */
  const MemoryFile& mesh_file() const { return mesh_file_; }

  /** Adds a supporting file to the in-memory mesh.
   @param name  The name of the supporting file as it is referenced in the mesh
                file or another supporting file.
   @param file  The file source data.
   @throws if `name` is already defined. */
  void AddSupportingFile(std::string_view name, FileSource file);

  /** Returns the list of supporting file names. Note these are the names given
   in AddSupportingFile() or as keys in the constructor's map and not the
   FileSource::description(). The names will be sorted alphabetically.

   The returne list of names should *not* be persisted. Calls to
   AddSupportingFile() may invalidate the underlying strings referenced by the
   returned string views. */
  std::vector<std::string_view> SupportingFileNames() const;

  /** Reports the number of supporting files. */
  int num_supporting_files() const { return ssize(supporting_files_); }

  /** Returns a pointer to the supporting file associated with the given name.
   The pointer should *not* be persisted. Calls to AddSupportingFile() may
   invalidate older pointers.
   @returns nullptr if the name isn't present in the supporting files. */
  const FileSource* file(std::string_view name) const;

  /** Reports if the mesh is empty. Note: the mesh is empty if the _mesh file_
   is undefined, even if there are multiple supporting files present. */
  bool empty() const;

  /* (Internal only) Exposes the map of supporting files. The signature of this
   map can change arbitrarily. */
  const string_map<FileSource>& supporting_files() const {
    return supporting_files_;
  }

 private:
  /* They key for the actual mesh file data in `data`. */
  MemoryFile mesh_file_;

  /** The optional collection of supporting files. */
  string_map<FileSource> supporting_files_;
};

/** Provides a general abstraction to the definition of a mesh. A mesh
 definition can come disk or memory. APIs that support both can take as
 specification an instance of %MeshSource to communicate that ability. */
class MeshSource {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MeshSource);

  /** Constructs from a file path. */
  // NOLINTNEXTLINE(runtime/explicit) This conversion is desirable.
  MeshSource(std::filesystem::path path);

  template <typename StringLike>
  // NOLINTNEXTLINE(runtime/explicit) This conversion is desirable.
  MeshSource(StringLike path_string)
      : MeshSource(std::filesystem::path(std::move(path_string))) {}

  /** Constructs from an in-memory mesh. */
  // NOLINTNEXTLINE(runtime/explicit) This conversion is desirable.
  MeshSource(InMemoryMesh mesh);

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
  const std::string& extension() const { return extension_; }

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
  // We cache the extension so that we don't have recompute it each time.
  std::string extension_;
};

}  // namespace geometry
}  // namespace drake
