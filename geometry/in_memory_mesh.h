#pragma once

#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/file_source.h"
#include "drake/common/memory_file.h"
#include "drake/common/string_map.h"

namespace drake {
namespace geometry {

/** Representation of a mesh file stored in memory. At a minimum it includes the
 contents of a mesh file. If that mesh file makes reference to additional files
 (e.g., a .obj file can reference a .mtl which in turn references a .png file),
 then those files _can_ be included in `this` %InMemoryMesh's supporting files.
 Each supporting file is represented by a key-value pair. The key is the file
 URI as it appears in the referencing file and the value is a FileSource. All
 supporting files can be located on disk; only the main mesh file is strictly
 required to be in memory. Failure to provide the supporting files may or may
 not lead to errors; it depends on the context in which the mesh data is used.
 */
class InMemoryMesh final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(InMemoryMesh);

  /** Constructs an empty file. */
  InMemoryMesh();

  /** Constructs a file from the given mesh file and a set of optional
   supporting files. */
  explicit InMemoryMesh(MemoryFile mesh_file,
                        string_map<FileSource> supporting_files = {});

  ~InMemoryMesh();

  /** Returns the base mesh file. */
  const MemoryFile& mesh_file() const { return mesh_file_; }

  /** Adds a supporting file to the in-memory mesh.
   @param name  The name of the supporting file as it is referenced in the mesh
                file or another supporting file.
   @param file  The file source data.
   @throws if `name` is already defined. */
  void AddSupportingFile(std::string_view name, FileSource file_source);

  /** Returns the list of supporting file names. Note these are the names given
   in AddSupportingFile() or as keys in the constructor's map and not the
   FileSource::description(). The names will be in a consistent order,
   regardless of the order they are added to `this`.

   The returned list of names should *not* be persisted. Calls to
   AddSupportingFile() may invalidate the underlying strings referenced by the
   returned string views. */
  std::vector<std::string_view> SupportingFileNames() const;

  /** Reports the number of supporting files. */
  int num_supporting_files() const { return ssize(supporting_files_); }

  /** Returns a pointer to the supporting file associated with the given name.
   The pointer should *not* be persisted. Calls to AddSupportingFile() may
   invalidate older pointers.
   @returns nullptr if the name isn't present in the supporting files. */
  const FileSource* supporting_file(std::string_view name) const;

  /** Reports if the mesh is empty. Note: the mesh is empty if the _mesh file_
   is undefined, even if there are supporting files present. */
  bool empty() const;

 private:
  /* The main mesh file's contents (e.g., a .obj or .gltf file, but not a .mtl
   or .bin). */
  MemoryFile mesh_file_;

  /** The optional collection of supporting files. */
  string_map<FileSource> supporting_files_;
};

}  // namespace geometry
}  // namespace drake
