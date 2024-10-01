#pragma once

#include <string>

#include "drake/common/file_source.h"
#include "drake/common/fmt.h"
#include "drake/common/memory_file.h"
#include "drake/common/string_map.h"

namespace drake {
namespace geometry {

/** Representation of a mesh file stored in memory. At a minimum it includes the
 contents of a mesh file. If that mesh file makes reference to additional files
 (e.g., a .obj file can reference a .mtl which in turn references a .png file),
 then those files _can_ be included in `this` %InMemoryMesh's supporting files.
 Each supporting file is represented by a key-value pair. The key is the
 file-referencing string as it appears in the referencing file and the value is
 a FileSource. All supporting files can be located on disk; only the main mesh
 file is strictly required to be in memory. Failure to provide the supporting
 files may or may not lead to errors; it depends on the context in which the
 mesh data is used. */
struct InMemoryMesh final {
  /** The main mesh file's contents (e.g., a .obj or .gltf file, but not a .mtl
   or .bin). */
  MemoryFile mesh_file;

  /** The optional collection of supporting files. */
  string_map<FileSource> supporting_files;

  /** Returns a string representation. */
  std::string to_string() const;
};

}  // namespace geometry
}  // namespace drake

DRAKE_FORMATTER_AS(, drake::geometry, InMemoryMesh, x, x.to_string())
