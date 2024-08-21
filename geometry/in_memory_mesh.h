#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/memory_file.h"

namespace drake {
namespace geometry {

/** Representation of a mesh file stored in memory. Eventually, this will also
 include a family of supporting files (e.g., material files, image files, etc.)
*/
class InMemoryMesh {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(InMemoryMesh);

  /** Constructs an empty file. */
  InMemoryMesh();

  /** Constructs a file from the given mesh file. */
  explicit InMemoryMesh(MemoryFile mesh_file);

  ~InMemoryMesh();

  /** Returns the base mesh file. */
  const MemoryFile& mesh_file() const { return mesh_file_; }

  /** Reports if the mesh is empty. */
  bool empty() const;

 private:
  /* The main mesh file's contents (e.g., a .obj or .gltf file, but not a .mtl
   or .bin). */
  MemoryFile mesh_file_;
};

}  // namespace geometry
}  // namespace drake
