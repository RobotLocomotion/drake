#pragma once

#include "drake/common/file_contents.h"
#include "drake/common/string_map.h"

namespace drake {
namespace geometry {

/** Representation of a mesh file stored in memory. At a minimum it should
 include the contents of a mesh file. If that mesh file makes reference to
 additional files (e.g., a .obj file can reference a .mtl which in turn
 references a .png file), then those files must be included in supporting
 files. Each supporting file is a represented by a key-value pair. The key
 is the URI as it appears in the referencing file and the value is the file's
 contents. */
struct InMemoryMesh {
  // TODO(SeanCurtis-TRI): FileContents must include mime-type so knowing how
  // to interpret the bits is self-contained.
  /** They key for the actual mesh file data in `data`. */
  common::FileContents mesh_file;

  /** The optional collection of supporting files. */
  string_map<common::FileContents> supporting_files;
};

}  // namespace geometry
}  // namespace drake
