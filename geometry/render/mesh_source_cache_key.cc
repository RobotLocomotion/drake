#include "drake/geometry/render/mesh_source_cache_key.h"

#include <filesystem>

#include "drake/common/fmt.h"

namespace fs = std::filesystem;

namespace drake {
namespace geometry {
namespace render {
namespace internal {

std::string GetMeshSourceCacheKey(const MeshSource& mesh_source,
                                  bool is_convex) {
  std::string prefix;
  // TODO(SeanCurtis-TRI): This uses the sha of the core mesh file to identify
  // a unique mesh. However, there is a weird, adversarial case:
  //
  //  - User loads a geometry from foo.mesh that references foo.bin.
  //    - the data for foo.mesh and foo.bin gets processed and loaded into
  //      the cache.
  //  - The hash of foo.mesh serves as its unique key.
  //  - User then changes contents of foo.bin.
  //  - User loads another geometry with foo.mesh (which has now appreciably
  //    changed because foo.bin is different).
  //  - The cache will believe it is the same file as before, even though the
  //    geometry has changed and not load the new version, using the old
  //    instead.
  //
  // For example, if a user runs a simulation, rendering images out, resets
  // the simulation with changes to the foo.bin file (e.g. material properties
  // in a .mtl file), with the intent of creating a visual variant of the
  // previous simulation, the geometry in the second pass will not have
  // changed appearance.
  //
  // This problem applies to both on-disk and in-memory meshes. For on-disk
  // meshes, the problem is worse because it strictly uses the mesh file name
  // as cache id and therefore won't even detect changes in the core mesh file
  // (let alone any of the supporting files).
  //
  // To address this we'll have to have a key predicated on the contents on
  // the whole file *ecosystem* so that we can detect if any part of the file
  // family is unique, creating, in some sense, a unique geometry.
  if (mesh_source.is_in_memory()) {
    prefix = mesh_source.in_memory().mesh_file.sha256().to_string();
  } else {
    DRAKE_DEMAND(mesh_source.is_path());
    std::error_code path_error;
    const fs::path canonical_path =
        fs::canonical(mesh_source.path(), path_error);
    if (path_error) {
      throw std::runtime_error(
          fmt::format("Unable to access mesh file '{}': {}",
                      mesh_source.path().string(), path_error.message()));
    }
    prefix = canonical_path.string();
  }
  // Note: We're using "?" as a separator. It isn't valid for filenames,
  // so using it guarantees we won't collide with potential file names.
  return prefix + (is_convex ? "?convex" : "");
}

}  // namespace internal
}  // namespace render
}  // namespace geometry
}  // namespace drake
