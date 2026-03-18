#pragma once

#include <string>

#include "drake/geometry/mesh_source.h"

namespace drake {
namespace geometry {
namespace render {
namespace internal {

/* Returns a unique cache key for the given mesh source.

The key uniquely identifies the mesh data. For on-disk meshes, it uses the
canonicalized file path. For in-memory meshes, it uses the SHA256 hash of
the mesh file contents.

@param mesh_source   The mesh source to generate a key for.
@param is_convex     Whether the mesh is being used as a convex shape. This
                     affects the key because convex shapes undergo additional
                     processing (convex hull computation).
@returns A string suitable for use as a cache key.
@throws std::runtime_error if the mesh source path cannot be canonicalized. */
std::string GetMeshSourceCacheKey(const MeshSource& mesh_source,
                                  bool is_convex);

}  // namespace internal
}  // namespace render
}  // namespace geometry
}  // namespace drake
