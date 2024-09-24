#pragma once

#include <filesystem>

#include "drake/geometry/in_memory_mesh.h"

namespace drake {
namespace geometry {

/** Given a file path to a .gltf file, loads the .gltf file contents into memory
along with any supporting .bin and image files.

Note: the supporting files will all be path-variants. The supporting files are
not explicitly loaded into memory and their existence/readability is *not*
validated. */
InMemoryMesh ReadGltfToMemory(const std::filesystem::path& gltf_path);

}  // namespace geometry
}  // namespace drake
