#pragma once

#include <filesystem>

#include "drake/geometry/in_memory_mesh.h"

namespace drake {
namespace geometry {

/** Given a file path to a .gltf file, loads the .gltf file contents into
 memory. All named .bin and image files are loaded into its supporting files as
 path-valued FileSource instances (i.e., absolute paths for the external files
 are included).

Note: the path-valued supporting files are not validated with respect to their
accessibility or even their existence. */
InMemoryMesh ReadGltfToMemory(const std::filesystem::path& gltf_path);

}  // namespace geometry
}  // namespace drake
