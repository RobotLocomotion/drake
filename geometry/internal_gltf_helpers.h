#pragma once

#include <filesystem>

#include "drake/geometry/in_memory_mesh.h"

namespace drake {
namespace geometry {
namespace internal {

/* Given a file path to a .gltf file, loads the .gltf file contents into memory
along with any supporting .bin and .image files. Note: in applications where
only the geometry matters, images can be excluded by setting `include_images`
to false. */
InMemoryMesh PreParseGltf(const std::filesystem::path gltf_path,
                          bool include_images = true);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
