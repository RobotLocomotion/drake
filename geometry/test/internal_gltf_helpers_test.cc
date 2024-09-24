#include "drake/geometry/internal_gltf_helpers.h"

#include <filesystem>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/temp_directory.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

namespace fs = std::filesystem;

GTEST_TEST(PreParseGltfTest, MultiFileGltf) {
  const fs::path gltf_path = FindResourceOrThrow(
      "drake/geometry/render/test/meshes/fully_textured_pyramid.gltf");

  // By default, we should load all external files (images and .bin files).
  const InMemoryMesh mesh = PreParseGltf(gltf_path);
  EXPECT_EQ(mesh.supporting_files.size(), 9);
  for (const auto& file :
       {"fully_textured_pyramid_emissive.png",
        "fully_textured_pyramid_normal.png", "fully_textured_pyramid_omr.png",
        "fully_textured_pyramid_base_color.png",
        "fully_textured_pyramid_emissive.ktx2",
        "fully_textured_pyramid_normal.ktx2", "fully_textured_pyramid_omr.ktx2",
        "fully_textured_pyramid_base_color.ktx2",
        "fully_textured_pyramid.bin"}) {
    ASSERT_TRUE(mesh.supporting_files.contains(file));
    // It's a path-valued FileSource.
    const FileSource& source = mesh.supporting_files.at(std::string(file));
    const auto* path = std::get_if<fs::path>(&source);
    ASSERT_NE(path, nullptr);
    EXPECT_EQ(*path, gltf_path.parent_path() / file);
  }
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
