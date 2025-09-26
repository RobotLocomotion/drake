#include "drake/geometry/read_gltf_to_memory.h"

#include <filesystem>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace geometry {
namespace {

namespace fs = std::filesystem;

GTEST_TEST(PreParseGltfTest, WithImages) {
  const fs::path gltf_path = FindResourceOrThrow(
      "drake/geometry/render/test/meshes/fully_textured_pyramid.gltf");

  const InMemoryMesh mesh = ReadGltfToMemory(gltf_path);
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

// This glTF has no images (and no "images" array). Confirm it still works.
GTEST_TEST(PreParseGltfTest, NoImageArray) {
  const fs::path gltf_path =
      FindResourceOrThrow("drake/geometry/test/cube_with_hole.gltf");

  const InMemoryMesh mesh = ReadGltfToMemory(gltf_path);

  EXPECT_EQ(mesh.supporting_files.size(), 1);
  ASSERT_TRUE(mesh.supporting_files.contains("cube_with_hole.bin"));
}

GTEST_TEST(PreParseGltfTest, BadGltf) {
  const fs::path dir = temp_directory();
  const fs::path gltf_path = dir / "invalid.gltf";
  {
    std::ofstream file(gltf_path);
    DRAKE_DEMAND(file.is_open());
    file << "not valid json\n";
  }

  DRAKE_EXPECT_THROWS_MESSAGE(ReadGltfToMemory(gltf_path),
                              ".*Error parsing the glTF file.*");
}

}  // namespace
}  // namespace geometry
}  // namespace drake
