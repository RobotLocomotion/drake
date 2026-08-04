#include "drake/geometry/meshcat_internal.h"

#include <filesystem>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "drake/common/find_resource.h"
#include "drake/common/sha256.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/meshcat_file_storage_internal.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

namespace fs = std::filesystem;

using math::RigidTransformd;
using nlohmann::json;

GTEST_TEST(MeshcatInternalTest, GetMeshcatStaticResource) {
  // This matches the list of URLs in the API doc.
  const std::vector<const char*> urls{
      "/favicon.ico",
      "/meshcat.html",
      "/meshcat.js",
  };
  for (const auto& url : urls) {
    SCOPED_TRACE(fmt::format("url = {}", url));
    const std::optional<std::string_view> result =
        GetMeshcatStaticResource(url);
    ASSERT_TRUE(result);
    EXPECT_FALSE(result->empty());
  }
}

GTEST_TEST(MeshcatInternalTest, UuidGenerator) {
  UuidGenerator dut;
  std::string foo = dut.GenerateRandom();
  std::string bar = dut.GenerateRandom();
  EXPECT_NE(foo, bar);

  const std::string_view pattern =
      "[[:xdigit:]]{8,8}-"
      "[[:xdigit:]]{4,4}-"
      "[[:xdigit:]]{4,4}-"
      "[[:xdigit:]]{4,4}-"
      "[[:xdigit:]]{12,12}";
  EXPECT_THAT(foo, testing::MatchesRegex(pattern));
  EXPECT_THAT(bar, testing::MatchesRegex(pattern));
}

GTEST_TEST(UnbundleGltfAssetsTest, DataUri) {
  // Load a glTF that has exactly one *bundled* asset.
  const fs::path gltf_filename =
      FindResourceOrThrow("drake/geometry/render/test/meshes/cube1.gltf");
  std::string gltf_contents = ReadFileOrThrow(gltf_filename);
  const size_t orig_size = gltf_contents.size();

  // Unbundle it.
  FileStorage storage;
  std::vector<std::shared_ptr<const MemoryFile>> assets =
      UnbundleGltfAssets(gltf_filename, &gltf_contents, &storage);

  // The contents got smaller due to unbundling (but not too small).
  DRAKE_DEMAND(orig_size >= 4096);
  EXPECT_LT(gltf_contents.size(), 2048);
  EXPECT_GT(gltf_contents.size(), 1024);

  // One asset was added to storage, identical to cube2.bin.
  const Sha256 expected_sha256 = Sha256::Checksum(ReadFileOrThrow(
      FindResourceOrThrow("drake/geometry/render/test/meshes/cube2.bin")));
  ASSERT_EQ(assets.size(), 1);
  EXPECT_EQ(assets.front()->sha256(), expected_sha256);

  // Make sure the new URI seems correct.
  EXPECT_THAT(std::string{json::parse(gltf_contents)["buffers"][0]["uri"]},
              testing::EndsWith(expected_sha256.to_string()));
}

GTEST_TEST(UnbundleGltfAssetsTest, InMemoryData) {
  const fs::path gltf_path = FindResourceOrThrow(
      "drake/geometry/render/test/meshes/fully_textured_pyramid.gltf");
  const fs::path gltf_dir = gltf_path.parent_path();
  // We'll read the .bin file from in-memory.
  string_map<FileSource> supporting_files{
      {"fully_textured_pyramid.bin",
       MemoryFile::Make(gltf_dir / "fully_textured_pyramid.bin")}};
  // We'll read the images from disk. UnbundleGltfAssets doesn't vary its logic
  // based on which array the URI comes from, so this is enough to test both
  // cases: in-memory and on-disk.
  for (const auto& f_name :
       {"fully_textured_pyramid_emissive.png",
        "fully_textured_pyramid_normal.png", "fully_textured_pyramid_omr.png",
        "fully_textured_pyramid_base_color.png",
        "fully_textured_pyramid_emissive.ktx2",
        "fully_textured_pyramid_normal.ktx2", "fully_textured_pyramid_omr.ktx2",
        "fully_textured_pyramid_base_color.ktx2"}) {
    supporting_files.insert({f_name, gltf_dir / f_name});
  }
  const MeshSource source(
      InMemoryMesh{MemoryFile::Make(gltf_path), std::move(supporting_files)});
  DRAKE_DEMAND(source.is_in_memory());
  // Unbundle it.
  FileStorage storage;
  std::string gltf_contents = source.in_memory().mesh_file.contents();

  std::vector<std::shared_ptr<const MemoryFile>> assets =
      UnbundleGltfAssets(source, &gltf_contents, &storage);
  // One asset for each supporting file (some in-memory, some on-disk).
  EXPECT_EQ(assets.size(), source.in_memory().supporting_files.size());

  const json gltf_json = json::parse(gltf_contents);

  // The in-memory file URI was updated in the gltf and placed into storage.
  const Sha256 bin_sha =
      std::get<MemoryFile>(
          source.in_memory().supporting_files.at("fully_textured_pyramid.bin"))
          .sha256();
  // File storage provides a version-based prefix to the sha.
  EXPECT_THAT(std::string{gltf_json["buffers"][0]["uri"]},
              testing::EndsWith(bin_sha.to_string()));
  EXPECT_NE(storage.Find(bin_sha), nullptr);

  // An on-disk file URI was likewise updated. We'll test one, and assume that
  // they all got handled.
  const Sha256 png_sha =
      MemoryFile::Make(gltf_dir / "fully_textured_pyramid_emissive.png")
          .sha256();
  // We happen to know that the emissive texture is texture 0.
  EXPECT_THAT(std::string{gltf_json["images"][0]["uri"]},
              testing::EndsWith(png_sha.to_string()));
  EXPECT_NE(storage.Find(png_sha), nullptr);
}

std::string MakeGltfWithUri(std::string_view uri) {
  return fmt::format(R"""(
{{
    "buffers": [
        {{
            "uri": "{}"
        }}
    ]
}}
)""",
                     uri);
}

GTEST_TEST(UnbundleGltfAssetsTest, DataUriBad) {
  // This URI is not base64 encoded; glTF data URIs must always be base64.
  std::string uri = "data:application/octet-stream,Hello!";
  std::string gltf_contents = MakeGltfWithUri(uri);
  const std::string orig = gltf_contents;
  FileStorage storage;
  auto assets =
      UnbundleGltfAssets(fs::path("foo.gltf"), &gltf_contents, &storage);
  EXPECT_EQ(assets.size(), 0);
  EXPECT_EQ(storage.size(), 0);
  EXPECT_EQ(gltf_contents, orig);
}

GTEST_TEST(UnbundleGltfAssetsTest, RelativeUri) {
  // Load a glTF that has exactly one *unbundled* asset.
  const fs::path gltf_filename =
      FindResourceOrThrow("drake/geometry/render/test/meshes/cube2.gltf");
  std::string gltf_contents = ReadFileOrThrow(gltf_filename);

  // Unbundle it.
  FileStorage storage;
  std::vector<std::shared_ptr<const MemoryFile>> assets =
      UnbundleGltfAssets(gltf_filename, &gltf_contents, &storage);

  // One asset was added to storage, identical to cube2.bin.
  const Sha256 expected_sha256 = Sha256::Checksum(ReadFileOrThrow(
      FindResourceOrThrow("drake/geometry/render/test/meshes/cube2.bin")));
  ASSERT_EQ(assets.size(), 1);
  EXPECT_EQ(assets.front()->sha256(), expected_sha256);

  // Make sure the new URI seems correct.
  EXPECT_THAT(std::string{json::parse(gltf_contents)["buffers"][0]["uri"]},
              testing::EndsWith(expected_sha256.to_string()));
}

GTEST_TEST(UnbundleGltfAssetsTest, RelativeUriBad) {
  std::string uri = "no-such-file.bin";
  std::string gltf_contents = MakeGltfWithUri(uri);
  const std::string orig = gltf_contents;
  FileStorage storage;
  auto assets =
      UnbundleGltfAssets(fs::path("foo.gltf"), &gltf_contents, &storage);
  EXPECT_EQ(assets.size(), 0);
  EXPECT_EQ(storage.size(), 0);
  EXPECT_EQ(gltf_contents, orig);
}

GTEST_TEST(UnbundleGltfAssetsTest, JsonParseError) {
  std::string gltf_contents = "Hello, world!";
  const std::string orig = gltf_contents;
  FileStorage storage;
  auto assets =
      UnbundleGltfAssets(fs::path("foo.gltf"), &gltf_contents, &storage);
  EXPECT_EQ(assets.size(), 0);
  EXPECT_EQ(storage.size(), 0);
  EXPECT_EQ(gltf_contents, orig);
}

GTEST_TEST(TransformGeometryNameTest, SampledResults) {
  SceneGraph<double> scene_graph;
  const SourceId s_id = scene_graph.RegisterSource("test");
  const RigidTransformd I = RigidTransformd::Identity();

  // Test pairs: (registered geometry name, expected transformed name).
  // clang-format off
  std::vector<std::pair<std::string, std::string>> test_names{
      {"unchanged", "unchanged"},
      {"::prefixed", "/prefixed"},
      {"single:colon", "single:colon"},
      {"simple::path", "simple/path"},
      {"extra:::colon", "extra/:colon"},
      {"empty::::slash", "empty//slash"},
      {"suffixed::", "suffixed/"},
      {"final_trio:::", "final_trio/:"}};
  // clang-format on
  for (const auto& [name, transformed] : test_names) {
    const GeometryId id = scene_graph.RegisterAnchoredGeometry(
        s_id, std::make_unique<GeometryInstance>(I, Sphere(1.0), name));
    EXPECT_EQ(TransformGeometryName(id, scene_graph.model_inspector()),
              transformed);
  }
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
