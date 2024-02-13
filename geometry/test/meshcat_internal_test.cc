#include "drake/geometry/meshcat_internal.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "drake/common/find_resource.h"
#include "drake/common/sha256.h"
#include "drake/geometry/meshcat_file_storage_internal.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using nlohmann::json;

GTEST_TEST(MeshcatInternalTest, GetMeshcatStaticResource) {
  // This matches the list of URLs in the API doc.
  const std::vector<const char*> urls{
      "/",           "/favicon.ico",  "/index.html", "/meshcat.html",
      "/meshcat.js", "/stats.min.js",
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
  const std::string gltf_filename =
      FindResourceOrThrow("drake/geometry/render/test/meshes/cube1.gltf");
  std::string gltf_contents = ReadFileOrThrow(gltf_filename);
  const size_t orig_size = gltf_contents.size();

  // Unbundle it.
  FileStorage storage;
  std::vector<std::shared_ptr<const FileStorage::Handle>> assets =
      UnbundleGltfAssets(gltf_filename, &gltf_contents, &storage);

  // The contents got smaller due to unbundling (but not too small).
  DRAKE_DEMAND(orig_size >= 4096);
  EXPECT_LT(gltf_contents.size(), 2048);
  EXPECT_GT(gltf_contents.size(), 1024);

  // One asset was added to storage, identical to cube2.bin.
  const Sha256 expected_sha256 = Sha256::Checksum(ReadFileOrThrow(
      FindResourceOrThrow("drake/geometry/render/test/meshes/cube2.bin")));
  ASSERT_EQ(assets.size(), 1);
  EXPECT_EQ(assets.front()->sha256, expected_sha256);

  // Make sure the new URI seems correct.
  EXPECT_THAT(json::parse(gltf_contents)["buffers"][0]["uri"],
              testing::EndsWith(expected_sha256.to_string()));
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
  auto assets = UnbundleGltfAssets("foo.gltf", &gltf_contents, &storage);
  EXPECT_EQ(assets.size(), 0);
  EXPECT_EQ(storage.size(), 0);
  EXPECT_EQ(gltf_contents, orig);
}

GTEST_TEST(UnbundleGltfAssetsTest, RelativeUri) {
  // Load a glTF that has exactly one *unbundled* asset.
  const std::string gltf_filename =
      FindResourceOrThrow("drake/geometry/render/test/meshes/cube2.gltf");
  std::string gltf_contents = ReadFileOrThrow(gltf_filename);

  // Unbundle it.
  FileStorage storage;
  std::vector<std::shared_ptr<const FileStorage::Handle>> assets =
      UnbundleGltfAssets(gltf_filename, &gltf_contents, &storage);

  // One asset was added to storage, identical to cube2.bin.
  const Sha256 expected_sha256 = Sha256::Checksum(ReadFileOrThrow(
      FindResourceOrThrow("drake/geometry/render/test/meshes/cube2.bin")));
  ASSERT_EQ(assets.size(), 1);
  EXPECT_EQ(assets.front()->sha256, expected_sha256);

  // Make sure the new URI seems correct.
  EXPECT_THAT(json::parse(gltf_contents)["buffers"][0]["uri"],
              testing::EndsWith(expected_sha256.to_string()));
}

GTEST_TEST(UnbundleGltfAssetsTest, RelativeUriBad) {
  std::string uri = "no-such-file.bin";
  std::string gltf_contents = MakeGltfWithUri(uri);
  const std::string orig = gltf_contents;
  FileStorage storage;
  auto assets = UnbundleGltfAssets("foo.gltf", &gltf_contents, &storage);
  EXPECT_EQ(assets.size(), 0);
  EXPECT_EQ(storage.size(), 0);
  EXPECT_EQ(gltf_contents, orig);
}

GTEST_TEST(UnbundleGltfAssetsTest, JsonParseError) {
  std::string gltf_contents = "Hello, world!";
  const std::string orig = gltf_contents;
  FileStorage storage;
  auto assets = UnbundleGltfAssets("foo.gltf", &gltf_contents, &storage);
  EXPECT_EQ(assets.size(), 0);
  EXPECT_EQ(storage.size(), 0);
  EXPECT_EQ(gltf_contents, orig);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
