#include "drake/geometry/render_gltf_client/merge_gltf.h"

#include <filesystem>
#include <fstream>
#include <functional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"

namespace drake {
namespace geometry {
namespace render_gltf_client {
namespace internal {
namespace {

using nlohmann::json;
using std::string;
using std::vector;
GTEST_TEST(Json, Stuff) {
  json source;
  EXPECT_FALSE(source.contains("data"));
  json& data = source["data"];
  data.push_back(10);
  EXPECT_TRUE(data.is_array());
  json& value = data.back();
  EXPECT_TRUE(value.is_number());
}

/* Specification of a test case. Because each function under test is responsible
 for merging a particular array, each test case examines only the targeted
 array (see array_name). */
struct MergeCase {
  /* The string representation of the json for a gltf file. */
  string gltf1;
  /* The string representation of the json for a gltf file. */
  string gltf2;
  /* The name of the array that should match expected. If empty, the whole
   gltf is compared. */
  string array_name;
  /* The string representation of the expected json produced by merging. */
  string expected;
  /* A description of what is being tested -- displayed in a scoped trace. */
  string description;
  /* The merge function that should be called. */
  std::function<void(json*, json&&)> merge{};
};

/* Evaluates a test case, confirming the merged result is as expected. */
void Evaluate(const MergeCase& test_case) {
  json gltf1 = json::parse(test_case.gltf1);
  json gltf2 = json::parse(test_case.gltf2);
  test_case.merge(&gltf1, std::move(gltf2));
  const json expected = json::parse(test_case.expected);
  if (test_case.array_name.empty()) {
    EXPECT_EQ(gltf1, expected) << test_case.description;
  } else {
    EXPECT_EQ(gltf1[test_case.array_name], expected) << test_case.description;
  }
}

/* The test harness for the various merge functions. This includes families of
 static functions that generate test cases.

 The vast majority of the merging logic consists of appending the contents
 of one *source* gltf's named arrays to a *target*'s corresponding named
 array. For some named arrays, it is truly that simple. For other arrays, it
 is necessary to bump indices contained in the elements of the source array
 which reference other *independent* arrays. Finally, in some cases, we
 actually cull some attributes.

 The testing strategy we employee is one of mocking. While there are correct
 gltf semantics (e.g., some fields are required, some values must lie in a
 certain range), these semantics don't matter for these tests because the
 device under test simply operates on the json. So, we can supplant correct
 gltf contents with convenient alternatives. */
class MergeTest : public testing::TestWithParam<MergeCase> {
 public:
  /* Creates a collection of test cases where an element in the sources array
   contains indices into another independent array. The indices get offset by
   the *size* of the independent array when concatenated to the corresponding
   target array. Only the independent array's size matters, not the contents.
   So, we populate it with arbitrary garbage entries.

   We make sure that the source array has multiple elements to confirm that
   we iterate through its contents.

   There are four variations of the test, based on which gltf has the array
   under test (one, both, or neither). When the array is present in the
   target file, we fill it with arbitrary values (we just want to see the
   *concatenation*) with the source array's contents.

   @param cases         The new cases will be added to this vector.
   @param array         The *name* of the array being tested.
   @param independent   The *name* of the independent array that the source
                        array's elements index into.
   @param merge         The function that merges the named arrays. */
  static void BumpIndex(vector<MergeCase>* cases, const std::string& array,
                        const std::string& independent,
                        const std::string& index,
                        std::function<void(json*, json&&)> merge) {
    cases->push_back(
        {.gltf1 = fmt::format(R"({{"{}":[10, 20], "{}":[0, 1]}})", array,
                              independent),
         .gltf2 = fmt::format(R"({{"{0}":[{{"{1}":10}}, {{"{1}":20}}]}})",
                              array, index),
         .array_name = array,
         .expected =
             fmt::format(R"([10, 20, {{"{0}":12}}, {{"{0}":22}}])", index),
         .description = fmt::format(
             "The {} {} indices get bumped - array in both.", array, index),
         .merge = merge});
    cases->push_back(
        {.gltf1 = fmt::format(R"({{"{}":[10, 20], "{}":[0, 1]}})", array,
                              independent),
         .gltf2 = "{}",
         .array_name = array,
         .expected = "[10, 20]",
         .description = fmt::format(
             "The {} {} indices get bumped - array in 1.", array, index),
         .merge = merge});
    cases->push_back(
        {.gltf1 = fmt::format(R"({{"{}":[0, 1]}})", independent),
         .gltf2 = fmt::format(R"({{"{0}":[{{"{1}":10}}, {{"{1}":20}}]}})",
                              array, index),
         .array_name = array,
         .expected = fmt::format(R"([{{"{0}":12}}, {{"{0}":22}}])", index),
         .description = fmt::format(
             "The {} {} indices get bumped - array in 2.", array, index),
         .merge = merge});
    cases->push_back(
        {.gltf1 = fmt::format(R"({{"{}":[0, 1]}})", independent),
         .gltf2 = "{}",
         .array_name = array,
         .expected = "null",
         .description = fmt::format(
             "The {} {} indices get bumped - array in neither.", array, index),
         .merge = merge});
  }

  /* Creates a test case where the elements of the array in question is copied
   verbatim. We don't inspect or modify the contents in any way. So, we put some
   unique garbage into the arrays and test that the code really doesn't care.

   We make sure that the source array has multiple elements to confirm that
   we iterate through its contents.

   This function can also be used to test that, even for elements that would
   require index modification (see BumpIndex()), all *other* element properties
   get copied verbatim. This works because this test case doesn't define the
   independent array, so any indices that *would* be present don't get offset.
   If the merge function specifically depends on finding particular entries,
   this would fail (confirming that it doesn't even introspect the contents).

   As with BumpIndex(), it creates a family of such tests based on which of the
   gltfs have the named array under question.

   @param cases         The new cases will be added to this vector.
   @param array         The *name* of the array being tested.
   @param merge         The function that merges the named arrays. */
  static void VerbatimCopy(vector<MergeCase>* cases, const std::string& array,
                           std::function<void(json*, json&&)> merge) {
    cases->push_back(
        {.gltf1 = fmt::format("{{\"{}\":[0, 1]}}", array),
         .gltf2 = fmt::format(
             R"({{"{}":[{{"one":1}},
                       {{"two":{{"arabic":2, "roman":"II"}}}}]}})",
             array),
         .array_name = array,
         .expected = R"([0, 1, {"one":1}, {"two":{"arabic":2, "roman":"II"}}])",
         .description = fmt::format(
             "The {} array contents copied verbatim - array in both.", array),
         .merge = merge});
    cases->push_back(
        {.gltf1 = fmt::format("{{\"{}\":[0, 1]}}", array),
         .gltf2 = "{}",
         .array_name = array,
         .expected = "[0, 1]",
         .description = fmt::format(
             "The {} array contents copied verbatim - array in 1.", array),
         .merge = merge});
    cases->push_back(
        {.gltf1 = "{}",
         .gltf2 = fmt::format(
             R"({{"{}":[{{"one":1}},
                       {{"two":{{"arabic":2, "roman":"II"}}}}]}})",
             array),
         .array_name = array,
         .expected = R"([{"one":1}, {"two":{"arabic":2, "roman":"II"}}])",
         .description = fmt::format(
             "The {} array contents copied verbatim - array in 2.", array),
         .merge = merge});
    cases->push_back(
        {.gltf1 = "{}",
         .gltf2 = "{}",
         .array_name = array,
         .expected = "null",
         .description = fmt::format(
             "The {} array contents copied verbatim - array in neither.",
             array),
         .merge = merge});
  }

  static vector<MergeCase> MergeScenesCases() {
    /* Merging scenes does a bit more work. A scene in the source may simply be
     concatenated to the target's "scenes" array (with the scene's node indices
     bumped appropriately). But where the two gltf files have scenes with
     matching names, those scenes get merged. The merging logic requires a
     bespoke test.

     We can't use the test case APIs to test bumping node indices because the
     nodes are not direct children of a "scene" element. */

    std::function<void(json*, json&&)> merge = MergeScenes;
    return vector<MergeCase>{
        {.gltf1 = R"({"scenes":[{"name":"Scene", "nodes":[0],
                                 "extras":{"v":2}}],
                      "nodes":[0]})",
         .gltf2 = "{}",
         .array_name = "scenes",
         .expected = R"([{"name":"Scene", "nodes":[0], "extras":{"v":2}}])",
         .description = "Only target has scenes.",
         .merge = merge},
        {.gltf1 = R"({"nodes":[0,1]})",
         .gltf2 =
             R"({"scenes":[{"name":"Scene", "nodes":[0], "extras":{"v":1}}]})",
         .array_name = "scenes",
         .expected = R"([{"name":"Scene", "nodes":[2], "extras":{"v":1}}])",
         .description = "Only source has scenes, nodes in both.",
         .merge = merge},
        {.gltf1 = R"({"scenes":[{"name":"Scene", "nodes":[0]}],
                      "nodes":[0, 1]})",
         .gltf2 = R"({"scenes":[{"name":"Scene", "nodes":[0]}], "nodes":[0]})",
         .array_name = "scenes",
         .expected = R"([{"name":"Scene", "nodes":[0, 2]}])",
         .description = "Scene names match.",
         .merge = merge},
        {.gltf1 = R"({"scenes":[{"name":"SceneA", "nodes":[0]}],
                      "nodes":[0, 1]})",
         .gltf2 = R"({"scenes":[{"name":"SceneB", "nodes":[0, 2]}],
                                 "nodes":[0,1,2]})",
         .array_name = "scenes",
         .expected = R"([{"name":"SceneA", "nodes":[0]},
                         {"name":"SceneB", "nodes":[2, 4]}])",
         .description = "Scene names don't match.",
         .merge = merge},
    };
  }

  static vector<MergeCase> MergeNodesCases() {
    std::function<void(json*, json&&)> merge = MergeNodes;
    /* A "node" has indices to bump, "mesh" or "camera". But it may also contain
     an array of node indices, "children". "mesh" and "camera" can use the
     test case framework, but the "children" requires a bespoke test.

     Furthermore, we explicitly erase the possible "skin" element. */
    vector<MergeCase> cases{
        {.gltf1 = R"({"nodes":[{"name":"A","children":[1]},
                             {"name":"B","mesh":0}],
                    "meshes":[0, 1]})",
         .gltf2 = R"({"nodes":[{"name":"C","children":[0, 1]}, {"name":"D"}]})",
         .array_name = "nodes",
         .expected = R"([{"name":"A","children":[1]}, {"name":"B","mesh":0},
                       {"name":"C","children":[2,3]}, {"name":"D"}])",
         .description = "Nodes in gltf1 cause children in 2 to offset.",
         .merge = merge},
        {.gltf1 = "{}",
         .gltf2 = R"({"nodes":[{"name":"C","skin":1}]})",
         .array_name = "nodes",
         .expected = R"([{"name":"C"}])",
         .description = R"(Remove "skin" child.)",
         .merge = merge},
    };

    BumpIndex(&cases, "nodes", "meshes", "mesh", merge);
    BumpIndex(&cases, "nodes", "cameras", "camera", merge);
    VerbatimCopy(&cases, "nodes", merge);
    return cases;
  }

  static vector<MergeCase> MergeMeshesCases() {
    std::function<void(json*, json&&)> merge = MergeMeshes;
    /* Meshes cannot use the test case creation utilities.
      1. Meshes delete two properties mesh.weights and
         mesh.primitives[i].targets.
      2. The indices that need to get bumped are not direct children of the
         "mesh", but are buried in primitives arrays that get visited.
      3. It has an unbounded set of "attributes" that get their indices
         bumped.
     So, these test cases get spelled out explicitly. */
    vector<MergeCase> cases{
        {.gltf1 = R"({"materials":[0, 1]})",
         .gltf2 = R"({"meshes":[{"name":"A", "primitives":[{"material":2}]}]})",
         .array_name = "meshes",
         .expected = R"([{"name":"A", "primitives":[{"material":4}]}])",
         .description = "Material index gets bumped.",
         .merge = merge},
        {.gltf1 = R"({"accessors":[0, 1]})",
         .gltf2 = R"({"meshes":[{"name":"A", "primitives":[{"indices":2}]}]})",
         .array_name = "meshes",
         .expected = R"([{"name":"A", "primitives":[{"indices":4}]}])",
         .description = "Vertex accessor gets bumped.",
         .merge = merge},
        {.gltf1 = R"({"accessors":[0, 1]})",
         .gltf2 = R"({"meshes":[{"name":"A",
                                 "primitives":[
                                    {"attributes":{"key1":1}},
                                    {"attributes":{"key2":3, "key4":5}}
                                  ]}]})",
         .array_name = "meshes",
         .expected = R"([{"name":"A",
                          "primitives":[
                             {"attributes":{"key1":3}},
                             {"attributes":{"key2":5, "key4":7}}
                           ]}])",
         .description = "Primitives attributes get bumped.",
         .merge = merge},
        {.gltf1 = R"({})",
         .gltf2 = R"({"meshes":[{"primitives":[{"mode":17,
                                                "extras":{"v":2}}]}]})",
         .array_name = "meshes",
         .expected = R"([{"primitives":[{"mode":17,
                                         "extras":{"v":2}}]}])",
         .description = "Non-index primitive fields propagate.",
         .merge = merge},
        {.gltf1 = R"({})",
         .gltf2 = R"({"meshes":[{"primitives":[{"mode":17,
                                                "targets":{"v":2}}]}]})",
         .array_name = "meshes",
         .expected = R"([{"primitives":[{"mode":17}]}])",
         .description = "Targets get removed from primitives.",
         .merge = merge},
        {.gltf1 = R"({"meshes":[{"name":"A"}]})",
         .gltf2 = R"({"meshes":[{"name":"B", "weights":[0, 1]}]})",
         .array_name = "meshes",
         .expected = R"([{"name":"A"}, {"name":"B"}])",
         .description = "Weights get stripped.",
         .merge = merge},
    };
    VerbatimCopy(&cases, "meshes", merge);
    return cases;
  }

  static vector<MergeCase> MergeMaterialsCases() {
    std::function<void(json*, json&&)> merge = MergeMaterials;
    /* Merging materials requires delving into the elements recursively. The
     indices to bump are not directly children of the "material" element. So,
     we test the bump logic directly. */
    vector<MergeCase> cases{
        {.gltf1 = R"({"textures":[0, 1]})",
         .gltf2 = R"({"materials":[
                         {"normalTexture":{"index":2},
                          "occlusionTexture":{"index":3},
                          "emissiveTexture":{"index":4},
                          "pbrMetallicRoughness":{
                            "baseColorTexture":{"index":5},
                            "metallicRoughnessTexture":{"index":6}
                          }},
                         {"normalTexture":{"index":17}}
                         ]})",
         .array_name = "materials",
         .expected = R"([{"normalTexture":{"index":4},
                          "occlusionTexture":{"index":5},
                          "emissiveTexture":{"index":6},
                          "pbrMetallicRoughness":{
                            "baseColorTexture":{"index":7},
                            "metallicRoughnessTexture":{"index":8}
                          }},
                          {"normalTexture":{"index":19}}])",
         .description = "Texture indices get bumped on all materials.",
         .merge = merge},
    };
    VerbatimCopy(&cases, "materials", merge);
    return cases;
  }

  static vector<MergeCase> MergeCamerasCases() {
    vector<MergeCase> cases;
    VerbatimCopy(&cases, "cameras", MergeCameras);
    return cases;
  }

  static vector<MergeCase> MergeAccessorsCases() {
    vector<MergeCase> cases;
    BumpIndex(&cases, "accessors", "bufferViews", "bufferView", MergeAccessors);
    VerbatimCopy(&cases, "accessors", MergeAccessors);
    return cases;
  }

  static vector<MergeCase> MergeBufferViewsCases() {
    vector<MergeCase> cases;
    BumpIndex(&cases, "bufferViews", "buffers", "buffer", MergeBufferViews);
    VerbatimCopy(&cases, "bufferViews", MergeBufferViews);
    return cases;
  }

  static vector<MergeCase> MergeBuffersCases() {
    vector<MergeCase> cases;
    VerbatimCopy(&cases, "buffers", MergeBuffers);
    return cases;
  }

  static vector<MergeCase> MergeTexturesCases() {
    vector<MergeCase> cases;
    BumpIndex(&cases, "textures", "samplers", "sampler", MergeTextures);
    BumpIndex(&cases, "textures", "images", "source", MergeTextures);
    VerbatimCopy(&cases, "textures", MergeTextures);
    return cases;
  }

  static vector<MergeCase> MergeImageCases() {
    vector<MergeCase> cases;
    BumpIndex(&cases, "images", "bufferViews", "bufferView", MergeImages);
    VerbatimCopy(&cases, "images", MergeImages);
    return cases;
  }

  static vector<MergeCase> MergeSamplersCases() {
    vector<MergeCase> cases;
    VerbatimCopy(&cases, "samplers", MergeSamplers);
    return cases;
  }
};

TEST_P(MergeTest, Evaluate) {
  Evaluate(GetParam());
}

INSTANTIATE_TEST_SUITE_P(Scenes, MergeTest,
                         testing::ValuesIn(MergeTest::MergeScenesCases()));
INSTANTIATE_TEST_SUITE_P(Nodes, MergeTest,
                         testing::ValuesIn(MergeTest::MergeNodesCases()));
INSTANTIATE_TEST_SUITE_P(Meshes, MergeTest,
                         testing::ValuesIn(MergeTest::MergeMeshesCases()));
INSTANTIATE_TEST_SUITE_P(Materials, MergeTest,
                         testing::ValuesIn(MergeTest::MergeMaterialsCases()));
INSTANTIATE_TEST_SUITE_P(Cameras, MergeTest,
                         testing::ValuesIn(MergeTest::MergeCamerasCases()));
INSTANTIATE_TEST_SUITE_P(Accessors, MergeTest,
                         testing::ValuesIn(MergeTest::MergeAccessorsCases()));
INSTANTIATE_TEST_SUITE_P(BufferViews, MergeTest,
                         testing::ValuesIn(MergeTest::MergeBufferViewsCases()));
INSTANTIATE_TEST_SUITE_P(Buffers, MergeTest,
                         testing::ValuesIn(MergeTest::MergeBuffersCases()));
INSTANTIATE_TEST_SUITE_P(Textures, MergeTest,
                         testing::ValuesIn(MergeTest::MergeTexturesCases()));
INSTANTIATE_TEST_SUITE_P(Images, MergeTest,
                         testing::ValuesIn(MergeTest::MergeImageCases()));
INSTANTIATE_TEST_SUITE_P(Samplers, MergeTest,
                         testing::ValuesIn(MergeTest::MergeSamplersCases()));

/* Simply call MergeGltf on real glTF files and makes sure it doesn't throw. */
GTEST_TEST(MergeGltf, Smoke) {
  json target = ReadJsonFile(FindResourceOrThrow(
      "drake/geometry/render_gltf_client/test/red_box.gltf"));

  json source = ReadJsonFile(FindResourceOrThrow(
      "drake/geometry/render_gltf_client/test/textured_green_box.gltf"));

  EXPECT_NO_THROW(MergeGltf(&target, std::move(source)));

  /* Save the merged gltf for inspection.
   bazel-testlogs/geometry/render_gltf_client/merge_gltf_test/test.outputs/output.zip.
   */
  if (const char* dir = std::getenv("TEST_UNDECLARED_OUTPUTS_DIR")) {
    /* target now contains the merged result. */
    json& merged = target;
    std::ofstream f(std::filesystem::path(dir) / "merged.gltf");
    f << std::setw(2);
    f << merged;
  }
}

}  // namespace
}  // namespace internal
}  // namespace render_gltf_client
}  // namespace geometry
}  // namespace drake
