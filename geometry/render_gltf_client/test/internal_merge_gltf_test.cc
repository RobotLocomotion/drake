#include "drake/geometry/render_gltf_client/internal_merge_gltf.h"

#include <filesystem>
#include <fstream>
#include <functional>
#include <ostream>
#include <regex>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace geometry {
namespace render_gltf_client {
namespace internal {
namespace {

namespace fs = std::filesystem;

using nlohmann::json;
using std::string;
using std::vector;

// Test the conversion between Eigen and glTF json. We'll simply create a weird
// 4x4 matrix and make sure that translating in and out of json is idempotent.
GTEST_TEST(GltfMergeTest, JsonEigenConversion) {
  Matrix4<double> M;
  M << 1.1, 2, 3.5, 4, 5, 6, 7, 8, 9.8, 10, 11, 12, 13, 14, 15, 16;
  const json M_json = GltfMatrixFromEigenMatrix(M);
  const Matrix4<double> M_return = EigenMatrixFromGltfMatrix(M_json);
  EXPECT_TRUE(CompareMatrices(M_return, M));
}

using MergeFunction = std::function<void(json*, json&&)>;

/* Specification of a test case. Because each function under test is responsible
 for merging a particular array, each test case examines only the targeted
 array (see array_name).

 To make the test case instantiation more compact, we write json using ' instead
 of ". This allows us to use simple string literals: "'key':'value'" instead of
 raw literals: ""key":"value"". The single quotes are swapped for double
 quotes in the Evaluate() test function. This applies to `target`, `source`, and
 `expected`. */
struct MergeCase {
  /* A description of what is being tested -- displayed for test failure. */
  string description;
  /* A string representation of the json for a glTF file. This glTF will be
   modified. */
  string target;
  /* A string representation of the json for a glTF file. This glTF will be
   merged into target. */
  string source;
  /* The name of the array that should match expected. */
  string array_name;
  /* A string representation of the expected json produced by merging. */
  string expected;
  /* The merge function that should be called. */
  MergeFunction merge{};

  friend std::ostream& operator<<(std::ostream& out,
                                  const MergeCase& merge_case) {
    out << merge_case.description;
    return out;
  }
};

/* The test harness for the various merge functions. This includes families of
 static functions that generate test cases.

 The vast majority of the merging logic consists of appending the contents
 of one *source* glTF's named arrays to a *target*'s corresponding named
 array. For some named arrays, it is truly that simple. For other arrays, it
 is necessary to bump indices contained in the elements of the source array
 which reference other *independent* arrays. Finally, in some cases (noted
 explicitly in their test cases), we actually cull some attributes.

 The testing strategy we employee is one of mocking. While there are correct
 glTF semantics (e.g., some fields are required, some values must lie in a
 certain range), these semantics don't matter for these tests because the
 device under test simply operates on the json. So, we can supplant correct
 glTF contents with convenient alternatives. */
class MergeTest : public testing::TestWithParam<MergeCase> {
 public:
  /* Creates a collection of test cases where an element in the sources array
   contains indices into another independent array. The indices get offset by
   the *size* of the independent array when concatenated to the corresponding
   target array. Only the independent array's size matters, not the contents.
   So, we populate it with arbitrary garbage entries.

   We make sure that the source array has multiple elements to confirm that
   we iterate through its contents.

   There are four variations of the test, based on which glTF has the array
   under test (one, the other, both, or neither). When the array is present in
   the target file, we fill it with arbitrary values (we just want to see the
   *concatenation*) with the source array's contents.

   @param cases         The new cases will be appended to this vector.
   @param array         The *name* of the array being tested.
   @param independent   The *name* of the independent array that the source
                        array's elements index into.
   @param index         The *name* of the element that is an index into the
                        independent array.
   @param merge         The function that merges the named arrays. This is the
                        device under test. */
  static void BumpIndex(vector<MergeCase>* cases, const string& array,
                        const string& independent, const string& index,
                        MergeFunction merge) {
    DRAKE_DEMAND(cases != nullptr);
    cases->push_back(
        {.description = fmt::format(
             "The {} {} indices get bumped - array in both.", array, index),
         .target =
             fmt::format("{{'{}':[10, 20], '{}':[0, 1]}}", array, independent),
         .source = fmt::format("{{'{0}':[{{'{1}':10}}, {{'{1}':20}}]}}", array,
                               index),
         .array_name = array,
         .expected = fmt::format("[10, 20, {{'{0}':12}}, {{'{0}':22}}]", index),
         .merge = merge});
    cases->push_back(
        {.description = fmt::format(
             "The {} {} indices get bumped - array in target.", array, index),
         .target =
             fmt::format("{{'{}':[10, 20], '{}':[0, 1]}}", array, independent),
         .source = "{}",
         .array_name = array,
         .expected = "[10, 20]",
         .merge = merge});
    cases->push_back(
        {.description = fmt::format(
             "The {} {} indices get bumped - array in source.", array, index),
         .target = fmt::format("{{'{}':[0, 1]}}", independent),
         .source = fmt::format("{{'{0}':[{{'{1}':10}}, {{'{1}':20}}]}}", array,
                               index),
         .array_name = array,
         .expected = fmt::format("[{{'{0}':12}}, {{'{0}':22}}]", index),
         .merge = merge});
    cases->push_back(
        {.description = fmt::format(
             "The {} {} indices get bumped - array in neither.", array, index),
         .target = fmt::format("{{'{}':[0, 1]}}", independent),
         .source = "{}",
         .array_name = array,
         .expected = "null",
         .merge = merge});
  }

  /* Creates a test case where the elements of the array in question are copied
   verbatim. We don't inspect or modify the contents in any way. So, we put some
   unique garbage into the arrays to confirm that the dut really doesn't care.
   Note: passing this test implies that esoteric tags like "extensions" or
   "extras" will propagate through for those entities that simply get appended
   to arrays instead of getting merged together.

   We make sure that the source array has multiple elements to confirm that
   we iterate through its contents.

   This function can also be used to test that, even for elements that would
   require index modification (see BumpIndex()), all *other* element properties
   get copied verbatim. This works because this test case doesn't define the
   independent array, so any indices that *would* be present don't get offset.
   If the merge function specifically depends on finding particular entries,
   this would fail (confirming that it doesn't even introspect the contents).

   As with BumpIndex(), it creates a family of such tests based on which of the
   glTFs have the named array under question.

   @param cases         The new cases will be added to this vector.
   @param array         The *name* of the array being tested.
   @param merge         The function that merges the named arrays. This is the
                        device under test. */
  static void VerbatimCopy(vector<MergeCase>* cases, const string& array,
                           MergeFunction merge) {
    DRAKE_DEMAND(cases != nullptr);
    cases->push_back(
        {.description = fmt::format(
             "The {} array contents copied verbatim - array in both.", array),
         .target = fmt::format("{{'{}':[0, 1]}}", array),
         .source = fmt::format(
             "{{'{}':[{{'one':1}}, {{'two':{{'arabic':2, 'roman':'II'}}}}]}}",
             array),
         .array_name = array,
         .expected = "[0, 1, {'one':1}, {'two':{'arabic':2, 'roman':'II'}}]",
         .merge = merge});
    cases->push_back(
        {.description = fmt::format(
             "The {} array contents copied verbatim - array in target.", array),
         .target = fmt::format("{{'{}':[0, 1]}}", array),
         .source = "{}",
         .array_name = array,
         .expected = "[0, 1]",
         .merge = merge});
    cases->push_back(
        {.description = fmt::format(
             "The {} array contents copied verbatim - array in source.", array),
         .target = "{}",
         .source = fmt::format(
             "{{'{}':[{{'one':1}}, {{'two':{{'arabic':2, 'roman':'II'}}}}]}}",
             array),
         .array_name = array,
         .expected = "[{'one':1}, {'two':{'arabic':2, 'roman':'II'}}]",
         .merge = merge});
    cases->push_back(
        {.description = fmt::format(
             "The {} array contents copied verbatim - array in neither.",
             array),
         .target = "{}",
         .source = "{}",
         .array_name = array,
         .expected = "null",
         .merge = merge});
  }

  static vector<MergeCase> MergeExtensionsAndExtrasCases() {
    /* Drake attempts to merge sets of "extensions" and "extras" blobs in only
     three cases: the root glTF file, the assets, and scenes that get merged.
     These test cases primarily use the root glTF to test the merging logic but
     also include a few token tests to confirm that the logic is applied to the
     extensions and extras in "asset". The scene merging is handled in the scene
     test.

     Failure to merge is handled in its own test harness (MergeFailureTest). */
    auto merge = [](json* j1, json&& j2) {
      MergeRecord record("test_target");
      MergeGltf(j1, std::move(j2), "test_source", &record);
    };
    // Note: "asset" must exist for MergeGltf to work.
    const string asset_prefix = "{'asset':{'version':'2.0'}";
    return vector<MergeCase>{
        // Successful merges on extensions.
        {.description = "Target only extensions are preserved",
         .target = asset_prefix + ", 'extensions':{'one':1}}",
         .source = asset_prefix + "}",
         .array_name = "extensions",
         .expected = "{'one':1}",
         .merge = merge},
        {.description = "Source only extensions are preserved",
         .target = asset_prefix + "}",
         .source = asset_prefix + ", 'extensions':{'one':1}}",
         .array_name = "extensions",
         .expected = "{'one':1}",
         .merge = merge},
        {.description = "Non-colliding data merges",
         .target = asset_prefix + ", 'extensions':{'two':2}}",
         .source = asset_prefix + ", 'extensions':{'one':1}}",
         .array_name = "extensions",
         .expected = "{'one':1, 'two':2}",
         .merge = merge},
        {.description = "Target has no extensions, source is non-object",
         .target = asset_prefix + "}",
         .source = asset_prefix + ", 'extensions':2}",
         .array_name = "extensions",
         .expected = "2",
         .merge = merge},
        {.description = "Source has non-object extensions, target has none",
         .target = asset_prefix + ", 'extensions':1}",
         .source = asset_prefix + "}",
         .array_name = "extensions",
         .expected = "1",
         .merge = merge},
        {.description = "Source and object have shallow, non-colliding "
                        "duplications in extensions",
         .target = asset_prefix + ", 'extensions':{'one':1}}",
         .source = asset_prefix + ", 'extensions':{'one':1, 'two':2}}",
         .array_name = "extensions",
         .expected = "{'one':1, 'two':2}",
         .merge = merge},
        {.description = "Source and object have deep, non-colliding "
                        "duplications in extensions",
         .target = asset_prefix + ", 'extensions':{'one':{'a':1}}}",
         .source = asset_prefix + ", 'extensions':{'one':{'a':1}, 'b':2}}",
         .array_name = "extensions",
         .expected = "{'one':{'a':1}, 'b':2}",
         .merge = merge},
        // Evidence that extras are being merged; complex successful and
        // unsuccessful case.
        {.description = "glTFs have deep, non-colliding duplications in extras",
         .target = asset_prefix + ", 'extras':{'one':{'a':1}}}",
         .source = asset_prefix + ", 'extras':{'one':{'a':1}, 'b':2}}",
         .array_name = "extras",
         .expected = "{'one':{'a':1}, 'b':2}",
         .merge = merge},
        // Evidence that extras and extensions are being merged in "assets".
        {.description =
             "glTFs have deep, non-colliding duplications in assets[extras]",
         .target = "{'asset':{'version':'2.0', 'extras':{'one':{'a':1}}}}",
         .source =
             "{'asset':{'version':'2.0', 'extras':{'one':{'a':1}, 'b':2}}}",
         .array_name = "asset",
         .expected = "{'generator':'Drake glTF merger', 'version':'2.0', "
                     "'extras':{'one':{'a':1}, 'b':2}}",
         .merge = merge},
        {.description = "glTFs have deep, non-colliding duplications in "
                        "assets[extensions]",
         .target = "{'asset':{'version':'2.0', 'extensions':{'one':{'a':1}}}}",
         .source =
             "{'asset':{'version':'2.0', 'extensions':{'one':{'a':1}, 'b':2}}}",
         .array_name = "asset",
         .expected = "{'generator':'Drake glTF merger', 'version':'2.0', "
                     "'extensions':{'one':{'a':1}, 'b':2}}",
         .merge = merge},
    };
  }

  static vector<MergeCase> MergeExtensionsDeclarationCases() {
    /* Both the "extensionsUsed" and "extensionsRequired" arrays use the same
     code. We'll rigorously test one of those arrays and then provide a token
     test for the other to make sure it gets processed correctly. */
    return vector<MergeCase>{
        {.description = "Target only is passed through",
         .target = "{'extensionsUsed':['A', 'B']}",
         .source = "{}",
         .array_name = "extensionsUsed",
         .expected = "['A', 'B']",
         .merge = MergeExtensionsUsed},
        {.description = "Source only is preserved",
         .target = "{}",
         .source = "{'extensionsUsed':['A', 'B']}",
         .array_name = "extensionsUsed",
         .expected = "['A', 'B']",
         .merge = MergeExtensionsUsed},
        {.description = "Source and target are merged without duplication",
         .target = "{'extensionsUsed':['A', 'B']}",
         .source = "{'extensionsUsed':['C', 'B', 'D']}",
         .array_name = "extensionsUsed",
         .expected = "['A', 'B', 'C', 'D']",
         .merge = MergeExtensionsUsed},
        {.description = "Source and target are merged without duplication - "
                        "extensionsRequired",
         .target = "{'extensionsRequired':['B', 'A']}",
         .source = "{'extensionsRequired':['C', 'B', 'D']}",
         .array_name = "extensionsRequired",
         .expected = "['B', 'A', 'C', 'D']",
         .merge = MergeExtensionsRequired},
    };
  }

  static vector<MergeCase> MergeScenesCases() {
    /* In merging scenes, we need to test the following aspects:
       - only default scenes get merged
         - both explicit declaration and default definition of "default".
       - When merging the target nodes get appended to the source node with
         offset values.
       - Extras and extensions get merged (where appropriate). Cases where
         merging fails uses the MergeFailureTest infrastructure.
    */
    MergeFunction merge = [](json* j1, json&& j2) {
      MergeRecord record("test_target");
      MergeDefaultScenes(j1, std::move(j2), "test_source", &record);
    };
    return vector<MergeCase>{
        // Cases that cover respect for the default scene *and* confirm that
        // merged nodes get appended and offset appropriately.
        //
        // Recognizing that the "right" source scene got merged is a bit
        // subtle. Below, we have source scenes S0 and S1 which reference source
        // nodes 0 and 1, respectively. Both of those nodes get merged, changing
        // their indices to 2 and 3. So, if the expected scene has node 2, it
        // means we merged S0. If it has index 3, we merged S1.
        //
        // When we become more strategic about what assets get merged, we'll
        // have to be equally more strategic in detecting success (i.e., by
        // giving the nodes unique names and confirming the nodes we've merged
        // have the expected names).
        {.description = "Target and source default scenes inferred as zero.",
         .target = "{'scenes':[{'name':'T0', 'nodes':[0]}, "
                   "           {'name':'T1', 'nodes':[1]}], "
                   "'nodes':[0, 1]}",  // We need nodes in target to get offset.
         .source = "{'scenes':[{'name':'S0', 'nodes':[0]}, "
                   "           {'name':'S1', 'nodes':[1]}]}",
         .array_name = "scenes",
         .expected = "[{'name':'T0', 'nodes':[0, 2]}, "
                     " {'name':'T1', 'nodes':[1]}]",
         .merge = merge},
        {.description = "Target and source default scenes explicitly zero.",
         .target = "{'scenes':[{'name':'T0', 'nodes':[0]}, "
                   "           {'name':'T1', 'nodes':[1]}], "
                   "'scene':0, "
                   "'nodes':[0, 1]}",  // We need nodes in target to get offset.
         .source = "{'scenes':[{'name':'S0', 'nodes':[0]}, "
                   "           {'name':'S1', 'nodes':[1]}], "
                   "'scene':0}",
         .array_name = "scenes",
         .expected = "[{'name':'T0', 'nodes':[0, 2]}, "
                     " {'name':'T1', 'nodes':[1]}]",
         .merge = merge},
        {.description = "Target scene is 0, source is non-zero.",
         .target = "{'scenes':[{'name':'T0', 'nodes':[0]}, "
                   "           {'name':'T1', 'nodes':[1]}], "
                   "'scene':0, "
                   "'nodes':[0, 1]}",  // We need nodes in target to get offset.
         .source = "{'scenes':[{'name':'S0', 'nodes':[0]}, "
                   "           {'name':'S1', 'nodes':[1]}], "
                   "'scene':1}",
         .array_name = "scenes",
         .expected = "[{'name':'T0', 'nodes':[0, 3]}, "
                     " {'name':'T1', 'nodes':[1]}]",
         .merge = merge},
        {.description = "Target scene is non-zero, source is zero.",
         .target = "{'scenes':[{'name':'T0', 'nodes':[0]}, "
                   "           {'name':'T1', 'nodes':[1]}], "
                   "'scene':1, "
                   "'nodes':[0, 1]}",  // We need nodes in target to get offset.
         .source = "{'scenes':[{'name':'S0', 'nodes':[0]}, "
                   "           {'name':'S1', 'nodes':[1]}], "
                   "'scene':0}",
         .array_name = "scenes",
         .expected = "[{'name':'T0', 'nodes':[0]}, "
                     " {'name':'T1', 'nodes':[1, 2]}]",
         .merge = merge},
        // Successful merging of extensions and extras.
        {.description = "Merged scenes with non-colliding extras merge.",
         .target = "{'scenes':[{'name':'S0', 'nodes':[0], "
                   "            'extras':{'one':{'a':1}}}],"
                   " 'nodes':[0, 1]}",
         .source = "{'scenes':[{'name':'T0', 'nodes':[0], "
                   "            'extras':{'v':1}}], "
                   " 'nodes':[0]}",
         .array_name = "scenes",
         .expected = "[{'name':'S0', 'nodes':[0, 2], "
                     "  'extras':{'v':1, 'one':{'a':1}}}]",
         .merge = merge},
        {.description = "Merged scenes with non-colliding extensions merge.",
         .target = "{'scenes':[{'name':'S0', 'nodes':[0], "
                   "            'extensions':{'one':{'a':1}}}], "
                   " 'nodes':[0, 1]}",
         .source = "{'scenes':[{'name':'T0', 'nodes':[0], "
                   "            'extensions':{'v':1}}], "
                   " 'nodes':[0]}",
         .array_name = "scenes",
         .expected = "[{'name':'S0', 'nodes':[0, 2], "
                     "  'extensions':{'v':1, 'one':{'a':1}}}]",
         .merge = merge},
    };
  }

  static vector<MergeCase> MergeNodesCases() {
    MergeFunction merge = MergeNodes;
    /* A "node" has indices to bump, "mesh" or "camera". But it may also contain
     an array of node indices, "children". "mesh" and "camera" can use the
     test case framework, but the "children" requires a bespoke test.

     Furthermore, we explicitly confirm removal of the "skin" element. */
    vector<MergeCase> cases{
        {.description = "Nodes in target cause children in 2 to offset.",
         .target = "{'nodes':[{'name':'A','children':[1]}, "
                   "{'name':'B','mesh':0}], 'meshes':[0, 1]}",
         .source = "{'nodes':[{'name':'C','children':[0, 1]}, {'name':'D'}]}",
         .array_name = "nodes",
         .expected = "[{'name':'A','children':[1]}, {'name':'B','mesh':0}, "
                     "{'name':'C','children':[2,3]}, {'name':'D'}]",
         .merge = merge},
        {.description = "Remove 'skin' child.",
         .target = "{}",
         .source = "{'nodes':[{'name':'C','skin':1}]}",
         .array_name = "nodes",
         .expected = "[{'name':'C'}]",
         .merge = merge},
    };

    BumpIndex(&cases, "nodes", "meshes", "mesh", merge);
    BumpIndex(&cases, "nodes", "cameras", "camera", merge);
    VerbatimCopy(&cases, "nodes", merge);
    return cases;
  }

  static vector<MergeCase> MergeMeshesCases() {
    MergeFunction merge = MergeMeshes;
    /* Meshes cannot use the test case creation utilities.
      1. Meshes delete two properties mesh.weights and
         mesh.primitives[i].targets.
      2. The indices that need to get bumped are not direct children of the
         "mesh", but are buried in primitives arrays that get visited.
      3. It has an unbounded set of "attributes" that get their indices
         bumped.
     So, these test cases get spelled out explicitly. */
    vector<MergeCase> cases{
        {.description = "Material index gets bumped.",
         .target = "{'materials':[0, 1]}",
         .source = "{'meshes':[{'name':'A', 'primitives':[{'material':2}]}]}",
         .array_name = "meshes",
         .expected = "[{'name':'A', 'primitives':[{'material':4}]}]",
         .merge = merge},
        {.description = "Vertex accessor gets bumped.",
         .target = "{'accessors':[0, 1]}",
         .source = "{'meshes':[{'name':'A', 'primitives':[{'indices':2}]}]}",
         .array_name = "meshes",
         .expected = "[{'name':'A', 'primitives':[{'indices':4}]}]",
         .merge = merge},
        {.description = "Primitives attributes get bumped.",
         .target = "{'accessors':[0, 1]}",
         .source =
             "{'meshes':[{'name':'A', 'primitives':[{'attributes':{'key1':1}}, "
             "{'attributes':{'key2':3, 'key4':5}}]}]}",
         .array_name = "meshes",
         .expected = "[{'name':'A', 'primitives':[{'attributes':{'key1':3}}, "
                     "{'attributes':{'key2':5, 'key4':7}}]}]",
         .merge = merge},
        {.description = "Non-index primitive fields propagate.",
         .target = "{'accessors':[0, 1]}",
         .source =
             "{'meshes':[{'primitives':[{'mode':17, 'extras':{'v':2}}]}]}",
         .array_name = "meshes",
         .expected = "[{'primitives':[{'mode':17, 'extras':{'v':2}}]}]",
         .merge = merge},
        {.description = "Targets get removed from primitives.",
         .target = "{}",
         .source =
             "{'meshes':[{'primitives':[{'mode':17, 'targets':{'v':2}}]}]}",
         .array_name = "meshes",
         .expected = "[{'primitives':[{'mode':17}]}]",
         .merge = merge},
        {.description = "Weights get stripped.",
         .target = "{'meshes':[{'name':'A'}]}",
         .source = "{'meshes':[{'name':'B', 'weights':[0, 1]}]}",
         .array_name = "meshes",
         .expected = "[{'name':'A'}, {'name':'B'}]",
         .merge = merge},
    };
    VerbatimCopy(&cases, "meshes", merge);
    return cases;
  }

  static vector<MergeCase> MergeMaterialsCases() {
    MergeFunction merge = MergeMaterials;
    /* Merging materials requires delving into the elements recursively. The
     indices to bump are not directly children of the "material" element. So,
     we test the bump logic directly. */
    vector<MergeCase> cases{
        {.description = "Texture indices get bumped on all materials.",
         .target = "{'textures':[0, 1]}",
         .source =
             "{'materials':[{'normalTexture':{'index':2}, "
             "'occlusionTexture':{'index':3}, 'emissiveTexture':{'index':4}, "
             "'pbrMetallicRoughness':{'baseColorTexture':{'index':5}, "
             "'metallicRoughnessTexture':{'index':6}}}, "
             "{'normalTexture':{'index':17}}]}",
         .array_name = "materials",
         .expected =
             "[{'normalTexture':{'index':4}, 'occlusionTexture':{'index':5}, "
             "'emissiveTexture':{'index':6}, "
             "'pbrMetallicRoughness':{'baseColorTexture':{'index':7}, "
             "'metallicRoughnessTexture':{'index':8}}}, "
             "{'normalTexture':{'index':19}}]",
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
    auto merge = [](json* j1, json&& j2) {
      MergeBuffers(j1, std::move(j2));
    };
    VerbatimCopy(&cases, "buffers", merge);
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
    auto merge = [](json* j1, json&& j2) {
      MergeImages(j1, std::move(j2));
    };
    BumpIndex(&cases, "images", "bufferViews", "bufferView", merge);
    VerbatimCopy(&cases, "images", merge);
    return cases;
  }

  static vector<MergeCase> MergeSamplersCases() {
    vector<MergeCase> cases;
    VerbatimCopy(&cases, "samplers", MergeSamplers);
    return cases;
  }
};

TEST_P(MergeTest, Evaluate) {
  const MergeCase& test_case = GetParam();
  auto swap_quotes = [](const string& in) {
    return std::regex_replace(in, std::regex("'"), string("\""));
  };
  json target = json::parse(swap_quotes(test_case.target));
  json source = json::parse(swap_quotes(test_case.source));
  test_case.merge(&target, std::move(source));
  const json expected = json::parse(swap_quotes(test_case.expected));
  EXPECT_EQ(target[test_case.array_name], expected) << test_case;
}

INSTANTIATE_TEST_SUITE_P(
    ExtensionsAndExtras, MergeTest,
    testing::ValuesIn(MergeTest::MergeExtensionsAndExtrasCases()));
INSTANTIATE_TEST_SUITE_P(
    ExtensionDeclarations, MergeTest,
    testing::ValuesIn(MergeTest::MergeExtensionsDeclarationCases()));
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

/* Specification of a failure test case. Merging only fails if there is a
 collision in merging "extras" or "extensions" in the glTF, "Scene", or "asset"
 elements. These test cases are simpler.

 To make the test case instantiation more compact, we write json using ' instead
 of ". This allows us to use simple string literals: "'key':'value'" instead of
 raw literals: ""key":"value"". The single quotes are swapped for double
 quotes in the Evaluate() test function. This applies to `target`, `source`, and
 `expected`. */
struct MergeFailureCase {
  /* A description of what is being tested -- displayed for test failure. */
  string description;
  /* A string representation of the json for a glTF file. This glTF will be
   modified. */
  string target;
  /* A string representation of the json for a glTF file. This glTF will be
   merged into target. */
  string source;
  /* A regular expression string to match against the expected exception
   message. */
  string expected_error;
  /* The merge function that should be called. */
  std::function<void(json*, json&&, const string&, MergeRecord*)> merge{};

  friend std::ostream& operator<<(std::ostream& out,
                                  const MergeFailureCase& merge_case) {
    out << merge_case.description;
    return out;
  }
};

/* The test harness for the merge failure due to collisions in "extras" or
 "extensions". This includes families of static functions that generate test
 cases. See MergeTest documentation for further elaboration on testing strategy.
 */
class MergeFailureTest : public testing::TestWithParam<MergeFailureCase> {
 public:
  /* The minutiae of the collision logic are tested through these test cases.
   Specifically, we test it via the "extensions" blob. We'll provide a token
   test for "extras" for evidence that it is calling the same API.

   Token tests are likewise provided in their own groups for merging scenes
   and assets. Just enough test to show that it's exercising the same API. */
  static vector<MergeFailureCase> MergeGltfCases() {
    /* Note: "asset" must exist for MergeGltf to work (note the fact that we
     have ended with the closing bracket; so extensions and extras are
     *siblings* of "asset", and children of glTF). */
    const string asset_prefix = "{'asset':{'version':'2.0'}";
    return vector<MergeFailureCase>{
        // Robust tests on extensions.
        {.description = "Target and source have non-object extensions",
         .target = asset_prefix + ", 'extensions':1}",
         .source = asset_prefix + ", 'extensions':2}",
         .expected_error =
             ".*glTF.extensions.*has a primitive.*has a primitive.",
         .merge = MergeGltf},
        {.description = "Target has primitive, source has object in extensions",
         .target = asset_prefix + ", 'extensions':1}",
         .source = asset_prefix + ", 'extensions':{'a':1}}",
         .expected_error = ".*glTF.extensions.*has a primitive.*has an object.",
         .merge = MergeGltf},
        {.description = "Target has object, source has primitive in extensions",
         .target = asset_prefix + ", 'extensions':{'b':2}}",
         .source = asset_prefix + ", 'extensions':1}",
         .expected_error = ".*glTF.extensions.*has an object.*has a primitive.",
         .merge = MergeGltf},
        {.description = "Shallow collision between extensions",
         .target = asset_prefix + ", 'extensions':{'b':2}}",
         .source = asset_prefix + ", 'extensions':{'b':3}}",
         .expected_error =
             ".*glTF.extensions.b.*defines it as 2.*defines it as 3.",
         .merge = MergeGltf},
        {.description = "Deep collision between extensions",
         .target = asset_prefix + ", 'extensions':{'b':{'a':1}}}",
         .source = asset_prefix + ", 'extensions':{'b':{'a':2}}}",
         .expected_error = ".*glTF.extensions.b.*defines it as "
                           ".\"a\":1.*defines it as .\"a\":2.*",
         .merge = MergeGltf},
        // Token test on extras.
        {.description = "Deep collision between extras",
         .target = asset_prefix + ", 'extras':{'b':{'a':1}}}",
         .source = asset_prefix + ", 'extras':{'b':{'a':2}}}",
         .expected_error = ".*glTF.extras.b.*defines it as "
                           ".\"a\":1.*defines it as .\"a\":2.*",
         .merge = MergeGltf},
    };
  }

  /* Minimum set of tests to provide evidence that merging extensions/extras in
   the "asset" element is using the same API as tested for merging glTF. */
  static vector<MergeFailureCase> MergeAssetCases() {
    /* Note: "asset" must exist for MergeGltf to work (note the fact that we
     haven't provided the closing bracket; so extensions and extras are
     *children* of "asset"). */
    const string asset_prefix = "{'asset':{'version':'2.0'";
    return vector<MergeFailureCase>{
        {.description = "Deep collision between asset.extras",
         .target = asset_prefix + ", 'extras':{'b':{'a':1}}}}",
         .source = asset_prefix + ", 'extras':{'b':{'a':2}}}}",
         .expected_error = ".*asset.extras.b.*defines it as "
                           ".\"a\":1.*defines it as .\"a\":2.*",
         .merge = MergeGltf},
        {.description = "Deep collision between asset.extensions",
         .target = asset_prefix + ", 'extensions':{'b':{'a':1}}}}",
         .source = asset_prefix + ", 'extensions':{'b':{'a':2}}}}",
         .expected_error = ".*asset.extensions.b.*defines it as "
                           ".\"a\":1.*defines it as .\"a\":2.*",
         .merge = MergeGltf},
    };
  }

  /* Minimum set of tests to provide evidence that merging extensions/extras in
   a Scene element is using the same API as tested for merging glTF. */
  static vector<MergeFailureCase> MergeSceneCases() {
    return vector<MergeFailureCase>{
        {.description = "Merged scenes with colliding extras.",
         .target = "{'scenes':[{'name':'T0', 'nodes':[0], "
                   "            'extras':{'one':{'a':1}}}], "
                   " 'nodes':[0, 1]}",
         .source = "{'scenes':[{'name':'S0', 'nodes':[0], "
                   "            'extras':{'one':2, 'v':1}}], "
                   " 'nodes':[0]}",
         .expected_error = ".*default_scene.extras.one.*defines it as "
                           ".\"a\":1.*defines it as 2.*",
         .merge = MergeDefaultScenes},
        {.description = "Merged scenes with colliding extensions.",
         .target = "{'scenes':[{'name':'T0', 'nodes':[0], "
                   "            'extensions':{'one':{'a':1}}}], "
                   " 'nodes':[0, 1]}",
         .source = "{'scenes':[{'name':'S0', 'nodes':[0], "
                   "            'extensions':{'one':2, 'v':1}}], "
                   " 'nodes':[0]}",
         .expected_error = ".*default_scene.extensions.one.*defines it as "
                           ".\"a\":1.*defines it as 2.*",
         .merge = MergeDefaultScenes},
    };
  }
};

INSTANTIATE_TEST_SUITE_P(Gltf, MergeFailureTest,
                         testing::ValuesIn(MergeFailureTest::MergeGltfCases()));
INSTANTIATE_TEST_SUITE_P(
    Asset, MergeFailureTest,
    testing::ValuesIn(MergeFailureTest::MergeAssetCases()));
INSTANTIATE_TEST_SUITE_P(
    Scene, MergeFailureTest,
    testing::ValuesIn(MergeFailureTest::MergeSceneCases()));

TEST_P(MergeFailureTest, Evaluate) {
  const MergeFailureCase& test_case = GetParam();
  auto swap_quotes = [](const string& in) {
    return std::regex_replace(in, std::regex("'"), string("\""));
  };
  json target = json::parse(swap_quotes(test_case.target));
  json source = json::parse(swap_quotes(test_case.source));
  const string target_name = "target_name";
  const string source_name = "source_name";
  // Before merging source into target, we need to register all of the target
  // extension/extra elements with the merge record. We'll merge it into a
  // minimal glTF and let the merging functionality record the data for us. This
  // will allow the subsequent collisions with source to be able to author
  // the exceptions properly.
  MergeRecord record("from empty");
  json merged_target{{"asset", {{"version", "2.0"}}}};
  test_case.merge(&merged_target, std::move(target), target_name, &record);

  DRAKE_EXPECT_THROWS_MESSAGE(
      test_case.merge(&merged_target, std::move(source), source_name, &record),
      test_case.expected_error);
}

/* Simply call MergeGltf on real glTF files and makes sure it doesn't throw. */
GTEST_TEST(MergeGltf, Smoke) {
  json target = ReadJsonFile(fs::path(FindResourceOrThrow(
      "drake/geometry/render_gltf_client/test/red_box.gltf")));

  json source = ReadJsonFile(fs::path(FindResourceOrThrow(
      "drake/geometry/render_gltf_client/test/textured_green_box.gltf")));

  MergeRecord record("test_target");
  EXPECT_NO_THROW(
      MergeGltf(&target, std::move(source), "test_source", &record));

  /* Save the merged glTF for inspection.
   bazel-testlogs/geometry/render_gltf_client/internal_merge_gltf_test/test.outputs/output.zip.
   */
  if (const char* dir = std::getenv("TEST_UNDECLARED_OUTPUTS_DIR")) {
    /* target now contains the merged result. */
    json& merged = target;
    std::ofstream f(fs::path(dir) / "merged.gltf");
    // Set the stream so the json gets "pretty-formatted" with a 2-space
    // indent.
    f << std::setw(2);
    f << merged;
  }
}

}  // namespace
}  // namespace internal
}  // namespace render_gltf_client
}  // namespace geometry
}  // namespace drake
