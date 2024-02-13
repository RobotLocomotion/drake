#include "drake/geometry/render_gltf_client/internal_merge_gltf.h"

#include <algorithm>
#include <fstream>
#include <functional>
#include <string>
#include <utility>

#include <common_robotics_utilities/base64_helpers.hpp>
#include <fmt/format.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/fmt_ostream.h"
#include "drake/common/ssize.h"
#include "drake/common/text_logging.h"

// For more explanation of the glTF fun and games:
// https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html

namespace drake {
namespace geometry {
namespace render_gltf_client {
namespace internal {

using json = nlohmann::json;
using std::string;

namespace {

// Returns the size of the named array, if present. Zero otherwise.
int ArraySize(const json& j, std::string_view array_name) {
  if (j.contains(array_name)) {
    DRAKE_DEMAND(j[array_name].is_array() || j[array_name].is_null());
    return ssize(j[array_name]);
  }
  return 0;
}

// If the json pointed to by `j_ptr` has a child with the given `name`, its
// integer value gets incremented by `offset`.
// @pre If present, (*j_ptr)[name] is an integer value.
void MaybeOffsetNamedIndex(json* j_ptr, std::string_view name, int offset) {
  json& j = *j_ptr;
  if (j.contains(name)) {
    int new_value = offset + j[name].get<int>();
    j[name] = new_value;
  }
}

// Same as MaybeOffsetNamedIndex(), but the child is an array of integers.
// @pre If present, (*j_ptr)[array_name] is an array of integers.
void MaybeOffsetIndexArray(json* j_ptr, std::string_view array_name,
                           int offset) {
  json& j = *j_ptr;
  if (j.contains(array_name)) {
    json& j_array = j[array_name];
    DRAKE_DEMAND(j_array.is_array() || j_array.is_null());
    for (int n = 0; n < ssize(j_array); ++n) {
      j_array[n] = j_array[n].get<int>() + offset;
    }
  }
}

// Walks the json tree rooted at `j_ptr` looking for int-valued descendent keys
// with the given `name`. Each such value found gets incremented by offset.
// @pre Every descendant with the given `name` is int-valued.
void MaybeOffsetNamedIndexInTree(json* j_ptr, std::string_view name,
                                 int offset) {
  if (j_ptr->is_object()) {
    MaybeOffsetNamedIndex(j_ptr, name, offset);
    for (auto& [key, value] : j_ptr->items()) {
      MaybeOffsetNamedIndexInTree(&value, name, offset);
    }
  }
}

// Merges *names* of extensions from j2 into j1 (preventing duplicates). This
// should not be used for merging an "extensions" property which contains
// arbitrary objects.
void MergeExtensionNames(json* j1, json&& j2, const std::string& array_name) {
  if (j2.contains(array_name)) {
    json& extensions1 = (*j1)[array_name];
    for (auto& extension2 : j2[array_name]) {
      if (std::any_of(extensions1.begin(), extensions1.end(),
                      [&extension2](const auto& value) {
                        return value == extension2;
                      })) {
        // Don't duplicate extension names.
        continue;
      }
      extensions1.push_back(std::move(extension2));
    }
  }
}

// Provides information about the scope for extra/extensions merging. It
// indicates the element that "contains" the extras/extensions being merged.
// Used to craft exception messages.
enum class ContainerType {
  // The root of the glTF data.
  kGltf,
  // The "asset" node under the root.
  kAsset,
  // The item in the "scenes" array being treated as the default scene.
  kDefaultScene,
};
std::string_view to_string(ContainerType x) {
  switch (x) {
    case ContainerType::kGltf:
      return "glTF";
    case ContainerType::kAsset:
      return "asset";
    case ContainerType::kDefaultScene:
      return "default_scene";
  }
  DRAKE_UNREACHABLE();
}

// Attempts to merge the key-value pairs from j2 into j1. To merge successfully,
// all every key in j2 that appears in j1 must have the same value. If
// successful, every key-value pair in j2 is in j1 upon return.
//
// Note: a "successful" merge could still leave j1 unchanged, because either
// there was nothing in j2, or j2's contents were already present in j1.
//
// @throws if merge was not successful.
// @pre j1->is_object() && j2.is_object().
void MergeTrees(json* j1, json&& j2, const std::string& blob_name,
                ContainerType container_type,
                const std::filesystem::path& j2_path, MergeRecord* record) {
  DRAKE_DEMAND(j1->is_object() && j2.is_object());
  // First confirm there are no collisions.
  for (auto& [key, value] : j2.items()) {
    if (j1->contains(key)) {
      if ((*j1)[key] != value) {
        const std::filesystem::path& j1_path = record->FindSourcePath(*j1);
        throw std::runtime_error(fmt::format(
            "Error in merging '{}.{}.{}'; two glTF files have different "
            "values. '{}' defines it as {}, but '{}' defines it as {}.",
            to_string(container_type), blob_name, key, j1_path.string(),
            fmt_streamed((*j1)[key]), j2_path.string(), fmt_streamed(value)));
      }
    }
  }
  // There were no collisions, merging will now proceed and be successful by
  // default.
  for (auto& [key, value] : j2.items()) {
    if (j1->contains(key)) continue;
    (*j1)[key] = std::move(value);
    record->AddElementTree((*j1)[key], j2_path);
  }
}

// Merge user data. This can be used for merging the arbitrary collection of
// objects contained in either "extras" or "extensions". As documented, if there
// is a collision between the two json trees, we will throw with helpful info.
void MergeBlobs(json* j1, json&& j2, const std::string& blob_name,
                ContainerType container_type,
                const std::filesystem::path& j2_path, MergeRecord* record) {
  if (j2.contains(blob_name)) {
    json& blob2 = j2[blob_name];
    if (blob2.is_null()) {
      // The "blob" key existed, but nothing was stored. We do nothing.
      return;
    }
    json& blob1 = (*j1)[blob_name];
    // Merging logic. There are a limited number of cases where we can merge:
    //  1. blobs 1 and 2 are both objects with no colliding key-value pairs.
    //     We can simply merge 2 into 1.
    //  2. blob1 is null
    //     We can simply take blob2 verbatim (whether object or primitive).
    //  3. In all other cases, merging is not possible.
    if (blob1.is_object() && blob2.is_object()) {
      MergeTrees(&blob1, std::move(blob2), blob_name, container_type, j2_path,
                 record);
    } else if (blob1.is_null()) {
      // j1 doesn't have the blob, go ahead and replace it with j2's.
      blob1 = std::move(blob2);
      record->AddElementTree(blob1, j2_path);
    } else {
      const std::filesystem::path& j1_path = record->FindSourcePath(blob1);
      throw std::runtime_error(fmt::format(
          "Error in merging '{}.{}'. To merge, the must both be objects. "
          "'{}' has {} and '{}' has {}.",
          to_string(container_type), blob_name, j1_path.string(),
          blob1.is_object() ? "an object" : "a primitive", j2_path.string(),
          blob2.is_object() ? "an object" : "a primitive"));
    }
  }
}

// Attempts to merge the "extras" and "extensions" object blocks from j2 to to
// j1. This only needs to be called where we're attempting to merge nodes:
// the root glTF node and Scene nodes. All other nodes simply get concatenated
// to lists.
void MergeExtrasAndExtensions(json* j1, json&& j2, ContainerType container_type,
                              const std::filesystem::path& j2_path,
                              MergeRecord* record) {
  MergeBlobs(j1, std::move(j2), "extras", container_type, j2_path, record);
  MergeBlobs(j1, std::move(j2), "extensions", container_type, j2_path, record);
}

// If `item_inout` has a field named `uri` and it is not a `data:` URI, replaces
// the field's value with a base64-encoded `data:` URI.
//
// In glTF 2.0, URIs can only appear in two places:
//  "images": [ { "uri": "some.png" } ]
//  "buffers": [ { "uri": "some.bin", "byteLength": 1024 } ]
//
// When merging a glTF, we expect that our images- and buffers-handling logic
// must call this function as a subroutine.
void MaybeEmbedDataUri(nlohmann::json* item_inout,
                       const std::filesystem::path& base_path) {
  DRAKE_DEMAND(item_inout != nullptr);
  nlohmann::json& item = *item_inout;
  if (!item.contains("uri")) {
    return;
  }
  const std::string_view uri = item["uri"].template get<std::string_view>();
  if (uri.substr(0, 5) == "data:") {
    return;
  }
  const std::string content = ReadFileOrThrow(base_path / uri);
  item["uri"] =
      fmt::format("data:application/octet-stream;base64,{}",
                  common_robotics_utilities::base64_helpers::Encode(
                      std::vector<uint8_t>(content.begin(), content.end())));
}

}  // namespace

MergeRecord::MergeRecord(std::filesystem::path initial_path) {
  source_paths_.push_back(std::move(initial_path));
}

const std::filesystem::path& MergeRecord::FindSourcePath(
    const json& element) const {
  const auto iter = merged_trees_.find(&element);
  DRAKE_DEMAND(iter != merged_trees_.end());
  return source_paths_.at(iter->second);
}

void MergeRecord::AddElementTree(const json& root,
                                 const std::filesystem::path& source_path) {
  const int source_index = ssize(source_paths_);
  source_paths_.push_back(source_path);
  // Recursively register all of the json elements in the tree rooted at
  // `subtree`.
  std::function<void(const json&)> add_tree_recurse = [&](const json& subtree) {
    this->merged_trees_[&subtree] = source_index;
    if (subtree.is_object() || subtree.is_array()) {
      for (const auto& child : subtree) {
        add_tree_recurse(child);
      }
    }
  };
  add_tree_recurse(root);
}

json ReadJsonFile(const std::filesystem::path& json_path) {
  std::ifstream f(json_path);
  return json::parse(f);
}

json GltfMatrixFromEigenMatrix(const Matrix4<double>& matrix) {
  json result;
  // For glTF, transform matrix is a *column-major* matrix.
  for (int c = 0; c < 4; ++c) {
    for (int r = 0; r < 4; ++r) {
      result.push_back(matrix(r, c));
    }
  }
  return result;
}

Matrix4<double> EigenMatrixFromGltfMatrix(const json& matrix_json) {
  Matrix4<double> T;
  // For glTF, transform matrix is a *column-major* matrix.
  int i = -1;
  for (int c = 0; c < 4; ++c) {
    for (int r = 0; r < 4; ++r) {
      T(r, c) = matrix_json[++i].get<double>();
    }
  }
  return T;
}

void MergeDefaultScenes(json* j1, json&& j2,
                        const std::filesystem::path& j2_path,
                        MergeRecord* record) {
  int index_1 = j1->contains("scene") ? (*j1)["scene"].get<int>() : 0;
  int index_2 = j2.contains("scene") ? j2["scene"].get<int>() : 0;
  json& scene_1 = (*j1)["scenes"][index_1];
  json& scene_2 = j2["scenes"][index_2];
  if (scene_2.contains("nodes")) {
    const int node_offset = ArraySize(*j1, "nodes");
    // Offset all node indices in j2's scene.
    MaybeOffsetIndexArray(&scene_2, "nodes", node_offset);
    // Merge j2's scene's nodes into the j1's scene's nodes.
    json& nodes_1 = scene_1["nodes"];
    for (auto& n : scene_2["nodes"]) {
      nodes_1.push_back(std::move(n));
    }
  }
  MergeExtrasAndExtensions(&scene_1, std::move(scene_2),
                           ContainerType::kDefaultScene, j2_path, record);
}

void MergeNodes(json* j1, json&& j2) {
  if (j2.contains("nodes")) {
    json& nodes = (*j1)["nodes"];
    // Offsets to update used indices.
    const int node_offset = ArraySize(*j1, "nodes");
    const int mesh_offset = ArraySize(*j1, "meshes");
    const int cam_offset = ArraySize(*j1, "cameras");
    for (auto& node : j2["nodes"]) {
      MaybeOffsetNamedIndex(&node, "mesh", mesh_offset);
      MaybeOffsetNamedIndex(&node, "camera", cam_offset);
      MaybeOffsetIndexArray(&node, "children", node_offset);
      // We're not merging skins, so, drop the value.
      node.erase("skin");
      // The transform of the node is copied without change.
      nodes.push_back(std::move(node));
    }
  }
}

void MergeExtensionsUsed(json* j1, json&& j2) {
  MergeExtensionNames(j1, std::move(j2), "extensionsUsed");
}

void MergeExtensionsRequired(json* j1, json&& j2) {
  MergeExtensionNames(j1, std::move(j2), "extensionsRequired");
}

void MergeMeshes(json* j1, json&& j2) {
  if (j2.contains("meshes")) {
    json& meshes = (*j1)["meshes"];
    // Offsets to update used indices.
    const int mat_offset = ArraySize(*j1, "materials");
    const int index_offset = ArraySize(*j1, "accessors");
    for (auto& mesh : j2["meshes"]) {
      // We're not supporting animation, morph targets, or skinning; erase
      // "weights" and "targets".
      mesh.erase("weights");
      if (mesh.contains("primitives")) {
        for (auto& prim : mesh["primitives"]) {
          prim.erase("targets");
          MaybeOffsetNamedIndex(&prim, "material", mat_offset);
          MaybeOffsetNamedIndex(&prim, "indices", index_offset);
          if (prim.contains("attributes")) {
            json& attr = prim["attributes"];
            for (auto& [key, _] : attr.items()) {
              MaybeOffsetNamedIndex(&attr, key, index_offset);
            }
          }
        }
      }
      meshes.push_back(std::move(mesh));
    }
  }
}

void MergeMaterials(json* j1, json&& j2) {
  if (j2.contains("materials")) {
    json& materials = (*j1)["materials"];
    // Offsets to update used indices.
    const int tex_offset = ArraySize(*j1, "textures");
    for (auto& mat : j2["materials"]) {
      // We have to walk the tree looking for "index" keys; these should be
      // part of textures and should offset according to j1's texture count.
      MaybeOffsetNamedIndexInTree(&mat, "index", tex_offset);
      materials.push_back(std::move(mat));
    }
  }
}

void MergeCameras(json* j1, json&& j2) {
  if (j2.contains("cameras")) {
    json& cameras = (*j1)["cameras"];
    for (auto& cam : j2["cameras"]) {
      // Cameras can simply be copied over.
      cameras.push_back(std::move(cam));
    }
  }
}

void MergeAccessors(json* j1, json&& j2) {
  if (j2.contains("accessors")) {
    json& accessors = (*j1)["accessors"];
    // Offsets to update used indices.
    const int buf_offset = ArraySize(*j1, "bufferViews");
    for (auto& acc : j2["accessors"]) {
      MaybeOffsetNamedIndex(&acc, "bufferView", buf_offset);
      accessors.push_back(std::move(acc));
    }
  }
}

void MergeBufferViews(json* j1, json&& j2) {
  if (j2.contains("bufferViews")) {
    json& bufferViews = (*j1)["bufferViews"];
    // Offsets to update used indices.
    const int buf_offset = ArraySize(*j1, "buffers");
    for (auto& bv : j2["bufferViews"]) {
      MaybeOffsetNamedIndex(&bv, "buffer", buf_offset);
      bufferViews.push_back(std::move(bv));
    }
  }
}

void MergeBuffers(json* j1, json&& j2,
                  const std::filesystem::path& j2_base_path) {
  if (j2.contains("buffers")) {
    json& buffers = (*j1)["buffers"];
    for (auto& buffer : j2["buffers"]) {
      MaybeEmbedDataUri(&buffer, j2_base_path);
      buffers.push_back(std::move(buffer));
    }
  }
}

void MergeTextures(json* j1, json&& j2) {
  if (j2.contains("textures")) {
    json& textures = (*j1)["textures"];
    // Offsets to update used indices.
    const int source_offset = ArraySize(*j1, "images");
    const int sampler_offset = ArraySize(*j1, "samplers");
    for (auto& texture : j2["textures"]) {
      MaybeOffsetNamedIndex(&texture, "source", source_offset);
      MaybeOffsetNamedIndex(&texture, "sampler", sampler_offset);
      textures.push_back(std::move(texture));
    }
  }
}

void MergeImages(json* j1, json&& j2,
                 const std::filesystem::path& j2_base_path) {
  if (j2.contains("images")) {
    json& images = (*j1)["images"];
    // Offsets to update used indices.
    const int buf_offset = ArraySize(*j1, "bufferViews");
    for (auto& image : j2["images"]) {
      MaybeOffsetNamedIndex(&image, "bufferView", buf_offset);
      MaybeEmbedDataUri(&image, j2_base_path);
      images.push_back(std::move(image));
    }
  }
}

void MergeSamplers(json* j1, json&& j2) {
  // TODO(SeanCurtis-TRI): This may include samplers for animations that don't
  // get merged. If we don't merge animations, it would be better to omit
  // the samplers. However, samplers are small and unused samplers are probably
  // harmless.
  if (j2.contains("samplers")) {
    json& samplers = (*j1)["samplers"];
    for (auto& sampler : j2["samplers"]) {
      // Samplers can simply be copied over.
      samplers.push_back(std::move(sampler));
    }
  }
}

void MergeGltf(json* j1, json&& j2, const std::filesystem::path& j2_path,
               MergeRecord* record) {
  json& asset1 = (*j1)["asset"];
  json& asset2 = j2["asset"];
  DRAKE_DEMAND(!(asset1.is_null() || asset2.is_null()));
  const std::filesystem::path j2_directory = j2_path.parent_path();

  asset1["generator"] = "Drake glTF merger";
  // TODO(SeanCurtis-TRI): We're not doing anything to the copyright. Should we?
  DRAKE_DEMAND(asset1["version"].get<string>() == "2.0");
  DRAKE_DEMAND(asset2["version"].get<string>() == "2.0");
  MergeExtrasAndExtensions(j1, std::move(j2), ContainerType::kGltf, j2_path,
                           record);
  MergeExtrasAndExtensions(&asset1, std::move(asset2), ContainerType::kAsset,
                           j2_path, record);

  // Don't change the order. Because we mutate j1 as we go, we need to make sure
  // we only mutate something after we've processed everything that depends on
  // it.
  // https://github.com/KhronosGroup/glTF-Tutorials/blob/master/gltfTutorial/gltfTutorial_002_BasicGltfStructure.md
  MergeDefaultScenes(j1, std::move(j2), j2_path, record);
  MergeNodes(j1, std::move(j2));
  MergeMeshes(j1, std::move(j2));
  MergeMaterials(j1, std::move(j2));
  MergeCameras(j1, std::move(j2));
  MergeAccessors(j1, std::move(j2));
  MergeTextures(j1, std::move(j2));
  MergeImages(j1, std::move(j2), j2_directory);
  MergeSamplers(j1, std::move(j2));
  MergeBufferViews(j1, std::move(j2));
  MergeBuffers(j1, std::move(j2), j2_directory);
  MergeExtensionsUsed(j1, std::move(j2));
  MergeExtensionsRequired(j1, std::move(j2));

  // NOTE: For now we're omitting skins, animations and morphs.
}

}  // namespace internal
}  // namespace render_gltf_client
}  // namespace geometry
}  // namespace drake
