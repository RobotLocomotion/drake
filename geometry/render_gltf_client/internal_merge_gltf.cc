#include "drake/geometry/render_gltf_client/internal_merge_gltf.h"

#include <algorithm>
#include <fstream>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <fmt/format.h>

#include "drake/common/drake_assert.h"
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

// Attempts to merge the key-value pairs from j2 into j1. The merging is
// "successful" if every key-value pair in j2 is in j1 upon return. An
// "unsuccessful" merging means j1 is unchanged. The merging will be
// unsuccessful if any key in j2 is already present in j1, but their respective
// values differ.
//
// Note: a "successful" merge could still leave j1 unchanged, because either
// there was nothing in j2, or j2's contents were already present in j1.
//
// @returns true if the merge was successful.
// @pre j1->is_object() && j2.is_object().
bool MergeTrees(json* j1, json&& j2) {
  DRAKE_DEMAND(j1->is_object() && j2.is_object());
  // First confirm
  for (auto& [key, value] : j2.items()) {
    if (j1->contains(key)) {
      if ((*j1)[key] != value) {
        return false;
      }
      continue;
    }
  }
  // There were no collisions, merging will no proceed and be successful by
  // default.
  for (auto& [key, value] : j2.items()) {
    if (j1->contains(key)) continue;
    (*j1)[key] = std::move(value);
  }
  return true;
}

// Merge user data. This can be used for merging the arbitrary collection of
// objects contained in either "extras" or "extensions". As documented, if there
// is a collision between the two json trees, j2's values will be omitted.
void MergeBlobs(json* j1, json&& j2, const std::string& blob_name,
                const char* container_type) {
  if (j2.contains(blob_name)) {
    json& blob2 = j2[blob_name];
    if (blob2.is_null()) {
      // The "blob" key existed, but nothing was stored. We do nothing.
      return;
    }
    json& blob1 = (*j1)[blob_name];
    // Merging logic. There are a limited number of cases where we can merge:
    //  1. blob1 is null
    //     We can simply take blob2 verbatim (whether object or primitive).
    //  2. blobs 1 and 2 are both objects with no colliding key-value pairs.
    //     We can simply merge 2 into 1.
    //  3. In all other cases, merging is not possible.
    if (blob1.is_object() && blob2.is_object()) {
      if (!MergeTrees(&blob1, std::move(blob2))) {
        log()->warn(
            "Attempting to merge the \"{0}\" between two {1} objects; there "
            "were collisions in the \"{0}\" -- same key name but different "
            "values. The second glTF's \"{0}\" were ignored.",
            blob_name, container_type);
      }
    } else if (blob1.is_null()) {
      // j1 doesn't have the blob, go ahead and replace it with j2's.
      blob1 = std::move(blob2);
    } else {
      log()->warn(
          "Attempting to merge the \"{0}\" between two {1} objects; one "
          "holds an object, the other a primitive. They cannot be merged. The "
          "\"{0}\" will not be merged.",
          blob_name, container_type);
    }
  }
}

// Attempts to merge the "extras" and "extensions" object blocks from j2 to to
// j1. This only needs to be called where we're attempting to merge nodes:
// the root glTF node and Scene nodes. All other nodes simply get concatenated
// to lists.
void MergeExtrasAndExtensions(json* j1, json&& j2, const char* container_type) {
  MergeBlobs(j1, std::move(j2), "extras", container_type);
  MergeBlobs(j1, std::move(j2), "extensions", container_type);
}

}  // namespace

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

void MergeScenes(json* j1, json&& j2) {
  std::map<string, int> names_in_1;
  json& j1_scenes = (*j1)["scenes"];
  // Identify all of the named scenes in j1. Names are optional.
  for (int s = 0; s < ssize(j1_scenes); ++s) {
    const json& scene = j1_scenes[s];
    if (scene.contains("name")) {
      names_in_1.insert({scene["name"].get<string>(), s});
    }
  }

  // For each scene in j2, update its node values and either merge scenes or
  // append scenes.
  if (j2.contains("scenes")) {
    const int node_offset = ArraySize(*j1, "nodes");
    for (auto& scene : j2["scenes"]) {
      // Offset all node indices in this scene.
      MaybeOffsetIndexArray(&scene, "nodes", node_offset);

      // If the scene needs to be merged, merge it.
      if (scene.contains("name")) {
        const string& name = scene["name"].get<string>();
        if (names_in_1.count(name) > 0) {
          json& j1_scene1 = j1_scenes[names_in_1[name]];
          if (scene.contains("nodes")) {
            // Merge this scene's nodes into the named scene.
            json& nodes = j1_scene1["nodes"];
            for (auto& n : scene["nodes"]) {
              nodes.push_back(std::move(n));
            }
            // TODO(SeanCurtis-TRI): This omits the extensions and extras that
            // may be stored with j2's scene. Consider merging them as well.
            // It's unclear what to do if two identically named scenes have the
            // same extras property, but different values. Probably just throw.
            // Note: where j2's scene doesn't require merging, those properties
            // are copied automatically.
          }
          MergeExtrasAndExtensions(&j1_scene1, std::move(scene), "scene");
          // We've merged; no further required for this scene.
          continue;
        }
      }

      // We didn't merge the scenes, so we just need to append.
      j1_scenes.push_back(std::move(scene));
    }
  }
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

void MergeBuffers(json* j1, json&& j2) {
  if (j2.contains("buffers")) {
    json& buffers = (*j1)["buffers"];
    for (auto& bv : j2["buffers"]) {
      // Buffers can simply be copied over.
      buffers.push_back(std::move(bv));
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

void MergeImages(json* j1, json&& j2) {
  if (j2.contains("images")) {
    json& images = (*j1)["images"];
    // Offsets to update used indices.
    const int buf_offset = ArraySize(*j1, "bufferViews");
    for (auto& image : j2["images"]) {
      MaybeOffsetNamedIndex(&image, "bufferView", buf_offset);
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

void MergeGltf(json* j1, json&& j2) {
  json& asset1 = (*j1)["asset"];
  json& asset2 = j2["asset"];
  DRAKE_DEMAND(!(asset1.is_null() && asset2.is_null()));

  asset1["generator"] = "Drake glTF merger";
  // TODO(SeanCurtis-TRI): We're not doing anything to the copyright. Should we?
  DRAKE_DEMAND(asset1["version"].get<string>() == "2.0");
  DRAKE_DEMAND(asset2["version"].get<string>() == "2.0");
  MergeExtrasAndExtensions(j1, std::move(j2), "glTF");
  MergeExtrasAndExtensions(&asset1, std::move(asset2), "asset");

  // Don't change the order. Because we mutate j1 as we go, we need to make sure
  // we only mutate something after we've processed everything that depends on
  // it.
  // https://github.com/KhronosGroup/glTF-Tutorials/blob/master/gltfTutorial/gltfTutorial_002_BasicGltfStructure.md
  MergeScenes(j1, std::move(j2));
  MergeNodes(j1, std::move(j2));
  MergeMeshes(j1, std::move(j2));
  MergeMaterials(j1, std::move(j2));
  MergeCameras(j1, std::move(j2));
  MergeAccessors(j1, std::move(j2));
  MergeTextures(j1, std::move(j2));
  MergeImages(j1, std::move(j2));
  MergeSamplers(j1, std::move(j2));
  MergeBufferViews(j1, std::move(j2));
  MergeBuffers(j1, std::move(j2));
  MergeExtensionsUsed(j1, std::move(j2));
  MergeExtensionsRequired(j1, std::move(j2));

  // NOTE: For now we're omitting skins, animations and morphs.
}

}  // namespace internal
}  // namespace render_gltf_client
}  // namespace geometry
}  // namespace drake
