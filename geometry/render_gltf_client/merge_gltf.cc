#include "drake/geometry/render_gltf_client/merge_gltf.h"

#include <fstream>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <fmt/format.h>

#include "drake/common/drake_assert.h"
#include "drake/common/fmt_ostream.h"
#include "drake/common/ssize.h"

// For more explanation of the gltf fun and games:
// https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html

namespace drake {
namespace geometry {
namespace render_gltf_client {
namespace internal {

using json = nlohmann::json;
using std::string;

#define PRINT(j) fmt::print("{}: {}\n", #j, fmt_streamed(j))

namespace {

// Returns the ize of the named array
int ArraySize(const json& j, std::string_view array_name) {
  if (j.contains(array_name)) {
    DRAKE_DEMAND(j[array_name].is_array() || j[array_name].is_null());
    return ssize(j[array_name]);
  }
  return 0;
}

void MaybeOffsetNamedIndex(json* j_ptr, std::string_view name, int offset) {
  json& j = *j_ptr;
  if (j.contains(name)) {
    int new_value = offset + j[name].get<int>();
    j[name] = new_value;
  }
}

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

// Walks the json tree rooted at j_ptr looking for int-valued keys with the
// given `name`. Each such value found gets incremented by offset.
void MaybeOffsetNamedIndexInTree(json* j_ptr, std::string_view name,
                                 int offset) {
  MaybeOffsetNamedIndex(j_ptr, name, offset);
  for (auto& [key, value] : j_ptr->items()) {
    // N.B. Be careful with this recursive call, if the `value` is a leaf (e.g,,
    // an int, string, or bool), calling items() on it will return a single
    // pair of a null key with itself. It's a stackoverflow bomb!
    if (value.is_object() || value.is_array()) {
      // Don't bother recursing if this isn't a container.
      MaybeOffsetNamedIndexInTree(&value, name, offset);
    }
  }
}

}  // namespace

json ReadJsonFile(const std::filesystem::path& json_path) {
  std::ifstream f(json_path);
  return json::parse(f);
}

void MergeScenes(json* j1, json&& j2) {
  std::map<string, int> names_in_1;
  json& j1_scenes = (*j1)["scenes"];
  // Identify all of the named scenes in j1.
  for (int s = 0; s < ssize(j1_scenes); ++s) {
    const json& scene = j1_scenes[s];
    names_in_1.insert({scene["name"].get<string>(), s});
  }

  // For each scene in j2, update its node values and either merge scenes or
  // append scenes.
  if (j2.contains("scenes")) {
    const int node_offset = ArraySize(*j1, "nodes");
    for (auto& scene : j2["scenes"]) {
      // Offset all node indices in this scene.
      MaybeOffsetIndexArray(&scene, "nodes", node_offset);

      const string& name = scene["name"].get<string>();
      if (names_in_1.count(name) > 0) {
        if (scene.contains("nodes")) {
          // Merge this scene's nodes into the named scene.
          json& nodes = j1_scenes[names_in_1[name]]["nodes"];
          for (auto& n : scene["nodes"]) {
            nodes.push_back(std::move(n));
          }
          // TODO(SeanCurtis-TRI): This omits the extensions and extras that
          // may be stored with j2's scene. Consider merging them as well. It's
          // unclear what to do if two identically named scenes have the same
          // extras property, but different values.
          // Note: where j2's scene doesn't require merging, those properties
          // are copied automatically.
        }
      } else {
        // Simply append the uniquely named scene to scenes.
        j1_scenes.push_back(std::move(scene));
      }
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

void MergeMeshes(json* j1, json&& j2) {
  if (j2.contains("meshes")) {
    json& meshes = (*j1)["meshes"];
    // Offsets to update used indices.
    const int mat_offset = ArraySize(*j1, "materials");
    const int index_offset = ArraySize(*j1, "accessors");
    for (auto& mesh : j2["meshes"]) {
      // We're not supporting animation, morph targets, or skinning.
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
    const int image_offset = ArraySize(*j1, "images");
    const int sampler_offset = ArraySize(*j1, "samplers");
    for (auto& texture : j2["textures"]) {
      MaybeOffsetNamedIndex(&texture, "source", image_offset);
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
  (*j1)["asset"]["generator"] = "Drake gltf merger";

  // Don't change the order. Because we mutate j1 as we go, we need to make sure
  // we only mutate something after we've processed everything that depends on
  // it.
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

  // TODO(SeanCurtis-TRI): Handle the version better. Requirements:
  //  1. j1 should be empty or should have j1["asset"]["version"] == "2.0"
  //  2. j2 should be empty or should have j2["asset"]["version"] == "2.0"
  //  3. Output should have j1["asset"]["version"] = "2.0"
  (*j1)["asset"]["version"] = "2.0";

  // NOTE: For now we're omitting skins and animations.
}

}  // namespace internal
}  // namespace render_gltf_client
}  // namespace geometry
}  // namespace drake
