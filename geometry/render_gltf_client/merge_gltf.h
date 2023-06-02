#pragma once

#include <filesystem>

#include <drake_vendor/nlohmann/json.hpp>

namespace drake {
namespace geometry {
namespace render_gltf_client {
namespace internal {

nlohmann::json ReadJsonFile(const std::filesystem::path& json_path);

/* Merges the scenes from j2 into j1.

 Upon return, j1 will contain the following scenes:

   - Each of the scenes in j1 with a name that isn't shared by j2.
   - Each of the scenes in j2 with a name that isn't shared by j1.
   - For scenes in j1 and j2 with the same name, the node list of j2's scene
     will be offset by the number of nodes in j1 and appended to the node array
     of j1's identically named scene. */
void MergeScenes(nlohmann::json* j1, nlohmann::json&& j2);

/* Merges the nodes from j2 into j1.

 All index references in j2's elements are updated based on the number of
 indices already used by j1. */
void MergeNodes(nlohmann::json* j1, nlohmann::json&& j2);

/* Merges the meshes from j2 into j1.

 All index references in j2's elements are updated based on the number of
 indices already used by j1. */
void MergeMeshes(nlohmann::json* j1, nlohmann::json&& j2);

/* Merges the materials from j2 into j1.

 All index references in j2's elements are updated based on the number of
 indices already used by j1. */
void MergeMaterials(nlohmann::json* j1, nlohmann::json&& j2);

/* Merges the cameras from j2 into j1.

 All index references in j2's elements are updated based on the number of
 indices already used by j1. */
void MergeCameras(nlohmann::json* j1, nlohmann::json&& j2);

/* Merges the accessors from j2 into j1.

 All index references in j2's elements are updated based on the number of
 indices already used by j1. */
void MergeAccessors(nlohmann::json* j1, nlohmann::json&& j2);

/* Merges the bufferViews from j2 into j1.

 All index references in j2's elements are updated based on the number of
 indices already used by j1. */
void MergeBufferViews(nlohmann::json* j1, nlohmann::json&& j2);

/* Merges the buffers from j2 into j1.

 All index references in j2's elements are updated based on the number of
 indices already used by j1. */
void MergeBuffers(nlohmann::json* j1, nlohmann::json&& j2);

/* Merges the textures from j2 into j1.

 All index references in j2's elements are updated based on the number of
 indices already used by j1. */
void MergeTextures(nlohmann::json* j1, nlohmann::json&& j2);

/* Merges the images from j2 into j1.

 All index references in j2's elements are updated based on the number of
 indices already used by j1. */
void MergeImages(nlohmann::json* j1, nlohmann::json&& j2);

/* Merges the images from j2 into j1.

 All index references in j2's elements are updated based on the number of
 indices already used by j1. */
void MergeSamplers(nlohmann::json* j1, nlohmann::json&& j2);

// TODO(SeanCurtis-TRI): For elements that aren't copied over (e.g. animation),
// its corresponding *data* should be removed from the buffer(s). This is an
//  awkward task because:
//  a. We need to cut out the right part of the data (go from encoded string
//    to bytes and back again).
//  b. All bufferViews that reference the buffer at higher byte addresses need
//     to be identified and offset.

/* Merges the j2 into j1.

 This explicitly excludes skin data, animation, and morph target elements
 (although the underlying data contained in buffers remains).

 All index references in j2's elements are updated based on the number of
 indices already used by j1. */
void MergeGltf(nlohmann::json* j1, nlohmann::json&& j2);

}  // namespace internal
}  // namespace render_gltf_client
}  // namespace geometry
}  // namespace drake
