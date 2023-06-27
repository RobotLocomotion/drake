#pragma once

#include <filesystem>

#include <drake_vendor/nlohmann/json.hpp>

namespace drake {
namespace geometry {
namespace render_gltf_client {
namespace internal {

/* Returns the parsed result of the indicated json file. */
nlohmann::json ReadJsonFile(const std::filesystem::path& json_path);

/* Note: glTF interrelates elements of arrays by having one element refer
 to another by the other's index into the array. For all of these merging
 functions, when the array elements have such indices, the merged index values
 get incremented by the size of the target array before merging.
*/

/* Merges the "scenes" array from j2 into j1.

 Upon return, j1 will contain the following scenes:

   - Each scene from j1 and j2 whose name only appears in one of the files.
   - For scenes in j1 and j2 with the same name, the node list of j2's scene
     appended to the scene in j1. Any extras or extensions in j2's scene will
     be lost. */
void MergeScenes(nlohmann::json* j1, nlohmann::json&& j2);

/* Merges the "nodes" array from j2 into j1. */
void MergeNodes(nlohmann::json* j1, nlohmann::json&& j2);

/* Merges the "meshes" array from j2 into j1. */
void MergeMeshes(nlohmann::json* j1, nlohmann::json&& j2);

/* Merges the "materials" array from j2 into j1. */
void MergeMaterials(nlohmann::json* j1, nlohmann::json&& j2);

/* Merges the "cameras" array from j2 into j1. */
void MergeCameras(nlohmann::json* j1, nlohmann::json&& j2);

/* Merges the "accessors" array from j2 into j1. */
void MergeAccessors(nlohmann::json* j1, nlohmann::json&& j2);

/* Merges the "bufferViews" array from j2 into j1. */
void MergeBufferViews(nlohmann::json* j1, nlohmann::json&& j2);

/* Merges the "buffers" array from j2 into j1. */
void MergeBuffers(nlohmann::json* j1, nlohmann::json&& j2);

/* Merges the "textures" array from j2 into j1. */
void MergeTextures(nlohmann::json* j1, nlohmann::json&& j2);

/* Merges the "images" array from j2 into j1. */
void MergeImages(nlohmann::json* j1, nlohmann::json&& j2);

/* Merges the "samplers" array from j2 into j1. */
void MergeSamplers(nlohmann::json* j1, nlohmann::json&& j2);

// TODO(SeanCurtis-TRI): For elements that aren't copied over (e.g. animation),
// their corresponding *data* should be removed from the buffer(s). This is an
//  awkward task because:
//  a. We need to cut out the right part of the data (go from encoded string
//    to bytes and back again).
//  b. All bufferViews that reference the buffer at higher byte addresses need
//     to be identified and offset.

/* Merges the glTF data stored in the json j2 into j1.

 This explicitly excludes skin data, animation, and morph target elements
 (although the underlying data contained in buffers remains).

 All index references in j2's elements are updated based on the number of
 indices already used by j1.

 @pre Both j1 and j2 indicate version 2.0 glTF files. */
void MergeGltf(nlohmann::json* j1, nlohmann::json&& j2);

}  // namespace internal
}  // namespace render_gltf_client
}  // namespace geometry
}  // namespace drake
