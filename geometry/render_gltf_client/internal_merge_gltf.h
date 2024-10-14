#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include <nlohmann/json.hpp>

#include "drake/common/drake_export.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/mesh_source.h"

namespace drake {
namespace geometry {
namespace render_gltf_client {
namespace internal {

/* Returns the parsed result of the json file indicated by the given source. */
nlohmann::json ReadJsonFile(const MeshSource& mesh_source);

/* Creates the glTF matrix array from the given 4x4 matrix.
 @returns A json list of 16 number values. */
nlohmann::json GltfMatrixFromEigenMatrix(const Matrix4<double>& matrix);

/* Creates a 4x4 matrix from the the glTF json entry representing the matrix.
 @param matrix_json  A list of 16 number values. */
Matrix4<double> EigenMatrixFromGltfMatrix(const nlohmann::json& matrix_json);

/* Note: glTF interrelates elements of arrays by having one element refer
 to another by the other's index into the array. For all of these merging
 functions, when the array elements have such indices, the merged index values
 get incremented by the size of the target array before merging.
*/

/* We attempt to merge extensions and extras for the glTF, scene, and asset
 elements. If there is a conflict in the values, we report the conflict. This
 struct provides a record of where merged extras and extensions come from so
 we can effectively report the source of the conflict. */
class DRAKE_NO_EXPORT MergeRecord {
 public:
  /* The initial merge record should be created with the name of the initial
   target glTF's source. */
  explicit MergeRecord(std::string initial_name);

  ~MergeRecord();

  /* Finds the source name for the given element.
   @pre element is part of this merge record. */
  const std::string& FindSourceName(const nlohmann::json& element) const;

  /* Adds the json tree with the given `root` to the record associated with the
   given `source_name`. `source_name` will appear in error messages if there
   is a problem with an element, so should be name that is meaningful to the
   end user. */
  void AddElementTree(const nlohmann::json& root,
                      const std::string& source_name);

 private:
  /* A map from a json pointer in the target glTF structure to the index of the
   source name from which it came. The mapped values should all be valid
   indices into `source_names_`.

   Note: This works because nlohmann::json is linked-list-esque. Each node is
   allocated on the heap and they don't move just because additional children
   get included. */
  std::unordered_map<const nlohmann::json*, int> merged_trees_;

  /* The names of all sources contributing to the composition of j1. */
  std::vector<std::string> source_names_;
};

/* Merges the default scene from j2 into j1's default scene.

 For each glTF source, the default scene is defined by the optional "scenes"
 property. If undefined, it is interpreted as zero. It will also attempt to
 merge the extras and extensions between the two scenes.

 @param j1       The glTF root element.
 @param j2       The glTF root element.
 @param j2_name  A label to use for referring to the glTF in j2 in error
                 messages if problems are encountered while processing j2.
                 If multiple glTF files are merged, each should have a unique
                 name.
 @param record   The accumulated record of the merge.
 @pre Both j1 and j2 have a valid default scene.
 @throws if there are merge conflicts between the scenes' "extra" or
 "extensions" data. */
void MergeDefaultScenes(nlohmann::json* j1, nlohmann::json&& j2,
                        const std::string& j2_name, MergeRecord* record);

/* Merges the "extensionsUsed" array from j2 into j1. */
void MergeExtensionsUsed(nlohmann::json* j1, nlohmann::json&& j2);

/* Merges the "extensionsRequired" array from j2 into j1. */
void MergeExtensionsRequired(nlohmann::json* j1, nlohmann::json&& j2);

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
//
// With the fact that we only merge the default scenes, this means we could be
// merging all sorts of unused data. Nodes not referenced by the default scene
// are wasted. Any meshes referenced *only* by those nodes are likewise wasted.
// This can be pushed down to all images, textures, samplers, etc., not just
// the data embedded in buffers.
//
// To make a truly minimal merged result, we'd have to walk the tree once to
// find out what we'll actually need based on default scene merging and then
// merge only that. However, at this point our informal (hopefully soon to
// become formal) advice is to use glTF files with a single scene (the normal
// result of simply exporting geometry from a tool like blender). So, this
// extra work would probably be premature. When we change to not just allow but
// exploit multiple scenes, we'll have to be more targeted here.

/* Merges the glTF data stored in the json j2 into j1.

 Generally, merging consists of concatenating the elements of the arrays in
 j2 into the corresponding arrays in j1. Indices in j2 referencing those
 elements get offset to reflect their new positions in j1's arrays. However,
 there are some special rules for merging:

 1. Only the default scenes get merged. If either glTF hasn't specified the
    "scene" property, it is assumed to be the 0th scene.
    - The scene *name* is never modified.
 2. We attempt to merge the "extras" and "extensions" at the glTF level, and in
    the "asset" and merged Scene properties. If there is any problem in merging,
    we throw an exception detailing the problem.
 3. This process makes no effort to distinguish between data and file URIs. They
    simply get merged as is; no action can be taken here to reconcile file URIs
    in j2 with the non-existent location of j1. The caller takes full
    responsibility to handle this in one of two ways:
      a. make j2 an embedded glTF (so there are no file URIs to resolve), or
      b. track the files referenced by file URI and place them relative to j1's
         final, on-disk location.
    Note: RenderEngineGltfClient converts all file uris into data uris before
    invoking this, implementing option (a).

 This explicitly excludes skin data, animation, and morph target elements
 (although the underlying data contained in buffers remains).

 @param[in/out] j1    The glTF's json structure to merge into.
 @param[in] j2        The glTF's json to merge from.
 @param[in] j2_name   A label to use for referring to the glTF in j2 in error
                      messages if problems are encountered while processing j2.
 @param[out] record   The records of the origins of components in the merged
                      glTF json.
 @pre Both j1 and j2 indicate version 2.0 glTF files. */
void MergeGltf(nlohmann::json* j1, nlohmann::json&& j2,
               const std::string& j2_name, MergeRecord* record);

}  // namespace internal
}  // namespace render_gltf_client
}  // namespace geometry
}  // namespace drake
