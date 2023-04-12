#pragma once

#include <vector>

#include <Eigen/Dense>
#include <tiny_obj_loader.h>

namespace drake {
namespace geometry {
namespace internal {

/* The data representing a mesh. The triangle mesh is defined by `indices`. Row
 t represents a triangle by the triplet of vertex indices: tᵥ₀, tᵥ₁, tᵥ₂. The
 indices map into the rows of `positions`, `normals`, and `uvs`. I.e., for
 vertex index v, the position of that vertex is at `positions.row(v)`, its
 corresponding normal is at `normals.row(v)`, and its texture coordinates are at
 `uvs.row(v)`.

 For now, all vertex quantities (`positions`, `normals` and `uvs`) are
 guaranteed (as well as the `indices` data). In the future, `uvs` may become
 optional.  */
struct MeshData {
  Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> positions;
  Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> normals;
  Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor> uvs;
  Eigen::Matrix<unsigned int, Eigen::Dynamic, 3, Eigen::RowMajor> indices;

  /** See docs for `has_tex_coord` below.  */
  static constexpr bool kHasTexCoordDefault{true};

  /** This flag indicates that this mesh has texture coordinates to support
   maps.
   If True, the values of `uvs` will be nontrivial.
   If False, the values of `uvs` will be all zeros, but will still have the
   correct size.  */
  bool has_tex_coord{kHasTexCoordDefault};
};

/* Loads a mesh's vertices and indices (faces) from an OBJ description given
 in the input stream. It does not load textures. Note that while this
 functionality seems similar to ReadObjToTriangleSurfaceMesh, RenderEngineGl
 cannot use TriangleSurfaceMesh. Rendering requires normals and texture
 coordinates; TriangleSurfaceMesh was not designed with those quantities in
 mind.

 If no texture coordinates are specified by the file, it will be indicated in
 the returned MeshData. See MeshData::has_tex_coord for more detail.

 @throws std::exception if a) tinyobj::LoadObj() fails, (b) there are no faces
                           or normals, c) faces fail to reference normals, or d)
                           faces fail to reference the texture coordinates if
                           they are present.
 @pre                              */
MeshData LoadMeshFromObj(const tinyobj::attrib_t attrib,
                         std::vector<tinyobj::shape_t> shapes,
                         std::string_view description);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
