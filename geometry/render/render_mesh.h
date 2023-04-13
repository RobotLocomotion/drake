#pragma once

#include <optional>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/diagnostic_policy.h"
#include "drake/geometry/rgba.h"

namespace drake {
namespace geometry {
namespace internal {

/* Specifies a mesh material as currently supported by Drake. We expect this
 definition to grow with time. */
struct RenderMaterial {
  Rgba diffuse;
  std::string diffuse_map;
};

/* The data representing a mesh with a single material. The triangle mesh is
 defined by `indices`. Row t represents a triangle by the triplet of vertex
 indices: tᵥ₀, tᵥ₁, tᵥ₂. The indices map into the rows of `positions`,
 `normals`, and `uvs`. I.e., for vertex index v, the position of that vertex is
 at `positions.row(v)`, its corresponding normal is at `normals.row(v)`, and its
 texture coordinates are at `uvs.row(v)`.

 For now, all vertex quantities (`positions`, `normals` and `uvs`) are
 guaranteed (as well as the `indices` data). In the future, `uvs` may become
 optional.  */
struct RenderMesh {
  Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> positions;
  Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> normals;
  Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor> uvs;
  Eigen::Matrix<unsigned int, Eigen::Dynamic, 3, Eigen::RowMajor> indices;

  /* See docs for `has_tex_coord` below.  */
  static constexpr bool kHasTexCoordDefault{true};

  /* This flag indicates that this mesh has texture coordinates to support
   maps.
   If True, the values of `uvs` will be nontrivial.
   If False, the values of `uvs` will be all zeros, but will still have the
   correct size.  */
  bool has_tex_coord{kHasTexCoordDefault};

  /* The specification of the material intrinsically associated with this mesh
   data. If it has no value, materials should be defined by user of the
   RenderMesh. */
  std::optional<RenderMaterial> material;
};

// TODO(SeanCurtis-TRI): All of this explanation needs to go into the trouble-
// shooting guide.
// TODO(SeanCurtis-TRI): Modify the API to return a *set* of RenderMesh, each
// with a unique material.
/* Returns a single instance of RenderMesh from the indicated obj file.

 Note: the mesh data's material will be defined iff the OBJ applies a single
 valid material to the whole mesh. The material may be omitted for any number
 of "validity" reasons, including (but not limited to):

   - The .mtl file is referenced via *absolute* path.
   - The .mtl file (which references an image relatively) is not in the same
     directory as the .obj file.
   - Specified texture image (map_Kd) is unavailable (or can't be found).
   - Texture image specified but missing texture coordinates across all
     triangles.
   - Invalid material name referenced.
   - Invalid mtl file referenced.

 It is up to the caller to decide what material to provide if none is associated
 with the mesh data.

 Note: This API is similar to ReadObjToTriangleSurfaceMesh, but it differs in
 several ways:

    - Support of per-vertex data that TriangleSurfaceMesh doesn't support
      (e.g., vertex normals, texture coordinates).
    - Material semantics.
    - The geometric data is reconditioned to be compatible with "geometry
      buffer" applications. (See RenderMesh for details.)

 If no texture coordinates are specified by the file, it will be indicated in
 the returned RenderMesh. See RenderMesh::has_tex_coord for more detail.

 @throws std::exception if a) tinyobj::LoadObj() fails, (b) there are no faces
                           or normals, c) faces fail to reference normals, or d)
                           faces fail to reference the texture coordinates if
                           they are present. */
RenderMesh LoadMeshFromObj(const std::string& obj_file_name,
                           const drake::internal::DiagnosticPolicy& policy);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
