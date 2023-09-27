#pragma once

#include <filesystem>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/diagnostic_policy.h"
#include "drake/geometry/geometry_properties.h"
#include "drake/geometry/render/render_material.h"
#include "drake/geometry/rgba.h"

namespace drake {
namespace geometry {
namespace internal {

/* The data representing a mesh with a single material. The triangle mesh is
 defined by `indices`. Row t represents a triangle by the triplet of vertex
 indices: tᵥ₀, tᵥ₁, tᵥ₂. The indices map into the rows of `positions`,
 `normals`, and `uvs`. I.e., for vertex index v, the position of that vertex is
 at `positions.row(v)`, its corresponding normal is at `normals.row(v)`, and its
 texture coordinates are at `uvs.row(v)`.

 For now, all vertex quantities (`positions`, `normals` and `uvs`) are
 guaranteed (as well as the `indices` data). In the future, `uvs` may become
 optional. */
struct RenderMesh {
  Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> positions;
  Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> normals;
  Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::RowMajor> uvs;
  Eigen::Matrix<unsigned int, Eigen::Dynamic, 3, Eigen::RowMajor> indices;

  /* Indicates the degree that UV coordinates have been assigned to the mesh.
   Only UvState::kFull supports texture images. No matter what, the `uvs` matrix
   will be appropriately sized. But only for kFull will the values be
   meaningful. */
  UvState uv_state{UvState::kNone};

  /* The specification of the material associated with this mesh data. */
  RenderMaterial material;
};

// TODO(SeanCurtis-TRI): All of this explanation, and general guidance for what
// meshes (and which features) are supported, needs to go into the trouble-
// shooting guide.
/* Returns a set of RenderMesh instances based on the objects and materials
 defined in the indicated obj file.

 For each unique material referenced in the obj file, a RenderMesh will be
 created. Even if there are errors in the material specification, the algorithm
 does its best to provide the "best" approximation of the declared material. The
 fact that it was declared and applied is respected. For example, the following
 are specification defects that nevertheless result in a RenderMaterial _based_
 on the mtl material and _not_ on the fallback logic:

   - Referencing a non-existent or unavailable texture.
   - Failing to specify *any* material properties at all beyond its name.
   - Specifying a texture but failing to provide texture coordinates.
   - etc.

 Note: This API is similar to ReadObjToTriangleSurfaceMesh, but it differs in
 several ways:

    - Support of per-vertex data that TriangleSurfaceMesh doesn't support
      (e.g., vertex normals, texture coordinates).
    - Material semantics.
    - The geometric data is reconditioned to be compatible with "geometry
      buffer" applications. (See RenderMesh for details.)
    - It supports multiple objects in the file. In fact, there may be more
      RenderMesh instances than objects defined because a single object with
      multiple materials will likewise be partitioned into separate RenderMesh
      instances.

 If texture coordinates are assigned to vertices, it will be indicated in
 the returned RenderMesh. See RenderMesh::uv_state for more detail.

 If the material includes texture paths, they will have been confirmed to both
 exist and be accessible.

 TODO(SeanCurtis-TRI): all throwing conditions should be converted to messages
 on the policy. Problems that would produce an "empty" mesh (e.g., no
 faces/normals) would log errors on the policy (and return the empty mesh).
 Others, like no uvs referenced, should emit a warning and return a non-empty
 mesh, representing the best approximation of what was parsed.

 @throws std::exception if a) tinyobj::LoadObj() fails, (b) there are no faces
                           or normals, c) faces fail to reference normals, or d)
                           faces fail to reference the texture coordinates if
                           they are present. */
std::vector<RenderMesh> LoadRenderMeshesFromObj(
    const std::filesystem::path& obj_path, const GeometryProperties& properties,
    const Rgba& default_diffuse,
    const drake::internal::DiagnosticPolicy& policy = {});

}  // namespace internal
}  // namespace geometry
}  // namespace drake
