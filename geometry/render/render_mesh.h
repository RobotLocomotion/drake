#pragma once

#include <filesystem>

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
 optional.  */
struct RenderMesh {
  Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> positions;
  Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> normals;
  Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::RowMajor> uvs;
  Eigen::Matrix<unsigned int, Eigen::Dynamic, 3, Eigen::RowMajor> indices;

  /* See docs for `has_tex_coord` below.  */
  static constexpr bool kHasTexCoordDefault{true};

  // TODO(SeanCurtis-TRI): this flag was necessary when materials and meshes
  // were resolved separately. Now that we resolve materials at the same time,
  // we should maintain the invariants that the presence of a texture map
  // implies the presence of texture coordinates. We make no guarantees about
  // the opposite direction -- that should only be relevant to efforts to add a
  // texture material to a RenderMesh that didn't originally parse into having
  // a texture material. It may or may not work. It also means that this
  // cannot be a struct; structs don't get to maintain invariants.
  /* This flag indicates that this mesh has texture coordinates to support
   maps.
   If True, the values of `uvs` will be nontrivial.
   If False, the values of `uvs` will be all zeros, but will still have the
   correct size.  */
  bool has_tex_coord{kHasTexCoordDefault};

  /* The specification of the material associated with this mesh data. */
  RenderMaterial material;
};

// TODO(SeanCurtis-TRI): All of this explanation, and general guidance for what
// meshes (and which features) are supported, needs to go into the trouble-
// shooting guide.
// TODO(SeanCurtis-TRI): Modify the API to return a *set* of RenderMesh, each
// with a unique material.
/* Returns a single instance of RenderMesh from the indicated obj file.

 The material definition will come from the obj's mtl file iff a single material
 is applied to all faces in the obj. Otherwise, it applies the fallback
 material protocol documented in MakeMeshFallbackMaterial() (the only time in
 which the `properties` and `default_diffuse` parameters are used).

 As long as there is a single material applied to all faces in the obj file,
 the material will be derived from the material library, even if the material
 specification is flawed. The derivation does its best to provide the "best"
 approximation of the declared material. But the fact it was declared and
 applied is respected. For example, the following are specification defects
 that nevertheless result in a RenderMaterial _based_ on the mtl material and
 _not_ on the fallback logic:

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

 If no texture coordinates are specified by the file, it will be indicated in
 the returned RenderMesh. See RenderMesh::has_tex_coord for more detail.

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
                           they are present.  */
RenderMesh LoadRenderMeshFromObj(
    const std::filesystem::path& obj_path, const GeometryProperties& properties,
    const Rgba& default_diffuse,
    const drake::internal::DiagnosticPolicy& policy = {});

}  // namespace internal
}  // namespace geometry
}  // namespace drake
