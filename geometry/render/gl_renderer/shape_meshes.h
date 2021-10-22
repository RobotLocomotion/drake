#pragma once

#include <istream>
#include <string>

#include <Eigen/Dense>

#include "drake/geometry/render/gl_renderer/opengl_includes.h"

namespace drake {
namespace geometry {
namespace render {
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
  Eigen::Matrix<GLfloat, Eigen::Dynamic, 3, Eigen::RowMajor> positions;
  Eigen::Matrix<GLfloat, Eigen::Dynamic, 3, Eigen::RowMajor> normals;
  Eigen::Matrix<GLfloat, Eigen::Dynamic, 2, Eigen::RowMajor> uvs;
  Eigen::Matrix<GLuint, Eigen::Dynamic, 3, Eigen::RowMajor> indices;

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

 @throws std::exception if a) there are no normals, b) faces fail to
                           reference normals, or c) faces fail to reference
                           the texture coordinates if they are present.  */
MeshData LoadMeshFromObj(std::istream* input_stream,
                         const std::string& filename = "from_string");

/* Overload of LoadMeshFromObj that reads the OBJ description from the given
 file. */
MeshData LoadMeshFromObj(const std::string& filename);

// TODO(SeanCurtis-TRI): Provide a geodesic sphere (or a tessellation like that
//  produced for hydroelastics).

/* Creates an OpenGL-compatible mesh representation of a unit sphere
 (radius = 1). The sphere is tessellated along longitude and latitude bands.

 Texture coordinates are assigned with the u-direction mapping to changes in
 longitude (i.e., in an east-west direction) and the v-direction maps to changes
 in latitude (i.e., in a north-south direction). The south pole has minimum
 v = 0 and the north pole has maximum v = 1. U ranges from [0, 1] with a seam at
 an arbitrary "prime meridian".

 @param longitude_bands     The number of triangle bands running from pole to
                            pole.
 @param latitude_bands      The number of triangle bands running parallel
                            with the sphere equator.
 @pre `longitude_bands` >= 3 and `latitude_bands` >= 2 (otherwise the sphere
      will have no volume).  */
MeshData MakeLongLatUnitSphere(int longitude_bands = 50,
                               int latitude_bands = 50);

/* Creates an OpenGL-compatible mesh representation of a unit cylinder; its
 radius and height are equal to 1. It is centered on the origin of its canonical
 frame C and aligned with Frame C's z axis: Cz.
 The cylinder's geometry consists of the two caps and the barrel and is
 tessellated as follows. The barrel is divided in two directions: into a
 given number of _bands_ of equal height along its length, and into a given
 number of strips of equal width around its curvature. At the top and bottom
 circular caps a triangular fan is created from a vertex at the center of each
 circular cap to all of the vertices on the edge where cap and barrel meet.

 Texture coordinates are assigned akin to how they are assigned to the sphere.
 The u-values change in the radial direction while the v-values change along the
 length direction. There is a seam in the mapping along an arbitrary
 longitudinal line (u = 0 on one side and u = 1 on the other).

 This mesh's triangles will not all be topologically connected via shared
 vertices. Discontinuities in surface normals lead to disjoint meshes (i.e.,
 the caps are separate from the barrel).

 @param num_strips  The number of strips the barrel's curvature is divided into.
 @param num_bands   The number of equal-width bands spanning the height of the
                    cylinder.
 @pre `num_strips` >= 3 (otherwise the cylinder will have no volume).
 @pre `num_bands` >= 1.  */
MeshData MakeUnitCylinder(int num_strips = 50, int num_bands = 1);

/* Creates an OpenGL-compatible mesh reprepsenting a square patch. The patch
 has edge length `measure` units long. The square patch is defined lying on the
 z = 0 plane in its canonical frame C, centered on the origin Co. The faces are
 wound such that the right-handed face normal points in the direction of the
 positive z-axis of Frame C.

 Texture coordinates are assigned so that the "mimimum" (smallest x- and
 y-values) map to (0, 0) and the opposite corner is (1, 1).

 The domain of the patch is divided into square sub regions based on the given
 `resolution`. There will be `resolution^2` square patches (each defined by
 two triangles).
 @pre `measure` > 0
 @pre `resolution >= 1`. */
MeshData MakeSquarePatch(GLfloat measure = 200, int resolution = 1);

/* Creates an OpenGL-compatible mesh representation of the unit box - all edges
 are length 1. The box is centered on the origin of its canonical frame C with
 its edges aligned to the frame's basis. Each face of the box is divided into a
 single pair of triangles.

 Texture coordinates are assigned so that the texture will be fully applied to
 each face once. The +/-x and +/-y faces are all oriented so that the top of
 the texture map will be in the +z direction. For the +z and -z faces, the
 orientation is not guaranteed.

 This mesh's triangles will not all be topologically connected via shared
 vertices. Discontinuities in surface normals lead to disjoint meshes (each box
 face is disconnected).

<!-- TODO(SeanCurtis-TRI): consider offering subdivisions if per-vertex
    properties yield undesirable artifacts for large boxes.  --> */
MeshData MakeUnitBox();

/* Creates an OpenGL-compatible mesh representation of a capsule with the given
 `radius` and `length`. The capsule is centered on the origin of its canonical
 frame C with its length aligned with C's z axis: Cz. The capsule is,
 conceptually, a cylinder capped with two hemispheres. The `length` is the
 measure of the cylinder's length. So, the *total* length of the capsule would
 be `length + 2 * radius`.

 Texture coordinates are assigned akin to the sphere and cylinder. The u-values
 change from 0 to 1 in the radial direction (with a seam at an arbitrary
 longitudinal line), and the v-values change from 0 to 1 from the south to the
 north poles based on fraction of the geodesic distance from pole to pole.

 The capsule is tessellated according to `samples`. The circular cross section
 of the cylindrical region of the capsule will have `samples` number of
 vertices. The two hemispherical caps will be tesselated such that triangles
 near the spherical equators have good aspect ratios. There is only a single
 band of quadrilaterals (split into pairs of triangles) in the cylindrical
 region.

 @param samples   The number of vertices to place around the barrel of the
                  capsule.
 @param radius    The radius of the hemispherical end-caps and cylindrical
                  barrel.
 @param length    The length of the cylindrical barrel.
 @pre `samples` >= 3 (otherwise the capsule will have no volume).
 @pre radius > 0 and length > 0.  */
MeshData MakeCapsule(int samples, double radius, double length);

}  // namespace internal
}  // namespace render
}  // namespace geometry
}  // namespace drake
