#pragma once

#include <istream>
#include <string>

#include <Eigen/Dense>

#include "drake/geometry/render/gl_renderer/opengl_includes.h"

namespace drake {
namespace geometry {
namespace render {
namespace internal {

// TODO(SeanCurtis-TRI): All of these methods ultimately need to provide
//  texture coordinates.

/* The data representing a mesh. The triangle mesh is defined by `indices`. Row
 t represents a triangle by the triplet of vertex indices: tᵥ₀, tᵥ₁, tᵥ₂. The
 indices map into the rows of `positions`, `normals`, and `uvs`. I.e., for
 vertex index v, the position of that vertex is at `positions.row(v)`, its
 corresponding normal is at `normals.row(v)`, and its texture coordinates are at
 `uvs.row(v)`.

 For now, uvs will be empty. Only `positions`, `indices`, and `normals` are
 guaranteed. Texture coordinates will come in the near future.  */
struct MeshData {
  Eigen::Matrix<GLfloat, Eigen::Dynamic, 3, Eigen::RowMajor> positions;
  Eigen::Matrix<GLfloat, Eigen::Dynamic, 3, Eigen::RowMajor> normals;
  Eigen::Matrix<GLfloat, Eigen::Dynamic, 2, Eigen::RowMajor> uvs;
  Eigen::Matrix<GLuint, Eigen::Dynamic, 3, Eigen::RowMajor> indices;
};

// TODO(SeanCurtis-TRI): Also parse texture coordinates.

/* Loads a mesh's vertices and indices (faces) from an OBJ description given
 in the input stream. It does not load textures. Note that while this
 functionality seems similar to ReadObjToSurfaceMesh, RenderEngineGl cannot use
 SurfaceMesh. Rendering requires normals and texture coordinates; SurfaceMesh
 was not designed with those quantities in mind.  */
MeshData LoadMeshFromObj(std::istream* input_stream,
                         const std::string& filename = "from_string");

/* Overload of LoadMeshFromObj that reads the OBJ description from the given
 file. */
MeshData LoadMeshFromObj(const std::string& filename);

// TODO(SeanCurtis-TRI): Provide a geodesic sphere (or a tessellation like that
//  produced for hydroelastics).
/* Creates an OpenGL-compatible mesh representation of a unit sphere
 (radius = 1). The sphere is tessellated along longitude and latitude bands.

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

 This mesh's triangles will not all be topologically connected via shared
 vertices. Discontinuities in surface normals lead to disjoint meshes (i.e.,
 the caps are separate from the barrel).

 @param num_strips  The number of strips the barrel's curvature is divided into.
 @param num_bands   The number of equal-width bands spanning the height of the
                    cylinder.
 @pre `num_strips` >= 3 (otherwise the cylinder will have no volume).
 @pre `num_bands` >= 1.  */
MeshData MakeUnitCylinder(int num_strips = 50, int num_bands = 1);

/* Creates an OpenGL-compaptible mesh reprepsenting a square patch. The patch
 has edge length `measure` units long. The square patch is defined lying on the
 z = 0 plane in its canonical frame C, centered on the origin Co. The faces are
 wound such that the right-handed face normal points in the direction of the
 positive z-axis of Frame C.

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
