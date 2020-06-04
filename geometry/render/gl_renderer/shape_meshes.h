#pragma once

#include <istream>
#include <string>
#include <utility>

#include <Eigen/Dense>

#include "drake/geometry/render/gl_renderer/opengl_includes.h"

namespace drake {
namespace geometry {
namespace render {
namespace internal {

// TODO(SeanCurtis-TRI): All of these methods ultimately need to provide
//  normals and texture coordinates.

// These are pseudo-public aliases -- they are exposed to other classes in the
// render::internal namespace, but aren't part of the public API.

/* The representation of all Nv vertex positions in the mesh encoded in a Nvx3
 matrix such that the ith row is the position for the ith vertex in the mesh
 (measured and expressed in the mesh's canonical frame M).

 The representation is, as the name implies, intended to facilitate defining
 OpenGl geometry constructs via vertex buffers.  */
using VertexBuffer = Eigen::Matrix<GLfloat, Eigen::Dynamic, 3, Eigen::RowMajor>;

/* The representation of all Nt mesh triangles, encoded in an Ntx3 matrix of
 index values. The ith row represents the ith triangle such that columns 0, 1,
 and 2 of that row are indices into the corresponding VertexBuffer. All values
 in this data should lie in the range [0, vertices.rows() - 1], for the
 corresponding set of vertices.

 The representation is, as the name implies, intended to facilitate defining
 OpenGl geometries via index buffers.  */
using IndexBuffer = Eigen::Matrix<GLuint, Eigen::Dynamic, 3, Eigen::RowMajor>;

// TODO(SeanCurtis-TRI): Also parse normals and texture coordinates.

/* Loads a mesh's vertices and indices (faces) from an OBJ description given
 in the input stream. It does not load textures. Note that while this
 functionality seems similar to ReadObjToSurfaceMesh, RenderEngineGl cannot use
 SurfaceMesh. Rendering requires normals and texture coordinates; SurfaceMesh
 was not designed with those quantities in mind.  */
std::pair<VertexBuffer, IndexBuffer> LoadMeshFromObj(
    std::istream* input_stream);

/* Overload of LoadMeshFromObj that reads the OBJ description from the given
 file. */
std::pair<VertexBuffer, IndexBuffer> LoadMeshFromObj(
    const std::string& filename);

// TODO(SeanCurtis-TRI): Provide a geodesic sphere (or a tessellation like that
//  produced for hydroelastics).
/* Creates an OpenGL-compaptible mesh representation of a unit sphere
 (radius = 1). The sphere is tessellated along longitude and latitude bands.

 @param longitude_bands     The number of triangle bands running from pole to
                            pole.
 @param latitude_bands      The number of triangle bands running parallel
                            with the sphere equator.
 @pre `longitude_bands` >= 3 and `latitude_bands` >= 2 (otherwise the sphere
      will have no volume).  */
std::pair<VertexBuffer, IndexBuffer> MakeLongLatUnitSphere(
    int longitude_bands = 50, int latitude_bands = 50);

/* Creates an OpenGL-compatible mesh representation of a unit cylinder; its
 radius and height are equal to 1. It is centered on the origin of its canonical
 frame C and aligned with Frame C's z axis: Cz.
 The cylinder's geometry consists of the two caps and the barrel and is
 tessellated as follows. The barrel is divided in two directions: into a
 given number of _bands_ of equal height along its length, and into a given
 number of strips of equal width around its curvature. At the top and bottom
 circular caps a triangular fan is created from a vertex at the center of each
 circular cap to all of the vertices on the edge where cap and barrel meet.

 @param num_strips  The number of strips the barrel's curvature is divided into.
 @param num_bands   The number of equal-width bands spanning the height of the
                    clinder.
 @pre `num_strips` >= 3 (otherwise the cylinder will have no volume).
 @pre `num_bands` >= 1.  */
std::pair<VertexBuffer, IndexBuffer> MakeUnitCylinder(int num_strips = 50,
                                                      int num_bands = 1);

/* Creates an OpenGL-compaptible mesh reprepsenting a square patch. The patch
 has edge length `measure` units long. The square patch is defined lying on the
 z = 0 plane in its canonical frame C, centered on the origin Co. The faces are
 wound such that the right-handed face normal points in the dirction of the
 positive z-axis of Frame C.

 The domain of the patch is divided into square sub regions based on the given
 `resolution`. There will be `resolution^2` square patches (each defined by
 two triangles).
 @pre `measure` > 0
 @pre `resolution >= 1`. */
std::pair<VertexBuffer, IndexBuffer> MakeSquarePatch(GLfloat measure = 200,
                                                     int resolution = 1);

/* Creates an OpenGL_compatible mesh representation of the unit box - all edges
 are length 1. The box is centered on the origin of its canonical frame C with
 its edges aligned to the frame's basis. Each face of the box is divided into a
 single pair of triangles.

<!-- TODO(SeanCurtis-TRI): consider offering subdivisions if per-vertex
    properties yield undesirable artifacts for large boxes.  --> */
std::pair<VertexBuffer, IndexBuffer> MakeUnitBox();

// TODO(SeanCurtis): capsule, ellipsoid.

}  // namespace internal
}  // namespace render
}  // namespace geometry
}  // namespace drake
