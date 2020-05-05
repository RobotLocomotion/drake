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
}  // namespace internal
}  // namespace render
}  // namespace geometry
}  // namespace drake
