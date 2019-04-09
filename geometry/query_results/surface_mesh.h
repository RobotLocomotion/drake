#pragma once

#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/type_safe_index.h"

namespace drake {
namespace geometry {

/**
 Index used to identify a vertex in a surface mesh.
 */
using SurfaceVertexIndex = TypeSafeIndex<class SurfaceVertexTag>;

/**
 Index used to identify a triangular face in a surface mesh.
 */
using SurfaceFaceIndex = TypeSafeIndex<class SurfaceFaceTag>;

/** %SurfaceVertex represents a vertex in SurfaceMesh of a contact surface
 between bodies M and N. Right now it has only one member variable, but we
 plan to add more later.
 @tparam T the underlying scalar type. Must be a valid Eigen scalar.
*/
template <class T>
class SurfaceVertex {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SurfaceVertex)

  /** Constructs SurfaceVertex.
   @param r_MV   position of vertex v in M's frame.
   */
  explicit SurfaceVertex(const Vector3<T>& r_MV)
      : r_MV_(r_MV) {}

  /** Returns the position of this vertex in M's frame.
   */
  const Vector3<T>& r_MV() const { return r_MV_; }

 private:
  // Position of this vertex in M's frame.
  Vector3<T> r_MV_;
};

/** %SurfaceFace represents a triangular face in SurfaceMesh of a contact
 surface between bodies M and N.
 */
class SurfaceFace {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SurfaceFace)

  /** Constructs ContactSurfaceFace.
   @param v0 Index of the first vertex in SurfaceMesh.
   @param v1 Index of the second vertex in SurfaceMesh.
   @param v2 Index of the last vertex in SurfaceMesh.
   @note   The order of the three vertices gives the counterclockwise normal
          direction towards increasing eₘ the scalar field on body M.
   */
  SurfaceFace(SurfaceVertexIndex v0,
              SurfaceVertexIndex v1,
              SurfaceVertexIndex v2)
      : vertex_({v0, v1, v2}) {}

  /** Constructs ContactSurfaceFace.
   @param v  array of three integer indices of the vertices of the face in
             SurfaceMesh.
   @note   The order of the three vertices gives the counterclockwise normal
          direction towards increasing eₘ the scalar field on body M.
   */
  explicit SurfaceFace(const int v[3])
      : vertex_({SurfaceVertexIndex(v[0]),
                 SurfaceVertexIndex(v[1]),
                 SurfaceVertexIndex(v[2])}) {}

  /** Returns the vertex index in SurfaceMesh of the i-th vertex of this face.
   @param i  The local index of the vertex in this face.
   @pre 0 <= i < 3
   */
  SurfaceVertexIndex vertex(int i) const {
    DRAKE_DEMAND(0 <= i && i < 3);
    return vertex_[i];
  }

 private:
  // The vertices of this face.
  std::array<SurfaceVertexIndex, 3> vertex_;
};

/** %SurfaceMesh represents a triangulated surface of a contact surface.
  A field variable can be defined on SurfaceMesh using SurfaceMeshField.
 */
template <class T>
class SurfaceMesh {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SurfaceMesh)

  /** Constructs a SurfaceMesh from faces and vertices.
    @param faces     The triangular faces that will be moved into this
                     SurfaceMesh. We require an rvalue reference.
    @param vertices  The vertices that will be moved into this SurfaceMesh.
                     We require an rvalue reference.
   */
  SurfaceMesh(std::vector<SurfaceFace>&& faces,
              std::vector<SurfaceVertex<T>>&& vertices)
  : faces_(std::move(faces)), vertices_(std::move(vertices))
  {}

  /** Returns the number of triangular faces.
   */
  int num_faces() { return faces_.size(); }

  /** Returns the number of vertices.
   */
  int num_vertices() { return vertices_.size(); }

  /** Returns the triangular face identified by a given index.
    @param f   The index of the triangular face.
   */
  const SurfaceFace& face(SurfaceFaceIndex f) const { return faces_[f]; }

  /** Returns the vertex identified by a given index.
    @param v  The index of the vertex.
   */
  const SurfaceVertex<T>& vertex(SurfaceVertexIndex v) const {
    return vertices_[v];
  }

 private:
  // Triangles comprising the surface.
  std::vector<SurfaceFace> faces_;
  // Shared vertices of the triangles.
  std::vector<SurfaceVertex<T>> vertices_;
};


}  // namespace geometry
}  // namespace drake

