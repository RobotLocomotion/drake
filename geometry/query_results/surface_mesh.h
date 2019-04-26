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
 Index for identifying a triangular face in a surface mesh.
 */
using SurfaceFaceIndex = TypeSafeIndex<class SurfaceFaceTag>;

/** %SurfaceVertex represents a vertex in SurfaceMesh of a contact surface
 between bodies M and N.
 @tparam T The underlying scalar type for coordinates, e.g., double
           or AutoDiffXd. Must be a valid Eigen scalar.
*/
template <class T>
class SurfaceVertex {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SurfaceVertex)

  /** Constructs SurfaceVertex.
   @param r_MV  displacement vector from the origin of M's frame to this
   vertex, expressed in M's frame.
   */
  explicit SurfaceVertex(const Vector3<T>& r_MV)
      : r_MV_(r_MV) {}

  /** Returns the displacement vector from the origin of M's frame to this
   vertex, expressed in M's frame.
   */
  const Vector3<T>& r_MV() const { return r_MV_; }

 private:
  // Displacement vector from the origin of M's frame to this vertex,
  // expressed in M's frame.
  Vector3<T> r_MV_;
};

/** %SurfaceFace represents a triangular face in a SurfaceMesh of a contact
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
          direction towards increasing eₘ the scalar field on body M. See
          ContactSurface.
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
 @tparam T The underlying scalar type for coordinates, e.g., double
           or AutoDiffXd. Must be a valid Eigen scalar.
 */
template <class T>
class SurfaceMesh {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SurfaceMesh)

  /**
   @name Interface to MeshField

   The following definitions are needed by a MeshField defined on this
   SurfaceMesh.

   MeshField uses the term _elements_ (inspired by Finite Element Method)
   for _faces_, i.e., triangles, in a triangulated surface mesh. (For a
   tetrahedral volume mesh, the term elements would be for tetrahedrons.)
  */
  //@{

  /**
   A surface mesh has the intrinsic dimension 2.  It is embedded in 3-d space.
   */
  static constexpr int kDim = 2;

  /**
    Index for identifying a vertex.
   */
  using VertexIndex = SurfaceVertexIndex;

  /**
    Index for identifying a triangular element.
   */
  using ElementIndex = SurfaceFaceIndex;

  /**
    Type of barycentric coordinates on a triangular element.
    Barycentric coordinates (b0, b1, b2) satisfies b0 + b1 + b2 = 1, so
    technically we could calculate one of the bᵢ from the others; however,
    there is no standard way to suppress one of the coordinates.
   */
  using Barycentric = Vector<T, kDim + 1>;

  /** Returns the triangular element identified by a given index.
    @param e   The index of the triangular element.
    @pre e = 0, 1, 2,..., num_faces()-1.
   */
  const SurfaceFace& element(ElementIndex e) const {
    DRAKE_DEMAND(0 <= e && e <= num_faces() - 1);
    return faces_[e];
  }

  /** Returns the vertex identified by a given index.
    @param v  The index of the vertex.
    @pre v = 0, 1, 2,...,num_vertices()-1.
   */
  const SurfaceVertex<T>& vertex(VertexIndex v) const {
    DRAKE_DEMAND(0 <= v && v <= num_vertices() - 1);
    return vertices_[v];
  }

  //@}

  /** Constructs a SurfaceMesh from faces and vertices.
    @param faces     The triangular faces.
    @param vertices  The vertices.
   */
  SurfaceMesh(std::vector<SurfaceFace>&& faces,
              std::vector<SurfaceVertex<T>>&& vertices)
      : faces_(std::move(faces)),
        area_(faces_.size()),
        vertices_(std::move(vertices)) {
    init();
  }

  /** Returns the number of triangular elements.
   */
  int num_faces() const { return faces_.size(); }

  /** Returns the number of vertices.
   */
  int num_vertices() const { return vertices_.size(); }

  /** Returns area of a triangular element
   */
  T area(SurfaceFaceIndex f) const { return area_[f]; }

 private:
  // Initialization.
  void init();
  // Evaluate area of a triangular face.
  T EvaluateFaceArea(SurfaceFaceIndex f);
  // The triangles that comprise the surface.
  std::vector<SurfaceFace> faces_;
  // Area of the triangles. Computed in initialization.
  std::vector<T> area_;
  // The vertices that are shared between the triangles.
  std::vector<SurfaceVertex<T>> vertices_;
};

template <class T>
void SurfaceMesh<T>::init() {
  for (SurfaceFaceIndex f(0); f < faces_.size(); ++f)
    area_[f] = EvaluateFaceArea(f);
}

template <class T>
T SurfaceMesh<T>::EvaluateFaceArea(SurfaceFaceIndex f) {
  const auto& r_MU = vertices_[faces_[f].vertex(0)].r_MV();
  const auto& r_MV = vertices_[faces_[f].vertex(1)].r_MV();
  const auto& r_MW = vertices_[faces_[f].vertex(2)].r_MV();
  const auto r_UV_M = r_MV - r_MU;
  const auto r_UW_M = r_MW - r_MU;
  const auto cross = r_UV_M.cross(r_UW_M);
  return T(0.5)*cross.norm();
}

}  // namespace geometry
}  // namespace drake

