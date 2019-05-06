#pragma once

#include <array>
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

/** %SurfaceVertex represents a vertex in SurfaceMesh.
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

/** %SurfaceFace represents a triangular face in a SurfaceMesh.
 */
class SurfaceFace {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SurfaceFace)

  /** Constructs SurfaceFace.
   @param v0 Index of the first vertex in SurfaceMesh.
   @param v1 Index of the second vertex in SurfaceMesh.
   @param v2 Index of the last vertex in SurfaceMesh.
   */
  SurfaceFace(SurfaceVertexIndex v0,
              SurfaceVertexIndex v1,
              SurfaceVertexIndex v2)
      : vertex_({v0, v1, v2}) {}

  /** Constructs SurfaceFace.
   @param v  array of three integer indices of the vertices of the face in
             SurfaceMesh.
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
    return vertex_.at(i);
  }

 private:
  // The vertices of this face.
  std::array<SurfaceVertexIndex, 3> vertex_;
};

// TODO(DamrongGuoy): mention interesting properties of the mesh, e.g., open
//  meshes, meshes with holes, non-manifold surface.
/** %SurfaceMesh represents a triangulated surface.
 @tparam T The underlying scalar type for coordinates, e.g., double
           or AutoDiffXd. Must be a valid Eigen scalar.
 */
template <class T>
class SurfaceMesh {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SurfaceMesh)

  /**
   @name Mesh type traits

   A collection of type traits to enable mesh consumers to be templated on mesh
   type. Each mesh type provides specific definitions of _vertex_, _element_,
   and _barycentric coordinates_. For %SurfaceMesh, an element is a triangle.
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
    Barycentric coordinates (b₀, b₁, b₂) satisfy b₀ + b₁ + b₂ = 1, bᵢ >= 0, so
    technically we could calculate one of the bᵢ from the others; however,
    there is no standard way to omit one of the coordinates.
   */
  using Barycentric = Vector<T, kDim + 1>;

  /** Returns the triangular element identified by a given index.
    @param e   The index of the triangular element.
    @pre e ∈ {0, 1, 2,..., num_faces()-1}.
   */
  const SurfaceFace& element(ElementIndex e) const {
    DRAKE_DEMAND(0 <= e && e < num_faces());
    return faces_[e];
  }

  /** Returns the vertex identified by a given index.
    @param v  The index of the vertex.
    @pre v ∈ {0, 1, 2,...,num_vertices()-1}.
   */
  const SurfaceVertex<T>& vertex(VertexIndex v) const {
    DRAKE_DEMAND(0 <= v && v < num_vertices());
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

  /** Returns the number of triangular elements in the mesh.
   */
  int num_faces() const { return faces_.size(); }

  /** Returns the number of vertices in the mesh.
   */
  int num_vertices() const { return vertices_.size(); }

  /** Returns area of a triangular element.
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
  const auto& r_MA = vertices_[faces_[f].vertex(0)].r_MV();
  const auto& r_MB = vertices_[faces_[f].vertex(1)].r_MV();
  const auto& r_MC = vertices_[faces_[f].vertex(2)].r_MV();
  const auto r_UV_M = r_MB - r_MA;
  const auto r_UW_M = r_MC - r_MA;
  const auto cross = r_UV_M.cross(r_UW_M);
  return T(0.5)*cross.norm();
}

}  // namespace geometry
}  // namespace drake

