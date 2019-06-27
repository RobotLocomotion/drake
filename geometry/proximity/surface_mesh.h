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
    Type of barycentric coordinates on a triangular element. Barycentric
    coordinates (b₀, b₁, b₂) satisfy b₀ + b₁ + b₂ = 1. It corresponds to a
    position on the plane of the triangle. If all bᵢ >= 0, it corresponds to
    a position inside the triangle or on the edges of the triangle. If some
    bᵢ < 0, it corresponds to a position on the plane of the triangle that is
    outside the triangle. Technically we could calculate one of the bᵢ from
    the others; however, there is no standard way to omit one of the
    coordinates.
   */
  using Barycentric = Vector<T, kDim + 1>;

  /** Type of cartesian coordinates. Mesh consumers can use it in conversion
   from Cartesian coordinates to barycentric coordinates.
   */
  using Cartesian = Vector<T, 3>;

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

  /** Returns the number of vertices in the mesh.
   */
  int num_vertices() const { return vertices_.size(); }

  //@}

  /** Constructs a SurfaceMesh from faces and vertices.
    @param faces     The triangular faces.
    @param vertices  The vertices.
   */
  SurfaceMesh(std::vector<SurfaceFace>&& faces,
              std::vector<SurfaceVertex<T>>&& vertices)
      : faces_(std::move(faces)), vertices_(std::move(vertices)),
        area_(faces_.size()) {  // Pre-allocate here, not yet calculated.
    CalcAreasAndCentroid();
  }

  /** Returns the number of triangular elements in the mesh.
   */
  int num_faces() const { return faces_.size(); }

  /** Returns area of a triangular element.
   */
  const T& area(SurfaceFaceIndex f) const { return area_[f]; }

  /** Returns the total area of all the faces of this surface mesh.
   */
  const T& total_area() const { return total_area_; }

  /** Returns the area-weighted geometric centroid of this surface mesh. The
   returned value is the position vector p_MSc from M's origin to the
   centroid Sc, expressed in frame M. (M is the frame in which this mesh's
   vertices are measured and expressed.) Note that the centroid is not
   necessarily a point on the surface. If the total mesh area is exactly
   zero, we define the centroid to be (0,0,0).

   The centroid location is calculated _per face_ not _per vertex_ so is
   insensitive to whether vertices are shared by faces.
   */
  const Vector3<T>& centroid() const { return p_MSc_; }

  /**
   Maps the barycentric coordinates `Q_barycentric` of a point Q in
   `element_index` to its position vector p_MQ.
   */
  Vector3<T> CalcCartesianFromBarycentric(
      ElementIndex element_index, const Vector3<T>& Q_barycentric) const {
    const SurfaceVertex<T> va = vertex(element(element_index).vertex(0));
    const SurfaceVertex<T> vb = vertex(element(element_index).vertex(1));
    const SurfaceVertex<T> vc = vertex(element(element_index).vertex(2));

    // This is just a linear transformation between the two coordinates,
    // Cartesian (C) and Barycentric (B). Form the transformation matrix:
    Matrix3<T> T_CB;
    T_CB.col(0) = va.r_MV();
    T_CB.col(1) = vb.r_MV();
    T_CB.col(2) = vc.r_MV();

    return T_CB * Q_barycentric;
  }

  /** Calculate barycentric coordinates with respect to the triangular face `f`
   of the projection of a position `p_M` to the plane of the face. This
   operation is expensive compared with going from barycentric to Cartesian.
   @param p_M  A position expressed in the frame M of the mesh.
   @param f    The index of a triangular face.
   @note  If the projection of p_M is outside the triangle, the
          barycentric coordinates (b₀, b₁, b₂) still satisfy b₀ + b₁ + b₂ = 1;
          however, some bᵢ will be negative.
   */
  Barycentric CalcBarycentric(const Cartesian& p_M, SurfaceFaceIndex f) const {
    const Vector3<T>& v0 = vertex(element(f).vertex(0)).r_MV();
    const Vector3<T>& v1 = vertex(element(f).vertex(1)).r_MV();
    const Vector3<T>& v2 = vertex(element(f).vertex(2)).r_MV();
    // Translate v0 to the origin. Cartesian coordinates change but
    // barycentric coordinates stay the same.
    //     v0 becomes u0 = v0 - v0 = 0 vector
    //     v1 becomes u1 = v1 - v0.
    //     v2 becomes u2 = v2 - v0.
    //     p_M becomes q = p_M - v0.
    //
    // Consider q* on the spanning plane through the origin, u1, u2:
    //     q* = b₀*u0 + b₁*u1 + b₂*u2
    //        = 0 + b₁*u1 + b₂*u2
    //        = b₁*u1 + b₂*u2
    //
    // Solve for b₁, b₂ that give q* "closest" to q in the least square sense:
    //
    //      ||   | ||b1|   |||
    //      |u1  u2||b2| ~ |q|
    //      ||   | |       |||
    //
    // return Barycentric (1-b₁-b₂, b₁, b₂)
    //
    Eigen::Matrix<T, 3, 2> A;
    A.col(0) << v1 - v0;
    A.col(1) << v2 - v0;
    Vector2<T> solution = A.colPivHouseholderQr().solve(p_M - v0);

    const T& b1 = solution(0);
    const T& b2 = solution(1);
    const T b0 = T(1.) - b1 - b2;
    return Barycentric(b0, b1, b2);
  }
  // TODO(DamrongGuoy): Investigate alternative calculation suggested by
  //  Alejandro Castro:
  // 1. Translate v0 to the origin.
  //        v0 becomes u0 = v0 - v0 = 0 vector
  //        v1 becomes u1 = v1 - v0.
  //        v2 becomes u2 = v2 - v0.
  //        p_M becomes x₀ₚ = p_M - v0.
  // 2. Calculate the unit normal vector n to the spanning plane S through
  //    the origin, u1, and u2.
  //        n = u1.cross(u2).normalize().
  // 3. Project x₀ₚ to xₛ on the plane S,
  //        xₛ = x₀ₚ - (x₀ₚ.dot(n))*n
  //
  // Now we have xₛ = b₀*u0 + b₁*u1 + b₂*u2 by barycentric coordinates.
  //                =   0   + b₁*u1 + b₂*u2
  //
  // 5. Solve for b₁ and b₂.
  //        (b₁*u1 + b₂*u2).dot(u1) = xₛ.dot(u1)
  //        (b₁*u1 + b₂*u2).dot(u2) = xₛ.dot(u2)
  //    Therefore, the 2x2 system:
  //        |u1.dot(u1)  u2.dot(u1)||b1| = |xₛ.dot(u1)|
  //        |u1.dot(u2)  u2.dot(u2)||b2|   |xₛ.dot(u2)|
  //
  // 6. return Barycentric(1-b₁-b₂, b₁, b₂)
  //
  // Optimization: save n, and the matrix |uᵢ.dot(uⱼ)| for later.
  //

 private:
  // Calculates the areas of each triangle, the total area, and the centorid of
  // the surface.
  void CalcAreasAndCentroid();

  // The triangles that comprise the surface.
  std::vector<SurfaceFace> faces_;
  // The vertices that are shared among the triangles.
  std::vector<SurfaceVertex<T>> vertices_;

  // Computed in initialization.

  // Area of the triangles.
  std::vector<T> area_;
  T total_area_{};

  // Area-weighted geometric centroid Sc of the surface mesh as an offset vector
  // from the origin of Frame M to point Sc, expressed in Frame M.
  Vector3<T> p_MSc_;
};

template <class T>
void SurfaceMesh<T>::CalcAreasAndCentroid() {
  total_area_ = 0;
  p_MSc_.setZero();

  for (SurfaceFaceIndex f(0); f < faces_.size(); ++f) {
    const SurfaceFace& face = faces_[f];
    const Vector3<T>& r_MA = vertices_[face.vertex(0)].r_MV();
    const Vector3<T>& r_MB = vertices_[face.vertex(1)].r_MV();
    const Vector3<T>& r_MC = vertices_[face.vertex(2)].r_MV();
    const auto r_UV_M = r_MB - r_MA;
    const auto r_UW_M = r_MC - r_MA;

    const auto cross = r_UV_M.cross(r_UW_M);
    const T face_area = T(0.5) * cross.norm();
    area_[f] = face_area;
    total_area_ += face_area;

    // Accumulate area-weighted surface centroid; must be divided by 3X the
    // total area afterwards.
    p_MSc_ += face_area * (r_MA + r_MB + r_MC);
  }

  // Finalize centroid.
  if (total_area_ != T(0.))
    p_MSc_ /= (3. * total_area_);
}

}  // namespace geometry
}  // namespace drake

