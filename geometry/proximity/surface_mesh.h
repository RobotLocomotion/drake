#pragma once

#include <array>
#include <set>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"
#include "drake/common/type_safe_index.h"
#include "drake/math/rigid_transform.h"

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

  DRAKE_DEPRECATED("2019-12-01", "Use TransformInPlace() instead.")
  void Transform(const math::RigidTransform<T>& X_NM) {
    TransformInPlace(X_NM);
  }

  /** Transforms this vertex position from its initial frame M to a new frame N.
   */
  void TransformInPlace(const math::RigidTransform<T>& X_NM) {
    r_MV_ = X_NM * r_MV_;
  }

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

  /** Reverses the order of the vertex indices -- this essentially flips the
   face normal based on the right-handed normal rule.
   */
  void ReverseWinding() {
    std::swap(vertex_[0], vertex_[1]);
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

   The barycentric coordinates for a point Q are notated a b_Q.
   */
  using Barycentric = Vector<T, kDim + 1>;

  /** Type of Cartesian coordinates. Mesh consumers can use it in conversion
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

  /**
   Returns the vertex identified by a given index.
   @param v  The index of the vertex.
   @pre v ∈ {0, 1, 2,...,num_vertices()-1}.
   */
  const SurfaceVertex<T>& vertex(VertexIndex v) const {
    DRAKE_DEMAND(0 <= v && v < num_vertices());
    return vertices_[v];
  }

  /**
   Gets the set of triangles that refer to the specified vertex.
   */
  const std::set<SurfaceFaceIndex>& referring_triangles(VertexIndex v) const {
    return referring_triangles_[v];
  }

  /** Returns the number of vertices in the mesh.
   */
  int num_vertices() const { return vertices_.size(); }

  //@}

  /**
   Constructs a SurfaceMesh from faces and vertices.
   @param faces     The triangular faces.
   @param vertices  The vertices.
   */
  SurfaceMesh(std::vector<SurfaceFace>&& faces,
              std::vector<SurfaceVertex<T>>&& vertices)
      : faces_(std::move(faces)), vertices_(std::move(vertices)),
        area_(faces_.size()) {  // Pre-allocate here, not yet calculated.
    SetReferringTriangles();
    CalcAreasAndCentroid();
  }

  /** Transforms the vertices of this mesh from its initial frame M to the new
   frame N.
   */
  void TransformVertices(const math::RigidTransform<T>& X_NM) {
    for (auto& v : vertices_) {
      v.TransformInPlace(X_NM);
    }
  }

  /** Reverses the ordering of all the faces' indices -- see
    SurfaceFace::ReverseWinding().
   */
  void ReverseFaceWinding() {
    for (auto& f : faces_) {
      f.ReverseWinding();
    }
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
      ElementIndex element_index, const Barycentric& b_Q) const {
    const SurfaceVertex<T> va = vertex(element(element_index).vertex(0));
    const SurfaceVertex<T> vb = vertex(element(element_index).vertex(1));
    const SurfaceVertex<T> vc = vertex(element(element_index).vertex(2));

    // This is just a linear transformation between the two coordinates,
    // Cartesian (C) and Barycentric (B). Form the transformation matrix:
    Matrix3<T> T_CB;
    T_CB.col(0) = va.r_MV();
    T_CB.col(1) = vb.r_MV();
    T_CB.col(2) = vc.r_MV();

    return T_CB * b_Q;
  }

  /** Calculate barycentric coordinates with respect to the triangular face `f`
   of the point Q'. Q' is the projection of the provided point Q on the plane
   of triangle `f`. If Q lies on the plane, Q = Q'. This operation is expensive
   compared with going from barycentric to Cartesian.
   @param p_MQ   The position of point Q measured and expressed in the mesh's
                 frame M.
   @param f      The index of a triangular face.
   @retval b_Q'  The barycentric coordinates of Q' (projection of Q onto `f`'s
                 plane) relative to triangle f.
   @note  If Q' is outside the triangle, the barycentric coordinates
          (b₀, b₁, b₂) still satisfy b₀ + b₁ + b₂ = 1; however, some bᵢ will be
          negative.
   */
  Barycentric CalcBarycentric(const Cartesian& p_MQ, SurfaceFaceIndex f) const {
    const Vector3<T>& v0 = vertex(element(f).vertex(0)).r_MV();
    const Vector3<T>& v1 = vertex(element(f).vertex(1)).r_MV();
    const Vector3<T>& v2 = vertex(element(f).vertex(2)).r_MV();
    // Translate the triangle to the origin to simplify calculations;
    // barycentric coordinates stay the same.
    //     u⃗i = v⃗i - v0
    //     p_MR = p_MQ - v0
    //
    // Consider R' on the spanning plane through the origin, u1, u2:
    //     R' = b₀*u0 + b₁*u1 + b₂*u2
    //        = 0 + b₁*u1 + b₂*u2
    //        = b₁*u1 + b₂*u2
    //
    // Solve for b₁, b₂ that give R' "closest" to R in the least square sense:
    //
    //      |      ||b1|
    //      |u⃗1  u⃗2||b2| ~ R'
    //      |      |
    //
    // return Barycentric (1-b₁-b₂, b₁, b₂)
    //
    Eigen::Matrix<T, 3, 2> A;
    A.col(0) << v1 - v0;
    A.col(1) << v2 - v0;
    Vector2<T> solution = A.colPivHouseholderQr().solve(p_MQ - v0);

    const T& b1 = solution(0);
    const T& b2 = solution(1);
    const T b0 = T(1.) - b1 - b2;
    return Barycentric(b0, b1, b2);
  }
  // TODO(DamrongGuoy): Investigate alternative calculation suggested by
  //  Alejandro Castro:
  // 1. Starting with the same ui and p_MR.
  // 2. Calculate the unit normal vector n to the spanning plane S through
  //    the origin, u1, and u2.
  //        n = u1.cross(u2).normalize().
  // 3. Project p_MR to p_MR' on the plane S,
  //        p_MR' = p_MR - (p_MR.dot(n))*n
  //
  // Now we have p_MR' = b₀*u⃗0 + b₁*u⃗1 + b₂*u⃗2 by barycentric coordinates.
  //                   =   0   + b₁*u1 + b₂*u2
  //
  // 5. Solve for b₁ and b₂.
  //        (b₁*u1 + b₂*u2).dot(u1) = p_MR'.dot(u1)
  //        (b₁*u1 + b₂*u2).dot(u2) = p_MR'.dot(u2)
  //    Therefore, the 2x2 system:
  //        |u1.dot(u1)  u2.dot(u1)||b1| = |p_MR'.dot(u1)|
  //        |u1.dot(u2)  u2.dot(u2)||b2|   |p_MR'.dot(u2)|
  //
  // 6. return Barycentric(1-b₁-b₂, b₁, b₂)
  //
  // Optimization: save n, and the inverse of matrix |uᵢ.dot(uⱼ)| for later.
  //

 private:
  // Calculates the areas of each triangle, the total area, and the centorid of
  // the surface.
  void CalcAreasAndCentroid();

  // Determines the triangular faces that refer to each vertex.
  void SetReferringTriangles() {
    referring_triangles_.resize(num_vertices());

    for (SurfaceFaceIndex i(0); i < num_faces(); ++i) {
      const int kNumVerticesPerFace = 3;
      for (int j = 0; j < kNumVerticesPerFace; ++j)
        referring_triangles_[element(i).vertex(j)].insert(i);
    }
  }

  // The triangles that comprise the surface.
  std::vector<SurfaceFace> faces_;
  // The vertices that are shared among the triangles.
  std::vector<SurfaceVertex<T>> vertices_;

  // Computed in initialization.

  // Triangles that reference each vertex.
  std::vector<std::set<SurfaceFaceIndex>> referring_triangles_;

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

