#pragma once

#include <algorithm>
#include <array>
#include <limits>
#include <set>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"
#include "drake/common/type_safe_index.h"
#include "drake/geometry/proximity/mesh_traits.h"
#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {

using PolygonalSurfaceFaceIndex = TypeSafeIndex<class PolygonalSurfaceFaceTag>;

/** %PolygonalSurfaceFace represents a polygonal face in a PolygonalSurfaceFace.
 */
class PolygonalSurfaceFace {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PolygonalSurfaceFace)

  /** Constructs PolygonalSurfaceFace.
   @param verts Indices of the polygonal face in counter-clockwise order.
   */
  explicit PolygonalSurfaceFace(std::vector<SurfaceVertexIndex> verts)
      : verts_(std::move(verts)) {}

  /** Returns the number of vertices in this face.
   */
  int num_vertices() const {
    return verts_.size();
  }

  /** Returns the vertex index in PolygonalSurfaceMesh of the i-th vertex of
   this face.
   @param i  The local index of the vertex in this face.
   @pre 0 <= i < num_vertices()
   */
  SurfaceVertexIndex vertex(int i) const {
    return verts_.at(i);
  }

  const std::vector<SurfaceVertexIndex>& vertices() const {
    return verts_;
  }

  /** Reverses the order of the vertex indices -- this essentially flips the
   face normal based on the right-handed normal rule.
   */
  void ReverseWinding() {
    std::reverse(verts_.begin(), verts_.end());
  }

 private:
  // The vertices of this face.
  std::vector<SurfaceVertexIndex> verts_;
};



/** %PolygonalSurfaceMesh represents a polygonal surface.
 @tparam T The underlying scalar type for coordinates, e.g., double
           or AutoDiffXd. Must be a valid Eigen scalar.
 */
template <class T>
class PolygonalSurfaceMesh {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PolygonalSurfaceMesh)

  /**
   @name Mesh type traits

   A collection of type traits to enable mesh consumers to be templated on mesh
   type. Each mesh type provides specific definitions of _vertex_, _element_,
   and _barycentric coordinates_. For %PolygonalSurfaceMesh, an element is a
   polygon.
   */
  //@{

  using ScalarType = T;

  /**
   Index for identifying a vertex.
   */
  using VertexIndex = SurfaceVertexIndex;
  /* Note: the vertex type itself is templated (as opposed to just being an
   alias for SurfaceVertex<T>), so that given a Mesh<AutoDiffXd> we can get a
   double valued version of its vertex as: Mesh<AutoDiffXd>::VertexType<double>.
   */
  template <typename U = T>
  using VertexType = SurfaceVertex<U>;

  /**
   Index for identifying a triangular element.
   */
  using ElementIndex = SurfaceFaceIndex;

  /** Returns the triangular element identified by a given index.
    @param e   The index of the triangular element.
    @pre e ∈ {0, 1, 2,..., num_faces()-1}.
   */
  const PolygonalSurfaceFace& element(ElementIndex e) const {
    DRAKE_DEMAND(0 <= e && e < num_faces());
    return faces_[e];
  }

  /** Returns the faces. */
  const std::vector<PolygonalSurfaceFace>& faces() const { return faces_; }

  /** Returns the vertices. */
  const std::vector<SurfaceVertex<T>>& vertices() const { return vertices_; }

  /**
   Returns the vertex identified by a given index.
   @param v  The index of the vertex.
   @pre v ∈ {0, 1, 2,...,num_vertices()-1}.
   */
  const SurfaceVertex<T>& vertex(VertexIndex v) const {
    DRAKE_DEMAND(0 <= v && v < num_vertices());
    return vertices_[v];
  }

  T vertex_value(VertexIndex v) const {
    DRAKE_DEMAND(0 <= v && v < num_vertices());
    return vertex_values_[v];
  }

  // Returning a copy of the vector is safer but slower than returning a const&.
  std::vector<T> vertex_values() const {
    return vertex_values_;
  }


  /** Returns the number of vertices in the mesh.
   */
  int num_vertices() const { return vertices_.size(); }

  /** Returns the number of face in the mesh. For %PolygonalSurfaceMesh, an
   element is a polygon. Returns the same number as num_faces() and enables
   mesh consumers to be templated on mesh type.
   */
  int num_elements() const { return num_faces(); }

  //@}

  /**
   Constructs a PolygonalSurfaceMesh from faces and vertices.
   @param faces     The triangular faces.
   @param vertices  The vertices.
   */
  PolygonalSurfaceMesh(std::vector<PolygonalSurfaceFace>&& faces,
              std::vector<SurfaceVertex<T>>&& vertices,
              std::vector<T>&& vertex_values)
      : faces_(std::move(faces)),
        vertices_(std::move(vertices)),
        vertex_values_(std::move(vertex_values)),
        area_(faces_.size()),  // Pre-allocate here, not yet calculated.
        face_normals_(faces_.size()) {  // Pre-allocate, not yet calculated.
    if (faces_.empty()) {
      throw std::logic_error("A PolygonalSurfaceMesh must contain at least"
                             " one polygon");
    }
    CalcAreasNormalsAndCentroid();
  }

  /** Transforms the vertices of this mesh from its initial frame M to the new
   frame N.
   */
  void TransformVertices(const math::RigidTransform<T>& X_NM) {
    for (auto& v : vertices_) {
      v.TransformInPlace(X_NM);
    }
    for (auto& n : face_normals_) {
      n = X_NM.rotation() * n;
    }
    p_MSc_ = X_NM * p_MSc_;
  }

  /** Reverses the ordering of all the faces' indices -- see
    PolygonalSurfaceFace::ReverseWinding().
   */
  void ReverseFaceWinding() {
    for (auto& f : faces_) {
      f.ReverseWinding();
    }
    for (auto& n : face_normals_) {
      n = -n;
    }
  }

  /** Returns the number of polygonal elements in the mesh.
   */
  int num_faces() const { return faces_.size(); }

  /** Returns area of a polygonal element.
   */
  const T& area(SurfaceFaceIndex f) const { return area_[f]; }

  /** Returns the total area of all the faces of this surface mesh.
   */
  const T& total_area() const { return total_area_; }

  /** Returns the unit face normal vector of a polygon. It respects the
   right-handed normal rule. A near-zero-area triangle may get an unreliable
   normal vector. A zero-area triangle will get a zero vector.
   @pre f ∈ {0, 1, 2,..., num_faces()-1}.
   */
  const Vector3<T>& face_normal(PolygonalSurfaceFaceIndex f) const {
    DRAKE_DEMAND(0 <= f && f < num_faces());
    return face_normals_[f];
  }

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

  // TODO(DamrongGuoy): Consider using an oriented bounding box in obb.h.
  //  Currently we have a problem that PolygonalSurfaceMesh and its vertices are
  //  templated on T, but Obb is for double only.
  /**
   Calculates the axis-aligned bounding box of this surface mesh M.
   @returns the center and the size vector of the box expressed in M's frame.
   */
  std::pair<Vector3<T>, Vector3<T>> CalcBoundingBox() const {
    Vector3<T> min_extent =
        Vector3<T>::Constant(std::numeric_limits<double>::max());
    Vector3<T> max_extent =
        Vector3<T>::Constant(std::numeric_limits<double>::lowest());
    for (SurfaceVertexIndex i(0); i < num_vertices(); ++i) {
      Vector3<T> vertex = this->vertex(i).r_MV();
      min_extent = min_extent.cwiseMin(vertex);
      max_extent = max_extent.cwiseMax(vertex);
    }
    Vector3<T> center = (max_extent + min_extent) / 2.0;
    Vector3<T> size = max_extent - min_extent;
    return std::make_pair(center, size);
  }

  // TODO(#12173): Consider NaN==NaN to be true in equality tests.
  /** Checks to see whether the given PolygonalSurfaceMesh object is equal
   via deep exact comparison. NaNs are treated as not equal as per the IEEE
   standard.
   @param mesh The mesh for comparison.
   @returns `true` if the given mesh is equal.
   */
  bool Equal(const PolygonalSurfaceMesh<T>& mesh) const {
    if (this == &mesh) return true;

    if (this->num_faces() != mesh.num_faces()) return false;
    if (this->num_vertices() != mesh.num_vertices()) return false;

    // Check face indices.
    for (SurfaceFaceIndex i(0); i < this->num_faces(); ++i) {
      const PolygonalSurfaceFace& face1 = this->element(i);
      const PolygonalSurfaceFace& face2 = mesh.element(i);
      if (face1.num_vertices() != face2.num_vertices()) return false;
      for (int j = 0; j < face1.num_vertices(); ++j)
        if (face1.vertex(j) != face2.vertex(j)) return false;
    }

    // Check vertices.
    for (SurfaceVertexIndex i(0); i < this->num_vertices(); ++i) {
      if (this->vertex(i).r_MV() != mesh.vertex(i).r_MV()) return false;
    }

    // All checks passed.
    return true;
  }


//  /** Calculates the gradient ∇u of a linear field u on the triangle `f`.
//   Field u is defined by the three field values `field_value[i]` at the i-th
//   vertex of the triangle. The gradient ∇u is expressed in the coordinates
//   frame of this mesh M.
//   */
//  template <typename FieldValue>
//  Vector3<FieldValue> CalcGradientVectorOfLinearField(
//      const std::array<FieldValue, 3>& field_value,
//      PolygonalSurfaceFaceIndex f) const {
//    Vector3<FieldValue> gradu_M = field_value[0] * CalcGradBarycentric(f, 0);
//    gradu_M += field_value[1] * CalcGradBarycentric(f, 1);
//    gradu_M += field_value[2] * CalcGradBarycentric(f, 2);
//    return gradu_M;
//  }

 private:
  // Calculates the areas and face normals of each triangle, the total area,
  // and the centroid of the surface.
  void CalcAreasNormalsAndCentroid();

  // The triangles that comprise the surface.
  std::vector<PolygonalSurfaceFace> faces_;
  // The vertices that are shared among the triangles.
  std::vector<SurfaceVertex<T>> vertices_;
  std::vector<T> vertex_values_;

  // Computed in initialization.

  // Area of the triangles.
  std::vector<T> area_;
  T total_area_{};

  // Face normal vector of the triangles.
  std::vector<Vector3<T>> face_normals_;

  // Area-weighted geometric centroid Sc of the surface mesh as an offset vector
  // from the origin of Frame M to point Sc, expressed in Frame M.
  Vector3<T> p_MSc_;
};

template <class T>
void PolygonalSurfaceMesh<T>::CalcAreasNormalsAndCentroid() {
  total_area_ = 0;
  p_MSc_.setZero();

  for (SurfaceFaceIndex f(0); f < faces_.size(); ++f) {
    const PolygonalSurfaceFace& face = faces_[f];
    const Vector3<T>& r_MA = vertices_[face.vertex(0)].r_MV();

    Vector3<T> p_MFc = r_MA;  // Begin accumulation the centroid of the polygon.

    Vector3<T> cross;
    T face_area(0);

    for (int v = 1; v < face.num_vertices() - 1; ++v) {
      const Vector3<T>& r_MB = vertices_[face.vertex(v)].r_MV();
      const Vector3<T>& r_MC = vertices_[face.vertex(v+1)].r_MV();
      const auto r_UV_M = r_MB - r_MA;
      const auto r_UW_M = r_MC - r_MA;

      cross = r_UV_M.cross(r_UW_M);
      face_area += cross.norm();

      p_MFc += r_MB;
    }

    p_MFc += vertices_[face.vertex(face.num_vertices() - 1)].r_MV();

    area_[f] = T(0.5) * face_area;
    total_area_ += face_area;

    // TODO(DamrongGuoy): Use the right norm.
    T norm = 1.0;

    face_normals_[f] = (norm != T(0.0)) ? cross / norm : cross;

    // Accumulate area-weighted surface centroid; must be divided by 3X the
    // total area afterwards.
    p_MSc_ += face_area * p_MFc;
  }

  // Finalize centroid.
  if (total_area_ != T(0.))
    p_MSc_ /= (3. * total_area_);
}

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class PolygonalSurfaceMesh)

}  // namespace geometry
}  // namespace drake
