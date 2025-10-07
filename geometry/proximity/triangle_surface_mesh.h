#pragma once

#include <array>
#include <limits>
#include <optional>
#include <set>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/mesh_traits.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
/** %SurfaceTriangle represents a triangular face in a TriangleSurfaceMesh.
 */
class SurfaceTriangle {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SurfaceTriangle);

  /** Constructs SurfaceTriangle.
   @param v0 Index of the first vertex in TriangleSurfaceMesh.
   @param v1 Index of the second vertex in TriangleSurfaceMesh.
   @param v2 Index of the last vertex in TriangleSurfaceMesh.
   @pre index values are non-negative. */
  SurfaceTriangle(int v0, int v1, int v2) : vertex_({v0, v1, v2}) {
    DRAKE_DEMAND(v0 >= 0 && v1 >= 0 && v2 >= 0);
  }

  /** Constructs SurfaceTriangle.
   @param v  array of three integer indices of the vertices of the triangle in
             TriangleSurfaceMesh.
   @pre index values are non-negative. */
  explicit SurfaceTriangle(const int v[3])
      : SurfaceTriangle(v[0], v[1], v[2]) {}

  /** Returns the number of vertices in this face. */
  int num_vertices() const { return 3; }

  /** Returns the vertex index in TriangleSurfaceMesh of the i-th vertex of this
   triangle.
   @param i  The local index of the vertex in this triangle.
   @pre 0 <= i < 3
   */
  int vertex(int i) const { return vertex_.at(i); }

  /** Reverses the order of the vertex indices -- this essentially flips the
   triangle normal based on the right-handed normal rule.
   */
  void ReverseWinding() { std::swap(vertex_[0], vertex_[1]); }

 private:
  // The vertices of this triangle.
  std::array<int, 3> vertex_;
};

// Forward declaration of TriangleSurfaceMeshTester<T> for friend access.
template <typename T>
class TriangleSurfaceMeshTester;

// TODO(DamrongGuoy): mention interesting properties of the mesh, e.g., open
//  meshes, meshes with holes, non-manifold surface.
/** %TriangleSurfaceMesh represents a union of triangles. The surface is not
 necessarily continuous.

 @tparam_nonsymbolic_scalar
 */
template <class T>
class TriangleSurfaceMesh {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(TriangleSurfaceMesh);

  /**
   @name Mesh type traits

   A collection of type traits to enable mesh consumers to be templated on mesh
   type. Each mesh type provides specific definitions of _vertex_, _element_,
   and _barycentric coordinates_. For %TriangleSurfaceMesh, an element is a
   triangle.
   */
  //@{

  using ScalarType = T;

  /**
   Number of vertices per element. A triangle has 3 vertices.
   */
  static constexpr int kVertexPerElement = 3;

  // TODO(SeanCurtis-TRI) This is very dissatisfying. The alias contained in a
  //  templated class doesn't depend on the class template parameter, but
  //  depends on some non-template-dependent property (kVertexPerElement).
  //  That means we *apparently* have different types:
  //    TriangleSurfaceMesh<double>::Barycentric<double>
  //    TriangleSurfaceMesh<AutoDiffXd>::Barycentric<double>
  // But, ultimately both become Vector3d and, because they are simply aliases,
  // are interchangeable. It would be nice to have some way of formulating this
  // that *doesn't* imply dependency on the scalar type of TriangleSurfaceMesh.
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
  template <typename U = T>
  using Barycentric = Vector<U, kVertexPerElement>;

  /** Returns the triangular element identified by a given index.
    @param e   The index of the triangular element.
    @pre e ∈ {0, 1, 2,..., num_triangles()-1}.
   */
  const SurfaceTriangle& element(int e) const {
    DRAKE_DEMAND(0 <= e && e < num_triangles());
    return triangles_[e];
  }

  /** Returns the centroid of a triangle measured and expressed in the mesh's
   frame. */
  Vector3<T> element_centroid(int t) const {
    DRAKE_DEMAND(0 <= t && t < num_triangles());
    /* We're not pre-computing and returning stored values because the current
     expectation is that the cost of storing the values (greater construction
     time, more complexity in code) is probably not justified based on how
     many times the centroid would be accessed. So, for now, we'll compute on
     the fly until we know that the query cost dominates. */
    const auto& tri = triangles_[t];
    return (vertices_M_[tri.vertex(0)] + vertices_M_[tri.vertex(1)] +
            vertices_M_[tri.vertex(2)]) /
           3;
  }

  /** Returns the triangles. */
  const std::vector<SurfaceTriangle>& triangles() const { return triangles_; }

  /** Returns the vertices. */
  const std::vector<Vector3<T>>& vertices() const { return vertices_M_; }

  /**
   Returns the vertex identified by a given index.
   @param v  The index of the vertex.
   @pre v ∈ {0, 1, 2,...,num_vertices()-1}.
   */
  const Vector3<T>& vertex(int v) const {
    DRAKE_DEMAND(0 <= v && v < num_vertices());
    return vertices_M_[v];
  }

  /** Returns the number of vertices in the mesh.
   */
  int num_vertices() const { return vertices_M_.size(); }

  /** Returns the number of triangles in the mesh. For %TriangleSurfaceMesh, an
   element is a triangle. Returns the same number as num_triangles() and enables
   mesh consumers to be templated on mesh type.
   */
  int num_elements() const { return num_triangles(); }

  //@}

  /**
   Constructs a TriangleSurfaceMesh from triangles and vertices.
   @param triangles The triangular triangles.
   @param vertices  The vertices.
   */
  TriangleSurfaceMesh(std::vector<SurfaceTriangle>&& triangles,
                      std::vector<Vector3<T>>&& vertices)
      : triangles_(std::move(triangles)),
        vertices_M_(std::move(vertices)),
        area_(triangles_.size()),  // Pre-allocate here, not yet calculated.
        face_normals_(triangles_.size()) {  // Pre-allocate, not yet calculated.
    if (triangles_.empty()) {
      throw std::logic_error("A mesh must contain at least one triangle");
    }
    ComputePositionDependentQuantities();
  }

  // TODO(SeanCurtis-TRI) This shouldn't be called "TransformVertices"; the name
  //  is misleading. It transforms more than just vertex positions. It should
  //  simply be called Transform and documented as "transforming the mesh's
  //  frame-dependent quantities from frame M to the new frame N".
  /** (Internal use only) Transforms the vertices of this mesh from its
   initial frame M to the new frame N.
   */
  void TransformVertices(const math::RigidTransform<T>& X_NM) {
    for (auto& v : vertices_M_) {
      v = X_NM * v;
    }
    for (auto& n : face_normals_) {
      n = X_NM.rotation() * n;
    }
    p_MSc_ = X_NM * p_MSc_;
  }

  /** (Internal use only) Reverses the ordering of all the triangles' indices
   -- see SurfaceTriangle::ReverseWinding().
   */
  void ReverseFaceWinding();

  /** Returns the number of triangles in the mesh.
   */
  int num_triangles() const { return triangles_.size(); }

  /** Returns area of triangle `t`.
   @pre t ∈ {0, 1, 2,..., num_triangles()-1}.
   */
  const T& area(int t) const {
    DRAKE_DEMAND(0 <= t && t < num_triangles());
    return area_[t];
  }

  /** Returns the total area of all the triangles of this surface mesh.
   */
  const T& total_area() const { return total_area_; }

  /** Returns the unit face normal vector of a triangle. It respects the
   right-handed normal rule. A near-zero-area triangle may get an unreliable
   normal vector. A zero-area triangle will get a zero vector.
   @pre t ∈ {0, 1, 2,..., num_triangles()-1}.
   */
  const Vector3<T>& face_normal(int t) const {
    DRAKE_DEMAND(0 <= t && t < num_triangles());
    return face_normals_[t];
  }

  /** Returns the area-weighted geometric centroid of this surface mesh. The
   returned value is the position vector p_MSc from M's origin to the
   centroid Sc, expressed in frame M. (M is the frame in which this mesh's
   vertices are measured and expressed.) Note that the centroid is not
   necessarily a point on the surface. If the total mesh area is exactly
   zero, we define the centroid to be (0,0,0).

   The centroid location is calculated _per face_ not _per vertex_ so is
   insensitive to whether vertices are shared by triangles.
   */
  const Vector3<T>& centroid() const { return p_MSc_; }

  /**
   Maps the barycentric coordinates `Q_barycentric` of a point Q in
   `element_index` to its position vector p_MQ.

   The return type depends on both the mesh's vertex position scalar type `T`
   and the Barycentric coordinate type `B` of the query point.  See
   @ref drake::geometry::promoted_numerical "promoted_numerical_t" for details.
   @pre `element_index` ∈ {0, 1, 2,..., num_triangles()-1}.
   */
  template <typename B>
  Vector3<promoted_numerical_t<T, B>> CalcCartesianFromBarycentric(
      int element_index, const Barycentric<B>& b_Q) const {
    const Vector3<T> va = vertex(element(element_index).vertex(0));
    const Vector3<T> vb = vertex(element(element_index).vertex(1));
    const Vector3<T> vc = vertex(element(element_index).vertex(2));

    // This is just a linear transformation between the two coordinates,
    // Cartesian (C) and Barycentric (B). Form the transformation matrix:
    Matrix3<promoted_numerical_t<T, B>> T_CB;
    T_CB.col(0) = va;
    T_CB.col(1) = vb;
    T_CB.col(2) = vc;

    return T_CB * b_Q;
  }

  /** Calculate barycentric coordinates with respect to the triangle `t`
   of the point Q'. Q' is the projection of the provided point Q on the plane
   of triangle `t`. If Q lies on the plane, Q = Q'. This operation is expensive
   compared with going from barycentric to Cartesian.

   The return type depends on both the mesh's vertex position scalar type `T`
   and the Cartesian coordinate type `C` of the query point.  See
   @ref drake::geometry::promoted_numerical "promoted_numerical_t" for details.

   @param p_MQ   The position of point Q measured and expressed in the mesh's
                 frame M.
   @param t      The index of a triangle.
   @retval b_Q'  The barycentric coordinates of Q' (projection of Q onto `t`'s
                 plane) relative to triangle t.
   @note  If Q' is outside the triangle, the barycentric coordinates
          (b₀, b₁, b₂) still satisfy b₀ + b₁ + b₂ = 1; however, some bᵢ will be
          negative.
   @pre t ∈ {0, 1, 2,..., num_triangles()-1}.
   @tparam C must be either `double` or `AutoDiffXd`. */
  template <typename C>
  Barycentric<promoted_numerical_t<T, C>> CalcBarycentric(
      const Vector3<C>& p_MQ, int t) const
#ifndef DRAKE_DOXYGEN_CXX
    requires scalar_predicate<C>::is_bool
#endif
  ;  // NOLINT(whitespace/semicolon)

  // TODO(DamrongGuoy): Consider using an oriented bounding box in obb.h.
  //  Currently we have a problem that TriangleSurfaceMesh and its vertices are
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
    for (int i = 0; i < num_vertices(); ++i) {
      Vector3<T> vertex = this->vertex(i);
      min_extent = min_extent.cwiseMin(vertex);
      max_extent = max_extent.cwiseMax(vertex);
    }
    Vector3<T> center = (max_extent + min_extent) / 2.0;
    Vector3<T> size = max_extent - min_extent;
    return std::make_pair(center, size);
  }

  // TODO(#12173): Consider NaN==NaN to be true in equality tests.
  /** Checks to see whether the given TriangleSurfaceMesh object is equal via
   deep exact comparison. NaNs are treated as not equal as per the IEEE
   standard.
   @param mesh The mesh for comparison.
   @returns `true` if the given mesh is equal.
   */
  bool Equal(const TriangleSurfaceMesh<T>& mesh) const {
    if (this == &mesh) return true;

    if (this->num_triangles() != mesh.num_triangles()) return false;
    if (this->num_vertices() != mesh.num_vertices()) return false;

    // Check face indices.
    for (int i = 0; i < this->num_triangles(); ++i) {
      const SurfaceTriangle& face1 = this->element(i);
      const SurfaceTriangle& face2 = mesh.element(i);
      for (int j = 0; j < 3; ++j)
        if (face1.vertex(j) != face2.vertex(j)) return false;
    }

    // Check vertices.
    for (int i = 0; i < this->num_vertices(); ++i) {
      if (this->vertex(i) != mesh.vertex(i)) return false;
    }

    // All checks passed.
    return true;
  }

  /** Calculates the gradient ∇u of a linear field u on the triangle `t`.
   Field u is defined by the three field values `field_value[i]` at the i-th
   vertex of the triangle. The gradient ∇u is expressed in the coordinates
   frame of this mesh M.
   */
  template <typename FieldValue>
  Vector3<FieldValue> CalcGradientVectorOfLinearField(
      const std::array<FieldValue, 3>& field_value, int t) const {
    std::optional<Vector3<FieldValue>> grad =
        MaybeCalcGradientVectorOfLinearField(field_value, t);
    if (!grad.has_value()) {
      throw std::runtime_error("Bad geometry; could not calculate gradient.");
    }
    return grad.value();
  }

  /** Calculates the gradient ∇u of a linear field u on the triangle `t`.
   Field u is defined by the three field values `field_value[i]` at the i-th
   vertex of the triangle. The gradient ∇u is expressed in the coordinates
   frame of this mesh M.
   */
  template <typename FieldValue>
  std::optional<Vector3<FieldValue>> MaybeCalcGradientVectorOfLinearField(
      const std::array<FieldValue, 3>& field_value, int t) const {
    Vector3<FieldValue> gradu_M = Vector3<FieldValue>::Zero();
    for (int k = 0; k < 3; ++k) {
      auto grad_k = MaybeCalcGradBarycentric(t, k);
      if (!grad_k.has_value()) {
        return {};
      }
      gradu_M += field_value[k] * grad_k.value();
    }
    return gradu_M;
  }

  /** Updates the position of all vertices in the mesh. Each sequential triple
   in p_MVs (e.g., 3i, 3i + 1, 3i + 2), i ∈ ℤ, is interpreted as a position
   vector associated with the iᵗʰ vertex. The position values are interpreted to
   be measured and expressed in the same frame as the mesh to be deformed.

   @param p_MVs  Vertex positions for the mesh's N vertices flattened into a
                 vector (where each position vector is measured and expressed in
                 the mesh's original frame).
   @throws std::exception if p_MVs.size() != 3 * num_vertices() */
  void SetAllPositions(const Eigen::Ref<const VectorX<T>>& p_MVs);

 private:
  // Calculates the areas and face normals of each triangle, the total area,
  // and the centroid of the surface.
  void ComputePositionDependentQuantities();

  // Calculates the gradient vector ∇bᵢ of the barycentric coordinate
  // function bᵢ of the i-th vertex of the triangle `t`. The gradient
  // vector ∇bᵢ is expressed in the coordinates frame of this mesh M.
  // @pre  0 ≤ i < 3.
  // @throws if triangle is degenerate.
  // TODO(rpoyner-tri): currently only used by the test helper; delete?
  Vector3<T> CalcGradBarycentric(int t, int i) const {
    auto result = MaybeCalcGradBarycentric(t, i);
    if (!result.has_value()) {
      throw std::runtime_error("Bad geometry; could not calculate gradient.");
    }
    return *result;
  }

  // Like CalcGradBarycentric, but returns std::nullopt if triangle is
  // degenerate.
  std::optional<Vector3<T>> MaybeCalcGradBarycentric(int t, int i) const;

  // The triangles that comprise the surface.
  std::vector<SurfaceTriangle> triangles_;
  // The vertices that are shared among the triangles, measured and expressed in
  // the mesh's frame M.
  std::vector<Vector3<T>> vertices_M_;

  // Computed in initialization.

  // Area of the triangles.
  std::vector<T> area_;
  T total_area_{};

  // Face normal vector of the triangles.
  std::vector<Vector3<T>> face_normals_;

  // Area-weighted geometric centroid Sc of the surface mesh as an offset vector
  // from the origin of Frame M to point Sc, expressed in Frame M.
  Vector3<T> p_MSc_;

  friend class TriangleSurfaceMeshTester<T>;
};

template <class T>
void TriangleSurfaceMesh<T>::ComputePositionDependentQuantities() {
  total_area_ = 0;
  p_MSc_.setZero();

  for (int f = 0; f < num_triangles(); ++f) {
    const SurfaceTriangle& face = triangles_[f];
    const Vector3<T>& r_MA = vertices_M_[face.vertex(0)];
    const Vector3<T>& r_MB = vertices_M_[face.vertex(1)];
    const Vector3<T>& r_MC = vertices_M_[face.vertex(2)];
    const auto r_UV_M = r_MB - r_MA;
    const auto r_UW_M = r_MC - r_MA;

    const auto cross = r_UV_M.cross(r_UW_M);
    const T norm = cross.norm();
    const T face_area = T(0.5) * norm;
    area_[f] = face_area;
    total_area_ += face_area;

    // TODO(DamrongGuoy): Provide a mechanism for users to set the face
    //  normals of skinny triangles since this calculation is not reliable.
    //  For example, the code that creates ContactSurface by
    //  triangle-tetrahedron intersection can set more reliable normal vectors
    //  for the skinny intersecting triangles.  Related to issue# 12110.
    face_normals_[f] = (norm != T(0.0)) ? cross / norm : cross;

    // Accumulate area-weighted surface centroid; must be divided by 3X the
    // total area afterwards.
    p_MSc_ += face_area * (r_MA + r_MB + r_MC);
  }

  // Finalize centroid.
  if (total_area_ != T(0.)) p_MSc_ /= (3. * total_area_);
}

template <typename T>
std::optional<Vector3<T>> TriangleSurfaceMesh<T>::MaybeCalcGradBarycentric(
    int t, int i) const {
  DRAKE_DEMAND(0 <= i && i < 3);
  DRAKE_DEMAND(0 <= t && t < num_triangles());
  // Vertex V corresponds to bᵢ in the barycentric coordinate in the triangle
  // indexed by `t`. A and B are the other two vertices of the triangle.
  // Positions of the vertices are expressed in frame M of the mesh.
  const Vector3<T>& p_MV = vertices_M_[triangles_[t].vertex(i)];
  const Vector3<T>& p_MA = vertices_M_[triangles_[t].vertex((i + 1) % 3)];
  const Vector3<T>& p_MB = vertices_M_[triangles_[t].vertex((i + 2) % 3)];

  // TODO(DamrongGuoy): Provide a mechanism for users to set the gradient
  //  vector in TriangleSurfaceMeshFieldLinear since this calculation is not
  //  reliable for zero- or almost-zero-area triangles. For example, the code
  //  that creates ContactSurface by triangle-tetrahedron intersection can set
  //  the pressure gradient along a contact polygon by projecting the compliant
  //  tetrahedron's pressure gradient onto the plane of the rigid triangle.

  // Let bᵥ be the barycentric coordinate function corresponding to vertex V.
  // bᵥ is a linear function of the points in the triangle.
  // bᵥ = 0 on the line through AB.
  // bᵥ = 1 on the line through V parallel to AB.
  // Therefore, bᵥ changes fastest in the direction perpendicular to AB
  // towards V with the rate of change 1/h, where h is the height of vertex V
  // from the base AB.
  //
  //    ──────────────V────────────── bᵥ = 1
  //                 ╱↑╲       ┊
  //                ╱ ┊ ╲      ┊
  //               ╱  ┊  ╲     ┊ h
  //              ╱   ┊H  ╲    ┊
  //             ╱    ┊    ╲   ┊
  //    ────────A━━━━━━━━━━━B──────── bᵥ = 0
  //
  // Let H be the height vector from the base AB to the vertex V, i.e., the
  // vector perpendicular to the line AB that starts from a point on the
  // line and ends at V. The length of H is h. The gradient vector ∇bᵥ is
  // along the unit vector H/h. Along that direction, bᵥ changes at the rate
  // of 1/h per unit distance. Therefore,
  //
  //       ∇bᵥ = H/h².
  //
  // We can calculate H from AV by subtracting its component along AB:
  //
  //       H = AV - (AV⋅AB)AB/|AB|²
  //
  const Vector3<T> p_AB_M = p_MB - p_MA;
  const T ab2 = p_AB_M.squaredNorm();
  const Vector3<T> p_AV_M = p_MV - p_MA;
  constexpr double kEps = std::numeric_limits<double>::epsilon();
  constexpr double kEps2 = kEps * kEps;
  // For a skinny triangle with the edge AB that has zero or almost-zero
  // length, we set the vector H to be AV.
  const Vector3<T> H_M =
      (ab2 <= kEps2) ? p_AV_M : p_AV_M - (p_AV_M.dot(p_AB_M)) * p_AB_M / ab2;
  const T h2 = H_M.squaredNorm();
  if (h2 <= kEps2) {
    return {};
  }
  return H_M / h2;
}

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class TriangleSurfaceMesh);

}  // namespace geometry
}  // namespace drake
