#pragma once

#include <array>
#include <limits>
#include <set>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/mesh_traits.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
/** %SurfaceFace represents a triangular face in a SurfaceMesh.
 */
class SurfaceFace {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SurfaceFace)

  /** Constructs SurfaceFace.
   @param v0 Index of the first vertex in SurfaceMesh.
   @param v1 Index of the second vertex in SurfaceMesh.
   @param v2 Index of the last vertex in SurfaceMesh.
   @pre index values are non-negative. */
  SurfaceFace(int v0, int v1, int v2) : vertex_({v0, v1, v2}) {
    DRAKE_DEMAND(v0 >= 0 && v1 >= 0 && v2 >= 0);
  }

  /** Constructs SurfaceFace.
   @param v  array of three integer indices of the vertices of the face in
             SurfaceMesh.
   @pre index values are non-negative. */
  explicit SurfaceFace(const int v[3]) : SurfaceFace(v[0], v[1], v[2]) {}

  /** Returns the vertex index in SurfaceMesh of the i-th vertex of this face.
   @param i  The local index of the vertex in this face.
   @pre 0 <= i < 3
   */
  int vertex(int i) const { return vertex_.at(i); }

  /** Reverses the order of the vertex indices -- this essentially flips the
   face normal based on the right-handed normal rule.
   */
  void ReverseWinding() {
    std::swap(vertex_[0], vertex_[1]);
  }

 private:
  // The vertices of this face.
  std::array<int, 3> vertex_;
};

namespace internal {
// Forward declaration for friend declaration.
template <typename> class MeshDeformer;
}  // namespace internal

// Forward declaration of SurfaceMeshTester<T>. SurfaceMesh<T> will
// grant friend access to SurfaceMeshTester<T>.
template <typename T> class SurfaceMeshTester;

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

  using ScalarType = T;

  /**
   Number of vertices per element. A triangle has 3 vertices.
   */
  static constexpr int kVertexPerElement = 3;

  // TODO(SeanCurtis-TRI) This is very dissatisfying. The alias contained in a
  //  templated class doesn't depend on the class template parameter, but
  //  depends on some non-template-dependent property (kVertexPerElement).
  //  That means we *apparently* have different types:
  //    SurfaceMesh<double>::Barycentric<double>
  //    SurfaceMesh<AutoDiffXd>::Barycentric<double>
  // But, ultimately both become Vector3d and, because they are simply aliases,
  // are interchangeable. It would be nice to have some way of formulating this
  // that *doesn't* imply dependency on the scalar type of SurfaceMesh.
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
    @pre e ∈ {0, 1, 2,..., num_faces()-1}.
   */
  const SurfaceFace& element(int e) const {
    DRAKE_DEMAND(0 <= e && e < num_faces());
    return faces_[e];
  }

  /** Returns the faces. */
  const std::vector<SurfaceFace>& faces() const { return faces_; }

  /** Returns the vertices. */
  const std::vector<Vector3<T>>& vertices() const { return vertices_; }

  /**
   Returns the vertex identified by a given index.
   @param v  The index of the vertex.
   @pre v ∈ {0, 1, 2,...,num_vertices()-1}.
   */
  const Vector3<T>& vertex(int v) const {
    DRAKE_DEMAND(0 <= v && v < num_vertices());
    return vertices_[v];
  }

  /** Returns the number of vertices in the mesh.
   */
  int num_vertices() const { return vertices_.size(); }

  /** Returns the number of triangles in the mesh. For %SurfaceMesh, an
   element is a triangle. Returns the same number as num_faces() and enables
   mesh consumers to be templated on mesh type.
   */
  int num_elements() const { return num_faces(); }

  //@}

  /**
   Constructs a SurfaceMesh from faces and vertices.
   @param faces     The triangular faces.
   @param vertices  The vertices.
   */
  SurfaceMesh(std::vector<SurfaceFace>&& faces,
              std::vector<Vector3<T>>&& vertices)
      : faces_(std::move(faces)),
        vertices_(std::move(vertices)),
        area_(faces_.size()),  // Pre-allocate here, not yet calculated.
        face_normals_(faces_.size()) {  // Pre-allocate, not yet calculated.
    if (faces_.empty()) {
      throw std::logic_error("A mesh must contain at least one triangle");
    }
    CalcAreasNormalsAndCentroid();
  }

  /** Transforms the vertices of this mesh from its initial frame M to the new
   frame N.
   */
  void TransformVertices(const math::RigidTransform<T>& X_NM) {
    for (auto& v : vertices_) {
      v = X_NM * v;
    }
    for (auto& n : face_normals_) {
      n = X_NM.rotation() * n;
    }
    p_MSc_ = X_NM * p_MSc_;
  }

  /** Reverses the ordering of all the faces' indices -- see
    SurfaceFace::ReverseWinding().
   */
  void ReverseFaceWinding() {
    for (auto& f : faces_) {
      f.ReverseWinding();
    }
    for (auto& n : face_normals_) {
      n = -n;
    }
  }

  /** Returns the number of triangular elements in the mesh.
   */
  int num_faces() const { return faces_.size(); }

  /** Returns area of a triangular element.
   @pre f ∈ {0, 1, 2,..., num_faces()-1}.
   */
  const T& area(int f) const {
    DRAKE_DEMAND(0 <= f && f < num_faces());
    return area_[f];
  }

  /** Returns the total area of all the faces of this surface mesh.
   */
  const T& total_area() const { return total_area_; }

  /** Returns the unit face normal vector of a triangle. It respects the
   right-handed normal rule. A near-zero-area triangle may get an unreliable
   normal vector. A zero-area triangle will get a zero vector.
   @pre f ∈ {0, 1, 2,..., num_faces()-1}.
   */
  const Vector3<T>& face_normal(int f) const {
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

  /**
   Maps the barycentric coordinates `Q_barycentric` of a point Q in
   `element_index` to its position vector p_MQ.

   The return type depends on both the mesh's vertex position scalar type `T`
   and the Barycentric coordinate type `B` of the query point.  See
   @ref drake::geometry::promoted_numerical "promoted_numerical_t" for details.
   @pre `element_index` ∈ {0, 1, 2,..., num_faces()-1}.
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

  /** Calculate barycentric coordinates with respect to the triangular face `f`
   of the point Q'. Q' is the projection of the provided point Q on the plane
   of triangle `f`. If Q lies on the plane, Q = Q'. This operation is expensive
   compared with going from barycentric to Cartesian.

   The return type depends on both the mesh's vertex position scalar type `T`
   and the Cartesian coordinate type `C` of the query point.  See
   @ref drake::geometry::promoted_numerical "promoted_numerical_t" for details.

   @param p_MQ   The position of point Q measured and expressed in the mesh's
                 frame M.
   @param f      The index of a triangular face.
   @retval b_Q'  The barycentric coordinates of Q' (projection of Q onto `f`'s
                 plane) relative to triangle f.
   @note  If Q' is outside the triangle, the barycentric coordinates
          (b₀, b₁, b₂) still satisfy b₀ + b₁ + b₂ = 1; however, some bᵢ will be
          negative.
   @pre f ∈ {0, 1, 2,..., num_faces()-1}.
   */
  template <typename C>
  Vector3<promoted_numerical_t<T, C>> CalcBarycentric(const Vector3<C>& p_MQ,
                                                      int f) const {
    const Vector3<T>& v0 = vertex(element(f).vertex(0));
    const Vector3<T>& v1 = vertex(element(f).vertex(1));
    const Vector3<T>& v2 = vertex(element(f).vertex(2));
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
    using ReturnType = promoted_numerical_t<T, C>;
    Eigen::Matrix<ReturnType, 3, 2> A;
    A.col(0) << v1 - v0;
    A.col(1) << v2 - v0;
    Vector2<ReturnType> solution = A.colPivHouseholderQr().solve(p_MQ - v0);

    const ReturnType& b1 = solution(0);
    const ReturnType& b2 = solution(1);
    const ReturnType b0 = T(1.) - b1 - b2;
    return {b0, b1, b2};
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

  // TODO(DamrongGuoy): Consider using an oriented bounding box in obb.h.
  //  Currently we have a problem that SurfaceMesh and its vertices are
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
  /** Checks to see whether the given SurfaceMesh object is equal via deep
   exact comparison. NaNs are treated as not equal as per the IEEE standard.
   @param mesh The mesh for comparison.
   @returns `true` if the given mesh is equal.
   */
  bool Equal(const SurfaceMesh<T>& mesh) const {
    if (this == &mesh) return true;

    if (this->num_faces() != mesh.num_faces()) return false;
    if (this->num_vertices() != mesh.num_vertices()) return false;

    // Check face indices.
    for (int i = 0; i < this->num_faces(); ++i) {
      const SurfaceFace& face1 = this->element(i);
      const SurfaceFace& face2 = mesh.element(i);
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

  /** Calculates the gradient ∇u of a linear field u on the triangle `f`.
   Field u is defined by the three field values `field_value[i]` at the i-th
   vertex of the triangle. The gradient ∇u is expressed in the coordinates
   frame of this mesh M.
   */
  template <typename FieldValue>
  Vector3<FieldValue> CalcGradientVectorOfLinearField(
      const std::array<FieldValue, 3>& field_value, int f) const {
    Vector3<FieldValue> gradu_M = field_value[0] * CalcGradBarycentric(f, 0);
    gradu_M += field_value[1] * CalcGradBarycentric(f, 1);
    gradu_M += field_value[2] * CalcGradBarycentric(f, 2);
    return gradu_M;
  }

 private:
  // Client attorney class that provides a means to modify vertex positions.
  friend class internal::MeshDeformer<SurfaceMesh<T>>;

  // Calculates the areas and face normals of each triangle, the total area,
  // and the centroid of the surface.
  void CalcAreasNormalsAndCentroid();

  // Calculates the gradient vector ∇bᵢ of the barycentric coordinate
  // function bᵢ of the i-th vertex of the triangle `f`. The gradient
  // vector ∇bᵢ is expressed in the coordinates frame of this mesh M.
  // @pre  0 ≤ i < 3.
  Vector3<T> CalcGradBarycentric(int f, int i) const;

  // The triangles that comprise the surface.
  std::vector<SurfaceFace> faces_;
  // The vertices that are shared among the triangles.
  std::vector<Vector3<T>> vertices_;

  // Computed in initialization.

  // Area of the triangles.
  std::vector<T> area_;
  T total_area_{};

  // Face normal vector of the triangles.
  std::vector<Vector3<T>> face_normals_;

  // Area-weighted geometric centroid Sc of the surface mesh as an offset vector
  // from the origin of Frame M to point Sc, expressed in Frame M.
  Vector3<T> p_MSc_;

  friend class SurfaceMeshTester<T>;
};

template <class T>
void SurfaceMesh<T>::CalcAreasNormalsAndCentroid() {
  total_area_ = 0;
  p_MSc_.setZero();

  for (int f = 0; f < num_faces(); ++f) {
    const SurfaceFace& face = faces_[f];
    const Vector3<T>& r_MA = vertices_[face.vertex(0)];
    const Vector3<T>& r_MB = vertices_[face.vertex(1)];
    const Vector3<T>& r_MC = vertices_[face.vertex(2)];
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
    face_normals_[f] = (norm != T(0.0))? cross / norm : cross;

    // Accumulate area-weighted surface centroid; must be divided by 3X the
    // total area afterwards.
    p_MSc_ += face_area * (r_MA + r_MB + r_MC);
  }

  // Finalize centroid.
  if (total_area_ != T(0.))
    p_MSc_ /= (3. * total_area_);
}

template <typename T>
Vector3<T> SurfaceMesh<T>::CalcGradBarycentric(int f, int i) const {
  DRAKE_DEMAND(0 <= i && i < 3);
  DRAKE_DEMAND(0 <= f && f < num_faces());
  // Vertex V corresponds to bᵢ in the barycentric coordinate in the triangle
  // indexed by `f`. A and B are the other two vertices of the triangle.
  // Positions of the vertices are expressed in frame M of the mesh.
  const Vector3<T>& p_MV = vertices_[faces_[f].vertex(i)];
  const Vector3<T>& p_MA = vertices_[faces_[f].vertex((i + 1) % 3)];
  const Vector3<T>& p_MB = vertices_[faces_[f].vertex((i + 2) % 3)];

  // TODO(DamrongGuoy): Provide a mechanism for users to set the gradient
  //  vector in SurfaceMeshFieldLinear since this calculation is not reliable
  //  for zero- or almost-zero-area triangles. For example, the code that
  //  creates ContactSurface by triangle-tetrahedron intersection can set the
  //  pressure gradient along a contact polygon by projecting the soft
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
    throw std::runtime_error("Bad triangle. Cannot compute gradient.");
  }
  return H_M / h2;
}

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class SurfaceMesh)

}  // namespace geometry
}  // namespace drake
