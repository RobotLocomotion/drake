#pragma once

#include <array>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/mesh_traits.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
/** %VolumeElement represents a tetrahedral element in a VolumeMesh. It is a
 topological entity in the sense that it only knows the indices of its vertices
 but not their coordinates.
 */
class VolumeElement {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(VolumeElement);

  /** Constructs VolumeElement.
   We follow the convention that the first three vertices define a triangle with
   its right-handed normal pointing inwards. The fourth vertex is then on the
   positive side of this first triangle.
   @warning This class does not enforce our convention for the ordering of the
   vertices.
   @param v0 Index of the first vertex in VolumeMesh.
   @param v1 Index of the second vertex in VolumeMesh.
   @param v2 Index of the third vertex in VolumeMesh.
   @param v3 Index of the last vertex in VolumeMesh.
   @pre All indices are non-negative.
   */
  VolumeElement(int v0, int v1, int v2, int v3) : vertex_({v0, v1, v2, v3}) {
    DRAKE_DEMAND(v0 >= 0 && v1 >= 0 && v2 >= 0 && v3 >= 0);
  }

  /** Constructs VolumeElement.
   @param v  Array of four integer indices of the vertices of the element in
             VolumeMesh.
   @pre All indices are non-negative.
   */
  explicit VolumeElement(const int v[4])
      : VolumeElement(v[0], v[1], v[2], v[3]) {}

  /** Returns the number of vertices in this element. */
  int num_vertices() const { return 4; }

  /** Returns the vertex index in VolumeMesh of the i-th vertex of this
   element.
   @param i  The local index of the vertex in this element.
   @pre 0 <= i < 4
   */
  int vertex(int i) const { return vertex_.at(i); }

  /** Checks to see whether the given VolumeElement use the same four indices in
   the same order. We check for equality to the last bit consistently with
   VolumeMesh::Equal(). Two permutations of the four vertex indices of a
   tetrahedron are considered different tetrahedra even though they span the
   same space.
   */
  bool Equal(const VolumeElement& e) const {
    return this->vertex_ == e.vertex_;
  }

 private:
  // The vertices of this element.
  std::array<int, 4> vertex_;
};

inline bool operator==(const VolumeElement& e1, const VolumeElement& e2) {
  return e1.Equal(e2);
}

inline bool operator!=(const VolumeElement& e1, const VolumeElement& e2) {
  return !(e1 == e2);
}

// Forward declaration of VolumeMeshTester<T>. VolumeMesh<T> will grant
// friend access to VolumeMeshTester<T>.
template <typename T>
class VolumeMeshTester;

/** %VolumeMesh represents a tetrahedral volume mesh.
 @tparam T  The underlying scalar type for coordinates, e.g., double or
            AutoDiffXd. Must be a valid Eigen scalar.
 */
template <class T>
class VolumeMesh {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(VolumeMesh);

  /**
   @name Mesh type traits

   A collection of type traits to enable mesh consumers to be templated on
   mesh type. Each mesh type provides specific definitions of _vertex_,
   _element_, and _barycentric coordinates_. For %VolumeMesh, an element is a
   tetrahedron.
   */
  //@{

  using ScalarType = T;

  /**
   Number of vertices per element. A tetrahedron has 4 vertices.
   */
  static constexpr int kVertexPerElement = 4;

  // TODO(SeanCurtis-TRI) This is very dissatisfying. The alias contained in a
  //  templated class doesn't depend on the class template parameter, but
  //  depends on some non-template-dependent property (kVertexPerElement).
  //  That means we *apparently* have different types:
  //    VolumeMesh<double>::Barycentric<double>
  //    VolumeMesh<AutoDiffXd>::Barycentric<double>
  // But, ultimately both become Vector4d and, because they are simply aliases,
  // are interchangeable. It would be nice to have some way of formulating this
  // that *doesn't* imply dependency on the scalar type of VolumeMesh.
  /** Type of barycentric coordinates on a tetrahedral element. Barycentric
   coordinates (b₀, b₁, b₂, b₃) satisfy b₀ + b₁ + b₂ + b₃ = 1. It corresponds
   to a position in the space. If all bᵢ >= 0, it corresponds to a position
   inside the tetrahedron or on the faces of the tetrahedron. If some bᵢ < 0,
   it corresponds to a position outside the tetrahedron. Technically we
   could calculate one of the bᵢ from the others; however, there is no
   standard way to omit one of the coordinates.
  */
  template <typename U = T>
  using Barycentric = Vector<U, kVertexPerElement>;

  //@}

  /** Constructor from a vector of vertices and from a vector of elements.
   Each element must be a valid VolumeElement following the vertex ordering
   convention documented in the VolumeElement class. This class however does not
   enforce this convention and it is thus the responsibility of the user.  */
  VolumeMesh(std::vector<VolumeElement>&& elements,
             std::vector<Vector3<T>>&& vertices);

  const VolumeElement& element(int e) const {
    DRAKE_DEMAND(0 <= e && e < num_elements());
    return elements_[e];
  }

  /** Returns the vertex identified by a given index.
   @param v  The index of the vertex.
   @pre v ∈ {0, 1, 2,...,num_vertices()-1}.
   */
  const Vector3<T>& vertex(int v) const {
    DRAKE_DEMAND(0 <= v && v < num_vertices());
    return vertices_M_[v];
  }

  /** Returns the inward facing normal of face f of element e.
   @param e The index of the element.
   @param f The index of the triangular face of the tetrahedral element e
            formed by the vertices [(f + 1) % 4, (f + 2) % 4, (f + 3) % 4].
   @pre e ∈ [0, num_elements())
   @pre f ∈ [0, 4)
   */
  const Vector3<T>& inward_normal(int e, int f) const {
    DRAKE_DEMAND(0 <= e && e < num_elements());
    DRAKE_DEMAND(0 <= f && f < kVertexPerElement);
    return inward_normals_M_[e][f];
  }

  /** Returns p_AB_M, the position vector from vertex A to vertex B in M, where
   A and B are specified by the element local indices a and b of element e.
   @param e The index of the element.
   @param a The element local index of vertex A.
   @param b The element local index of vertex B.
   @pre e ∈ [0, num_elements())
   @pre a ∈ [0, 4)
   @pre b ∈ [0, 4)
   @pre a < b
  */
  const Vector3<T>& edge_vector(int e, int a, int b) const {
    DRAKE_DEMAND(0 <= e && e < num_elements());
    DRAKE_DEMAND(0 <= a && a < kVertexPerElement);
    DRAKE_DEMAND(0 <= b && b < kVertexPerElement);
    DRAKE_DEMAND(a < b);
    // The following formula gives this table:
    // {a, b} = {0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}
    // index  =      0,      1,      2,      3,      4,      5
    const int index = a + b - !a;
    return edge_vectors_M_[e][index];
  }

  const std::vector<Vector3<T>>& vertices() const { return vertices_M_; }

  const std::vector<VolumeElement>& tetrahedra() const { return elements_; }

  /** Returns the number of tetrahedral elements in the mesh.
   */
  int num_elements() const { return elements_.size(); }

  /** Returns the number of vertices in the mesh.
   */
  int num_vertices() const { return vertices_M_.size(); }

  /** Calculates volume of a tetrahedral element. It is a signed volume, i.e.,
   it can be negative depending on the order of the four vertices of the
   tetrahedron.
   @pre `e ∈ [0, num_elements())`.
   */
  T CalcTetrahedronVolume(int e) const {
    const T volume =
        (this->edge_vector(e, 0, 3).dot(
            this->edge_vector(e, 0, 1).cross(this->edge_vector(e, 0, 2)))) /
        T(6.0);
    return volume;
  }

  /** Calculates the volume of `this` mesh by taking the sum of the volume of
   *  each tetrahedral element.
   */
  T CalcVolume() const {
    T volume(0.0);
    for (int e = 0; e < num_elements(); ++e) {
      volume += CalcTetrahedronVolume(e);
    }
    return volume;
  }

  /** Calculate barycentric coordinates with respect to the tetrahedron `e`
   of the point Q. This operation is expensive compared with going from
   barycentric to Cartesian.

   The return type depends on both the mesh's vertex position scalar type `T`
   and the Cartesian coordinate type `C` of the query point.  See
   @ref drake::geometry::promoted_numerical "promoted_numerical_t" for details.

   @param p_MQ  A position expressed in the frame M of the mesh.
   @param e     The index of a tetrahedral element.
   @note  If p_MQ is outside the tetrahedral element, the barycentric
          coordinates (b₀, b₁, b₂, b₃) still satisfy b₀ + b₁ + b₂ + b₃ = 1;
          however, some bᵢ will be negative.
   @tparam C must be either `double` or `AutoDiffXd`. */
  template <typename C>
  Barycentric<promoted_numerical_t<T, C>> CalcBarycentric(
      const Vector3<C>& p_MQ, int e) const
#ifndef DRAKE_DOXYGEN_CXX
    requires scalar_predicate<C>::is_bool
#endif
  ;  // NOLINT(whitespace/semicolon)

  /** Checks to see whether the given VolumeMesh object is equal via deep
   comparison (up to a tolerance). NaNs are treated as not equal as per the IEEE
   standard. The tolerance is applied to corresponding vertex positions; the ith
   vertex in each mesh can have a distance of no more than `vertex_tolerance`.
   @param mesh              The mesh for comparison.
   @param vertex_tolerance  The maximum distance allowed between two vertices to
                            be considered equal.
   @returns `true` if the given mesh is equal.
   */
  bool Equal(const VolumeMesh<T>& mesh, double vertex_tolerance = 0) const;

  /** Calculates the gradient ∇u of a linear field u on the tetrahedron `e`.
   Field u is defined by the four field values `field_value[i]` at the i-th
   vertex of the tetrahedron. The gradient ∇u is expressed in the coordinates
   frame of this mesh M.

   If the return value is std::nullopt, the tetrahedron is degenerate, and no
   reliable gradient could be computed.

   The return type depends on both the mesh's vertex position scalar type `T`
   and the given field's scalar type `FieldValue`.  See
   @ref drake::geometry::promoted_numerical "promoted_numerical_t" for details.
   */
  template <typename FieldValue>
  std::optional<Vector3<promoted_numerical_t<T, FieldValue>>>
  MaybeCalcGradientVectorOfLinearField(
      const std::array<FieldValue, 4>& field_value, int e) const {
    using ReturnType = promoted_numerical_t<T, FieldValue>;
    Vector3<ReturnType> gradu_M = Vector3<ReturnType>::Zero();
    for (int i = 0; i < 4; ++i) {
      auto grad_i = MaybeCalcGradBarycentric(e, i);
      if (!grad_i.has_value()) {
        return {};
      }
      gradu_M += field_value[i] * *grad_i;
    }
    return gradu_M;
  }

  /** Like MaybeCalcGradientVectorOfLinearField, but throws if the geometry is
   degenerate.

   @throws std::exception if the gradient could not be computed.
   */
  template <typename FieldValue>
  Vector3<promoted_numerical_t<T, FieldValue>> CalcGradientVectorOfLinearField(
      const std::array<FieldValue, 4>& field_value, int e) const {
    auto result = MaybeCalcGradientVectorOfLinearField(field_value, e);
    if (!result.has_value()) {
      throw std::runtime_error("Bad geometry; could not calculate gradient.");
    }
    return result.value();
  }

  /** Transforms the vertices of this mesh from its initial frame M to the new
   frame N.
   @param[in] transform  The transform X_NM relating the mesh in frame M to the
   new frame N. */
  void TransformVertices(const math::RigidTransform<T>& transform);

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
  // Calculates the gradient vector ∇bᵢ of the barycentric coordinate
  // function bᵢ of the i-th vertex of the tetrahedron `e`. The gradient
  // vector ∇bᵢ is expressed in the coordinates frame of this mesh M.
  // @pre  0 ≤ i < 4.
  // TODO(rpoyner-tri): currently only used by the test helper; delete?
  Vector3<T> CalcGradBarycentric(int e, int i) const {
    auto result = MaybeCalcGradBarycentric(e, i);
    if (!result.has_value()) {
      throw std::runtime_error("Bad geometry; could not calculate gradient.");
    }
    return *result;
  }

  // Calculates the inward facing normals and element edge vectors.
  void ComputePositionDependentQuantities();

  // Like CalcGradBarycentric, but returns std::nullopt instead of throwing on
  // degenerate geometry.
  std::optional<Vector3<T>> MaybeCalcGradBarycentric(int e, int i) const;

  // The tetrahedral elements that comprise the volume.
  std::vector<VolumeElement> elements_;
  // The vertices that are shared between the tetrahedral elements, measured and
  // expressed in the mesh's frame M.
  std::vector<Vector3<T>> vertices_M_;
  // Stores the inward facing normals of each face of the tetrahedron, measured
  // and expressed in the mesh's frame M. Index i stores the normal of the face
  // formed by vertices {0, 1, 2, 3} / {i}.
  std::vector<std::array<Vector3<T>, 4>> inward_normals_M_;
  // Stores the edge vectors of each tetrahedron, measured and
  // expressed in the mesh's frame M, in lexicographical order:
  // {0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}
  std::vector<std::array<Vector3<T>, 6>> edge_vectors_M_;

  friend class VolumeMeshTester<T>;
};

template <typename T>
std::optional<Vector3<T>> VolumeMesh<T>::MaybeCalcGradBarycentric(int e,
                                                                  int i) const {
  DRAKE_DEMAND(0 <= i && i < 4);
  // Vertex V corresponds to bᵢ in the barycentric coordinate in the
  // tetrahedron indexed by `e`.  A, B, and C are the remaining vertices of
  // the tetrahedron. Their positions are expressed in frame M of the mesh.
  const Vector3<T>& p_MV = vertices_M_[elements_[e].vertex(i)];
  const Vector3<T>& p_MA = vertices_M_[elements_[e].vertex((i + 1) % 4)];
  const Vector3<T>& p_MB = vertices_M_[elements_[e].vertex((i + 2) % 4)];
  const Vector3<T>& p_MC = vertices_M_[elements_[e].vertex((i + 3) % 4)];

  const Vector3<T> p_AV_M = p_MV - p_MA;
  const Vector3<T> p_AB_M = p_MB - p_MA;
  const Vector3<T> p_AC_M = p_MC - p_MA;

  // Let bᵥ be the barycentric coordinate function corresponding to vertex V.
  // bᵥ is a linear function of the points in the tetrahedron.
  // bᵥ = 0 on the plane through triangle ABC.
  // bᵥ = 1 on the plane through V parallel to ABC.
  // Therefore, bᵥ changes fastest in the direction of the face normal vector
  // of ABC towards V. The rate of change is 1/h, where h is the
  // height of vertex V from the base ABC.
  //
  //    ──────────────V────────────── plane bᵥ = 1
  //                 ╱ ╲       ┊
  //                ╱   ╲      ┊         Triangle ABC is perpendicular to
  //               ╱     ╲     ┊ h       this view, so ABC looks like a line
  //              ╱       ╲    ┊         segment instead of a triangle.
  //             ╱    ↑∇bᵥ ╲   ┊
  //    ────────A━━━B━━━━━━━C──────── plane bᵥ = 0
  //
  // We conclude that ∇bᵥ is the vector of length 1/h that is perpendicular to
  // ABC and points into the tetrahedron.
  //
  // To calculate ∇bᵥ, consider the scalar triple product (AB x AC)⋅AV, which
  // is the signed volume of the parallelepiped spanned by AB, AC, and AV, and
  // consider the cross product AB x AC, which is the area vector of the
  // parallelogram spanned by AB and AC. We have:
  //
  //       ∇bᵥ = normal vector inversely proportional to height
  //           = area vector / signed volume
  //           = (AB x AC) / ((AB x AC)⋅AV)
  //
  // Consider a non-degenerate tetrahedron (tetrahedron with volume well above
  // zero) with vertices V₀,V₁,V₂,V₃ (note that vertex Vᵢ is the local
  // iᵗʰ vertex of the tetrahedron, *not* the global iᵗʰ vertex of the whole
  // mesh). Assume V₀,V₁,V₂,V₃ is in positive orientation, i.e.,
  // (V₁ - V₀)x(V₂ - V₀) points towards V₃. Given the vertex V = Vᵢ,
  // i ∈ {0,1,2,3}, the vertices A = Vᵢ₊₁, B = Vᵢ₊₂, C = Vᵢ₊₃ form the vector
  // AB x AC that points from ABC towards V when i is 1 or 3, and
  // AB x AC points away from V when i is 0 or 2. When AB x AC points towards
  // V, the signed volume (AB x AC)⋅AV is positive, and when AB x AC points
  // away from V, the signed volume is negative. As a result, the vector
  // ∇bᵥ = (AB x AC) / ((AB x AC)⋅AV) always points from ABC towards V as
  // expected.
  //
  // If the tetrahedron is degenerate (tetrahedron with almost zero volume),
  // the calculation (AB x AC) / ((AB x AC)⋅AV) is not numerically reliable any
  // more due to rounding errors. Near-zero-area triangle ABC may have the
  // area-vector calculation AB x AC pointing in the wrong direction. If ABC is
  // well formed but V is almost co-planar with ABC (V is near a vertex of ABC,
  // near the spanning line of an edge of ABC, or anywhere on the spanning plane
  // of ABC), the signed volume calculation ((AB x AC)⋅AV) may become a
  // near-zero number with the wrong sign. In these degenerate cases, we
  // throw std::exception.
  //
  const Vector3<T> area_vector_M = p_AB_M.cross(p_AC_M);  // AB x AC
  const T signed_volume = area_vector_M.dot(p_AV_M);      // (AB x AC)⋅AV

  constexpr double kEps = std::numeric_limits<double>::epsilon();

  // TODO(DamrongGuoy): Find a better way to handle degeneracy. Right now we
  //  check the volume of the parallelepiped against the threshold equal the
  //  machine epsilon. We might want to scale the threshold with the
  //  dimensions (length, area) of the parts of the tetrahedron (6 edges, 4
  //  triangles). Furthermore, we might also want case analysis for various
  //  kinds of degenerate tetrahedra; for example, a tetrahedron with
  //  one or more near-zero-length edges, a tetrahedron with one or more
  //  near-zero-area obtuse triangles without near-zero-length edges,
  //  or a tetrahedron with near-zero volume without near-zero-area triangles.

  using std::abs;
  if (abs(signed_volume) <= kEps) {
    return {};
  }
  return area_vector_M / signed_volume;
}

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class VolumeMesh);

}  // namespace geometry
}  // namespace drake
