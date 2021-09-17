#pragma once

#include <array>
#include <limits>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/mesh_traits.h"

namespace drake {
namespace geometry {
/** %VolumeElement represents a tetrahedral element in a VolumeMesh. It is a
 topological entity in the sense that it only knows the indices of its vertices
 but not their coordinates.
 */
class VolumeElement {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(VolumeElement)

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

namespace internal {
// Forward declaration for friend declaration.
template <typename> class MeshDeformer;
}  // namespace internal

// Forward declaration of VolumeMeshTester<T>. VolumeMesh<T> will grant
// friend access to VolumeMeshTester<T>.
template <typename T> class VolumeMeshTester;

/** %VolumeMesh represents a tetrahedral volume mesh.
 @tparam T  The underlying scalar type for coordinates, e.g., double or
            AutoDiffXd. Must be a valid Eigen scalar.
 */
template <class T>
class VolumeMesh {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(VolumeMesh)

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
             std::vector<Vector3<T>>&& vertices)
      : elements_(std::move(elements)), vertices_(std::move(vertices)) {
    if (elements_.empty()) {
      throw std::logic_error("A mesh must contain at least one tetrahedron");
    }
  }

  const VolumeElement& element(int e) const {
    DRAKE_DEMAND(0 <= e && num_elements());
    return elements_[e];
  }

  /** Returns the vertex identified by a given index.
   @param v  The index of the vertex.
   @pre v ∈ {0, 1, 2,...,num_vertices()-1}.
   */
  const Vector3<T>& vertex(int v) const {
    DRAKE_DEMAND(0 <= v && v < num_vertices());
    return vertices_[v];
  }

  const std::vector<Vector3<T>>& vertices() const { return vertices_; }

  const std::vector<VolumeElement>& tetrahedra() const { return elements_; }

  /** Returns the number of tetrahedral elements in the mesh.
   */
  int num_elements() const { return elements_.size(); }

  /** Returns the number of vertices in the mesh.
   */
  int num_vertices() const { return vertices_.size(); }

  /** Calculates volume of a tetrahedral element.
   @pre `f ∈ [0, num_elements())`.
   */
  T CalcTetrahedronVolume(int e) const {
    // TODO(DamrongGuoy): Refactor this function out of VolumeMesh when we need
    //  it. CalcTetrahedronVolume(index) will call
    //  CalcTetrahedronVolume(Vector3, Vector3, Vector3, Vector3).
    const Vector3<T>& a = vertices_[elements_[e].vertex(0)];
    const Vector3<T>& b = vertices_[elements_[e].vertex(1)];
    const Vector3<T>& c = vertices_[elements_[e].vertex(2)];
    const Vector3<T>& d = vertices_[elements_[e].vertex(3)];
    // Assume the first three vertices a, b, c define a triangle with its
    // right-handed normal pointing towards the inside of the tetrahedra. The
    // fourth vertex, d, is on the positive side of the plane defined by a,
    // b, c. With this convention, the computed volume will be positive,
    // otherwise negative.
    const T volume = (d - a).dot((b - a).cross(c - a)) / T(6.0);
    DRAKE_ASSERT(volume > T(0));
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
   */
  template <typename C>
  Barycentric<promoted_numerical_t<T, C>> CalcBarycentric(
      const Vector3<C>& p_MQ, int e) const {
    // We have two conditions to satisfy.
    // 1. b₀ + b₁ + b₂ + b₃ = 1
    // 2. b₀*v0 + b₁*v1 + b₂*v2 + b₃*v3 = p_M.
    // Together they create this 4x4 linear system:
    //
    //      | 1  1  1  1 ||b₀|   | 1 |
    //      | |  |  |  | ||b₁| = | | |
    //      | v0 v1 v2 v3||b₂|   |p_M|
    //      | |  |  |  | ||b₃|   | | |
    //
    // q = p_M - v0 = b₀*u0 + b₁*u1 + b₂*u2 + b₃*u3
    //              = 0 + b₁*u1 + b₂*u2 + b₃*u3
    using ReturnType = promoted_numerical_t<T, C>;
    Matrix4<ReturnType> A;
    for (int i = 0; i < 4; ++i) {
      A.col(i) << ReturnType(1.0), vertex(element(e).vertex(i));
    }
    Vector4<ReturnType> b;
    b << ReturnType(1.0), p_MQ;
    const Vector4<ReturnType> b_Q = A.partialPivLu().solve(b);
    // TODO(DamrongGuoy): Save the inverse of the matrix instead of
    //  calculating it on the fly. We can reduce to 3x3 system too.  See
    //  issue #11653.
    return b_Q;
  }

  /** Checks to see whether the given VolumeMesh object is equal via deep
   comparison (up to a tolerance). NaNs are treated as not equal as per the IEEE
   standard. The tolerance is applied to corresponding vertex positions; the ith
   vertex in each mesh can have a distance of no more than `vertex_tolerance`.
   @param mesh              The mesh for comparison.
   @param vertex_tolerance  The maximum distance allowed between two vertices to
                            be considered equal.
   @returns `true` if the given mesh is equal.
   */
  bool Equal(const VolumeMesh<T>& mesh, double vertex_tolerance = 0) const {
    if (this == &mesh) return true;

    if (this->num_elements() != mesh.num_elements()) return false;
    if (this->num_vertices() != mesh.num_vertices()) return false;

    // Check tetrahedral elements.
    for (int i = 0; i < this->num_elements(); ++i) {
      if (!this->element(i).Equal(mesh.element(i))) return false;
    }
    // Check vertices.
    for (int i = 0; i < this->num_vertices(); ++i) {
      if ((this->vertex(i) - mesh.vertex(i)).norm() > vertex_tolerance) {
        return false;
      }
    }

    // All checks passed.
    return true;
  }

  /** Calculates the gradient ∇u of a linear field u on the tetrahedron `e`.
   Field u is defined by the four field values `field_value[i]` at the i-th
   vertex of the tetrahedron. The gradient ∇u is expressed in the coordinates
   frame of this mesh M.

   The return type depends on both the mesh's vertex position scalar type `T`
   and the given field's scalar type `FieldValue`.  See
   @ref drake::geometry::promoted_numerical "promoted_numerical_t" for details.
   */
  template <typename FieldValue>
  Vector3<promoted_numerical_t<T, FieldValue>> CalcGradientVectorOfLinearField(
      const std::array<FieldValue, 4>& field_value, int e) const {
    using ReturnType = promoted_numerical_t<T, FieldValue>;
    Vector3<ReturnType> gradu_M = field_value[0] * CalcGradBarycentric(e, 0);
    for (int i = 1; i < 4; ++i) {
      gradu_M += field_value[i] * CalcGradBarycentric(e, i);
    }
    return gradu_M;
  }

 private:
  // Client attorney class that provides a means to modify vertex positions.
  friend class internal::MeshDeformer<VolumeMesh<T>>;

  // Calculates the gradient vector ∇bᵢ of the barycentric coordinate
  // function bᵢ of the i-th vertex of the tetrahedron `e`. The gradient
  // vector ∇bᵢ is expressed in the coordinates frame of this mesh M.
  // @pre  0 ≤ i < 4.
  Vector3<T> CalcGradBarycentric(int e, int i) const;

  // The tetrahedral elements that comprise the volume.
  std::vector<VolumeElement> elements_;
  // The vertices that are shared between the tetrahedral elements.
  std::vector<Vector3<T>> vertices_;

  friend class VolumeMeshTester<T>;
};

template <typename T>
Vector3<T> VolumeMesh<T>::CalcGradBarycentric(int e, int i) const {
  DRAKE_DEMAND(0 <= i && i < 4);
  // Vertex V corresponds to bᵢ in the barycentric coordinate in the
  // tetrahedron indexed by `e`.  A, B, and C are the remaining vertices of
  // the tetrahedron. Their positions are expressed in frame M of the mesh.
  const Vector3<T>& p_MV = vertices_[elements_[e].vertex(i)];
  const Vector3<T>& p_MA = vertices_[elements_[e].vertex((i + 1) % 4)];
  const Vector3<T>& p_MB = vertices_[elements_[e].vertex((i + 2) % 4)];
  const Vector3<T>& p_MC = vertices_[elements_[e].vertex((i + 3) % 4)];

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
    throw std::runtime_error("Bad tetrahedron. Cannot compute gradient.");
  }
  return area_vector_M / signed_volume;
}

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class VolumeMesh)

}  // namespace geometry
}  // namespace drake
