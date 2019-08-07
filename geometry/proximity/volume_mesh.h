#pragma once

#include <array>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/type_safe_index.h"

namespace drake {
namespace geometry {

/**
 Index used to identify a vertex in a volume mesh.
 */
using VolumeVertexIndex = TypeSafeIndex<class VolumeVertexTag>;

/**
 Index for identifying a tetrahedral element in a volume mesh.
 */
using VolumeElementIndex = TypeSafeIndex<class VolumeElementTag>;

/** %VolumeVertex represents a vertex in VolumeMesh.
 @tparam T The underlying scalar type for coordinates, e.g., double or
           AutoDiffXd. Must be a valid Eigen scalar.
 */
template <class T>
class VolumeVertex {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(VolumeVertex)

  /** Constructs VolumeVertex.
   @param r_MV displacement vector from the origin of M's frame to this
               vertex, expressed in M's frame.
   */
  explicit VolumeVertex(const Vector3<T>& r_MV)
      : r_MV_(r_MV) {}

  /** Constructs VolumeVertex from the xyz components of a point V in a frame
   M.
   */
  VolumeVertex(const T& Vx_M, const T& Vy_M, const T& Vz_M)
      : r_MV_(Vx_M, Vy_M, Vz_M) {}

  /** Returns the displacement vector from the origin of M's frame to this
    vertex, expressed in M's frame.
   */
  const Vector3<T>& r_MV() const { return r_MV_; }

 private:
  // Displacement vector from the origin of M's frame to this vertex,
  // expressed in M's frame.
  Vector3<T> r_MV_;
};

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
   */
  VolumeElement(VolumeVertexIndex v0, VolumeVertexIndex v1,
                VolumeVertexIndex v2, VolumeVertexIndex v3)
      : vertex_({v0, v1, v2, v3}) {}

  /** Constructs VolumeElement.
   @param v  Array of four integer indices of the vertices of the element in
             VolumeMesh.
   */
  explicit VolumeElement(const int v[4])
      : vertex_({VolumeVertexIndex(v[0]), VolumeVertexIndex(v[1]),
                 VolumeVertexIndex(v[2]), VolumeVertexIndex(v[3])}) {}

  /** Returns the vertex index in VolumeMesh of the i-th vertex of this
   element.
   @param i  The local index of the vertex in this element.
   @pre 0 <= i < 4
   */
  VolumeVertexIndex vertex(int i) const {
    return vertex_.at(i);
  }

 private:
  // The vertices of this element.
  std::array<VolumeVertexIndex, 4> vertex_;
};

/** %VolumeMesh represents a tetrahedral volume mesh.
 @tparam T  The underlying scalar type for coordinates, e.g., double or
            AutoDiffXd. Must be a valid Eigen scalar.
 */
template <class T>
class VolumeMesh {
 public:
  DRAKE_DECLARE_COPY_AND_MOVE_AND_ASSIGN(VolumeMesh)

  /**
   @name Mesh type traits

   A collection of type traits to enable mesh consumers to be templated on
   mesh type. Each mesh type provides specific definitions of _vertex_,
   _element_, and _barycentric coordinates_. For %VolumeMesh, an element is a
   tetrahedron.
   */
  //@{

  static constexpr int kDim = 3;

  /** Index for identifying a vertex.
   */
  using VertexIndex = VolumeVertexIndex;

  /** Index for identifying a tetrahedral element.
   */
  using ElementIndex = VolumeElementIndex;

  /** Type of barycentric coordinates on a tetrahedral element. Barycentric
   coordinates (b₀, b₁, b₂, b₃) satisfy b₀ + b₁ + b₂ + b₃ = 1. It corresponds
   to a position in the space. If all bᵢ >= 0, it corresponds to a position
   inside the tetrahedron or on the faces of the tetrahedron. If some bᵢ < 0,
   it corresponds to a position outside the tetrahedron. Technically we
   could calculate one of the bᵢ from the others; however, there is no
   standard way to omit one of the coordinates.
  */
  using Barycentric = Vector<T, kDim + 1>;

  /** Type of Cartesian coordinates. Mesh consumers can use it in conversion
   from Cartesian coordinates to barycentric coordinates.
   */
  using Cartesian = Vector<T, 3>;

  //@}

  /** Constructor from a vector of vertices and from a vector of elements.
   Each element must be a valid VolumeElement following the vertex ordering
   convention documented in the VolumeElement class. This class however does not
   enforce this convention and it is thus the responsibility of the user.  */
  VolumeMesh(std::vector<VolumeElement>&& elements,
             std::vector<VolumeVertex<T>>&& vertices)
      : elements_(std::move(elements)), vertices_(std::move(vertices)) {}

  const VolumeElement& element(ElementIndex e) const {
    DRAKE_DEMAND(0 <= e && num_elements());
    return elements_[e];
  }

  /** Returns the vertex identified by a given index.
   @param v  The index of the vertex.
   @pre v ∈ {0, 1, 2,...,num_vertices()-1}.
   */
  const VolumeVertex<T>& vertex(VertexIndex v) const {
    DRAKE_DEMAND(0 <= v && v < num_vertices());
    return vertices_[v];
  }

  const std::vector<VolumeVertex<T>>& vertices() const { return vertices_; }

  const std::vector<VolumeElement>& tetrahedra() const { return elements_; }

  /** Returns the number of tetrahedral elements in the mesh.
   */
  int num_elements() const { return elements_.size(); }

  /** Returns the number of vertices in the mesh.
   */
  int num_vertices() const { return vertices_.size(); }

  /** Calculates volume of a tetrahedral element.
   */
  T CalcTetrahedronVolume(VolumeElementIndex e) const {
    // TODO(DamrongGuoy): Refactor this function out of VolumeMesh when we need
    //  it. CalcTetrahedronVolume(VolumeElementIndex) will call
    //  CalcTetrahedronVolume(Vector3, Vector3, Vector3, Vector3).
    const Vector3<T>& a = vertices_[elements_[e].vertex(0)].r_MV();
    const Vector3<T>& b = vertices_[elements_[e].vertex(1)].r_MV();
    const Vector3<T>& c = vertices_[elements_[e].vertex(2)].r_MV();
    const Vector3<T>& d = vertices_[elements_[e].vertex(3)].r_MV();
    // Assume the first three vertices a, b, c define a triangle with its
    // right-handed normal pointing towards the inside of the tetrahedra. The
    // fourth vertex, d, is on the positive side of the plane defined by a,
    // b, c. With this convention, the computed volume will be positive,
    // otherwise negative.
    const T volume = (d - a).dot((b - a).cross(c - a)) / T(6.0);
    DRAKE_ASSERT(volume > T(0));
    return volume;
  }

  /** Calculate barycentric coordinates with respect to the tetrahedron `e`
   of the point Q'. This operation is expensive compared with going from
   barycentric to Cartesian.
   @param p_MQ  A position expressed in the frame M of the mesh.
   @param e     The index of a tetrahedral element.
   @note  If p_MQ is outside the tetrahedral element, the barycentric
          coordinates (b₀, b₁, b₂, b₃) still satisfy b₀ + b₁ + b₂ + b₃ = 1;
          however, some bᵢ will be negative.
   */
  Barycentric CalcBarycentric(const Cartesian& p_MQ, ElementIndex e) const {
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

    Matrix4<T> A;
    for (int i = 0; i < 4; ++i) {
      A.col(i) << T(1.0), vertex(element(e).vertex(i)).r_MV();
    }
    Vector4<T> b;
    b << T(1.0), p_MQ;
    Barycentric b_Q = A.partialPivLu().solve(b);
    // TODO(DamrongGuoy): Save the inverse of the matrix instead of
    //  calculating it on the fly. We can reduce to 3x3 system too.  See
    //  issue #11653.
    return b_Q;
  }

 private:
  // The tetrahedral elements that comprise the volume.
  std::vector<VolumeElement> elements_;
  // The vertices that are shared between the tetrahedral elements.
  std::vector<VolumeVertex<T>> vertices_;
};
DRAKE_DEFINE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN_T(VolumeMesh)

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class VolumeMesh)

}  // namespace geometry
}  // namespace drake
