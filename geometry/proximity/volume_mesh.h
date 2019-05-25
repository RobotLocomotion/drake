#pragma once

#include <array>
#include <utility>
#include <vector>

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
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(VolumeMesh)

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
   coordinates (b₀, b₁, b₂, b₃) satisfy b₀ + b₁ + b₂ + b₃ = 1, bᵢ >= 0, so
   technically we could calculate one of the bᵢ from the others; however,
   there is no standard way to omit one of the coordinates.
  */
  using Barycentric = Vector<T, kDim + 1>;

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

  //@}

  VolumeMesh(std::vector<VolumeElement>&& elements,
             std::vector<VolumeVertex<T>>&& vertices)
      : elements_(std::move(elements)), vertices_(std::move(vertices)) {}

  /** Returns the number of tetrahedral elements in the mesh.
   */
  int num_elements() const { return elements_.size(); }

  /** Returns the number of vertices in the mesh.
   */
  int num_vertices() const { return vertices_.size(); }

 private:
  // The tetrahedral elements that comprise the volume.
  std::vector<VolumeElement> elements_;
  // The vertices that are shared between the tetrahedral elements.
  std::vector<VolumeVertex<T>> vertices_;
};

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class VolumeMesh)

}  // namespace geometry
}  // namespace drake
