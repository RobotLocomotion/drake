#pragma once

#include <string>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"

namespace drake {
namespace geometry {

// TODO(DamrongGuoy): Move this documentation to QueryObject.
/*
  The classes in this file collectively represent the contact surface Sₘₙ
  between two compliant bodies M and N.  A contact surface is a surface of
  equilibrium eₘ = eₙ, where eₘ and eₙ are the given scalar fields on
  bodies M and N:
                      eₘ : M → ℝ, eₙ : N → ℝ,
                      Sₘₙ = { q ∈ M ∩ N : eₘ(r_MQ) = eₙ(r_NQ) },
  where r_MQ and r_NQ are the position vectors of a point q expressed in M's
  frame and B's frame respectively.

  Conceptually we can define the function hₘₙ = eₘ - eₙ as a scalar field on
  M ∩ N:
                      hₘₙ : M ∩ N → ℝ,
                      hₘₙ(q) = eₘ(r_MQ) - eₙ(r_NQ).
  The function hₘₙ measures the difference between the two scalar fields
  eₘ and eₙ.

  A point q is on the contact surface Sₘₙ if and only if hₘₙ(q) = 0.

  Our data structure for the contact surface does not store hₘₙ, but it stores
  the vector field ∇hₘₙ on the contact surface Sₘₙ:
                 ∇hₘₙ : Sₘₙ → ℝ³,
                 ∇hₘₙ(q) = ∇eₘ(r_MQ) - ∇eₙ(r_NQ),
  where ∇eₘ : M → ℝ³ is the gradient vector field of eₘ on M, and similarly
  ∇eₙ : N → ℝ³ is the gradient vector field of eₙ on N.

  Mathematically the vector ∇hₘₙ(q) of q ∈ Sₘₙ is orthogonal to the contact
  surface Sₘₙ at q. The vector ∇hₘₙ(q) points in the direction of increasing
  eₘ and decreasing eₙ. It measures how fast eₘ and eₙ deviates from the other.

  Computationally our data structure represents the contact surface Sₘₙ as a
  triangulated surface mesh (with triangle-to-vertex topology). At each
  vertex v, we store the common scalar value:
                  e(v) = eₘ(r_MV) = eₙ(r_NV)
  and the vector value:
                  grad_h_M(v) = ∇hₘₙ(r_MV), expressed in M's frame.
  On each triangle, we provide interpolation of the scalar `e` and the vector
  `grad_h_M`.

  Due to discretization, the vector `grad_h_M` stored at a vertex v is not
  strictly orthogonal to any triangle sharing v.  Orthogonality does improve
  with finer discretization.
 */

/* Detailed decisions (will change as we go).
   1. Basic entities (vertices, triangles) are referenced by integer indices
      instead of pointers.
      upside: Straightforward copy operations of meshes and field variables.
              Serialization to write data files for visualization packages is
              straightforward.
              Useful for debugging, hashing, sorting (sorting pointers can
              introduce repeatability problems).
      downside: Integer index alone cannot access data directly; it needs
                tables/containers to access data.
                In dynamic settings(adding + removing), we need extra work to
                maintain continuous indexing, or we need an extra indirection
                to go from an arbitrary id to a continuous index.

   2. Triangle-to-vertex connectivity (triangles share vertices).
      upside: More compact, avoid duplication of vertex data, compared to
              triangle soup.
              Enable straightforward mapping from mesh to physical field
              variables.
      downside: Construction is more involved (manageable using hash tables)
                to avoid "double count".

   3. Vertex-to-triangle topology (not finalized yet)
      upside: Allow topology-based operations like traversal and averaging
              among all triangles sharing a vertex.
      downside: A vertex can have an arbitrary number of incident triangles.
                Extra data to maintain.
                Not all applications need it.
 */

/** Design concepts.
 *
 We roughly partition the data structures into domain discretization
 (triangulated surface meshes) and the physical field variables (pressure, slip,
 traction, etc.) defined on the domain, and the class ContactSurface combines
 both of them together.  This design has motivation from Finite Element
 representations.

 The collection of classes that represent a triangulated surface mesh consist
 of SurfaceVertex, SurfaceFace, and SurfaceMesh. The class that maps an
 underlying surface mesh to a physical field is MeshField. Users can define
 many MeshField's on the same SurfaceMesh.

 MeshField have two storage traits: vertex data and face data. Most field
 variables (e.g., pressure, slip, traction) are vertex data. In a few
 cases (e.g., face normal vector, face area), they are face data.

 ContactSurface has four built-in MeshField's: the scalar field e(), the
 vector field grad_h_M(), the face normal vector nhat_M(), and the face area().
 */

//------------------------------------------------------------------------------
/** @name                  Indexing in Contact Surfaces

 We use indexing in communication between various parts of data structures.
 For example, a triangular face identified by FaceIndex refers to its three
 vertices via their VertexIndex.  We also use VertexIndex or FaceIndex to look
 up field variables.

 Currently we have two classes of indices: VertexIndex and FaceIndex. They
 are suffice for linear elements. In the future, we may consider EdgeIndex,
 which is useful for quadratic elements.
 */
// TODO(DamrongGuoy): Use type-safe index for VertexIndex and FaceIndex.
/**
 Index of a vertex in a contact surface. A triangular face identifies its
 vertices using VertexIndex. We also use VertexIndex to look up a physical
 field variable in MeshField. VertexIndex starts with 0.
 */
typedef int VertexIndex;
/**
 Index of a triangular face in a contact surface. We use FaceIndex to identify
 a triangular face. FaceIndex starts with 0.
 */
typedef int FaceIndex;

//------------------------------------------------------------------------------
/** @name                       Field Variables

 Conceptually a field variable is represented by an array of field values: one
 value per one mesh entity. Currently we support two kinds of values: scalar
 and 3-d vector. We also support two storage schemes: vertex data and face data.

 Most field variables (e.g., pressure, slip, traction) are vertex data, which
 stores one field value per one mesh vertex and allow interpolation on a
 triangular face. A few cases (e.g., face normal vector, area of triangular
 face) are face data, which stores one field value per one mesh face.

 For vertex data, we use VertexIndex to look up the field value. For face
 data, we use FaceIndex to look up the field value.
 */

 /**
  %MeshField represents the value of a field variable. We store one value per
   one mesh entity (vertex or face).
 * @tparam FieldValueType a valid Eigen scalar or vector.
 * @tparam IndexType  VertexIndex or FaceIndex.
 */
template <class FieldValueType, class IndexType>
class MeshField {
 public:
  MeshField(const std::string& name,
            const std::vector<FieldValueType>& values);
  const FieldValueType& at(IndexType i) { return values_.at(i); }

  // TODO(DamrongGuoy): Check array bound.
  FieldValueType& operator[](IndexType i) { return values_[i]; }
  void set(IndexType i, const FieldValueType& value) { values_[i] = value; }
 private:
  std::string name_;
  std::vector<FieldValueType> values_;
};

//------------------------------------------------------------------------------
/** @name                   Domain Discretization

 We use triangulated surface meshes to represent our domains and mimic Finite
 Element representations. Currently we only use the linear triangular element
 (3-node triangle) with possible extension to the quadratic triangular element
 (6-node triangle) in the future.

 We provide a building block for interpolation on a triangular element. We
 also provide conversion between Cartesian coordinates and barycentric
 coordinates with respect to a triangular element.

 A physical field variable is stored in a related class MeshField (see Field
 Variables).
 */

/**
 %SurfaceVertex represents a vertex v in the contact surface between
 bodies M and N.  It provides:
   1. the `index` of this vertex in the contact surface,
   2. the position `r_MV` of this vertex, expressed in M's frame.

 @tparam T the underlying scalar type. Must be a valid Eigen scalar.
*/
template <class T>
class SurfaceVertex {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SurfaceVertex)

  SurfaceVertex() = default;

  /** Constructs SurfaceVertex.
   @param index  vertex index in the contact surface.
   @param r_MV   position of vertex v in M's frame.
   */
  SurfaceVertex(VertexIndex index, const Vector3<T>& r_MV)
      : index_(index), r_MV_(r_MV) {}

  // We provide API functions to read/write the values at the vertex, so
  // that, in the future, we can change the internal representation of vertex
  // data without affecting callers.
  VertexIndex index() const { return index_; }
  void set_index(const VertexIndex& index) { index_ = index; }
  const Vector3<T>& r_MV() const { return r_MV_; }
  void set_r_MV(const Vector3<T>& r_MV) { r_MV_ = r_MV; }

  friend std::ostream& operator<<(std::ostream& os,
                                  const SurfaceVertex<T>& vertex) {
    return os << "Surface vertex"
              << "\n  index() = " << vertex.index()
              << "\n  r_MV() = " << vertex.r_MV().transpose()
              << std::endl;
  }

 private:
  // Index of this vertex in the contact surface.
  VertexIndex index_;
  // Position of vertex v in M's frame.
  Vector3<T> r_MV_;
};

/*
  %SurfaceFace represents a triangle in the contact surface between
  bodies M and N. It provides:
    1. the `index` of this face in the contact surface,
    2. indices of its three vertices.
 */
class SurfaceFace {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SurfaceFace)

  SurfaceFace() = default;

  /** Constructs ContactSurfaceFace.
   @param f  FaceIndex of this face.
   @param v0 VertexIndex of the first vertex.
   @param v1 VertexIndex of the second vertex.
   @param v2 VertexIndex of the last vertex.
   @pre   The order of the three vertices gives the counterclockwise normal
          direction, i.e. (v1-v0)x(v2-v0), towards increasing eₘ the scalar
          field on body M.
   */
  SurfaceFace(FaceIndex f, VertexIndex v0, VertexIndex v1, VertexIndex v2)
      : index_(f), v_({v0, v1, v2}) {}

  // @pre 0 <= i < 3
  VertexIndex vertex(int i) const { return v_[i]; }

 protected:
  // Index of this face in the contact surface.
  FaceIndex index_;
  // The vertices of this face.
  std::array<VertexIndex, 3> v_;
};

/**
- a building block for interpolation of field values,
- unit normal vector in the direction of increasing scalar field eₘ of
body M, expressed in M's frame,
- area of the triangle,
- mapping between barycentric coordinates and Cartesian coordinates.
 */
template <class T>
class SurfaceMesh {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SurfaceMesh)
  // Converts between Cartesian coordinates r_MQ = (x,y,z) of a point q,
  // expressed in M's frame, to the barycentric coordinates b = (b0,b1,b2)
  // with respect to vertices v0, v1, v2 of this face of the contact surface,
  // where:
  //       (x,y,z) = b0 * v0 + b1 * v1 + b2 * v2.
  //
  // @note barycentric(cartesian(b)) = b; however,
  //       cartesian(barycentric(p)) = p if and only if p is on the plane
  //       of the triangle.
  Vector3<T> barycentric(FaceIndex f, const Vector3<T>& r_MQ);
  Vector3<T> cartesian(FaceIndex f, const Vector3<T>& barycentric);

  T interpolate(const MeshField<T, VertexIndex>& field, FaceIndex f,
                const Vector3<T>& barycentric);

  // Returns the unit normal vector nhat_M to the triangle, expressed in M's
  // frame. The normal vector is oriented towards increasing eₘ the scalar
  // field on body M.
  // @note   Due to discretization, nhat_M of this triangle and grad_h_M of
  //         its vertices are approximately parallel.  With finer
  //         discretization, they become more parallel.
  Vector3<T> nhat_M(FaceIndex f) { return nhat_M_.at(f); }

  // Returns the area of this triangle.
  T area(FaceIndex f) { return area_.at(f); }

  const SurfaceVertex<T>& vertex(VertexIndex v) { return vertices_[v]; }

 private:
  // Triangles comprising the surface.
  std::vector<SurfaceFace> faces_;
  // Shared vertices of the triangles.
  std::vector<SurfaceVertex<T>> vertices_;

  MeshField<T, FaceIndex> area_;
  MeshField<Vector3<T>, FaceIndex> nhat_M_;
};

//@tparam T the underlying scalar type. Must be a valid Eigen scalar.
template <class T>
class ContactSurface {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ContactSurface)

  GeometryId id_M() const { return id_M_; }
  GeometryId id_N() const { return id_N_; }

  // Linear interpolation of scalar field e()
  T e(FaceIndex f, const Vector3<T>& barycentric);
  // Spherical linear interpolation from vector field grad_h_M()
  Vector3<T> grad_h_M(FaceIndex f, const Vector3<T>& barycentric);

  Vector3<T> nhat_M(FaceIndex f) { return mesh_.nhat_M(f); }
  T area(FaceIndex f) { return mesh_.area(f); }

  const SurfaceVertex<T>& vertex(VertexIndex v) { return mesh_.vertex(v); }

 private:
  /// The id of the first geometry in the contact.
  GeometryId id_M_;
  /// The id of the second geometry in the contact.
  GeometryId id_N_;

  SurfaceMesh<T> mesh_;
  // Scalar field `e` at vertex v.
  MeshField<T, VertexIndex> e_;
  // Vector field ∇hₘₙ at vertex v, expressed in M's frame.
  MeshField<Vector3<T>, VertexIndex> grad_h_M_;
};

using ContactSurfaced = ContactSurface<double>;

}  // namespace geometry
}  // namespace drake
