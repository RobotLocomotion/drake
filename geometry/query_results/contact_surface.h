#pragma once

#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"

namespace drake {
namespace geometry {

template <class T>
class ContactSurfaceFace;
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

/* Design factors (will change as we go).
   1. Basic entities (vertex, triangle) are referenced by pointers instead of
      indexes.
      upside: Direct access to their attributes without tables/containers.
              No need for continuous indexing in dynamic settings (adding +
              removing).
      downside: Serialization for writing data files to external visualization
                package (unique id can help).

   2. Triangle-to-vertex topology.
      upside: More compact, avoid duplication of vertex data.
      downside: Construction is more involved (manageable using hash
                tables)

   3. Vertex-to-triangle topology.
      upside: Allow topology-based operations like averaging normal vectors
              of all triangles sharing a vertex.
      downside: A vertex can have an arbitrary number of incident triangles.
                Extra mapping data to maintain.

   4. Entity id or not.
      upside: Useful for debugging, hashing, sorting (sorting id's is much
              better than sorting pointers).
      downside: Extra code to maintain unique id.
 */

 /**
  %ContactSurfaceVertex represents a vertex v in the contact surface between
  bodies M and N.  It provides:
    - position `r_MV` of vertex v, expressed in M's frame,
    - a scalar value `e` of a scalar field at v,
    - a vector value `grad_h_M` of the vector field ∇hₘₙ at v, expressed in
      M's frame.

  @tparam T the underlying scalar type. Must be a valid Eigen scalar.
 */
template <class T>
class ContactSurfaceVertex {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ContactSurfaceVertex)

  ContactSurfaceVertex() = default;

  /** Constructs ContactSurfaceVertex.
   @param r_MV      vertex position.
   @param e         value of the scalar field at the vertex.
   @param grad_h_M  value of the vector field at the vertex.
   */
  ContactSurfaceVertex(Vector3<T> r_MV, T e, Vector3<T> grad_h_M)
  : r_MV_(r_MV), e_(e), grad_h_M_(grad_h_M) {
  }

  // We provide API functions to read/write the values at the vertex, so
  // that, in the future, we can change the internal representation of vertex
  // data without affecting callers.
  Vector3<T> r_MV() { return r_MV_; }
  void set_r_MV(Vector3<T> r_MV) { r_MV_ = r_MV; }
  T e() { return e_; }
  void set_e(T e) { e_ = e; }
  Vector3<T> grad_h_M() { return grad_h_M_; }
  void set_grad_h_M(Vector3<T> grad_h_M) { grad_h_M_ = grad_h_M; }


  friend std::ostream& operator<<(std::ostream& os,
                                 const ContactSurfaceVertex<T>& vertex) {
    return os << "Contact surface vertex"
              << "\n  r_MV() = " << vertex.r_MV().transpose()
              << "\n  e() = " << vertex.e()
              << "\n  grad_h_M() = " << vertex.grad_h_M()
              << std::endl;
  }

 private:
  /** Position of vertex v in M's frame. */
  Vector3<T> r_MV_;
  /** Scalar value at vertex v. */
  T e_;
  /** Vector value at vertex v, in M's frame. */
  Vector3<T> grad_h_M_;
};


/*
  %ContactSurfaceFace represents a triangle in the contact surface between
  bodies M and N. It provides:
    - access to its three vertices,
    - interpolation of field values of its vertices,
    - unit normal vector in the direction of increasing scalar field eₘ of
      body M, expressed in M's frame,
    - area of the triangle,
    - mapping between barycentric coordinates and Cartesian coordinates.

  @tparam T the underlying scalar type. Must be a valid Eigen scalar.
 */
template <class T>
class ContactSurfaceFace {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ContactSurfaceFace)

  ContactSurfaceFace() = default;

  /** Constructs ContactSurfaceFace.
   @param v0 the first vertex.
   @param v1 the second vertex.
   @param v2 the last vertex.
   @pre   The order of the three vertices gives the counterclockwise normal
          direction, i.e. (v1-v0)x(v2-v0), towards increasing eₘ the scalar
          field on body M.
   */
  ContactSurfaceFace(ContactSurfaceVertex<T>* vertex_A,
                     ContactSurfaceVertex<T>* vertex_B,
                     ContactSurfaceVertex<T>* vertex_C);


  ContactSurfaceVertex<T>* vertex_A();
  ContactSurfaceVertex<T>* vertex_B();
  ContactSurfaceVertex<T>* vertex_C();

  // @pre 0 <= i < 3
  ContactSurfaceVertex<T>* vertex(int i);

  // Converts between Cartesian coordinates r_MQ = (x,y,z) of a point q,
  // expressed in M's frame, to the barycentric coordinates b = (b0,b1,b2)
  // with respect to vertices v0, v1, v2 of this face of the contact surface,
  // where:
  //       (x,y,z) = b0 * v0 + b1 * v1 + b2 * v2.
  //
  // @note barycentric(cartesian(b)) = b; however,
  //       cartesian(barycentric(p)) = p if and only if p is on the plane
  //       of the triangle.
  Vector3<T> barycentric(const Vector3<T>& r_MQ);
  Vector3<T> cartesian(const Vector3<T>& barycentric);

  // Performs interpolation in barycentric coordinates.
  // Linear interpolation from scalar values at vertices.
  // Spherical linear interpolation from vector values at vertices.
  T e(const Vector3<T>& barycentric);
  Vector3<T> grad_h_M(const Vector3<T>& barycentric);

  // Returns the unit normal vector nhat_M to the triangle, expressed in M's
  // frame. The normal vector is oriented towards increasing eₘ the scalar
  // field on body M.
  // @note   Due to discretization, nhat_M of this triangle and grad_h_M of
  //         its vertices are approximately parallel.  With finer
  //         discretization, they become more parallel.
  Vector3<T> nhat_M();

  // Returns the area of this triangle.
  T area();

  // TODO(DamrongGuoy): Implement all the functions declare above.

 protected:
  // The vertices of the face.
  ContactSurfaceVertex<T>* v_[3];
  // The unit normal vector, computed on construction.
  Vector3<T> nhat_M_;
  // The area, computed on construction.
  T area_;
};

/// The contact surface computed by the geometry system.

template <class T>
class ContactSurface {
 public:
  typedef ContactSurfaceFace<T> FaceType;
  typedef ContactSurfaceVertex<T> VertexType;
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ContactSurface)

  const std::vector<FaceType>& triangles() const { return faces_; }
  const std::vector<VertexType>& vertices() const { return vertices_; }
  GeometryId id_A() const { return id_A_; }
  GeometryId id_B() const { return id_B_; }

 private:
  /// The id of the first geometry in the contact.
  GeometryId id_A_;
  /// The id of the second geometry in the contact.
  GeometryId id_B_;
  /// Triangles comprising the contact surface.
  std::vector<FaceType> faces_;
  // Vertices of the contact surface.
  std::vector<VertexType> vertices_;
};

//  template <class FaceType>
//  class ContactSurfaceType {
//  };
//  template <class T>
//  using ContactSurface = ContactSurfaceType<ContactSurfaceFace<T>>;

}  // namespace geometry
}  // namespace drake
