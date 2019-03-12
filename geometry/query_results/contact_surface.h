#pragma once

#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/query_results/mesh_field.h"
#include "drake/geometry/query_results/surface_mesh.h"

namespace drake {
namespace geometry {


/** The %ContactSurface characterizes the intersection of two penetrating
  geometries M and N as a contact surface with a scalar field and a vector
  field.

  <b> Mathematical Definitions </b>

  We describe the _contact_ _surface_ Sₘₙ between two compliant bodies M and
  N.  A contact surface is a surface of equilibrium eₘ = eₙ, where eₘ and
  eₙ are the given scalar fields on bodies M and N:

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

  <b> Computational Representation </b>

  Computationally our data structure represents the contact surface Sₘₙ as a
  triangulated _surface_ _mesh_, which is the division of the contact surface
  into triangular faces that share vertices. See SurfaceMesh for more details.

  We represent the scalar field e() and the vector field grad_h_M() on the
  surface mesh using the class SurfaceMeshField that can evaluate
  the field value at any point on any triangle of the SurfaceMesh.

  At each vertex v, we store the scalar value:

               e(v) = eₘ(r_MV) = eₙ(r_NV)

  and the vector value:

               grad_h_M(v) = ∇hₘₙ(r_MV), expressed in M's frame.

  On each triangle, we provide evaluation of the scalar `e` and the vector
  `grad_h_M`.

  Due to discretization, the vector `grad_h_M` at a vertex v is not
  strictly orthogonal to any triangle sharing v.  Orthogonality does improve
  with finer discretization.

  <b> Local Coordinates </b>

  A point p in a triangle with vertices v₀, v₁, v₂ can be specified by
  its _barycentric_ _coordinates_ (b0, b1, b2) by:

               p = b0 * v₀ + b1 * v₁ + b2 * v₂.

  In our code, we use the _standard_ _coordinates_ (s1, s2) that maps to the
  barycentric coordinates by:

               (b0, b1, b2) = (1 - s1 - s2, s1, s2).

  The standard coordinates (s1, s2) maps the standard right
  triangle with vertices (0,0), (1,0), (0,1) in ℝ² to the triangle with
  vertices v₀, v₁, v₂ in ℝ³ respectively.

  @tparam T the underlying scalar type. Must be a valid Eigen scalar.
 */
template <class T>
class ContactSurface {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ContactSurface)

  /** Returns the geometry id of the body M. */
  GeometryId id_M() const { return id_M_; }

  /** Returns the geometry id of the body N. */
  GeometryId id_N() const { return id_N_; }

  /** Evaluates the scalar field e() at a point in a triangle.
    The point p is specified by its standard coordinates (s1, s2).
    @param f The face index of the triangle.
    @param s The standard coordinates (s1, s2).
   */
  T e(SurfaceFaceIndex f, const Vector2<T>& s) {
    return e_.Evaluate(f, s);
  }

  /** Evaluate the vector field grad_h_M() on a triangle.
    The point p is specified by its standard coordinates (s1, s2).
    @param f The face index of the triangle.
    @param s The standard coordinates (s1, s2).
    @retval  ∇h The vector value is expressed in M's frame.
   */
  Vector3<T> grad_h_M(SurfaceFaceIndex f, const Vector2<T>& s) {
    return grad_h_M_.Evaluate(f, s);
  }

  /** Returns the number of triangular faces.
   */
  int num_faces() { return mesh_.num_faces(); }

  /** Returns the number of vertices.
   */
  int num_vertices() { return mesh_.num_vertices(); }

 private:
  /// The id of the first geometry in the contact.
  GeometryId id_M_;
  /// The id of the second geometry in the contact.
  GeometryId id_N_;

  SurfaceMesh<T> mesh_;
  // Scalar field `e`.
  SurfaceMeshField<T, T> e_;
  // Vector field ∇hₘₙ, expressed in M's frame.
  SurfaceMeshField<Vector3<T>, T> grad_h_M_;
};

using ContactSurfaced = ContactSurface<double>;

}  // namespace geometry
}  // namespace drake


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


/* Design concepts.

 We roughly partition the data structures into domain discretization
 (triangulated surface meshes) and the physical field variables (pressure, slip,
 traction, etc.) defined on the domain, and the class ContactSurface combines
 both of them together.  This design has motivation from Finite Element
 representations.

 The collection of classes that represent a triangulated surface mesh consist
 of SurfaceVertex, SurfaceFace, and SurfaceMesh. The class that maps an
 underlying surface mesh to a physical field is SurfaceMeshField. Users can
 define many SurfaceMeshField's on the same SurfaceMesh.

 SurfaceMeshField have two storage traits: vertex data and face data. Most field
 variables (e.g., pressure, slip, traction) are vertex data. In a few
 cases (e.g., face normal vector, face area), they are face data.

 ContactSurface has four built-in SurfaceMeshField's: the scalar field e(), the
 vector field grad_h_M(), the face normal vector nhat_M(), and the face area().
 */


//------------------------------------------------------------------------------
/*
  <b> Terminology from Finite Element Method </b>

  We borrow terminology from Finite Element Method:

      O.C. Zienkiewicz, R.L. Taylor & J.Z. Zhu.
      The Finite Element Method: Its Basis and Fundamentals.
      Chapter 3. Weak Forms and Finite Element Approximation.
      Chapter 6. Shape Functions, Derivatives, and Integration.

  We divide the domain into small regular shaped regions (e.g. triangles or
  tetrahedrons), each of which define a _finite_ _element_ domain, and
  the finite set of points shared by the finite elements define the
  _nodes_.  The division of the domain into elements and nodes is
  called a _finite_ _element_ _mesh_.

  Each of the two compliant bodies M and N is discretized into
  a 3D _volume_ _mesh_ consisting of tetrahedral elements.

  The contact surface is discretized into a 3D _surface_ _mesh_
  consisting of triangular elements. See SurfaceMesh.

  On each finite element E, we have one _shape_ _function_ Nᵢ for each
  node nᵢ of the element,

               Nᵢ : E → ℝ

  and the _finite_ _element_ _approximation_ uᵉ of an arbitrary field
  variable u on the element E is:

               uᵉ(x,y,z) = ∑ Nᵢ(x,y,z) * uᵢ

  where uᵢ is the value of u at the node nᵢ.

  <b>Example 1. _Linear_ _Triangular_ _Element_</b>

  A _linear_ _triangular_ _element_ E with three vertices v₀, v₁, v₂
  has its three nodes n₀, n₁, n₂ coincide with the vertices.  Its finite
  element approximation is:

               uᵉ = N₀ * u₀ + N₁ * u₁ + N₂ * u₂

  For a triangular element, it is beneficial to use a map from
  _parent_ _coordinate_ _system_ (L₀, L₁, L₂) (also known as
  _barycentric_ or _area_ _coordinates_) to a position p
  on the triangle:

               p(L₀, L₁, L₂) = L₀ * v₀ + L₁ * v₁ + L₂ * v₂.

  The parent coordinates are constrained by:

               L₀ + L₁ + L₂ = 1, Lᵢ ∈ [0,1].

  Geometrically we can define Lᵢ(p) as the ratio between the area of the
  triangle p, vᵢ₊₁, vᵢ₊₂ and the area of the triangular element E.

  For a linear triangular element, the shape function is the same as the
  parent coordinate function:

               Nᵢ = Lᵢ, i = 0,1,2

  Linear tetrahedral element is similar.


  <b>Example 2. _Quadratic_ _Tetrahedral_ _Element_</b>

  Currently we only use linear elements for contact surfaces; however, we will
  also discuss _quadratic_ _elements_, which might be useful for setting up
  a pressure field in body M that is discretized into a 3D volume mesh
  consisting of tetrahedral elements.

  Similar to a triangular element, a tetrahedral element with vertices
  v₀, v₁, v₂, v₃  has a map from _parent_ _coordinate_ _system_
  (L₀, L₁, L₂, L₃) to a position p in the tetrahedron:

               p(L₀, L₁, L₂, L₃) = L₀ * v₀ + L₁ * v₁ + L₂ * v₂ +  L₃ * v₃,
               L₀ + L₁ + L₂ + L₃ = 1, Lᵢ ∈ [0,1].

  A _quadratic_ _tetrahedral_ _element_ E has 10 nodes. It has one node at
  each of the four vertices and one node at the mid-side of each of the
  six edges:

               nᵢ = vᵢ, i = 0,1,2,3
               nᵢⱼ = (vᵢ + vⱼ)/2, 0 ≤ i < j ≤ 3

  The 10 _shape_ _functions_ of a quadratic tetrahedral element are
  constructed from the parent coordinate function like this:

               Nᵢ = (2 * Lᵢ - 1) * Lᵢ, i = 0,1,2,3
               Nᵢⱼ = 4 Lᵢ * Lⱼ, 0 ≤ i < j ≤ 3

  The _finite_ _element_ _approximation_ uᵉ of a field variable u on the
  above quadratic tetrahedral element E is:

               uᵉ = ∑(Nᵢ * uᵢ) + ∑(Nᵢⱼ * uᵢⱼ), 0 ≤ i ≤ 3; 0 ≤ i < j ≤ 3

  where uᵢ is the value of u at the node nᵢ, and uᵢⱼ is the value of u at the
  mid-side node nᵢⱼ.

  Quadratic triangular element is similar.
 */


