#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/query_results/mesh_field.h"
#include "drake/geometry/query_results/surface_mesh.h"

namespace drake {
namespace geometry {


// TODO(DamrongGuoy): Update the reference to the "pressure field model"
//  paper when it is accepted to the conference.

/** The %ContactSurface characterizes the intersection of two geometries M
  and N as a contact surface with a scalar field and a vector field, whose
  purpose is to support the hydroelastic pressure field contact model as
  described in:

      R. Elandt, E. Drumwright, M. Sherman, and Andy Ruina. A pressure
      field model for fast, robust approximation of net contact force
      and moment between nominally rigid objects. Submitted to the
      2019 IEEE/RSJ Intl. Conf. on Intelligent Robots and Systems.

  <h2> Mathematical Concepts </h2>

  In this section, we give motivation for the concept of contact surface from
  the hydroelastic pressure field contact model. Here the mathematical
  discussion is coordinate-free (treatment of the topic without reference to
  any particular coordinate system); however, our implementation heavily
  relies on coordinates frames. We borrow terminology from differential
  geometry.

  In this section, the mathematical term _compact set_ (a subset of Euclidean
  space that is closed and bounded) corresponds to the term _body_ (or the
  space occupied by the body) in robotics.

  We describe the contact surface Sₘₙ between two intersecting compact subsets
  M and N of ℝ³ with the scalar fields eₘ and eₙ defined on M ⊂ ℝ³ and N ⊂ ℝ³
  respectively:

                 eₘ : M → ℝ,
                 eₙ : N → ℝ.

  The _contact surface_ Sₘₙ is the surface of equilibrium eₘ = eₙ:

               Sₘₙ = { q ∈ M ∩ N : eₘ(q) = eₙ(q) }.

  Here we write the lower-case q for a point that is an element of a compact
  subset of the Euclidean space ℝ³.  In our implementation, it corresponds to
  a point Q, or r_MQ_M its displacement vector from the origin of the
  coordinates frame of the body M expressed in M's frame.

  We can define the scalar field eₘₙ on Sₘₙ as the common value of eₘ and eₙ:

               eₘₙ : Sₘₙ → ℝ,
               eₘₙ(q) = eₘ(q) = eₙ(q).

  We can also define the scalar field hₘₙ on M ∩ N as the difference between
  eₘ and eₙ:

               hₘₙ : M ∩ N → ℝ,
               hₘₙ(q) = eₘ(q) - eₙ(q).

  It follows that the gradient vector field ∇hₘₙ on M ∩ N equals the difference
  between the the gradient vector fields ∇eₘ and ∇eₙ:

               ∇hₘₙ : M ∩ N → ℝ³,
               ∇hₘₙ(q) = ∇eₘ(q) - ∇eₙ(q).

  By construction, q ∈ Sₘₙ if and only if hₘₙ(q) = 0. In other words, Sₘₙ is
  the zero level set of hₘₙ. It follows that, for q ∈ Sₘₙ, ∇hₘₙ(q) is
  orthogonal to the surface Sₘₙ at q in the direction of increasing eₘ - eₙ.

  Notice that the domain of eₘₙ is the two-dimensional surface Sₘₙ, while the
  domain of ∇hₘₙ is the three-dimensional compact set M ∩ N.
  Even though eₘₙ and ∇hₘₙ are defined on different domains (Sₘₙ and M ∩ N),
  our implementation only represents them on their common domain, i.e., Sₘₙ.

  <h2> Computational Representation </h2>

  This section resumes our standard terminology of @ref
  multibody_frames_and_bodies "Frames and Bodies". From now on, we will write
  M and N for body M and body N, each of which is associated with a coordinate
  frame. Quantities defined in those coordinate frames will be denoted using
  either _M or _N, respectively. We will write Sₘₙ, eₘₙ, and ∇hₘₙ for the
  mathematical concepts in the previous section, and use the terms
  mesh, e_MN, and grad_h_MN_M for the representation or approximation of
  the mathematical concepts.

  Our ContactSurface data structure represents the contact surface Sₘₙ as a
  triangulated surface mesh using SurfaceMesh, and represents the scalar field
  eₘₙ and the vector field ∇hₘₙ using MeshField.

  The term `mesh` refers to the surface mesh of Sₘₙ, which is a free
  surface that does not necessarily bound a volume. The term `e_MN`
  represents the scalar field eₘₙ on the `mesh`. The term `grad_h_MN_M`
  represents the vector field ∇hₘₙ on the `mesh` and is expressed
  in the coordinate frame of Body M.

  Mathematically ∇hₘₙ is defined on the common intersection of the space
  occupied by Body M and Body N; however, the member variable
  `grad_h_MN_M` restricts the domain of ∇hₘₙ to the surface mesh only.

  Due to discretization, the vector `grad_h_MN_M` at a vertex need not be
  strictly orthogonal to any triangle sharing the vertex. Orthogonality does
  improve with finer discretization.

  <h3> Barycentric Coordinates </h3>

  For a point Q on the surface mesh of the contact surface between Body M and
  Body N, r_MQ = (x,y,z) is the displacement vector from the origin of M's
  frame to Q expressed in the coordinate frame of M. We also have the
  _barycentric coordinates_ (b0, b1, b2) on a triangle of the surface mesh that
  contains Q. With vertices of the triangle labeled as v₀, v₁, v₂, we can
  map (b0, b1, b2) to r_MQ by:

               r_MQ = b0 * r_MV₀ + b1 * r_MV₁ + b2 * r_MV₂,
               b0 + b1 + b2 = 1, bᵢ ∈ [0,1],

  where r_MVᵢ is the displacement vector of the vertex labeled as vᵢ from the
  origin of M's frame, expressed in M's frame.

  We use the barycentric coordinates to evaluate the field values.

  @tparam T the underlying scalar type. Must be a valid Eigen scalar.
 */
template <typename T>
class ContactSurface {
 public:
  /** @name Does not allow copy; implements MoveConstructible, MoveAssignable
   */
  //@{
  ContactSurface(const ContactSurface&) = delete;
  ContactSurface& operator=(const ContactSurface&) = delete;
  ContactSurface(ContactSurface&&) = default;
  ContactSurface& operator=(ContactSurface&&) = default;
  //@}

  /** Constructs a ContactSurface from SurfaceMesh and MeshField's.
   @param id_M         the id of the first body M.
   @param id_N         the id of the second body N.
   @param mesh         the surface mesh of the contact surface Sₘₙ between M
                       and N.
   @param e_MN         represents the common scalar field eₘₙ on the surface
                       mesh.
   @param grad_h_MN_M  represents the vector field ∇hₘₙ on the surface mesh,
                       expressed in M's frame.
   */
  ContactSurface(GeometryId id_M, GeometryId id_N,
                 std::unique_ptr<SurfaceMesh<T>> mesh,
                 SurfaceMeshFieldLinear<T, T>&& e_MN,
                 SurfaceMeshFieldLinear<Vector3<T>, T>&& grad_h_MN_M)
      : id_M_(id_M),
        id_N_(id_N),
        mesh_(std::move(mesh)),
        e_MN_(std::move(e_MN)),
        grad_h_MN_M_(std::move(grad_h_MN_M)) {}

  /** Returns the geometry id of the body M. */
  GeometryId id_M() const { return id_M_; }

  /** Returns the geometry id of the body N. */
  GeometryId id_N() const { return id_N_; }

  /** Evaluates the scalar field eₘₙ at a point Q in a triangle.
    The point Q is specified by its barycentric coordinates.
    @param face         The face index of the triangle.
    @param barycentric  The barycentric coordinates of Q on the triangle.
   */
  T EvaluateE_MN(SurfaceFaceIndex face,
                 const typename SurfaceMesh<T>::Barycentric& barycentric) {
    return e_MN_.Evaluate(face, barycentric);
  }

  /** Evaluates the vector field ∇hₘₙ at a point Q on a triangle.
    The point Q is specified by its barycentric coordinates.
    @param face         The face index of the triangle.
    @param barycentric  The barycentric coordinates of Q on the triangle.
    @retval  grad_h_MN_M is the vector expressed in M's frame.
   */
  Vector3<T> EvaluateGrad_h_MN_M(
      SurfaceFaceIndex face,
      const typename SurfaceMesh<T>::Barycentric& barycentric) {
    return grad_h_MN_M_.Evaluate(face, barycentric);
  }

  /** Returns the number of triangular faces.
   */
  int num_faces() const { return mesh_->num_faces(); }

  /** Returns the number of vertices.
   */
  int num_vertices() const { return mesh_->num_vertices(); }

  /** Returns the reference to the surface mesh.
   */
  const SurfaceMesh<T>& mesh() const {
    DRAKE_DEMAND(mesh_.get() != nullptr);
    return *mesh_;
  }

 private:
  /** The id of the first body M */
  GeometryId id_M_;
  /** The id of the second body N. */
  GeometryId id_N_;
  /** The surface mesh of the contact surface Sₘₙ between M and N. */
  std::unique_ptr<SurfaceMesh<T>> mesh_;
  /** Represents the common scalar field eₘₙ on the surface mesh. */
  SurfaceMeshFieldLinear<T, T> e_MN_;
  /** Represents the vector field ∇hₘₙ on the surface mesh, expressed in M's
      frame */
  SurfaceMeshFieldLinear<Vector3<T>, T> grad_h_MN_M_;
};

}  // namespace geometry
}  // namespace drake


