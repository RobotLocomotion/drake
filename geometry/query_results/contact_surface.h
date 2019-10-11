#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/mesh_field_linear.h"
#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/math/rigid_transform.h"

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
  relies on coordinate frames. We borrow terminology from differential
  geometry.

  In this section, the mathematical term _compact set_ (a subset of Euclidean
  space that is closed and bounded) corresponds to the term _geometry_ (or the
  space occupied by the geometry) in SceneGraph.

  We describe the contact surface 𝕊ₘₙ between two intersecting compact subsets
  𝕄 and ℕ of ℝ³ with the scalar fields eₘ and eₙ defined on 𝕄 ⊂ ℝ³ and ℕ ⊂ ℝ³
  respectively:

                 eₘ : 𝕄 → ℝ,
                 eₙ : ℕ → ℝ.

  The _contact surface_ 𝕊ₘₙ is the surface of equilibrium eₘ = eₙ. It is the
  locus of points Q where eₘ(Q) equals eₙ(Q):

               𝕊ₘₙ = { Q ∈ 𝕄 ∩ ℕ : eₘ(Q) = eₙ(Q) }.

  We can define the scalar field eₘₙ on the surface 𝕊ₘₙ as a scalar function
  that assigns Q ∈ 𝕊ₘₙ the value of eₘ(Q), which is the same as eₙ(Q):

               eₘₙ : 𝕊ₘₙ → ℝ,
               eₘₙ(Q) = eₘ(Q) = eₙ(Q).

  We can also define the scalar field hₘₙ on 𝕄 ∩ ℕ as the difference between
  eₘ and eₙ:

               hₘₙ : 𝕄 ∩ ℕ → ℝ,
               hₘₙ(Q) = eₘ(Q) - eₙ(Q).

  It follows that the gradient vector field ∇hₘₙ on 𝕄 ∩ ℕ equals the difference
  between the the gradient vector fields ∇eₘ and ∇eₙ:

               ∇hₘₙ : 𝕄 ∩ ℕ → ℝ³,
               ∇hₘₙ(Q) = ∇eₘ(Q) - ∇eₙ(Q).

  By construction, Q ∈ 𝕊ₘₙ if and only if hₘₙ(Q) = 0. In other words, 𝕊ₘₙ is
  the zero level set of hₘₙ. It follows that, for Q ∈ 𝕊ₘₙ, ∇hₘₙ(Q) is
  orthogonal to the surface 𝕊ₘₙ at Q in the direction of increasing eₘ - eₙ.
  <!-- Note from PR discussion
    1. `∇hₘₙ` *is* a well-behaved vector (subject to some assumptions -- see
        below).
    2. The contact surface "clips" intersecting geometries M and N into disjoint
       geometries M' and N'. `∇hₘₙ` points *out* of M' and *into* N'.
    Assumptions:
    - `∇e` is differentiable and "points outward"

   TODO(DamrongGuoy):
   1. Document the above listed properties of `∇hₘₙ`.
   2. Add a todo indicating M' and N' should be illustrated in the docs.
   3. Explicitly add the assumptions on `e` that make this interpretation valid.
  -->

  Notice that the domain of eₘₙ is the two-dimensional surface 𝕊ₘₙ, while the
  domain of ∇hₘₙ is the three-dimensional compact set 𝕄 ∩ ℕ.
  Even though eₘₙ and ∇hₘₙ are defined on different domains (𝕊ₘₙ and 𝕄 ∩ ℕ),
  our implementation only represents them on their common domain, i.e., 𝕊ₘₙ.

  <h2> Barycentric Coordinates </h2>

  For Point Q on the surface mesh of the contact surface between Geometry M and
  Geometry N, r_WQ = (x,y,z) is the displacement vector from the origin of the
  world frame to Q expressed in the coordinate frame of W. We also have the
  _barycentric coordinates_ (b0, b1, b2) on a triangle of the surface mesh that
  contains Q. With vertices of the triangle labeled as v₀, v₁, v₂, we can
  map (b0, b1, b2) to r_WQ by:

               r_WQ = b0 * r_Wv₀ + b1 * r_Wv₁ + b2 * r_Wv₂,
               b0 + b1 + b2 = 1, bᵢ ∈ [0,1],

  where r_Wvᵢ is the displacement vector of the vertex labeled as vᵢ from the
  origin of the world frame, expressed in the world frame.

  We use the barycentric coordinates to evaluate the field values.

  @tparam T the underlying scalar type. Must be a valid Eigen scalar.
 */
template <typename T>
class ContactSurface {
 public:
  ContactSurface(const ContactSurface& surface) {
    *this = surface;
  }

  ContactSurface& operator=(const ContactSurface& surface) {
    if (&surface == this)
      return *this;

    id_M_ = surface.id_M_;
    id_N_ = surface.id_N_;
    mesh_W_ = std::make_unique<SurfaceMesh<T>>(*surface.mesh_W_);

    // We can't simply copy the mesh fields; the copies must contain pointers
    // to the new mesh. So, we use CloneAndSetMesh() instead.
    e_MN_.reset(static_cast<SurfaceMeshFieldLinear<T, T>*>(
        surface.e_MN_->CloneAndSetMesh(mesh_W_.get()).release()));
    grad_h_MN_W_.reset(static_cast<SurfaceMeshFieldLinear<Vector3<T>, T>*>(
        surface.grad_h_MN_W_->CloneAndSetMesh(mesh_W_.get()).release()));

    return *this;
  }

  ContactSurface(ContactSurface&&) = default;
  ContactSurface& operator=(ContactSurface&&) = default;

  /** Constructs a ContactSurface.
   @param id_M         The id of the first geometry M.
   @param id_N         The id of the second geometry N.
   @param mesh_W       The surface mesh of the contact surface 𝕊ₘₙ between M
                       and N. The mesh vertices are defined in the world frame.
   @param e_MN         Represents the scalar field eₘₙ on the surface mesh.
   @param grad_h_MN_W  Represents the vector field ∇hₘₙ on the surface mesh,
                       expressed in the world frame. Due to discretization,
                       `grad_h_MN_W` at a vertex need not be strictly
                       orthogonal to every triangle sharing the vertex.
                       Orthogonality generally does improve with finer
                       discretization.
   @note If the id_M is greater than the id_N, we will swap M and N.
         Therefore, grad_h_MN_W will switch its direction.
   */
  ContactSurface(
      GeometryId id_M, GeometryId id_N, std::unique_ptr<SurfaceMesh<T>> mesh_W,
      std::unique_ptr<SurfaceMeshFieldLinear<T, T>> e_MN,
      std::unique_ptr<SurfaceMeshFieldLinear<Vector3<T>, T>> grad_h_MN_W)
      : id_M_(id_M),
        id_N_(id_N),
        mesh_W_(std::move(mesh_W)),
        e_MN_(std::move(e_MN)),
        grad_h_MN_W_(std::move(grad_h_MN_W)) {
    if (id_N_ < id_M_) SwapMAndN();
  }

  /** Returns the geometry id of Geometry M. */
  GeometryId id_M() const { return id_M_; }

  /** Returns the geometry id of Geometry N. */
  GeometryId id_N() const { return id_N_; }

  // TODO(damrongguoy) Consider removing these evaluation methods and instead
  // make the fields accessible, and then evaluate the fields directly.

  /** Evaluates the scalar field eₘₙ at Point Q in a triangle.
    Point Q is specified by its barycentric coordinates.
    @param face         The face index of the triangle.
    @param barycentric  The barycentric coordinates of Q on the triangle.
   */
  T EvaluateE_MN(
      SurfaceFaceIndex face,
      const typename SurfaceMesh<T>::Barycentric& barycentric) const {
    return e_MN_->Evaluate(face, barycentric);
  }

  /** Evaluates the scalar field eₘₙ at the given vertex on the contact surface
    mesh.
    @param vertex       The index of the vertex in the mesh.
   */
  T EvaluateE_MN(SurfaceVertexIndex vertex) const {
    return e_MN_->EvaluateAtVertex(vertex);
  }

  /** Evaluates the vector field ∇hₘₙ at Point Q on a triangle.
    Point Q is specified by its barycentric coordinates.
    @param face         The face index of the triangle.
    @param barycentric  The barycentric coordinates of Q on the triangle.
    @retval  grad_h_MN_W is the vector expressed in the world frame.
   */
  Vector3<T> EvaluateGrad_h_MN_W(
      SurfaceFaceIndex face,
      const typename SurfaceMesh<T>::Barycentric& barycentric) const {
    return grad_h_MN_W_->Evaluate(face, barycentric);
  }

  /** Evaluates the vector field ∇hₘₙ at the given vertex on the contact surface
    mesh.
    @param vertex       The index of the vertex in the mesh.
    @retval  grad_h_MN_W is the vector expressed in the world frame.
   */
  Vector3<T> EvaluateGrad_h_MN_W(SurfaceVertexIndex vertex) const {
    return grad_h_MN_W_->EvaluateAtVertex(vertex);
  }

  DRAKE_DEPRECATED("2019-12-01", "Use mesh_W() instead.")
  const SurfaceMesh<T>& mesh() const { return mesh_W(); }

  /** Returns a reference to the surface mesh whose vertex
   positions are measured and expressed in the world frame.
   */
  const SurfaceMesh<T>& mesh_W() const {
    DRAKE_DEMAND(mesh_W_ != nullptr);
    return *mesh_W_;
  }

  /** Returns a reference to the scalar field eₘₙ. */
  const MeshField<T, SurfaceMesh<T>>& e_MN() const { return *e_MN_; }

  /** Returns a reference to the vector field ∇hₘₙ. */
  const MeshField<Vector3<T>, SurfaceMesh<T>>& grad_h_MN_W() const {
    return *grad_h_MN_W_;
  }

  /** Checks to see whether the given ContactSurface object is equal (all data
   match to bit-wise precision).
   Note: currently requires the fields of the objects to be of type
   MeshFieldLinear, otherwise the current simple checking of equal values at
   vertices is insufficient.
   @param surface The surface for comparison.
   @returns `true` if the given surface is equal.
   */
  bool Equal(const ContactSurface<T>& surface) const {
    // First check the meshes.
    if (!this->mesh_W().Equal(surface.mesh_W()))
      return false;

    // Now examine the pressure field.
    const MeshFieldLinear<T, SurfaceMesh<T>>* pressure_field =
        dynamic_cast<const MeshFieldLinear<T, SurfaceMesh<T>>*>(
            &(this->e_MN()));;
    DRAKE_DEMAND(pressure_field);
    if (!pressure_field->Equal(surface.e_MN()))
      return false;

    // Now examine the grad_h field.
    const MeshFieldLinear<Vector3<T>, SurfaceMesh<T>>* grad_h_field =
        dynamic_cast<const MeshFieldLinear<Vector3<T>, SurfaceMesh<T>>*>(
            &(this->grad_h_MN_W()));;
    DRAKE_DEMAND(grad_h_field);
    if (!grad_h_field->Equal(surface.grad_h_MN_W()))
      return false;

    // All checks passed.
    return true;
  }

 private:
  // Swaps M and N (modifying the data in place to reflect the change).
  void SwapMAndN() {
    std::swap(id_M_, id_N_);
    // TODO(SeanCurtis-TRI): Determine if this work is necessary. It is neither
    // documented nor tested that the face winding is guaranteed to be one way
    // or the other. Alternatively, this should be documented and tested.
    mesh_W_->ReverseFaceWinding();

    // Simply reverse the direction of the vector field.
    std::vector<Vector3<T>>& values = grad_h_MN_W_->mutable_values();
    for (SurfaceVertexIndex v(0); v < mesh_W_->num_vertices(); ++v) {
      values[v] = -values[v];
    }

    // Note: the scalar field does not depend on the order of M and N.
  }

  // The id of the first geometry M.
  GeometryId id_M_;
  // The id of the second geometry N.
  GeometryId id_N_;
  // The surface mesh of the contact surface 𝕊ₘₙ between M and N.
  std::unique_ptr<SurfaceMesh<T>> mesh_W_;
  // TODO(SeanCurtis-TRI): We can only construct from a linear field, so store
  //  it as such for now. This can be promoted once there's a construction that
  //  uses a different derivation.
  // Represents the scalar field eₘₙ on the surface mesh.
  std::unique_ptr<SurfaceMeshFieldLinear<T, T>> e_MN_;
  // Represents the vector field ∇hₘₙ on the surface mesh, expressed in M's
  // frame.
  std::unique_ptr<SurfaceMeshFieldLinear<Vector3<T>, T>> grad_h_MN_W_;
  // TODO(DamrongGuoy): Remove this when we allow direct access to e_MN and
  //  grad_h_MN.
  template <typename U> friend class ContactSurfaceTester;
};

}  // namespace geometry
}  // namespace drake
