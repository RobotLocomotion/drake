#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/proximity/surface_mesh_field.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {

/** The %ContactSurface characterizes the intersection of two geometries M
  and N as a contact surface with a scalar field and a vector field, whose
  purpose is to support the hydroelastic pressure field contact model as
  described in:

      R. Elandt, E. Drumwright, M. Sherman, and Andy Ruina. A pressure
      field model for fast, robust approximation of net contact force
      and moment between nominally rigid objects. IROS 2019: 8238-8245.

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
  between the gradient vector fields ∇eₘ and ∇eₙ:

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

  <h2> Discrete Representation </h2>

  In practice, the contact surface is approximated with a discrete triangle
  mesh. The triangle mesh's normals are defined *per face*. The normal of each
  face is guaranteed to point "out of" N and "into" M. They can be accessed via
  `mesh_W().face_normal(face_index)`.

  The pressure values on the contact surface are represented as a continuous,
  piecewise-linear function, accessed via e_MN().

  The normals of the mesh are discontinuous at triangle boundaries, but the
  pressure can be meaningfully evaluated over the entire domain of the mesh.

  When available, the values of ∇eₘ and ∇eₙ are represented as a discontinuous,
  piecewise-constant function over the triangles -- one vector per triangle.
  These quantities are accessed via EvaluateGradE_M_W() and EvaluateGradE_N_W(),
  respectively.

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

  @tparam_nonsymbolic_scalar
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

    if (surface.grad_eM_W_) {
      grad_eM_W_ =
          std::make_unique<std::vector<Vector3<T>>>(*surface.grad_eM_W_);
    }
    if (surface.grad_eN_W_) {
      grad_eN_W_ =
          std::make_unique<std::vector<Vector3<T>>>(*surface.grad_eN_W_);
    }

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
   @pre The face normals in `mesh_W` point *out of* geometry N and *into* M.
   @note If `id_M > id_N`, the labels will be swapped and the normals of the
         mesh reversed (to maintain the documented invariants). Comparing the
         input parameters with the members of the resulting %ContactSurface will
         reveal if such a swap has occurred.
   */
  ContactSurface(GeometryId id_M, GeometryId id_N,
                 std::unique_ptr<SurfaceMesh<T>> mesh_W,
                 std::unique_ptr<SurfaceMeshFieldLinear<T, T>> e_MN)
      : ContactSurface(id_M, id_N, std::move(mesh_W), std::move(e_MN), nullptr,
                       nullptr) {}

  /** Constructs a ContactSurface with the optional gradients of the constituent
   scalar fields.
   @param id_M         The id of the first geometry M.
   @param id_N         The id of the second geometry N.
   @param mesh_W       The surface mesh of the contact surface 𝕊ₘₙ between M
                       and N. The mesh vertices are defined in the world frame.
   @param e_MN         Represents the scalar field eₘₙ on the surface mesh.
   @param grad_eM_W    ∇eₘ sampled once per face, expressed in the world frame.
   @param grad_eN_W    ∇eₙ sampled once per face, expressed in the world frame.
   @pre The face normals in `mesh_W` point *out of* geometry N and *into* M.
   @pre If given, `grad_eM_W` and `grad_eN_W` must have as many entries as
        `mesh_W` has faces and the ith entry in each should correspond to the
        ith face in `mesh_W`.
   @note If `id_M > id_N`, the labels will be swapped and the normals of the
         mesh reversed (to maintain the documented invariants). Comparing the
         input parameters with the members of the resulting %ContactSurface will
         reveal if such a swap has occurred.
   */
  ContactSurface(GeometryId id_M, GeometryId id_N,
                 std::unique_ptr<SurfaceMesh<T>> mesh_W,
                 std::unique_ptr<SurfaceMeshFieldLinear<T, T>> e_MN,
                 std::unique_ptr<std::vector<Vector3<T>>> grad_eM_W,
                 std::unique_ptr<std::vector<Vector3<T>>> grad_eN_W)
      : id_M_(id_M),
        id_N_(id_N),
        mesh_W_(std::move(mesh_W)),
        e_MN_(std::move(e_MN)),
        grad_eM_W_(std::move(grad_eM_W)),
        grad_eN_W_(std::move(grad_eN_W)) {
    // If defined the gradient values must map 1-to-1 onto elements.
    DRAKE_THROW_UNLESS(grad_eM_W_ == nullptr ||
                       static_cast<int>(grad_eM_W_->size()) ==
                           mesh_W_->num_elements());
    DRAKE_THROW_UNLESS(grad_eN_W_ == nullptr ||
                       static_cast<int>(grad_eN_W_->size()) ==
                           mesh_W_->num_elements());
    if (id_N_ < id_M_) SwapMAndN();
  }

  /** Returns the geometry id of Geometry M. */
  GeometryId id_M() const { return id_M_; }

  /** Returns the geometry id of Geometry N. */
  GeometryId id_N() const { return id_N_; }

  /** @name  Evaluation of constituent pressure fields

   The %ContactSurface *provisionally* includes the gradients of the constituent
   pressure fields (∇eₘ and ∇eₙ) sampled on the contact surface. In order for
   these values to be included in an instance, the gradient for the
   corresponding mesh must be well defined. For example a rigid mesh will not
   have a well-defined pressure gradient; as stiffness goes to infinity, the
   geometry becomes rigid and the gradient _direction_ converges to the
   direction of the rigid mesh's surface normals, but the magnitude goes to
   infinity, producing a pressure gradient that would be some variant of
   `<∞, ∞, ∞>`.

   Accessing the gradient values must be pre-conditioned on a test that the
   particular instance of %ContactSurface actually contains the gradient data.
   The presence of gradient data for each geometry must be confirmed separately.

   The values ∇eₘ and ∇eₘ are piecewise constant over the %ContactSurface and
   can only be evaluate on a per-triangle basis.  */
  //@{

  /** @returns `true` if `this` contains values for ∇eₘ.  */
  bool HasGradE_M() const { return grad_eM_W_ != nullptr; }

  /** @returns `true` if `this` contains values for ∇eₙ.  */
  bool HasGradE_N() const { return grad_eN_W_ != nullptr; }

  /** Returns the value of ∇eₘ for the triangle with index `index`.
   @throws std::exception if HasGradE_M() returns false.
   @pre `index ∈ [0, mesh().num_faces())`.  */
  const Vector3<T>& EvaluateGradE_M_W(int index) const {
    if (grad_eM_W_ == nullptr) {
      throw std::runtime_error(
          "ContactSurface::EvaluateGradE_M_W() invalid; no gradient values "
          "stored. Mesh M may be rigid, or the constituent gradients weren't "
          "requested.");
    }
    return (*grad_eM_W_)[index];
  }

  /** Returns the value of ∇eₙ for the triangle with index `index`.
   @throws std::exception if HasGradE_N() returns false.
   @pre `index ∈ [0, mesh().num_faces())`.  */
  const Vector3<T>& EvaluateGradE_N_W(int index) const {
    if (grad_eN_W_ == nullptr) {
      throw std::runtime_error(
          "ContactSurface::EvaluateGradE_N_W() invalid; no gradient values "
          "stored. Mesh N may be rigid, or the constituent gradients weren't "
          "requested.");
    }
    return (*grad_eN_W_)[index];
  }

  //@}

  /** Returns a reference to the surface mesh whose vertex
   positions are measured and expressed in the world frame.
   */
  const SurfaceMesh<T>& mesh_W() const {
    DRAKE_DEMAND(mesh_W_ != nullptr);
    return *mesh_W_;
  }

  /** Returns a reference to the scalar field eₘₙ. */
  const SurfaceMeshFieldLinear<T, T>& e_MN() const { return *e_MN_; }

  // TODO(#12173): Consider NaN==NaN to be true in equality tests.
  /** Checks to see whether the given ContactSurface object is equal via deep
   exact comparison. NaNs are treated as not equal as per the IEEE standard.

   @param surface The contact surface for comparison.
   @returns `true` if the given contact surface is equal.
   */
  bool Equal(const ContactSurface<T>& surface) const {
    // First check the meshes.
    if (!this->mesh_W().Equal(surface.mesh_W())) return false;

    // Now examine the pressure field.
    if (!this->e_MN().Equal(surface.e_MN())) return false;

    // TODO(SeanCurtis-TRI) This isn't testing the following quantities:
    //  1. Geometry ids
    //  2. Gradients.

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

    // Note: the scalar field does not depend on the order of M and N.
    std::swap(grad_eM_W_, grad_eN_W_);
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

  // The gradients of the pressure fields eₘ and eₙ sampled on the contact
  // surface. There is one gradient value *per contact surface triangle*.
  // These quantities may not be defined if the gradient is not well-defined.
  // See class documentation for elaboration.
  std::unique_ptr<std::vector<Vector3<T>>> grad_eM_W_;
  std::unique_ptr<std::vector<Vector3<T>>> grad_eN_W_;

  // TODO(DamrongGuoy): Remove this when we allow direct access to e_MN.
  template <typename U> friend class ContactSurfaceTester;
};

}  // namespace geometry
}  // namespace drake
