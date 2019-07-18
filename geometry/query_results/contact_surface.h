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

  <h2> Computational Representation </h2>

  This section resumes our standard terminology of @ref
  multibody_frames_and_bodies "Frames and Bodies". From now on, we will write
  M and N for Geometry M and Geometry N, each of which is associated with a
  coordinate frame of the same name. Quantities defined in those coordinate
  frames will be denoted using either _M or _N, respectively.

  <h3> Barycentric Coordinates </h3>

  For Point Q on the surface mesh of the contact surface between Geometry M and
  Geometry N, r_MQ = (x,y,z) is the displacement vector from the origin of M's
  frame to Q expressed in the coordinate frame of M. We also have the
  _barycentric coordinates_ (b0, b1, b2) on a triangle of the surface mesh that
  contains Q. With vertices of the triangle labeled as v₀, v₁, v₂, we can
  map (b0, b1, b2) to r_MQ by:

               r_MQ = b0 * r_Mv₀ + b1 * r_Mv₁ + b2 * r_Mv₂,
               b0 + b1 + b2 = 1, bᵢ ∈ [0,1],

  where r_Mvᵢ is the displacement vector of the vertex labeled as vᵢ from the
  origin of M's frame, expressed in M's frame.

  We use the barycentric coordinates to evaluate the field values.

  <h2>Result invariance<h2>

  For two geometries, A and B, there is a mapping to M and N (as documented in
  this class). The mapping is based on the values of `id_A` and `id_B`. For
  fixed values of `id_A` and `id_B`, the mapping from (A, B) to (M, N) will
  _always_ be the same. (Although thre is no guarantee as to _what_ that mapping
  is.)
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
    mesh_M_ = std::make_unique<SurfaceMesh<T>>(*surface.mesh_M_);

    // We can't simply copy the mesh fields; the copies must contain pointers
    // to the new mesh. So, we use CloneAndSetMesh() instead.
    e_MN_.reset(static_cast<SurfaceMeshFieldLinear<T, T>*>(
        surface.e_MN_->CloneAndSetMesh(mesh_M_.get()).release()));
    grad_h_MN_M_.reset(static_cast<SurfaceMeshFieldLinear<Vector3<T>, T>*>(
        surface.grad_h_MN_M_->CloneAndSetMesh(mesh_M_.get()).release()));

    return *this;
  }

  ContactSurface(ContactSurface&&) = default;
  ContactSurface& operator=(ContactSurface&&) = default;

  // TODO(SeanCurtis-TRI): There are two quantities that care about frames:
  //  the mesh vertex positions and grad_h_MN. Is there *actual* value in
  //  defining them in one particular geometry's frame as opposed to some third
  //  frame (e.g., world frame)?
  /** Constructs a ContactSurface.
   @param id_M         The id of the first geometry M.
   @param id_N         The id of the second geometry N.
   @param mesh_M       The surface mesh of the contact surface 𝕊ₘₙ between M
                       and N. The mesh vertices are defined in frame M.
   @param e_MN         Represents the scalar field eₘₙ on the surface mesh.
   @param grad_h_MN_M  Represents the vector field ∇hₘₙ on the surface mesh,
                       expressed in M's frame. Due to discretization,
                       `grad_h_MN_M` at a vertex need not be strictly
                       orthogonal to every triangle sharing the vertex.
                       Orthogonality generally does improve with finer
                       discretization.
   @param X_MN         The pose of the frame of N in the frame of M.
   @note If the id_M is greater than the id_N, we will swap M and N.
         Therefore, the mesh_M and grad_h_MN_M will change their frames.
         Furthermore, the vector field grad_h_MN_M will switch its direction.
   */
  ContactSurface(
      GeometryId id_M, GeometryId id_N, std::unique_ptr<SurfaceMesh<T>> mesh_M,
      std::unique_ptr<SurfaceMeshFieldLinear<T, T>> e_MN,
      std::unique_ptr<SurfaceMeshFieldLinear<Vector3<T>, T>> grad_h_MN_M,
      const math::RigidTransform<T>& X_MN)
      : id_M_(id_M),
        id_N_(id_N),
        mesh_M_(std::move(mesh_M)),
        e_MN_(std::move(e_MN)),
        grad_h_MN_M_(std::move(grad_h_MN_M)) {
    if (id_N_ < id_M_) SwapMAndN(X_MN.inverse());
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
    @retval  grad_h_MN_M is the vector expressed in M's frame.
   */
  Vector3<T> EvaluateGrad_h_MN_M(
      SurfaceFaceIndex face,
      const typename SurfaceMesh<T>::Barycentric& barycentric) const {
    return grad_h_MN_M_->Evaluate(face, barycentric);
  }

  /** Evaluates the vector field ∇hₘₙ at the given vertex on the contact surface
    mesh.
    @param vertex       The index of the vertex in the mesh.
    @retval  grad_h_MN_M is the vector expressed in M's frame.
   */
  Vector3<T> EvaluateGrad_h_MN_M(SurfaceVertexIndex vertex) const {
    return grad_h_MN_M_->EvaluateAtVertex(vertex);
  }

  /** Returns a reference to the surface mesh.
   */
  const SurfaceMesh<T>& mesh() const {
    DRAKE_DEMAND(mesh_M_ != nullptr);
    return *mesh_M_;
  }

 private:
  // Swaps M and N (modifying the data in place to reflect the change in
  // frames).
  // @param X_NM  The pose of frame M in N.
  void SwapMAndN(const math::RigidTransform<T>& X_NM) {
    std::swap(id_M_, id_N_);
    mesh_M_->TransformVertices(X_NM);
    // TODO(SeanCurtis-TRI): Determine if this work is necessary. It is neither
    // documented nor tested that the face winding is guaranteed to be one way
    // or the other. Alternatively, this should be documented and tested.
    mesh_M_->ReverseFaceWinding();

    // TODO(SeanCurtis-TRI): We have no reasonable interface for transforming a
    //  mesh field. In this case, we want to re-express some vectors and then
    //  reverse them (i.e., reflect them around the origin). Generically, we
    //  should apply an arbitrary 3x3 matrix (which encompasses a rotation and a
    //  reflection). But that would *not* be a valid rotation matrix
    //  (handed-ness would be wrong).
    std::vector<Vector3<T>>& values = grad_h_MN_M_->mutable_values();
    for (SurfaceVertexIndex v(0); v < mesh_M_->num_vertices(); ++v) {
      const Vector3<T>& normal_MN_M = values[v];
      const Vector3<T> normal_MN_N = X_NM.rotation() * normal_MN_M;
      const Vector3<T> normal_NM_N = -normal_MN_N;
      values[v] = normal_NM_N;
    }

    // Note: the scalar field does not depend on frames.
  }

  // The id of the first geometry M.
  GeometryId id_M_;
  // The id of the second geometry N.
  GeometryId id_N_;
  // The surface mesh of the contact surface 𝕊ₘₙ between M and N.
  std::unique_ptr<SurfaceMesh<T>> mesh_M_;
  // TODO(SeanCurtis-TRI): We can only construct from a linear field, so store
  //  it as such for now. This can be promoted once there's a construction that
  //  uses a different derivation.
  // Represents the scalar field eₘₙ on the surface mesh.
  std::unique_ptr<SurfaceMeshFieldLinear<T, T>> e_MN_;
  // Represents the vector field ∇hₘₙ on the surface mesh, expressed in M's
  // frame.
  std::unique_ptr<SurfaceMeshFieldLinear<Vector3<T>, T>> grad_h_MN_M_;
  // TODO(DamrongGuoy): Remove this when we allow direct access to e_MN and
  //  grad_h_MN.
  template <typename U> friend class ContactSurfaceTester;
};

}  // namespace geometry
}  // namespace drake
