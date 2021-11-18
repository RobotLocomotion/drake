#pragma once

#include <memory>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/polygon_surface_mesh.h"
#include "drake/geometry/proximity/polygon_surface_mesh_field.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity/triangle_surface_mesh_field.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {

// TODO(SeanCurtis-TRI): It would be nice if this were *inside* `ContactSurface`
//  so it didn't have to be a scoped enumeration -- the class would handle the
//  scope. But, because ContactSurface is templated, I'd end up with different
//  enumerations that depend on scalar.  :-P  And going non-scoped seems a bad
//  idea with such simple designations of "kTriangle" and "kPolygon".
/** Reports on how a hydroelastic contact surface is represented. See
 @ref contact_surface_discrete_representation
 "the documentation in ContactSurface" for more details. */
enum class HydroelasticContactRepresentation { kTriangle, kPolygon };

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

  @anchor contact_surface_discrete_representation
  <h2> Discrete Representation </h2>

  In practice, hydroelastic geometries themselves have a discrete
  representation: either a triangular surface mesh for rigid geometries or a
  tetrahedral volume mesh for compliant geometry. This discretization leads to
  contact surfaces that are likewise discrete.

  Intersection between triangles and tetrahedra (or tetrahedra and tetrahedra)
  can produce polygons with up to eight sides. A %ContactSurface can represent
  the resulting surface as a mesh of such polygons, or as a mesh of tesselated
  triangles. The domains of the two representations are identical. The
  triangular version admits for simple, high-order integration over the domain.
  Every element is a triangle, and triangles will only disappear and reappear
  as their areas go to zero. However, this increases the total number of
  faces in the mesh by more than a factor of three over the polygonal mesh. The
  polygonal representation produces fewer faces, but high order integration over
  polygons is problematic. We recommend choosing the cheapest representation
  that nevertheless supports your required fidelity (see
  QueryObject::ComputeContactSurfaces()).

  The representation of any %ContactSurface instance can be reported by calling
  representation(). If it returns HydroelasticContactRepresentation::kTriangle,
  then the mesh and pressure field can be accessed via tri_mesh_W() and
  tri_e_MN(), respectively. If it returns
  HydroelasticContactRepresentation::kPolygon, then use poly_mesh_W() and
  poly_e_MN().

  Regardless of representation (polygon or triangle), the normal for each
  mesh face is guaranteed to point "out of" N and "into" M. They can be accessed
  via the mesh, e.g., `tri_mesh_W().face_normal(face_index)`. By definition,
  the normals of the mesh are discontinuous at triangle boundaries.

  The pressure values on the contact surface are represented as a continuous,
  piecewise-linear function, accessed via tri_e_MN() or poly_e_MN().

  When available, the values of ∇eₘ and ∇eₙ are represented as a discontinuous,
  piecewise-constant function over the faces -- one gradient vector per face.
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
  ContactSurface(const ContactSurface& surface) { *this = surface; }

  // TODO(SeanCurtis-TRI) Copy assignment, both constructors, and SwapMAndN
  //  would all be better defined in the .cc file. The primary reason they are
  //  not is that HydroelasticContactInfo and ContactResultsToLcm unit tests
  //  (which explicitly declare support for T = symbolic::Expression), blindly
  //  assume that the contact surface likewise supports symbolic::Expression
  //  (even though that is not the case). Their only meaningful actions is to
  //  copy/create instances. By leaving these functions in the header, those
  //  workflows continue to work. Ideally, they'd protect themselves against the
  //  fact that they are intsantiating types that don't *truly* support
  //  Expression and these functions can move into the .cc file.
  //  This has a downstream effect of requiring the surface meshes
  //  ReverseFaceWinding methods defined in the header as it gets invoked by
  //  SwapMAndN.

  ContactSurface& operator=(const ContactSurface& surface) {
    if (&surface == this) return *this;

    id_M_ = surface.id_M_;
    id_N_ = surface.id_N_;
    if (surface.is_triangle()) {
      mesh_W_ = std::make_unique<TriangleSurfaceMesh<T>>(surface.tri_mesh_W());
      // We can't simply copy the mesh fields; the copies must contain pointers
      // to the new mesh. So, we use CloneAndSetMesh() instead.
      // Set the *variant* to be the right type, regardless of what it may have
      // been before.
      e_MN_ = std::unique_ptr<TriangleSurfaceMeshFieldLinear<T, T>>();
      auto& e_MN =
          std::get<std::unique_ptr<TriangleSurfaceMeshFieldLinear<T, T>>>(
              e_MN_);
      e_MN.reset(static_cast<TriangleSurfaceMeshFieldLinear<T, T>*>(
          surface.tri_e_MN().CloneAndSetMesh(&tri_mesh_W()).release()));
    } else {
      mesh_W_ = std::make_unique<PolygonSurfaceMesh<T>>(surface.poly_mesh_W());
      // We can't simply copy the mesh fields; the copies must contain pointers
      // to the new mesh. So, we use CloneAndSetMesh() instead.
      // Set the *variant* to be the right type, regardless of what it may have
      // been before.
      e_MN_ = std::unique_ptr<PolygonSurfaceMeshFieldLinear<T, T>>();
      auto& e_MN =
          std::get<std::unique_ptr<PolygonSurfaceMeshFieldLinear<T, T>>>(e_MN_);
      e_MN.reset(static_cast<PolygonSurfaceMeshFieldLinear<T, T>*>(
          surface.poly_e_MN().CloneAndSetMesh(&poly_mesh_W()).release()));
    }

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

  /** @name Constructors

   The %ContactSurface can be constructed with either a polygon or triangle
   mesh representation. The constructor invoked determines the representation.

   The general shape of each constructor is identical. They take the unique
   identifiers for the two geometries in contact, a mesh representation, a
   field representation, and (optional) gradients of the contacting geometryies'
   pressure fields.

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
         reveal if such a swap has occurred. */
  //@{

  /** Constructs a %ContactSurface with a triangle mesh representation. */
  ContactSurface(GeometryId id_M, GeometryId id_N,
                 std::unique_ptr<TriangleSurfaceMesh<T>> mesh_W,
                 std::unique_ptr<TriangleSurfaceMeshFieldLinear<T, T>> e_MN,
                 std::unique_ptr<std::vector<Vector3<T>>> grad_eM_W = nullptr,
                 std::unique_ptr<std::vector<Vector3<T>>> grad_eN_W = nullptr)
    : ContactSurface(id_M, id_N, std::move(mesh_W), std::move(e_MN),
                     std::move(grad_eM_W), std::move(grad_eN_W), 1) {}

  /** Constructs a %ContactSurface with a polygonal mesh representation. */
  ContactSurface(GeometryId id_M, GeometryId id_N,
                 std::unique_ptr<PolygonSurfaceMesh<T>> mesh_W,
                 std::unique_ptr<PolygonSurfaceMeshFieldLinear<T, T>> e_MN,
                 std::unique_ptr<std::vector<Vector3<T>>> grad_eM_W = nullptr,
                 std::unique_ptr<std::vector<Vector3<T>>> grad_eN_W = nullptr)
    : ContactSurface(id_M, id_N, std::move(mesh_W), std::move(e_MN),
                     std::move(grad_eM_W), std::move(grad_eN_W), 1) {}

  //@}

  /** Returns the geometry id of Geometry M. */
  GeometryId id_M() const { return id_M_; }

  /** Returns the geometry id of Geometry N. */
  GeometryId id_N() const { return id_N_; }

  /** @name Representation-independent API

   These methods represent sugar which masks the details of the mesh
   representation. They facilitate querying various mesh quantities that are
   common to the two representations, so that code can access the properties
   without worrying about the representation. If necessary, the actual meshes
   and fields can be accessed directly via the representation-dependent APIs
   below. */
  //@{

  int num_faces() const {
    return is_triangle() ? tri_mesh_W().num_elements()
                         : poly_mesh_W().num_elements();
  }

  int num_vertices() const {
    return is_triangle() ? tri_mesh_W().num_vertices()
                         : poly_mesh_W().num_vertices();
  }

  const T& area(int face_index) const {
    return is_triangle() ? tri_mesh_W().area(face_index)
                         : poly_mesh_W().area(face_index);
  }

  const T& total_area() const {
    return is_triangle() ? tri_mesh_W().total_area()
                         : poly_mesh_W().total_area();
  }

  const Vector3<T>& face_normal(int face_index) const {
    return is_triangle() ? tri_mesh_W().face_normal(face_index)
                         : poly_mesh_W().face_normal(face_index);
  }

  // TODO(SeanCurtis-TRI): If TriangleSurfaceMesh pre-computed centroids and
  //  stored them, this could return a const reference. It's not clear, however,
  //  that this would get invoked enough to matter.
  Vector3<T> centroid(int face_index) const {
    return is_triangle() ? tri_mesh_W().element_centroid(face_index)
                         : poly_mesh_W().element_centroid(face_index);
  }

  const Vector3<T>& centroid() const {
    return is_triangle() ? tri_mesh_W().centroid() : poly_mesh_W().centroid();
  }

  //@}

  /** @name Representation-dependent API

   These functions provide insight into what representation the %ContactSurface
   instance uses, and provide access to the representation-dependent quantities:
   mesh and field. */
  //@{

  /** Simpley reports if this contact surface's mesh representation is triangle.
   Equivalent to:

       representation() == HydroelasticContactRepresentation::kTriangle

   and offered as convenient sugar. */
  bool is_triangle() const {
    return std::holds_alternative<std::unique_ptr<TriangleSurfaceMesh<T>>>(
        mesh_W_);
  }

  /** Reports the representation mode of this contact surface. If accessing the
   mesh or field directly, the APIs that can be successfully exercised are
   related to this methods return value. See below. */
  HydroelasticContactRepresentation representation() const {
    return is_triangle() ? HydroelasticContactRepresentation::kTriangle
                         : HydroelasticContactRepresentation::kPolygon;
  }

  /** Returns a reference to the surface mesh whose vertex positions are
   measured and expressed in the world frame.

   This method is provided for short-term legacy reasons. Once the polygonal
   mesh representation is fully supported across the entire contact model
   code path, this will be removed in favor of its explicitly declared
   `tri_` or `poly_` variant.

   @pre is_triangle() returns `true`. */
  const TriangleSurfaceMesh<T>& mesh_W() const {
    return tri_mesh_W();
  }

  /** Returns a reference to the scalar field eₘₙ.

   This method is provided for short-term legacy reasons. Once the polygonal
   mesh representation is fully supported across the entire contact model
   code path, this will be removed in favor of its explicitly declared
   `tri_` or `poly_` variant.

   @pre is_triangle() returns `true`. */
  const TriangleSurfaceMeshFieldLinear<T, T>& e_MN() const {
    return tri_e_MN();
  }

  /** Returns a reference to the _triangular_ surface mesh whose vertex
   positions are measured and expressed in the world frame. */
  const TriangleSurfaceMesh<T>& tri_mesh_W() const {
    DRAKE_DEMAND(is_triangle());
    return *std::get<std::unique_ptr<TriangleSurfaceMesh<T>>>(mesh_W_);
  }

  /** Returns a reference to the scalar field eₘₙ for the _triangle_ mesh. */
  const TriangleSurfaceMeshFieldLinear<T, T>& tri_e_MN() const {
    DRAKE_DEMAND(is_triangle());
    return *std::get<std::unique_ptr<TriangleSurfaceMeshFieldLinear<T, T>>>(
        e_MN_);
  }

  /** Returns a reference to the _polygonal_ surface mesh whose vertex
   positions are measured and expressed in the world frame. */
  const PolygonSurfaceMesh<T>& poly_mesh_W() const {
    DRAKE_DEMAND(!is_triangle());
    return *std::get<std::unique_ptr<PolygonSurfaceMesh<T>>>(mesh_W_);
  }

  /** Returns a reference to the scalar field eₘₙ for the _polygonal_ mesh. */
  const PolygonSurfaceMeshFieldLinear<T, T>& poly_e_MN() const {
    DRAKE_DEMAND(!is_triangle());
    return *std::get<std::unique_ptr<PolygonSurfaceMeshFieldLinear<T, T>>>(
        e_MN_);
  }

  //@}

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
   can only be evaluate on a per-face basis.  */
  //@{

  /** @returns `true` if `this` contains values for ∇eₘ.  */
  bool HasGradE_M() const { return grad_eM_W_ != nullptr; }

  /** @returns `true` if `this` contains values for ∇eₙ.  */
  bool HasGradE_N() const { return grad_eN_W_ != nullptr; }

  /** Returns the value of ∇eₘ for the face with index `index`.
   @throws std::exception if HasGradE_M() returns false.
   @pre `index ∈ [0, mesh().num_faces())`.  */
  const Vector3<T>& EvaluateGradE_M_W(int index) const;

  /** Returns the value of ∇eₙ for the face with index `index`.
   @throws std::exception if HasGradE_N() returns false.
   @pre `index ∈ [0, mesh().num_faces())`.  */
  const Vector3<T>& EvaluateGradE_N_W(int index) const;

  //@}

  // TODO(#12173): Consider NaN==NaN to be true in equality tests.
  /** Checks to see whether the given ContactSurface object is equal via deep
   exact comparison. NaNs are treated as not equal as per the IEEE standard.

   @param surface The contact surface for comparison.
   @returns `true` if the given contact surface is equal.
   */
  bool Equal(const ContactSurface<T>& surface) const;

 private:
  using MeshVariant = std::variant<std::unique_ptr<TriangleSurfaceMesh<T>>,
                                   std::unique_ptr<PolygonSurfaceMesh<T>>>;
  using FieldVariant =
      std::variant<std::unique_ptr<TriangleSurfaceMeshFieldLinear<T, T>>,
                   std::unique_ptr<PolygonSurfaceMeshFieldLinear<T, T>>>;

  // Main delegation constructor. The extra int parameter is to introduce a
  // disambiguation mechanism.
  ContactSurface(GeometryId id_M, GeometryId id_N, MeshVariant mesh_W,
                 FieldVariant e_MN,
                 std::unique_ptr<std::vector<Vector3<T>>> grad_eM_W,
                 std::unique_ptr<std::vector<Vector3<T>>> grad_eN_W, int)
      : id_M_(id_M),
        id_N_(id_N),
        mesh_W_(move(mesh_W)),
        e_MN_(move(e_MN)),
        grad_eM_W_(move(grad_eM_W)),
        grad_eN_W_(move(grad_eN_W)) {
    // If defined the gradient values must map 1-to-1 onto elements.
    if (is_triangle()) {
      DRAKE_THROW_UNLESS(grad_eM_W_ == nullptr ||
                         static_cast<int>(grad_eM_W_->size()) ==
                             tri_mesh_W().num_elements());
      DRAKE_THROW_UNLESS(grad_eN_W_ == nullptr ||
                         static_cast<int>(grad_eN_W_->size()) ==
                             tri_mesh_W().num_elements());
    } else {
      DRAKE_THROW_UNLESS(grad_eM_W_ == nullptr ||
                         static_cast<int>(grad_eM_W_->size()) ==
                             poly_mesh_W().num_elements());
      DRAKE_THROW_UNLESS(grad_eN_W_ == nullptr ||
                         static_cast<int>(grad_eN_W_->size()) ==
                             poly_mesh_W().num_elements());
    }
    if (id_N_ < id_M_) SwapMAndN();
  }

  // Swaps M and N (modifying the data in place to reflect the change).
  void SwapMAndN() {
    std::swap(id_M_, id_N_);
    // TODO(SeanCurtis-TRI): Determine if this work is necessary. It is neither
    // documented nor tested that the face winding is guaranteed to be one way
    // or the other. Alternatively, this should be documented and tested.
    std::visit([](auto&& mesh) { mesh->ReverseFaceWinding(); }, mesh_W_);

    // Note: the scalar field does not depend on the order of M and N.
    std::swap(grad_eM_W_, grad_eN_W_);
  }

  // The id of the first geometry M.
  GeometryId id_M_;
  // The id of the second geometry N.
  GeometryId id_N_;

  // The surface mesh of the contact surface 𝕊ₘₙ between M and N.
  MeshVariant mesh_W_;

  // Represents the scalar field eₘₙ on the surface mesh.
  FieldVariant e_MN_;

  // The gradients of the pressure fields eₘ and eₙ sampled on the contact
  // surface. There is one gradient value *per contact surface triangle*.
  // These quantities may not be defined if the gradient is not well-defined.
  // See class documentation for elaboration.
  std::unique_ptr<std::vector<Vector3<T>>> grad_eM_W_;
  std::unique_ptr<std::vector<Vector3<T>>> grad_eN_W_;

  template <typename U> friend class ContactSurfaceTester;
};

}  // namespace geometry
}  // namespace drake
