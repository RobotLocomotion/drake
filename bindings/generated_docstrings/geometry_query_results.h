#pragma once

// GENERATED FILE DO NOT EDIT
// This file contains docstrings for the Python bindings that were
// automatically extracted by mkdoc.py.

#include <array>
#include <utility>

#if defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

// #include "drake/geometry/query_results/contact_surface.h"
// #include "drake/geometry/query_results/deformable_contact.h"
// #include "drake/geometry/query_results/penetration_as_point_pair.h"
// #include "drake/geometry/query_results/signed_distance_pair.h"
// #include "drake/geometry/query_results/signed_distance_to_point.h"

// Symbol: pydrake_doc_geometry_query_results
constexpr struct /* pydrake_doc_geometry_query_results */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::geometry
    struct /* geometry */ {
      // Symbol: drake::geometry::ContactSurface
      struct /* ContactSurface */ {
        // Source: drake/geometry/query_results/contact_surface.h
        const char* doc =
R"""(The ContactSurface characterizes the intersection of two geometries M
and N as a contact surface with a scalar field and a vector field,
whose purpose is to support the hydroelastic pressure field contact
model as described in:

R. Elandt, E. Drumwright, M. Sherman, and A. Ruina. A pressure field
model for fast, robust approximation of net contact force and moment
between nominally rigid objects. IROS 2019: 8238-8245.

Mathematical Concepts -----------------------

In this section, we give motivation for the concept of contact surface
from the hydroelastic pressure field contact model. Here the
mathematical discussion is coordinate-free (treatment of the topic
without reference to any particular coordinate system); however, our
implementation heavily relies on coordinate frames. We borrow
terminology from differential geometry.

In this section, the mathematical term *compact set* (a subset of
Euclidean space that is closed and bounded) corresponds to the term
*geometry* (or the space occupied by the geometry) in SceneGraph.

We describe the contact surface 𝕊ₘₙ between two intersecting compact
subsets 𝕄 and ℕ of ℝ³ with the scalar fields eₘ and eₙ defined on 𝕄 ⊂
ℝ³ and ℕ ⊂ ℝ³ respectively:

eₘ : 𝕄 → ℝ, eₙ : ℕ → ℝ.

The *contact surface* 𝕊ₘₙ is the surface of equilibrium eₘ = eₙ. It is
the locus of points Q where eₘ(Q) equals eₙ(Q):

𝕊ₘₙ = { Q ∈ 𝕄 ∩ ℕ : eₘ(Q) = eₙ(Q) }.

We can define the scalar field eₘₙ on the surface 𝕊ₘₙ as a scalar
function that assigns Q ∈ 𝕊ₘₙ the value of eₘ(Q), which is the same as
eₙ(Q):

eₘₙ : 𝕊ₘₙ → ℝ, eₘₙ(Q) = eₘ(Q) = eₙ(Q).

We can also define the scalar field hₘₙ on 𝕄 ∩ ℕ as the difference
between eₘ and eₙ:

hₘₙ : 𝕄 ∩ ℕ → ℝ, hₘₙ(Q) = eₘ(Q) - eₙ(Q).

It follows that the gradient vector field ∇hₘₙ on 𝕄 ∩ ℕ equals the
difference between the gradient vector fields ∇eₘ and ∇eₙ:

∇hₘₙ : 𝕄 ∩ ℕ → ℝ³, ∇hₘₙ(Q) = ∇eₘ(Q) - ∇eₙ(Q).

By construction, Q ∈ 𝕊ₘₙ if and only if hₘₙ(Q) = 0. In other words,
𝕊ₘₙ is the zero level set of hₘₙ. It follows that, for Q ∈ 𝕊ₘₙ,
∇hₘₙ(Q) is orthogonal to the surface 𝕊ₘₙ at Q in the direction of
increasing eₘ - eₙ.

Notice that the domain of eₘₙ is the two-dimensional surface 𝕊ₘₙ,
while the domain of ∇hₘₙ is the three-dimensional compact set 𝕄 ∩ ℕ.
Even though eₘₙ and ∇hₘₙ are defined on different domains (𝕊ₘₙ and 𝕄 ∩
ℕ), our implementation only represents them on their common domain,
i.e., 𝕊ₘₙ.

Discrete Representation -------------------------

In practice, hydroelastic geometries themselves have a discrete
representation: either a triangular surface mesh for rigid geometries
or a tetrahedral volume mesh for compliant geometry. This
discretization leads to contact surfaces that are likewise discrete.

Intersection between triangles and tetrahedra (or tetrahedra and
tetrahedra) can produce polygons with up to eight sides. A
ContactSurface can represent the resulting surface as a mesh of such
polygons, or as a mesh of tessellated triangles. The domains of the
two representations are identical. The triangular version admits for
simple, high-order integration over the domain. Every element is a
triangle, and triangles will only disappear and reappear as their
areas go to zero. However, this increases the total number of faces in
the mesh by more than a factor of three over the polygonal mesh. The
polygonal representation produces fewer faces, but high order
integration over polygons is problematic. We recommend choosing the
cheapest representation that nevertheless supports your required
fidelity (see QueryObject∷ComputeContactSurfaces()).

The representation of any ContactSurface instance can be reported by
calling representation(). If it returns
HydroelasticContactRepresentation∷kTriangle, then the mesh and
pressure field can be accessed via tri_mesh_W() and tri_e_MN(),
respectively. If it returns
HydroelasticContactRepresentation∷kPolygon, then use poly_mesh_W() and
poly_e_MN().

Regardless of representation (polygon or triangle), the normal for
each mesh face is guaranteed to point "out of" N and "into" M. They
can be accessed via the mesh, e.g.,
``tri_mesh_W().face_normal(face_index)``. By definition, the normals
of the mesh are discontinuous at triangle boundaries.

The pressure values on the contact surface are represented as a
continuous, piecewise-linear function, accessed via tri_e_MN() or
poly_e_MN().

When available, the values of ∇eₘ and ∇eₙ are represented as a
discontinuous, piecewise-constant function over the faces -- one
gradient vector per face. These quantities are accessed via
EvaluateGradE_M_W() and EvaluateGradE_N_W(), respectively.

Barycentric Coordinates -------------------------

For Point Q on the surface mesh of the contact surface between
Geometry M and Geometry N, r_WQ = (x,y,z) is the displacement vector
from the origin of the world frame to Q expressed in the coordinate
frame of W. We also have the *barycentric coordinates* (b0, b1, b2) on
a triangle of the surface mesh that contains Q. With vertices of the
triangle labeled as v₀, v₁, v₂, we can map (b0, b1, b2) to r_WQ by:

r_WQ = b0 * r_Wv₀ + b1 * r_Wv₁ + b2 * r_Wv₂, b0 + b1 + b2 = 1, bᵢ ∈
[0,1],

where r_Wvᵢ is the displacement vector of the vertex labeled as vᵢ
from the origin of the world frame, expressed in the world frame.

We use the barycentric coordinates to evaluate the field values.)""";
        // Symbol: drake::geometry::ContactSurface::ContactSurface<T>
        struct /* ctor */ {
          // Source: drake/geometry/query_results/contact_surface.h
          const char* doc_was_unable_to_choose_unambiguous_names = R"""()""";
        } ctor;
        // Symbol: drake::geometry::ContactSurface::Equal
        struct /* Equal */ {
          // Source: drake/geometry/query_results/contact_surface.h
          const char* doc =
R"""(Checks to see whether the given ContactSurface object is equal via
deep exact comparison. NaNs are treated as not equal as per the IEEE
standard.

Parameter ``surface``:
    The contact surface for comparison.

Returns:
    ``True`` if the given contact surface is equal.)""";
        } Equal;
        // Symbol: drake::geometry::ContactSurface::EvaluateGradE_M_W
        struct /* EvaluateGradE_M_W */ {
          // Source: drake/geometry/query_results/contact_surface.h
          const char* doc =
R"""(Returns the value of ∇eₘ for the face with index ``index``.

Raises:
    RuntimeError if HasGradE_M() returns false.

Precondition:
    ``index ∈ [0, mesh().num_faces())``.)""";
        } EvaluateGradE_M_W;
        // Symbol: drake::geometry::ContactSurface::EvaluateGradE_N_W
        struct /* EvaluateGradE_N_W */ {
          // Source: drake/geometry/query_results/contact_surface.h
          const char* doc =
R"""(Returns the value of ∇eₙ for the face with index ``index``.

Raises:
    RuntimeError if HasGradE_N() returns false.

Precondition:
    ``index ∈ [0, mesh().num_faces())``.)""";
        } EvaluateGradE_N_W;
        // Symbol: drake::geometry::ContactSurface::HasGradE_M
        struct /* HasGradE_M */ {
          // Source: drake/geometry/query_results/contact_surface.h
          const char* doc =
R"""(Returns:
    ``True`` if ``this`` contains values for ∇eₘ.)""";
        } HasGradE_M;
        // Symbol: drake::geometry::ContactSurface::HasGradE_N
        struct /* HasGradE_N */ {
          // Source: drake/geometry/query_results/contact_surface.h
          const char* doc =
R"""(Returns:
    ``True`` if ``this`` contains values for ∇eₙ.)""";
        } HasGradE_N;
        // Symbol: drake::geometry::ContactSurface::area
        struct /* area */ {
          // Source: drake/geometry/query_results/contact_surface.h
          const char* doc = R"""()""";
        } area;
        // Symbol: drake::geometry::ContactSurface::centroid
        struct /* centroid */ {
          // Source: drake/geometry/query_results/contact_surface.h
          const char* doc = R"""()""";
        } centroid;
        // Symbol: drake::geometry::ContactSurface::face_normal
        struct /* face_normal */ {
          // Source: drake/geometry/query_results/contact_surface.h
          const char* doc = R"""()""";
        } face_normal;
        // Symbol: drake::geometry::ContactSurface::id_M
        struct /* id_M */ {
          // Source: drake/geometry/query_results/contact_surface.h
          const char* doc = R"""(Returns the geometry id of Geometry M.)""";
        } id_M;
        // Symbol: drake::geometry::ContactSurface::id_N
        struct /* id_N */ {
          // Source: drake/geometry/query_results/contact_surface.h
          const char* doc = R"""(Returns the geometry id of Geometry N.)""";
        } id_N;
        // Symbol: drake::geometry::ContactSurface::is_triangle
        struct /* is_triangle */ {
          // Source: drake/geometry/query_results/contact_surface.h
          const char* doc =
R"""(Simply reports if this contact surface's mesh representation is
triangle. Equivalent to:

representation() == HydroelasticContactRepresentation∷kTriangle

and offered as convenient sugar.)""";
        } is_triangle;
        // Symbol: drake::geometry::ContactSurface::num_faces
        struct /* num_faces */ {
          // Source: drake/geometry/query_results/contact_surface.h
          const char* doc = R"""()""";
        } num_faces;
        // Symbol: drake::geometry::ContactSurface::num_vertices
        struct /* num_vertices */ {
          // Source: drake/geometry/query_results/contact_surface.h
          const char* doc = R"""()""";
        } num_vertices;
        // Symbol: drake::geometry::ContactSurface::poly_e_MN
        struct /* poly_e_MN */ {
          // Source: drake/geometry/query_results/contact_surface.h
          const char* doc =
R"""(Returns a reference to the scalar field eₘₙ for the *polygonal* mesh.

Precondition:
    ``is_triangle()`` returns ``False``.)""";
        } poly_e_MN;
        // Symbol: drake::geometry::ContactSurface::poly_mesh_W
        struct /* poly_mesh_W */ {
          // Source: drake/geometry/query_results/contact_surface.h
          const char* doc =
R"""(Returns a reference to the *polygonal* surface mesh whose vertex
positions are measured and expressed in the world frame.

Precondition:
    ``is_triangle()`` returns ``False``.)""";
        } poly_mesh_W;
        // Symbol: drake::geometry::ContactSurface::representation
        struct /* representation */ {
          // Source: drake/geometry/query_results/contact_surface.h
          const char* doc =
R"""(Reports the representation mode of this contact surface. If accessing
the mesh or field directly, the APIs that can be successfully
exercised are related to this methods return value. See below.)""";
        } representation;
        // Symbol: drake::geometry::ContactSurface::total_area
        struct /* total_area */ {
          // Source: drake/geometry/query_results/contact_surface.h
          const char* doc = R"""()""";
        } total_area;
        // Symbol: drake::geometry::ContactSurface::tri_e_MN
        struct /* tri_e_MN */ {
          // Source: drake/geometry/query_results/contact_surface.h
          const char* doc =
R"""(Returns a reference to the scalar field eₘₙ for the *triangle* mesh.

Precondition:
    ``is_triangle()`` returns ``True``.)""";
        } tri_e_MN;
        // Symbol: drake::geometry::ContactSurface::tri_mesh_W
        struct /* tri_mesh_W */ {
          // Source: drake/geometry/query_results/contact_surface.h
          const char* doc =
R"""(Returns a reference to the *triangular* surface mesh whose vertex
positions are measured and expressed in the world frame.

Precondition:
    ``is_triangle()`` returns ``True``.)""";
        } tri_mesh_W;
      } ContactSurface;
      // Symbol: drake::geometry::HydroelasticContactRepresentation
      struct /* HydroelasticContactRepresentation */ {
        // Source: drake/geometry/query_results/contact_surface.h
        const char* doc =
R"""(Reports on how a hydroelastic contact surface is represented. See
contact_surface_discrete_representation "the documentation in
ContactSurface" for more details.)""";
        // Symbol: drake::geometry::HydroelasticContactRepresentation::kPolygon
        struct /* kPolygon */ {
          // Source: drake/geometry/query_results/contact_surface.h
          const char* doc = R"""()""";
        } kPolygon;
        // Symbol: drake::geometry::HydroelasticContactRepresentation::kTriangle
        struct /* kTriangle */ {
          // Source: drake/geometry/query_results/contact_surface.h
          const char* doc = R"""()""";
        } kTriangle;
      } HydroelasticContactRepresentation;
      // Symbol: drake::geometry::PenetrationAsPointPair
      struct /* PenetrationAsPointPair */ {
        // Source: drake/geometry/query_results/penetration_as_point_pair.h
        const char* doc =
R"""(A characterization of the intersection of two penetrating geometries.
The characterization consists of a pair of points and a normal. The
points represent a point on each geometry that most deeply penetrates
the other geometry (in the normal direction). For convenience, the
penetration depth is provided and is equal to:

depth = ``(p_WCb - p_WCa) ⋅ nhat_BA_W``

(`depth` is strictly positive when there is penetration and otherwise
not defined.)

Template parameter ``T``:
    The underlying scalar type. Must be a valid Eigen scalar.)""";
        // Symbol: drake::geometry::PenetrationAsPointPair::SwapAAndB
        struct /* SwapAAndB */ {
          // Source: drake/geometry/query_results/penetration_as_point_pair.h
          const char* doc =
R"""(Swaps the interpretation of geometries A and B.)""";
        } SwapAAndB;
        // Symbol: drake::geometry::PenetrationAsPointPair::depth
        struct /* depth */ {
          // Source: drake/geometry/query_results/penetration_as_point_pair.h
          const char* doc = R"""(The penetration depth.)""";
        } depth;
        // Symbol: drake::geometry::PenetrationAsPointPair::id_A
        struct /* id_A */ {
          // Source: drake/geometry/query_results/penetration_as_point_pair.h
          const char* doc =
R"""(The id of the first geometry in the contact.)""";
        } id_A;
        // Symbol: drake::geometry::PenetrationAsPointPair::id_B
        struct /* id_B */ {
          // Source: drake/geometry/query_results/penetration_as_point_pair.h
          const char* doc =
R"""(The id of the second geometry in the contact.)""";
        } id_B;
        // Symbol: drake::geometry::PenetrationAsPointPair::nhat_BA_W
        struct /* nhat_BA_W */ {
          // Source: drake/geometry/query_results/penetration_as_point_pair.h
          const char* doc =
R"""(The unit-length normal which defines the penetration direction,
pointing from geometry B into geometry A, measured and expressed in
the world frame. It *approximates* the normal to the plane on which
the contact patch lies.)""";
        } nhat_BA_W;
        // Symbol: drake::geometry::PenetrationAsPointPair::p_WCa
        struct /* p_WCa */ {
          // Source: drake/geometry/query_results/penetration_as_point_pair.h
          const char* doc =
R"""(The point on A that most deeply penetrates B, measured and expressed
in the world frame.)""";
        } p_WCa;
        // Symbol: drake::geometry::PenetrationAsPointPair::p_WCb
        struct /* p_WCb */ {
          // Source: drake/geometry/query_results/penetration_as_point_pair.h
          const char* doc =
R"""(The point on B that most deeply penetrates A, measured and expressed
in the world frame.)""";
        } p_WCb;
      } PenetrationAsPointPair;
      // Symbol: drake::geometry::SignedDistancePair
      struct /* SignedDistancePair */ {
        // Source: drake/geometry/query_results/signed_distance_pair.h
        const char* doc =
R"""(The data for reporting the signed distance between two geometries, A
and B. It provides the id's of the two geometries, the witness points
Ca and Cb on the surfaces of A and B, the signed distance, and
nhat_BA_W a direction of fastest increasing distance (always unit
length and always point outward from B's surface).

- When A and B are separated, distance > 0.
- When A and B are touching or penetrating, distance <= 0.
- By definition, nhat_AB_W must be in the opposite direction of nhat_BA_W.
- (p_WCa - p_Wcb) = distance · nhat_BA_W.

Warning:
    For two geometries that are just touching (i.e., distance = 0),
    the underlying code can guarantee a correct value for nhat_BA_W
    only when one geometry is a sphere, and the other geometry is a
    sphere, a box, or a cylinder. Otherwise, the underlying code is
    not in place yet to guarantee a correct value for nhat_BA_W when
    surfaces are just touching, and the vector will be populated by
    NaN values.

Template parameter ``T``:
    The underlying scalar type. Must be a valid Eigen scalar.)""";
        // Symbol: drake::geometry::SignedDistancePair::SignedDistancePair<T>
        struct /* ctor */ {
          // Source: drake/geometry/query_results/signed_distance_pair.h
          const char* doc =
R"""(Constructor

Parameter ``a``:
    The id of the first geometry (A).

Parameter ``b``:
    The id of the second geometry (B).

Parameter ``p_ACa_in``:
    The witness point on geometry A's surface, in A's frame.

Parameter ``p_BCb_in``:
    The witness point on geometry B's surface, in B's frame.

Parameter ``dist``:
    The signed distance between p_A and p_B.

Parameter ``nhat_BA_W_in``:
    A direction of fastest increasing distance.

Precondition:
    nhat_BA_W_in is unit-length.)""";
        } ctor;
        // Symbol: drake::geometry::SignedDistancePair::SwapAAndB
        struct /* SwapAAndB */ {
          // Source: drake/geometry/query_results/signed_distance_pair.h
          const char* doc =
R"""(Swaps the interpretation of geometries A and B.)""";
        } SwapAAndB;
        // Symbol: drake::geometry::SignedDistancePair::distance
        struct /* distance */ {
          // Source: drake/geometry/query_results/signed_distance_pair.h
          const char* doc =
R"""(The signed distance between p_ACa and p_BCb.)""";
        } distance;
        // Symbol: drake::geometry::SignedDistancePair::id_A
        struct /* id_A */ {
          // Source: drake/geometry/query_results/signed_distance_pair.h
          const char* doc =
R"""(The id of the first geometry in the pair.)""";
        } id_A;
        // Symbol: drake::geometry::SignedDistancePair::id_B
        struct /* id_B */ {
          // Source: drake/geometry/query_results/signed_distance_pair.h
          const char* doc =
R"""(The id of the second geometry in the pair.)""";
        } id_B;
        // Symbol: drake::geometry::SignedDistancePair::nhat_BA_W
        struct /* nhat_BA_W */ {
          // Source: drake/geometry/query_results/signed_distance_pair.h
          const char* doc =
R"""(A direction of fastest increasing distance.)""";
        } nhat_BA_W;
        // Symbol: drake::geometry::SignedDistancePair::p_ACa
        struct /* p_ACa */ {
          // Source: drake/geometry/query_results/signed_distance_pair.h
          const char* doc =
R"""(The witness point on geometry A's surface, expressed in A's frame.)""";
        } p_ACa;
        // Symbol: drake::geometry::SignedDistancePair::p_BCb
        struct /* p_BCb */ {
          // Source: drake/geometry/query_results/signed_distance_pair.h
          const char* doc =
R"""(The witness point on geometry B's surface, expressed in B's frame.)""";
        } p_BCb;
      } SignedDistancePair;
      // Symbol: drake::geometry::SignedDistanceToPoint
      struct /* SignedDistanceToPoint */ {
        // Source: drake/geometry/query_results/signed_distance_to_point.h
        const char* doc =
R"""(The data for reporting the signed distance from a query point to a
geometry. Reports the result of a signed distance query between a
query point Q and geometry G. This includes G's id, the signed
distance, the nearest point N on the surface of G, and the gradient of
the signed distance with respect to the position of Q. Generally, the
gradient of the signed distance function is not defined everywhere.
The value reported in this struct depends on the query function
returning it. Refer to the query function's documentation for what
value it will report for otherwise undefined gradient values.

Template parameter ``T``:
    The underlying scalar type. Must be a valid Eigen scalar.)""";
        // Symbol: drake::geometry::SignedDistanceToPoint::SignedDistanceToPoint<T>
        struct /* ctor */ {
          // Source: drake/geometry/query_results/signed_distance_to_point.h
          const char* doc =
R"""(Constructs SignedDistanceToPoint struct from calculated results.

Parameter ``id_G_in``:
    The id of the geometry G to which we measure distance from the
    query point Q.

Parameter ``p_GN_in``:
    The position of the nearest point N on G's surface from the query
    point Q, expressed in G's frame.

Parameter ``distance_in``:
    The signed distance from the query point Q to the nearest point N
    on the surface of geometry G. It is positive if Q is outside G. It
    is negative if Q is inside G. It is zero if Q is on the boundary
    of G.

Parameter ``grad_W_in``:
    The gradient vector of the distance function with respect to the
    query point Q, expressed in world frame W.

Note:
    ``grad_W`` is not well defined everywhere. For example, when
    computing the distance from a point to a sphere, and the point
    coincides with the center of the sphere, grad_W is not well
    defined (as it can be computed as p_GQ / |p_GQ|, but the
    denominator is 0). When grad_W is not well defined, and we
    instantiate SignedDistanceToPoint<T> with T being an
    AutoDiffScalar (like AutoDiffXd), the gradient of the query result
    is not well defined either, so the user should use the gradient in
    p_GN, distance and grad_W with caution.

Precondition:
    grad_W_in must not contain NaN.)""";
        } ctor;
        // Symbol: drake::geometry::SignedDistanceToPoint::distance
        struct /* distance */ {
          // Source: drake/geometry/query_results/signed_distance_to_point.h
          const char* doc =
R"""(The signed distance from the query point Q to the nearest point N on
the surface of geometry G. It is positive if Q is outside G. It is
negative if Q is inside G. It is zero if Q is on the boundary of G.)""";
        } distance;
        // Symbol: drake::geometry::SignedDistanceToPoint::grad_W
        struct /* grad_W */ {
          // Source: drake/geometry/query_results/signed_distance_to_point.h
          const char* doc =
R"""(The gradient vector of the distance function with respect to the query
point Q, expressed in world frame W.)""";
        } grad_W;
        // Symbol: drake::geometry::SignedDistanceToPoint::id_G
        struct /* id_G */ {
          // Source: drake/geometry/query_results/signed_distance_to_point.h
          const char* doc =
R"""(The id of the geometry G to which we measure distance from the query
point Q.)""";
        } id_G;
        // Symbol: drake::geometry::SignedDistanceToPoint::p_GN
        struct /* p_GN */ {
          // Source: drake/geometry/query_results/signed_distance_to_point.h
          const char* doc =
R"""(The position of the nearest point N on G's surface from the query
point Q, expressed in G's frame.)""";
        } p_GN;
      } SignedDistanceToPoint;
    } geometry;
  } drake;
} pydrake_doc_geometry_query_results;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
