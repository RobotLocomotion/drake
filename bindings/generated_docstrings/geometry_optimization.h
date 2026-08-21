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

// #include "drake/geometry/optimization/affine_ball.h"
// #include "drake/geometry/optimization/affine_subspace.h"
// #include "drake/geometry/optimization/c_iris_collision_geometry.h"
// #include "drake/geometry/optimization/cartesian_product.h"
// #include "drake/geometry/optimization/convex_hull.h"
// #include "drake/geometry/optimization/convex_set.h"
// #include "drake/geometry/optimization/cspace_free_box.h"
// #include "drake/geometry/optimization/cspace_free_internal.h"
// #include "drake/geometry/optimization/cspace_free_polytope.h"
// #include "drake/geometry/optimization/cspace_free_polytope_base.h"
// #include "drake/geometry/optimization/cspace_free_structs.h"
// #include "drake/geometry/optimization/cspace_separating_plane.h"
// #include "drake/geometry/optimization/geodesic_convexity.h"
// #include "drake/geometry/optimization/graph_of_convex_sets.h"
// #include "drake/geometry/optimization/hpolyhedron.h"
// #include "drake/geometry/optimization/hyperellipsoid.h"
// #include "drake/geometry/optimization/hyperrectangle.h"
// #include "drake/geometry/optimization/implicit_graph_of_convex_sets.h"
// #include "drake/geometry/optimization/intersection.h"
// #include "drake/geometry/optimization/iris.h"
// #include "drake/geometry/optimization/iris_internal.h"
// #include "drake/geometry/optimization/minkowski_sum.h"
// #include "drake/geometry/optimization/point.h"
// #include "drake/geometry/optimization/spectrahedron.h"
// #include "drake/geometry/optimization/vpolytope.h"

// Symbol: pydrake_doc_geometry_optimization
constexpr struct /* pydrake_doc_geometry_optimization */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::geometry
    struct /* geometry */ {
      // Symbol: drake::geometry::optimization
      struct /* optimization */ {
        // Symbol: drake::geometry::optimization::AffineBall
        struct /* AffineBall */ {
          // Source: drake/geometry/optimization/affine_ball.h
          const char* doc =
R"""(Implements an ellipsoidal convex set represented as an affine scaling
of the unit ball {Bu + center | |u|₂ ≤ 1}. B must be a square matrix.

Compare this with an alternative parametrization of the ellipsoid: {x
| (x-center)ᵀAᵀA(x-center) ≤ 1}, which utilizes a quadratic form. The
two representations are related by B = A⁻¹ if A and B are invertible.

The quadratic form parametrization is implemented in Hyperellipsoid.
It can represent unbounded sets, but not sets along a
lower-dimensional affine subspace. The AffineBall parametrization can
represent sets along a lower-dimensional affine subspace, but not
unbounded sets.

An AffineBall can never be empty -- it always contains its center.
This includes the zero-dimensional case.)""";
          // Symbol: drake::geometry::optimization::AffineBall::AffineBall
          struct /* ctor */ {
            // Source: drake/geometry/optimization/affine_ball.h
            const char* doc_0args =
R"""(Constructs a default (zero-dimensional, nonempty) set.)""";
            // Source: drake/geometry/optimization/affine_ball.h
            const char* doc_2args =
R"""(Constructs the ellipsoid from a transformation matrix B and
translation center. B describes the linear transformation that is
applied to the unit ball in order to produce the ellipsoid, and center
describes the translation of the center of the ellipsoid from the
origin.

Precondition:
    B.rows() == B.cols().

Precondition:
    B.cols() == center.size().)""";
            // Source: drake/geometry/optimization/affine_ball.h
            const char* doc_1args =
R"""(Constructs an AffineBall from a Hyperellipsoid.

Precondition:
    ellipsoid.IsBounded().)""";
          } ctor;
          // Symbol: drake::geometry::optimization::AffineBall::B
          struct /* B */ {
            // Source: drake/geometry/optimization/affine_ball.h
            const char* doc =
R"""(Returns the affine transformation matrix B.)""";
          } B;
          // Symbol: drake::geometry::optimization::AffineBall::MakeAffineBallFromLineSegment
          struct /* MakeAffineBallFromLineSegment */ {
            // Source: drake/geometry/optimization/affine_ball.h
            const char* doc =
R"""(Constructs an affine ball such that its main diameter is the line
segment from ``x_1`` to ``x_2``, and the length of all other diameters
are 2 * ``epsilon``.

Precondition:
    x_1.size() == x_2.size().

Precondition:
    epsilon >= 0.

Raises:
    RuntimeError if ‖x_1 - x_2‖₂ is less than 1e-9.)""";
          } MakeAffineBallFromLineSegment;
          // Symbol: drake::geometry::optimization::AffineBall::MakeAxisAligned
          struct /* MakeAxisAligned */ {
            // Source: drake/geometry/optimization/affine_ball.h
            const char* doc =
R"""(Constructs an axis-aligned AffineBall with the implicit form
(x₀-c₀)²/r₀² + (x₁-c₁)²/r₁² + ... + (x_N - c_N)²/r_N² ≤ 1, where c is
shorthand for ``center`` and r is shorthand for ``radius``.

Precondition:
    radius.size() == center.size().

Precondition:
    radius[i] >= 0, for all i.)""";
          } MakeAxisAligned;
          // Symbol: drake::geometry::optimization::AffineBall::MakeHypersphere
          struct /* MakeHypersphere */ {
            // Source: drake/geometry/optimization/affine_ball.h
            const char* doc =
R"""(Constructs a hypersphere with ``radius`` and ``center``.

Precondition:
    radius >= 0.)""";
          } MakeHypersphere;
          // Symbol: drake::geometry::optimization::AffineBall::MakeUnitBall
          struct /* MakeUnitBall */ {
            // Source: drake/geometry/optimization/affine_ball.h
            const char* doc =
R"""(Constructs the L₂-norm unit ball in ``dim`` dimensions, {x | |x|₂ <= 1
}.

Precondition:
    dim >= 0.)""";
          } MakeUnitBall;
          // Symbol: drake::geometry::optimization::AffineBall::MinimumVolumeCircumscribedEllipsoid
          struct /* MinimumVolumeCircumscribedEllipsoid */ {
            // Source: drake/geometry/optimization/affine_ball.h
            const char* doc =
R"""(Constructs the minimum-volume ellipsoid which contains all of the
given points. This is commonly referred to as the outer Löwner-John
ellipsoid.

If all of the points lie along a proper affine subspace, this method
instead computes the minimum-k-volume ellipsoid, where k is the affine
dimension of the convex hull of ``points``.

Parameter ``points``:
    is a d-by-n matrix, where d is the ambient dimension and each
    column represents one point.

Parameter ``rank_tol``:
    the tolerance used to detect if the data lies in an affine
    subspace. The affine ball is computed in the subspace spanned by
    the left singular vectors of the data matrix whose associated
    singular values are larger than ``rank_tol`` *
    ``max_singular_value``. The default is 1e-6 to be compatible with
    common solver tolerances.

Raises:
    RuntimeError if the MathematicalProgram fails to solve. This can
    happen due to numerical issues caused by ``rank_tol`` being set
    too low.

Raises:
    RuntimeError if points includes NaNs or infinite values.

Precondition:
    points.rows() >= 1.

Precondition:
    points.cols() >= 1.)""";
          } MinimumVolumeCircumscribedEllipsoid;
          // Symbol: drake::geometry::optimization::AffineBall::Serialize
          struct /* Serialize */ {
            // Source: drake/geometry/optimization/affine_ball.h
            const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
          } Serialize;
          // Symbol: drake::geometry::optimization::AffineBall::center
          struct /* center */ {
            // Source: drake/geometry/optimization/affine_ball.h
            const char* doc = R"""(Returns the center of the ellipsoid.)""";
          } center;
        } AffineBall;
        // Symbol: drake::geometry::optimization::AffineSubspace
        struct /* AffineSubspace */ {
          // Source: drake/geometry/optimization/affine_subspace.h
          const char* doc =
R"""(An affine subspace (also known as a "flat", a "linear variety", or a
"linear manifold") is a vector subspace of some Euclidean space,
potentially translated so as to not pass through the origin. Examples
include points, lines, and planes (not necessarily through the
origin).

An affine subspace is described by a basis of its corresponding vector
subspace, plus a translation. This description is not unique as any
point in the affine subspace can be used as a translation, and any
basis of the corresponding vector subspace is valid.

An affine subspace can never be empty, because a vector subspace can
never be empty. Thus, the translation will always be contained in the
flat. An affine subspace is bounded if it is a point, which is when
the basis has zero columns.)""";
          // Symbol: drake::geometry::optimization::AffineSubspace::AffineDimension
          struct /* AffineDimension */ {
            // Source: drake/geometry/optimization/affine_subspace.h
            const char* doc =
R"""(Returns the affine dimension of this set. For an affine subspace, this
is simply the number of columns in the basis_ matrix. A point will
have affine dimension zero.)""";
          } AffineDimension;
          // Symbol: drake::geometry::optimization::AffineSubspace::AffineSubspace
          struct /* ctor */ {
            // Source: drake/geometry/optimization/affine_subspace.h
            const char* doc_0args =
R"""(Constructs a default (zero-dimensional, nonempty) affine subspace.)""";
            // Source: drake/geometry/optimization/affine_subspace.h
            const char* doc_2args_basis_translation =
R"""(Constructs the affine subspace from an n-by-m matrix describing the
basis, where n is the ambient dimension, and m is the dimension of the
subspace, and from an n-dimensional vector describing the translation.
The set is {x | x = translation + basis*y, y ∈ Rᵐ} The columns must be
linearly independent.

Precondition:
    basis.rows() == translation.size().)""";
            // Source: drake/geometry/optimization/affine_subspace.h
            const char* doc_2args_set_tol =
R"""(Constructs an affine subspace as the affine hull of another convex
set. The generic approach is to find a feasible point in the set, and
then iteratively compute feasible vectors until we have a basis that
spans the set. If you pass in a convex set whose points are
matrix-valued (e.g. a Spectrahedron), then the affine subspace will
work over a flattened representation of those coordinates. (So a
Spectrahedron with n-by-n matrices will output an AffineSubspace with
ambient dimension (n * (n+1)) / 2.)

``tol`` sets the numerical precision of the computation. For each
dimension, a pair of feasible points are constructed, so as to
maximize the displacement in that dimension. If their displacement
along that dimension is larger than tol, then the vector connecting
the points is added as a basis vector.

Raises:
    RuntimeError if ``set`` is empty.

Raises:
    RuntimeError if ``tol < 0``.

For several subclasses of ConvexSet, there is a closed-form
computation (or more efficient numerical computation) that is
preferred. - AffineBall: Can be computed via a rank-revealing
decomposition; ``tol`` is used as the numerical tolerance for the rank
of the matrix. Pass ``std∷nullopt`` for ``tol`` to use Eigen's
automatic tolerance computation. - AffineSubspace: Equivalent to the
copy-constructor; ``tol`` is ignored. - CartesianProduct: Can compute
the affine hull of each factor individually; ``tol`` is propagated to
the constituent calls. (This is not done if the Cartesian product has
an associated affine transformation.) - Hyperellipsoid: Always equal
to the whole ambient space; ``tol`` is ignored. - Hyperrectangle: Can
be computed in closed-form; ``tol`` has the same meaning as in the
generic affine hull computation. - Point: Can be computed in
closed-form; ``tol`` is ignored. This also encompasses sets which are
obviously a singleton point, as determined via MaybeGetPoint. -
VPolytope: Can be computed via a singular value decomposition; ``tol``
is used as the numerical tolerance for the rank of the matrix. Pass
``std∷nullopt`` for ``tol`` to use Eigen's automatic tolerance
computation.)""";
          } ctor;
          // Symbol: drake::geometry::optimization::AffineSubspace::ContainedIn
          struct /* ContainedIn */ {
            // Source: drake/geometry/optimization/affine_subspace.h
            const char* doc =
R"""(Returns ``True`` if ``this`` AffineSubspace is contained in ``other``.
This is computed by checking if ``translation()`` is in ``other`` and
then checking if each basis vector is in the span of the basis of
``other``. The latter step requires finding a least-squares solution,
so a nonzero tolerance (``tol``) is almost always necessary. (You may
have to adjust the default tolerance depending on the dimension of
your space and the scale of your basis vectors.))""";
          } ContainedIn;
          // Symbol: drake::geometry::optimization::AffineSubspace::IsNearlyEqualTo
          struct /* IsNearlyEqualTo */ {
            // Source: drake/geometry/optimization/affine_subspace.h
            const char* doc =
R"""(Returns true if the two AffineSubspaces describe the same set, by
checking that each set is contained in the other.)""";
          } IsNearlyEqualTo;
          // Symbol: drake::geometry::optimization::AffineSubspace::OrthogonalComplementBasis
          struct /* OrthogonalComplementBasis */ {
            // Source: drake/geometry/optimization/affine_subspace.h
            const char* doc =
R"""(Returns an orthonormal basis of the vector subspace which is
orthogonal to this AffineSubspace.)""";
          } OrthogonalComplementBasis;
          // Symbol: drake::geometry::optimization::AffineSubspace::Serialize
          struct /* Serialize */ {
            // Source: drake/geometry/optimization/affine_subspace.h
            const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
          } Serialize;
          // Symbol: drake::geometry::optimization::AffineSubspace::ToGlobalCoordinates
          struct /* ToGlobalCoordinates */ {
            // Source: drake/geometry/optimization/affine_subspace.h
            const char* doc =
R"""(Given a point y in the basis of the AffineSubspace, with the zero
point at translation_, returns the coordinates of y in the standard
basis of the ambient space. If the AffineSubspace is a point, it has
an empty basis, so the only possible local coordinates are also empty
(and should be passed in as a length-zero vector). Each column of the
input should be a vector in the affine subspace, represented in its
local coordinates, and the corresponding column of the output will be
its representation in the coordinate system of the ambient space.

Precondition:
    y.rows() == AffineDimension())""";
          } ToGlobalCoordinates;
          // Symbol: drake::geometry::optimization::AffineSubspace::ToLocalCoordinates
          struct /* ToLocalCoordinates */ {
            // Source: drake/geometry/optimization/affine_subspace.h
            const char* doc =
R"""(Given a point x in the standard basis of the ambient space, returns
the coordinates of x in the basis of the AffineSubspace, with the zero
point at translation_. The component of x that is orthogonal to the
AffineSubspace (if it exists) is discarded, so
ToGlobalCoordinates(ToLocalCoordinates(x)) is equivalent to
Project(x). Note that if the AffineSubspace is a point, the basis is
empty, so the local coordinates will also be empty (and returned as a
length-zero vector). Each column of the input should be a vector in
the ambient space, and the corresponding column of the output will be
its representation in the local coordinates of the affine subspace.

Precondition:
    x.rows() == ambient_dimension())""";
          } ToLocalCoordinates;
          // Symbol: drake::geometry::optimization::AffineSubspace::basis
          struct /* basis */ {
            // Source: drake/geometry/optimization/affine_subspace.h
            const char* doc =
R"""(Returns the basis in an n-by-m matrix, where n is the ambient
dimension, and m is the number of vectors in the basis.)""";
          } basis;
          // Symbol: drake::geometry::optimization::AffineSubspace::translation
          struct /* translation */ {
            // Source: drake/geometry/optimization/affine_subspace.h
            const char* doc =
R"""(Returns the translation as a length n vector.)""";
          } translation;
        } AffineSubspace;
        // Symbol: drake::geometry::optimization::CIrisCollisionGeometry
        struct /* CIrisCollisionGeometry */ {
          // Source: drake/geometry/optimization/c_iris_collision_geometry.h
          const char* doc =
R"""(This class contains the necessary information about the collision
geometry used in C-IRIS. Most notably it transcribes the geometric
condition that the collision geometry is on one side of the plane to
mathematical constraints. For the detailed algorithm please refer to
the paper Certified Polyhedral Decompositions of Collision-Free
Configuration Space by Hongkai Dai*, Alexandre Amice*, Peter Werner,
Annan Zhang and Russ Tedrake.)""";
          // Symbol: drake::geometry::optimization::CIrisCollisionGeometry::CIrisCollisionGeometry
          struct /* ctor */ {
            // Source: drake/geometry/optimization/c_iris_collision_geometry.h
            const char* doc =
R"""(Parameter ``geometry``:
    The actual geometry object. ``geometry`` must outlive this
    CIrisCollisionGeometry object.

Parameter ``body_index``:
    The index of the body to which this geometry is fixed.

Parameter ``id``:
    The ID of this geometry.

Parameter ``X_BG``:
    The pose of the geometry (G) in the attached body frame (B).)""";
          } ctor;
          // Symbol: drake::geometry::optimization::CIrisCollisionGeometry::OnPlaneSide
          struct /* OnPlaneSide */ {
            // Source: drake/geometry/optimization/c_iris_collision_geometry.h
            const char* doc =
R"""(Impose the constraint that the geometry is on a given side of the
plane {x | aᵀx+b=0}.

For example, to impose the constraint that a polytope is on the
positive side of the plane, we consider the following constraints
aᵀp_AVᵢ + b ≥ 1 (1) where Vᵢ is the i'th vertex of the polytope. (1)
says rational functions are non-negative.

To impose the constraint that a sphere is on positive side of the
plane, we consider the following constraints aᵀp_AS + b ≥ r*|a| (2a)
aᵀp_AS + b ≥ 1 (2b) where S is the sphere center, r is the sphere
radius.

We can reformulate (2a) as the following constraint ⌈aᵀp_AS + b aᵀ⌉ is
psd. (3) ⌊ a (aᵀp_AS + b)/r²*I₃⌋ (3) is equivalent to the rational
⌈1⌉ᵀ*⌈aᵀp_AS + b aᵀ⌉*⌈1⌉ ⌊y⌋ ⌊ a (aᵀp_AS+ b)/r²*I₃⌋ ⌊y⌋ is positive.

Parameter ``a``:
    The normal vector in the separating plane. a is expressed in frame
    A. Note that ``a`` doesn't need to have a unit length.

Parameter ``b``:
    The constant term in the separating plane.

Parameter ``X_AB_multilinear``:
    The pose of the collision geometry body (B) in the expressed frame
    A, written as a multilinear polynomial. This quantity is generated
    from
    RationalForwardKinematics∷CalcBodyPoseAsMultilinearPolynomial.

Parameter ``rational_forward_kin``:
    This object is constructed with the MultibodyPlant containing this
    collision geometry.

Parameter ``plane_side``:
    Whether the geometry is on the positive or negative side of the
    plane.

Parameter ``y_slack``:
    The slack variable y in the documentation above, used for
    non-polytopic geometries. For spheres and capsules, y_slack has
    size 3. For cylinders, y_slack has size 2.

Parameter ``rationals``:
    We append new rational functions to ``rationals``. If these new
    rational functions are positive, then the geometry is on a given
    side of the plane.

Precondition:
    rationals != nullptr)""";
          } OnPlaneSide;
          // Symbol: drake::geometry::optimization::CIrisCollisionGeometry::X_BG
          struct /* X_BG */ {
            // Source: drake/geometry/optimization/c_iris_collision_geometry.h
            const char* doc = R"""()""";
          } X_BG;
          // Symbol: drake::geometry::optimization::CIrisCollisionGeometry::body_index
          struct /* body_index */ {
            // Source: drake/geometry/optimization/c_iris_collision_geometry.h
            const char* doc = R"""()""";
          } body_index;
          // Symbol: drake::geometry::optimization::CIrisCollisionGeometry::geometry
          struct /* geometry */ {
            // Source: drake/geometry/optimization/c_iris_collision_geometry.h
            const char* doc = R"""()""";
          } geometry;
          // Symbol: drake::geometry::optimization::CIrisCollisionGeometry::id
          struct /* id */ {
            // Source: drake/geometry/optimization/c_iris_collision_geometry.h
            const char* doc = R"""()""";
          } id;
          // Symbol: drake::geometry::optimization::CIrisCollisionGeometry::num_rationals
          struct /* num_rationals */ {
            // Source: drake/geometry/optimization/c_iris_collision_geometry.h
            const char* doc =
R"""(Returns the number of rationals in the condition "this geometry is on
one side of the plane.")""";
          } num_rationals;
          // Symbol: drake::geometry::optimization::CIrisCollisionGeometry::type
          struct /* type */ {
            // Source: drake/geometry/optimization/c_iris_collision_geometry.h
            const char* doc = R"""()""";
          } type;
        } CIrisCollisionGeometry;
        // Symbol: drake::geometry::optimization::CIrisGeometryType
        struct /* CIrisGeometryType */ {
          // Source: drake/geometry/optimization/c_iris_collision_geometry.h
          const char* doc =
R"""(The supported type of geometries in C-IRIS.)""";
          // Symbol: drake::geometry::optimization::CIrisGeometryType::kCapsule
          struct /* kCapsule */ {
            // Source: drake/geometry/optimization/c_iris_collision_geometry.h
            const char* doc = R"""()""";
          } kCapsule;
          // Symbol: drake::geometry::optimization::CIrisGeometryType::kCylinder
          struct /* kCylinder */ {
            // Source: drake/geometry/optimization/c_iris_collision_geometry.h
            const char* doc = R"""()""";
          } kCylinder;
          // Symbol: drake::geometry::optimization::CIrisGeometryType::kPolytope
          struct /* kPolytope */ {
            // Source: drake/geometry/optimization/c_iris_collision_geometry.h
            const char* doc = R"""()""";
          } kPolytope;
          // Symbol: drake::geometry::optimization::CIrisGeometryType::kSphere
          struct /* kSphere */ {
            // Source: drake/geometry/optimization/c_iris_collision_geometry.h
            const char* doc = R"""()""";
          } kSphere;
        } CIrisGeometryType;
        // Symbol: drake::geometry::optimization::CSpaceSeparatingPlane
        struct /* CSpaceSeparatingPlane */ {
          // Source: drake/geometry/optimization/cspace_separating_plane.h
          const char* doc =
R"""(Wraps the information that a pair of collision geometries are
separated by a plane. One collision geometry is on the "positive" side
of the separating plane, namely {x| aᵀx + b ≥ δ} (with δ ≥ 0}, and the
other collision geometry is on the "negative" side of the separating
plane, namely {x|aᵀx+b ≤ −δ}.

Template parameter ``T``:
    The type of decision_variables. T= symbolic∷Variable or double.)""";
          // Symbol: drake::geometry::optimization::CSpaceSeparatingPlane::CSpaceSeparatingPlane<T>
          struct /* ctor */ {
            // Source: drake/geometry/optimization/cspace_separating_plane.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::geometry::optimization::CSpaceSeparatingPlane::a
          struct /* a */ {
            // Source: drake/geometry/optimization/cspace_separating_plane.h
            const char* doc = R"""()""";
          } a;
          // Symbol: drake::geometry::optimization::CSpaceSeparatingPlane::b
          struct /* b */ {
            // Source: drake/geometry/optimization/cspace_separating_plane.h
            const char* doc = R"""()""";
          } b;
          // Symbol: drake::geometry::optimization::CSpaceSeparatingPlane::decision_variables
          struct /* decision_variables */ {
            // Source: drake/geometry/optimization/cspace_separating_plane.h
            const char* doc = R"""()""";
          } decision_variables;
          // Symbol: drake::geometry::optimization::CSpaceSeparatingPlane::expressed_body
          struct /* expressed_body */ {
            // Source: drake/geometry/optimization/cspace_separating_plane.h
            const char* doc = R"""()""";
          } expressed_body;
          // Symbol: drake::geometry::optimization::CSpaceSeparatingPlane::geometry
          struct /* geometry */ {
            // Source: drake/geometry/optimization/cspace_separating_plane.h
            const char* doc =
R"""(Return the geometry on the specified side.)""";
          } geometry;
          // Symbol: drake::geometry::optimization::CSpaceSeparatingPlane::geometry_pair
          struct /* geometry_pair */ {
            // Source: drake/geometry/optimization/cspace_separating_plane.h
            const char* doc = R"""()""";
          } geometry_pair;
          // Symbol: drake::geometry::optimization::CSpaceSeparatingPlane::negative_side_geometry
          struct /* negative_side_geometry */ {
            // Source: drake/geometry/optimization/cspace_separating_plane.h
            const char* doc = R"""()""";
          } negative_side_geometry;
          // Symbol: drake::geometry::optimization::CSpaceSeparatingPlane::plane_degree
          struct /* plane_degree */ {
            // Source: drake/geometry/optimization/cspace_separating_plane.h
            const char* doc = R"""()""";
          } plane_degree;
          // Symbol: drake::geometry::optimization::CSpaceSeparatingPlane::positive_side_geometry
          struct /* positive_side_geometry */ {
            // Source: drake/geometry/optimization/cspace_separating_plane.h
            const char* doc = R"""()""";
          } positive_side_geometry;
        } CSpaceSeparatingPlane;
        // Symbol: drake::geometry::optimization::CalcPlane
        struct /* CalcPlane */ {
          // Source: drake/geometry/optimization/cspace_separating_plane.h
          const char* doc =
R"""(Computes the parameters a, b in the plane { x | aᵀx+b=0 }. a and b are
both polynomials of ``vars_for_plane``. The coefficients of these
polynomials are in ``decision_variables`` in graded reverse
lexicographic order.

Template parameter ``D``:
    , S, V The valid combination of D, S, V are 1.
    D=symbolic∷Variable, S=symbolic∷Variable, V=symbolic∷Polynomial.
    2. D=double, S=symbolic∷Variable, V=symbolic∷Polynomial 3.
    D=double, S=double, V=double)""";
        } CalcPlane;
        // Symbol: drake::geometry::optimization::CartesianProduct
        struct /* CartesianProduct */ {
          // Source: drake/geometry/optimization/cartesian_product.h
          const char* doc =
R"""(The Cartesian product of convex sets is a convex set: S = X₁ × X₂ × ⋯
× Xₙ = {(x₁, x₂, ..., xₙ) | x₁ ∈ X₁, x₂ ∈ X₂, ..., xₙ ∈ Xₙ}.

This class also supports a generalization of this concept in which the
coordinates are transformed by the linear map, {x | y = Ax + b, y ∈ Y₁
× Y₂ × ⋯ × Yₙ}, with the default values set to the identity map. This
concept is required for reasoning about cylinders in arbitrary poses
as cartesian products, and more generally for describing any affine
transform of a CartesianProduct.

Special behavior for IsEmpty: If there are no sets in the product,
returns nonempty by convention. See:
https://en.wikipedia.org/wiki/Empty_product#Nullary_Cartesian_product
Otherwise, if any set in the cartesian product is empty, the whole
product is empty.)""";
          // Symbol: drake::geometry::optimization::CartesianProduct::A
          struct /* A */ {
            // Source: drake/geometry/optimization/cartesian_product.h
            const char* doc =
R"""(Returns a copy of the matrix A if it has been set, or nullopt
otherwise.)""";
          } A;
          // Symbol: drake::geometry::optimization::CartesianProduct::CartesianProduct
          struct /* ctor */ {
            // Source: drake/geometry/optimization/cartesian_product.h
            const char* doc_0args =
R"""(Constructs a default (zero-dimensional, nonempty) set.)""";
            // Source: drake/geometry/optimization/cartesian_product.h
            const char* doc_1args_sets =
R"""(Constructs the product from a vector of convex sets.)""";
            // Source: drake/geometry/optimization/cartesian_product.h
            const char* doc_2args_setA_setB =
R"""(Constructs the product from a pair of convex sets.)""";
            // Source: drake/geometry/optimization/cartesian_product.h
            const char* doc_3args_sets_A_b =
R"""(Constructs the product of convex sets in the transformed coordinates:
{x | y = Ax + b, y ∈ Y₁ × Y₂ × ⋯ × Yₙ}.

Raises:
    RuntimeError when ``A`` is not full column rank.)""";
            // Source: drake/geometry/optimization/cartesian_product.h
            const char* doc_3args_query_object_geometry_id_reference_frame =
R"""(Constructs a CartesianProduct from a SceneGraph geometry and pose in
the ``reference_frame`` frame, obtained via the QueryObject. If
``reference_frame`` frame is std∷nullopt, then it will be expressed in
the world frame.

Although any geometry that can be used as a ConvexSet could also be a
(trivial) CartesianProduct, we restrict this constructor to handling
Cylinder geometry, which constructs the (non-trivial) Cartesian
product of a HyperEllipsoid and an HPolyhedron. Most other SceneGraph
geometry types are supported by at least one of the ConvexSet class
constructors.

Raises:
    RuntimeError if geometry_id does not correspond to a Cylinder.)""";
          } ctor;
          // Symbol: drake::geometry::optimization::CartesianProduct::b
          struct /* b */ {
            // Source: drake/geometry/optimization/cartesian_product.h
            const char* doc =
R"""(Returns a copy of the vector b if it has been set, or nullopt
otherwise.)""";
          } b;
          // Symbol: drake::geometry::optimization::CartesianProduct::factor
          struct /* factor */ {
            // Source: drake/geometry/optimization/cartesian_product.h
            const char* doc =
R"""(Returns a reference to the ConvexSet defining the ``index`` factor in
the product.)""";
          } factor;
          // Symbol: drake::geometry::optimization::CartesianProduct::num_factors
          struct /* num_factors */ {
            // Source: drake/geometry/optimization/cartesian_product.h
            const char* doc =
R"""(The number of factors (or sets) used in the product.)""";
          } num_factors;
        } CartesianProduct;
        // Symbol: drake::geometry::optimization::CheckIfSatisfiesConvexityRadius
        struct /* CheckIfSatisfiesConvexityRadius */ {
          // Source: drake/geometry/optimization/geodesic_convexity.h
          const char* doc =
R"""(Given a convex set, and a list of indices corresponding to continuous
revolute joints, checks whether or not the set satisfies the convexity
radius. See §6.5.3 of "A Panoramic View of Riemannian Geometry",
Marcel Berger for a general definition of convexity radius. When
dealing with continuous revolute joints, respecting the convexity
radius entails that each convex set has a width of stricty less than π
along each dimension corresponding to a continuous revolute joint.

Raises:
    RuntimeError if continuous_revolute_joints has repeated entries,
    or if any entry is outside the interval [0,
    convex_set.ambient_dimension()).)""";
        } CheckIfSatisfiesConvexityRadius;
        // Symbol: drake::geometry::optimization::ComputePairwiseIntersections
        struct /* ComputePairwiseIntersections */ {
          // Source: drake/geometry/optimization/geodesic_convexity.h
          const char* doc_5args_convex_sets_A_convex_sets_B_continuous_revolute_joints_preprocess_bbox_parallelism =
R"""(Computes the pairwise intersections between two lists of convex sets,
returning a list of edges, and a list of their corresponding offsets.
Each edge is a tuple in the form [index_A, index_B], where index_A is
the index of the set in ``convex_sets_A`` and index_B is the index of
the set in ``convex_sets_B``. The corresponding entry in the list of
offsets (i.e., the entry at the same index) is the translation that is
applied to all the points in the index_A'th set in ``convex_sets_A``
to align them with the index_B'th set in ``convex_sets_B``. This
translation may only have non-zero entries along the dimensions
corresponding to ``continuous_revolute_joints``. All non-zero entries
are integer multiples of 2π as the translation of the sets still
represents the same configurations for the indices in
``continuous_revolute_joints``.

Parameter ``convex_sets_A``:
    is a vector of convex sets. Pairwise intersections will be
    computed between ``convex_sets_A`` and ``convex_sets_B``.

Parameter ``convex_sets_B``:
    is the other vector of convex sets.

Parameter ``continuous_revolute_joints``:
    is a list of joint indices corresponding to continuous revolute
    joints.

Parameter ``preprocess_bbox``:
    is a flag for whether the function should precompute axis-aligned
    bounding boxes (AABBs) for every set. This can speed up the
    pairwise intersection checks, by determining some sets to be
    disjoint without needing to solve an optimization problem.
    However, it does require some overhead to compute those bounding
    boxes.

Parameter ``parallelism``:
    specifies the number of threads to use.

Raises:
    if ``continuous_revolute_joints`` has repeated entries, or if any
    entry is outside the interval [0, ambient_dimension), where
    ambient_dimension is the ambient dimension of the convex sets in
    ``convex_sets_A`` and ``convex_sets_B``.

Raises:
    if ``convex_sets_A`` or ``convex_sets_B`` are empty.

Raises:
    if any entry of ``convex_sets_A`` or ``convex_sets_B`` is a
    nullptr.)""";
          // Source: drake/geometry/optimization/geodesic_convexity.h
          const char* doc_6args_convex_sets_A_convex_sets_B_continuous_revolute_joints_bboxes_A_bboxes_B_parallelism =
R"""(Overload of ``ComputePairwiseIntersections`` allowing the user to
supply axis- aligned bounding boxes if they're known a priori, to save
on computation time.

Parameter ``bboxes_A``:
    is a vector of Hyperrectangles, allowing the user to manually pass
    in the AABBs of each set in ``convex_sets_A`` to avoid
    recomputation.

Parameter ``bboxes_B``:
    serves the same role to ``convex_sets_B`` as ``bboxes_A`` does to
    ``convex_sets_A``.

Warning:
    The function does not check that the entries of bboxes_A are
    indeed the AABBs corresponding to the sets in ``convex_sets_A``
    (and likewise for bboxes_B).

Raises:
    if ``convex_sets_A.size() != bboxes_A.size()``

Raises:
    if ``convex_sets_B.size() != bboxes_B.size()``

Raises:
    if not all entries of ``convex_sets_A``, `convex_sets_B`,
    ``bboxes_A``, and ``bboxes_B`` have the same ambient dimension.

Raises:
    if any entry of ``convex_sets_A`` or ``convex_sets_B`` is a
    nullptr.)""";
          // Source: drake/geometry/optimization/geodesic_convexity.h
          const char* doc_4args_convex_sets_continuous_revolute_joints_preprocess_bbox_parallelism =
R"""(Convenience overload to compute pairwise intersections within a list
of convex sets. Equivalent to calling
ComputePairwiseIntersections(convex_sets, convex_sets,
continuous_revolute_joints).

Parameter ``convex_sets``:
    is a vector of convex sets. Pairwise intersections will be
    computed within ``convex_sets``.

Parameter ``continuous_revolute_joints``:
    is a list of joint indices corresponding to continuous revolute
    joints.

Parameter ``preprocess_bbox``:
    is a flag for whether the function should precompute axis-aligned
    bounding boxes for every set. This can speed up the pairwise
    intersection checks, by determining some sets to be disjoint
    without needing to solve an optimization problem.

Parameter ``parallelism``:
    specifies the number of threads to use.

Raises:
    if ``continuous_revolute_joints`` has repeated entries, or if any
    entry is outside the interval [0, ambient_dimension), where
    ambient_dimension is the ambient dimension of the convex sets in
    ``convex_sets``.

Raises:
    if ``convex_sets`` is empty.

Raises:
    if any entry of ``convex_sets`` is a nullptr.)""";
          // Source: drake/geometry/optimization/geodesic_convexity.h
          const char* doc_4args_convex_sets_continuous_revolute_joints_bboxes_parallelism =
R"""(Overload of ``ComputePairwiseIntersections`` allowing the user to
supply axis- aligned bounding boxes if they're known a priori, to save
on computation time.

Parameter ``bboxes``:
    is a vector of Hyperrectangles, allowing the user to manually pass
    in the AABBs of each set in ``convex_sets`` to avoid
    recomputation.

Warning:
    The function does not check that the entries are indeed the AABBs
    corresponding to the sets in ``convex_sets``.

Raises:
    if ``convex_sets.size() != bboxes.size()``

Raises:
    if not all entries of ``convex_sets`` and ``bboxes`` have the same
    ambient dimension.

Raises:
    if any entry of ``convex_sets`` is a nullptr.)""";
        } ComputePairwiseIntersections;
        // Symbol: drake::geometry::optimization::ConvexHull
        struct /* ConvexHull */ {
          // Source: drake/geometry/optimization/convex_hull.h
          const char* doc =
R"""(Implements the convex hull of a set of convex sets. The convex hull of
multiple sets is defined as the smallest convex set that contains all
the sets. Given non-empty convex sets {X₁, X₂, ..., Xₙ}, the convex
hull is the set of all convex combinations of points in the sets, i.e.
{∑ᵢ λᵢ xᵢ | xᵢ ∈ Xᵢ, λᵢ ≥ 0, ∑ᵢ λᵢ = 1}.)""";
          // Symbol: drake::geometry::optimization::ConvexHull::ConvexHull
          struct /* ctor */ {
            // Source: drake/geometry/optimization/convex_hull.h
            const char* doc =
R"""(Constructs the convex hull from a vector of convex sets.

Parameter ``sets``:
    A vector of convex sets that define the convex hull.

Parameter ``remove_empty_sets``:
    If true, the constructor will check if any of the sets are empty
    and will not consider them. If false, the constructor will not
    check if any of the sets are empty.

Warning:
    If remove_empty_sets is set to false, but some of the sets are in
    fact empty, then unexpected and incorrect results may occur. Only
    set this flag to false if you are sure that your sets are
    non-empty and performance in the constructor is critical.)""";
          } ctor;
          // Symbol: drake::geometry::optimization::ConvexHull::element
          struct /* element */ {
            // Source: drake/geometry/optimization/convex_hull.h
            const char* doc =
R"""(Returns a reference to the convex set at the given index (including
empty sets).)""";
          } element;
          // Symbol: drake::geometry::optimization::ConvexHull::empty_sets_removed
          struct /* empty_sets_removed */ {
            // Source: drake/geometry/optimization/convex_hull.h
            const char* doc =
R"""(Returns true if ``this`` was constructed with remove_empty_sets=true.)""";
          } empty_sets_removed;
          // Symbol: drake::geometry::optimization::ConvexHull::num_elements
          struct /* num_elements */ {
            // Source: drake/geometry/optimization/convex_hull.h
            const char* doc =
R"""(Returns the number of convex sets defining the convex hull (including
empty sets).)""";
          } num_elements;
          // Symbol: drake::geometry::optimization::ConvexHull::participating_sets
          struct /* participating_sets */ {
            // Source: drake/geometry/optimization/convex_hull.h
            const char* doc =
R"""(Returns the participating sets in the convex hull. If the constructor
was called with remove_empty_sets=false, this function will return the
original sets, including potentially empty sets.)""";
          } participating_sets;
          // Symbol: drake::geometry::optimization::ConvexHull::sets
          struct /* sets */ {
            // Source: drake/geometry/optimization/convex_hull.h
            const char* doc = R"""(Returns the participating convex sets.)""";
          } sets;
        } ConvexHull;
        // Symbol: drake::geometry::optimization::ConvexSet
        struct /* ConvexSet */ {
          // Source: drake/geometry/optimization/convex_set.h
          const char* doc =
R"""(Abstract base class for defining a convex set.)""";
          // Symbol: drake::geometry::optimization::ConvexSet::AddPointInNonnegativeScalingConstraints
          struct /* AddPointInNonnegativeScalingConstraints */ {
            // Source: drake/geometry/optimization/convex_set.h
            const char* doc_3args =
R"""(Let S be this convex set. When S is bounded, this method adds the
convex constraints to imply


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    x ∈ t S,
    t ≥ 0,

.. raw:: html

    </details>

where x is a point in ℜⁿ (with n the ambient_dimension) and t is a
scalar.

When S is unbounded, then the behavior is almost identical, except
when t=0. In this case, the constraints imply t ≥ 0, x ∈ t S ⊕ rec(S),
where rec(S) is the recession cone of S (the asymptotic directions in
which S is not bounded) and ⊕ is the Minkowski sum. For t > 0, this is
equivalent to x ∈ t S, but for t = 0, we have only x ∈ rec(S).

Raises:
    RuntimeError if ambient_dimension() == 0)""";
            // Source: drake/geometry/optimization/convex_set.h
            const char* doc_7args =
R"""(Let S be this convex set. When S is bounded, this method adds the
convex constraints to imply


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    A * x + b ∈ (c' * t + d) S,
    c' * t + d ≥ 0,

.. raw:: html

    </details>

where A is an n-by-m matrix (with n the ambient_dimension), b is a
vector of size n, c is a vector of size p, x is a point in ℜᵐ, and t
is a point in ℜᵖ.

When S is unbounded, then the behavior is almost identical, except
when c' * t+d=0. In this case, the constraints imply


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    A * x + b ∈ (c' * t + d) S ⊕ rec(S),
    c' * t + d ≥ 0,

.. raw:: html

    </details>

where rec(S) is the recession cone of S (the asymptotic directions in
which S is not bounded) and ⊕ is the Minkowski sum. For c' * t + d >
0, this is equivalent to A * x + b ∈ (c' * t + d) S, but for c' * t +
d = 0, we have only A * x + b ∈ rec(S).

Raises:
    RuntimeError if ambient_dimension() == 0)""";
          } AddPointInNonnegativeScalingConstraints;
          // Symbol: drake::geometry::optimization::ConvexSet::AddPointInSetConstraints
          struct /* AddPointInSetConstraints */ {
            // Source: drake/geometry/optimization/convex_set.h
            const char* doc =
R"""(Adds a constraint to an existing MathematicalProgram enforcing that
the point defined by vars is inside the set.

Returns:
    (new_vars, new_constraints) Some of the derived class will add new
    decision variables to enforce this constraint, we return all the
    newly added decision variables as new_vars. The meaning of these
    new decision variables differs in each subclass. If no new
    variables are added, then we return an empty Eigen vector. Also we
    return all the newly added constraints to ``prog`` through this
    function.

Raises:
    RuntimeError if ambient_dimension() == 0)""";
          } AddPointInSetConstraints;
          // Symbol: drake::geometry::optimization::ConvexSet::AffineHullShortcut
          struct /* AffineHullShortcut */ {
            // Source: drake/geometry/optimization/convex_set.h
            const char* doc =
R"""(When there is a more efficient strategy to compute the affine hull of
this set, returns affine hull as an AffineSubspace. When no efficient
conversion exists, returns null. The default base class implementation
returns null. This method is used by the AffineSubspace constructor to
short-circuit the generic iterative approach. (This function is static
to allow calling it from the AffineSubspace constructor, but is
conceptially a normal member function.) The return type is ConvexSet
to avoid a forward declaration; any non-null result must always have
the AffineSubspace as its runtime type.)""";
          } AffineHullShortcut;
          // Symbol: drake::geometry::optimization::ConvexSet::CalcVolume
          struct /* CalcVolume */ {
            // Source: drake/geometry/optimization/convex_set.h
            const char* doc =
R"""(Computes the exact volume for the convex set.

Note:
    Not every convex set can report an exact volume. In that case, use
    CalcVolumeViaSampling() instead.

Raises:
    RuntimeError if ``has_exact_volume()`` returns ``False``.

Raises:
    if ambient_dimension() == 0.)""";
          } CalcVolume;
          // Symbol: drake::geometry::optimization::ConvexSet::CalcVolumeViaSampling
          struct /* CalcVolumeViaSampling */ {
            // Source: drake/geometry/optimization/convex_set.h
            const char* doc =
R"""(Calculates an estimate of the volume of the convex set using sampling
and performing Monte Carlo integration.

Note:
    this method is intended to be used for low to moderate dimensions
    (d<15). For larger dimensions, a telescopic product approach has
    yet to be implemented. See, e.g.,
    https://proceedings.mlr.press/v151/chevallier22a/chevallier22a.pdf

Parameter ``generator``:
    a random number generator.

Parameter ``desired_rel_accuracy``:
    the desired relative accuracy of the volume estimate in the sense
    that the estimated volume is likely to be within the interval
    defined by (1±2*desired_rel_accuracy)*true_volume with probability
    of at least* 0.95 according to the Law of Large Numbers.
    https://people.math.umass.edu/~lr7q/ps_files/teaching/math456/Chapter6.pdf
    The computation will terminate when the relative error is less
    than rel_accuracy or when the maximum number of samples is
    reached.

Parameter ``max_num_samples``:
    the maximum number of samples to use.

Precondition:
    ``desired_rel_accuracy`` is in the range [0,1].

Returns:
    a pair the estimated volume of the set and an upper bound for the
    relative accuracy

Raises:
    if ambient_dimension() == 0.

Raises:
    if the minimum axis-aligned bounding box of the set cannot be
    computed.)""";
          } CalcVolumeViaSampling;
          // Symbol: drake::geometry::optimization::ConvexSet::Clone
          struct /* Clone */ {
            // Source: drake/geometry/optimization/convex_set.h
            const char* doc = R"""(Creates a unique deep copy of this set.)""";
          } Clone;
          // Symbol: drake::geometry::optimization::ConvexSet::ConvexSet
          struct /* ctor */ {
            // Source: drake/geometry/optimization/convex_set.h
            const char* doc =
R"""(For use by derived classes to construct a ConvexSet.

Parameter ``has_exact_volume``:
    Derived classes should pass ``True`` if they've implemented
    DoCalcVolume() to return a value (at least sometimes).)""";
          } ctor;
          // Symbol: drake::geometry::optimization::ConvexSet::DoAddPointInNonnegativeScalingConstraints
          struct /* DoAddPointInNonnegativeScalingConstraints */ {
            // Source: drake/geometry/optimization/convex_set.h
            const char* doc_3args =
R"""(Non-virtual interface implementation for
AddPointInNonnegativeScalingConstraints().

Precondition:
    x.size() == ambient_dimension()

Precondition:
    ambient_dimension() > 0)""";
            // Source: drake/geometry/optimization/convex_set.h
            const char* doc_7args =
R"""(Non-virtual interface implementation for
AddPointInNonnegativeScalingConstraints(). Subclasses must override to
add the constraints needed to keep the point A * x + b in the
non-negative scaling of the set. Note that subclasses do not need to
add the constraint c * t + d ≥ 0 as it is already added.

Precondition:
    ambient_dimension() > 0

Precondition:
    A.rows() == ambient_dimension()

Precondition:
    A.rows() == b.rows()

Precondition:
    A.cols() == x.size()

Precondition:
    c.rows() == t.size())""";
          } DoAddPointInNonnegativeScalingConstraints;
          // Symbol: drake::geometry::optimization::ConvexSet::DoAddPointInSetConstraints
          struct /* DoAddPointInSetConstraints */ {
            // Source: drake/geometry/optimization/convex_set.h
            const char* doc =
R"""(Non-virtual interface implementation for AddPointInSetConstraints().

Precondition:
    vars.size() == ambient_dimension()

Precondition:
    ambient_dimension() > 0)""";
          } DoAddPointInSetConstraints;
          // Symbol: drake::geometry::optimization::ConvexSet::DoAffineHullShortcut
          struct /* DoAffineHullShortcut */ {
            // Source: drake/geometry/optimization/convex_set.h
            const char* doc =
R"""(NVI implementation of DoAffineHullShortcut, which trivially returns
null. Derived classes that have efficient algorithms should override
this method.)""";
          } DoAffineHullShortcut;
          // Symbol: drake::geometry::optimization::ConvexSet::DoCalcVolume
          struct /* DoCalcVolume */ {
            // Source: drake/geometry/optimization/convex_set.h
            const char* doc =
R"""(Non-virtual interface implementation for CalcVolume(). This will
*only* be called if has_exact_volume() returns true and
ambient_dimension() > 0)""";
          } DoCalcVolume;
          // Symbol: drake::geometry::optimization::ConvexSet::DoClone
          struct /* DoClone */ {
            // Source: drake/geometry/optimization/convex_set.h
            const char* doc =
R"""(Non-virtual interface implementation for Clone().)""";
          } DoClone;
          // Symbol: drake::geometry::optimization::ConvexSet::DoIsBoundedShortcut
          struct /* DoIsBoundedShortcut */ {
            // Source: drake/geometry/optimization/convex_set.h
            const char* doc =
R"""(Non-virtual interface implementation for DoIsBoundedShortcut().
Trivially returns std∷nullopt. This allows a derived class to
implement its own boundedness checks, to potentially avoid the more
expensive base class checks.

Precondition:
    ambient_dimension() >= 0)""";
          } DoIsBoundedShortcut;
          // Symbol: drake::geometry::optimization::ConvexSet::DoIsBoundedShortcutParallel
          struct /* DoIsBoundedShortcutParallel */ {
            // Source: drake/geometry/optimization/convex_set.h
            const char* doc =
R"""(Non-virtual interface implementation for
DoIsBoundedShortcutParallel(). Trivially returns std∷nullopt. This
allows a derived class to implement its own boundedness checks that
leverage parallelization, to potentially avoid the more expensive base
class checks.

Precondition:
    ambient_dimension() >= 0)""";
          } DoIsBoundedShortcutParallel;
          // Symbol: drake::geometry::optimization::ConvexSet::DoIsEmpty
          struct /* DoIsEmpty */ {
            // Source: drake/geometry/optimization/convex_set.h
            const char* doc =
R"""(Non-virtual interface implementation for IsEmpty(). The default
implementation solves a feasibility optimization problem, but derived
classes can override with a custom (more efficient) implementation.
Zero-dimensional sets are considered to be nonempty by default. Sets
which can be zero-dimensional and empty must handle this behavior in
their derived implementation of DoIsEmpty.)""";
          } DoIsEmpty;
          // Symbol: drake::geometry::optimization::ConvexSet::DoMaybeGetFeasiblePoint
          struct /* DoMaybeGetFeasiblePoint */ {
            // Source: drake/geometry/optimization/convex_set.h
            const char* doc =
R"""(Non-virtual interface implementation for MaybeGetFeasiblePoint(). The
default implementation solves a feasibility optimization problem, but
derived classes can override with a custom (more efficient)
implementation.)""";
          } DoMaybeGetFeasiblePoint;
          // Symbol: drake::geometry::optimization::ConvexSet::DoMaybeGetPoint
          struct /* DoMaybeGetPoint */ {
            // Source: drake/geometry/optimization/convex_set.h
            const char* doc =
R"""(Non-virtual interface implementation for MaybeGetPoint(). The default
implementation returns nullopt. Sets that can model a single point
should override with a custom implementation.

Precondition:
    ambient_dimension() >= 0.)""";
          } DoMaybeGetPoint;
          // Symbol: drake::geometry::optimization::ConvexSet::DoPointInSet
          struct /* DoPointInSet */ {
            // Source: drake/geometry/optimization/convex_set.h
            const char* doc =
R"""(Non-virtual interface implementation for PointInSet().

Precondition:
    x.size() == ambient_dimension()

Precondition:
    ambient_dimension() >= 0)""";
          } DoPointInSet;
          // Symbol: drake::geometry::optimization::ConvexSet::DoPointInSetShortcut
          struct /* DoPointInSetShortcut */ {
            // Source: drake/geometry/optimization/convex_set.h
            const char* doc =
R"""(A non-virtual interface implementation for PointInSet() that should be
used when the PointInSet() can be computed more efficiently than
solving a convex program.

Returns:
    Returns true if and only if x is known to be in the set. Returns
    false if and only if x is known to not be in the set. Returns
    std∷nullopt if a shortcut implementation is not provided (i.e. the
    method has not elected to decide whether the point x is in the
    set).

For example, membership in a VPolytope cannot be verified without
solving a linear program and so no shortcut implementation should be
provided. On the other hand, membership in an HPolyhedron can be
checked by checking the inequality Ax ≤ b and so a shortcut is
possible.)""";
          } DoPointInSetShortcut;
          // Symbol: drake::geometry::optimization::ConvexSet::DoProjectionShortcut
          struct /* DoProjectionShortcut */ {
            // Source: drake/geometry/optimization/convex_set.h
            const char* doc =
R"""(Non-virtual interface implementation for DoProjectionShortcut().

This allows a derived class to implement a method which computes the
projection of some, but not necessarily all, of the ``points`` more
efficiently than the generic implementation.

The default implementation checks whether each column of ``points`` is
in the set using DoPointInSetShortcut. Points in the set are given a
distance of 0 and are projected to themselves.

Parameter ``points``:
    are the points which we wish to project to the convex set.

Parameter ``projected_points``:
    are the projection of ``points`` onto the convex set.

Returns:
    A vector ``distances`` which is the same size as
    ``points``.cols().These are the distances from ``points`` to the
    convex set. If distances[i] has a value, then
    projected_points->col(i) is the projection of points.col(i) onto
    the set. If distances[i] is nullopt, then the projection of
    points.col(i) has not yet been computed, and so the value at
    projected_points->col(i) is meaningless.

Precondition:
    ambient_dimension() >= 0

Precondition:
    distances.size() == points.cols())""";
          } DoProjectionShortcut;
          // Symbol: drake::geometry::optimization::ConvexSet::DoToShapeWithPose
          struct /* DoToShapeWithPose */ {
            // Source: drake/geometry/optimization/convex_set.h
            const char* doc =
R"""(Non-virtual interface implementation for ToShapeWithPose().

Precondition:
    ambient_dimension() == 3)""";
          } DoToShapeWithPose;
          // Symbol: drake::geometry::optimization::ConvexSet::HandleZeroAmbientDimensionConstraints
          struct /* HandleZeroAmbientDimensionConstraints */ {
            // Source: drake/geometry/optimization/convex_set.h
            const char* doc =
R"""(Instances of subclasses such as CartesianProduct and MinkowskiSum can
have constituent sets with zero ambient dimension, which much be
handled in a special manner when calling methods such as
DoAddPointInSetConstraints. If the set is empty, a trivially
infeasible constraint must be added. We also warn the user when this
happens, since they probably didn't intend it to occur. If the set is
nonempty, then it's the unique zero-dimensional vector space {0}, and
no additional variables or constraints are needed. If a new variable
is created, return it, to optionally be stored (as in
AddPointInSetConstraints), or not be stored (as in
DoAddPointInNonnegativeScalingConstraints).)""";
          } HandleZeroAmbientDimensionConstraints;
          // Symbol: drake::geometry::optimization::ConvexSet::IntersectsWith
          struct /* IntersectsWith */ {
            // Source: drake/geometry/optimization/convex_set.h
            const char* doc =
R"""(Returns true iff the intersection between ``this`` and ``other`` is
non-empty.

Raises:
    RuntimeError if the ambient dimension of ``other`` is not the same
    as that of ``this``.)""";
          } IntersectsWith;
          // Symbol: drake::geometry::optimization::ConvexSet::IsBounded
          struct /* IsBounded */ {
            // Source: drake/geometry/optimization/convex_set.h
            const char* doc =
R"""(Returns true iff the set is bounded, e.g., there exists an
element-wise finite lower and upper bound for the set. Note: for some
derived classes, this check is trivial, but for others it can require
solving a number of (typically small) optimization problems. Each
derived class documents the cost of its boundedness test and whether
it honors the request for parallelism. (Derived classes which do not
have a specialized check will, by default, honor parallelism
requests.) Note that the overhead of multithreading may lead to slower
runtimes for simple, low-dimensional sets, but can enable major
speedups for more challenging problems.

Parameter ``parallelism``:
    requests the number of cores to use when solving mathematical
    programs to check boundedness, subject to whether a particular
    derived class honors parallelism.)""";
          } IsBounded;
          // Symbol: drake::geometry::optimization::ConvexSet::IsEmpty
          struct /* IsEmpty */ {
            // Source: drake/geometry/optimization/convex_set.h
            const char* doc =
R"""(Returns true iff the set is empty. Note: for some derived classes,
this check is trivial, but for others, it can require solving a
(typically small) optimization problem. Check the derived class
documentation for any notes. Zero-dimensional sets must be handled
specially. There are two possible sets in a zero-dimensional space --
the empty set, and the whole set (which is simply the "zero vector
space", {0}.) For more details, see:
https://en.wikipedia.org/wiki/Examples_of_vector_spaces#Trivial_or_zero_vector_space
Zero-dimensional sets are considered to be nonempty by default. Sets
which can be zero-dimensional and empty must handle this behavior in
their derived implementation of DoIsEmpty. An example of such a
subclass is VPolytope.)""";
          } IsEmpty;
          // Symbol: drake::geometry::optimization::ConvexSet::MaybeGetFeasiblePoint
          struct /* MaybeGetFeasiblePoint */ {
            // Source: drake/geometry/optimization/convex_set.h
            const char* doc =
R"""(Returns a feasible point within this convex set if it is nonempty, and
nullopt otherwise.)""";
          } MaybeGetFeasiblePoint;
          // Symbol: drake::geometry::optimization::ConvexSet::MaybeGetPoint
          struct /* MaybeGetPoint */ {
            // Source: drake/geometry/optimization/convex_set.h
            const char* doc =
R"""(If this set trivially contains exactly one point, returns the value of
that point. Otherwise, returns nullopt. By "trivially", we mean that
representation of the set structurally maps to a single point; if
checking for point-ness would require solving an optimization program,
returns nullopt. In other words, this is a relatively cheap function
to call.)""";
          } MaybeGetPoint;
          // Symbol: drake::geometry::optimization::ConvexSet::PointInSet
          struct /* PointInSet */ {
            // Source: drake/geometry/optimization/convex_set.h
            const char* doc =
R"""(Returns true iff the point x is contained in the set. If the ambient
dimension is zero, then if the set is nonempty, the point is trivially
in the set, and if the set is empty, the point is trivially not in the
set.)""";
          } PointInSet;
          // Symbol: drake::geometry::optimization::ConvexSet::Projection
          struct /* Projection */ {
            // Source: drake/geometry/optimization/convex_set.h
            const char* doc =
R"""(Computes in the L₂ norm the distance and the nearest point in this
convex set to every column of ``points``. If this set is empty, we
return nullopt.

Precondition:
    points.rows() == ambient_dimension().

Raises:
    if the internal convex optimization solver fails.)""";
          } Projection;
          // Symbol: drake::geometry::optimization::ConvexSet::Serialize
          struct /* Serialize */ {
            // Source: drake/geometry/optimization/convex_set.h
            const char* doc =
R"""(Implements non-virtual base class serialization.)""";
          } Serialize;
          // Symbol: drake::geometry::optimization::ConvexSet::ToShapeWithPose
          struct /* ToShapeWithPose */ {
            // Source: drake/geometry/optimization/convex_set.h
            const char* doc =
R"""(Constructs a Shape and a pose of the set in the world frame for use in
the SceneGraph geometry ecosystem.

Raises:
    RuntimeError if ambient_dimension() != 3 or if the functionality
    for a particular set has not been implemented yet.)""";
          } ToShapeWithPose;
          // Symbol: drake::geometry::optimization::ConvexSet::ambient_dimension
          struct /* ambient_dimension */ {
            // Source: drake/geometry/optimization/convex_set.h
            const char* doc =
R"""(Returns the dimension of the vector space in which the elements of
this set are evaluated. Contrast this with the ``affine dimension``:
the dimension of the smallest affine subset of the ambient space that
contains our set. For example, if we define a set using ``A*x = b``,
where ``A`` has linearly independent rows, then the ambient dimension
is the dimension of ``x``, but the affine dimension of the set is
``ambient_dimension() - rank(A)``.)""";
          } ambient_dimension;
          // Symbol: drake::geometry::optimization::ConvexSet::has_exact_volume
          struct /* has_exact_volume */ {
            // Source: drake/geometry/optimization/convex_set.h
            const char* doc =
R"""(Returns true if the exact volume can be computed for this convex set
instance.

Note:
    This value reasons about to the generic case of the convex set
    class rather than the specific instance of the convex set. For
    example, the exact volume of a box is trivival to compute, but if
    the box is created as a HPolyhedron, then the exact volume cannot
    be computed.)""";
          } has_exact_volume;
        } ConvexSet;
        // Symbol: drake::geometry::optimization::ConvexSets
        struct /* ConvexSets */ {
          // Source: drake/geometry/optimization/convex_set.h
          const char* doc =
R"""(Provides the recommended container for passing a collection of
ConvexSet instances.)""";
        } ConvexSets;
        // Symbol: drake::geometry::optimization::CspaceFreeBox
        struct /* CspaceFreeBox */ {
          // Source: drake/geometry/optimization/cspace_free_box.h
          const char* doc =
R"""(This class tries to find large axis-aligned bounding boxes in the
configuration space, such that all configurations in the boxes are
collision free. Note that we don't guarantee to find the largest box.)""";
          // Symbol: drake::geometry::optimization::CspaceFreeBox::CspaceFreeBox
          struct /* ctor */ {
            // Source: drake/geometry/optimization/cspace_free_box.h
            const char* doc =
R"""(Parameter ``plant``:
    The plant for which we compute the C-space free boxes. It must
    outlive this CspaceFreeBox object.

Parameter ``scene_graph``:
    The scene graph that has been connected with ``plant``. It must
    outlive this CspaceFreeBox object.

Parameter ``plane_order``:
    The order of the polynomials in the plane to separate a pair of
    collision geometries.

Note:
    CspaceFreeBox knows nothing about contexts. The plant and
    scene_graph must be fully configured before instantiating this
    class.)""";
          } ctor;
          // Symbol: drake::geometry::optimization::CspaceFreeBox::FindSeparationCertificateGivenBox
          struct /* FindSeparationCertificateGivenBox */ {
            // Source: drake/geometry/optimization/cspace_free_box.h
            const char* doc =
R"""(Finds the certificates that the C-space box {q | q_box_lower <= q <=
q_box_upper} is collision free.

Parameter ``q_box_lower``:
    The lower bound of the C-space box.

Parameter ``q_box_upper``:
    The upper bound of the C-space box.

Parameter ``ignored_collision_pairs``:
    We ignore the pair of geometries in ``ignored_collision_pairs``.

Parameter ``certificates``:
    Contains the certificate we successfully found for each pair of
    geometries. Notice that depending on ``options``, the program
    could search for the certificate for each geometry pair in
    parallel, and will terminate the search once it fails to find the
    certificate for any pair. At termination, the pair of geometries
    whose optimization hasn't been finished will not show up in
    ``certificates``.

Returns ``success``:
    If true, then we have certified that the C-space box {q |
    q_box_lower<=q<=q_box_upper} is collision free. Otherwise
    success=false.)""";
          } FindSeparationCertificateGivenBox;
          // Symbol: drake::geometry::optimization::CspaceFreeBox::SeparatingPlaneLagrangians
          struct /* SeparatingPlaneLagrangians */ {
            // Source: drake/geometry/optimization/cspace_free_box.h
            const char* doc =
R"""(When searching for the separating plane, we want to certify that the
numerator of a rational is non-negative in the C-space box q_box_lower
<= q <= q_box_upper (or equivalently s_box_lower <= s <= s_box_upper).
Hence for each of the rational we will introduce Lagrangian
multipliers for the polytopic constraint s - s_box_lower >= 0,
s_box_upper - s >= 0.)""";
            // Symbol: drake::geometry::optimization::CspaceFreeBox::SeparatingPlaneLagrangians::GetSolution
            struct /* GetSolution */ {
              // Source: drake/geometry/optimization/cspace_free_box.h
              const char* doc =
R"""(Substitutes the decision variables in each Lagrangians with its value
in result, returns the substitution result.)""";
            } GetSolution;
            // Symbol: drake::geometry::optimization::CspaceFreeBox::SeparatingPlaneLagrangians::SeparatingPlaneLagrangians
            struct /* ctor */ {
              // Source: drake/geometry/optimization/cspace_free_box.h
              const char* doc = R"""()""";
            } ctor;
            // Symbol: drake::geometry::optimization::CspaceFreeBox::SeparatingPlaneLagrangians::mutable_s_box_lower
            struct /* mutable_s_box_lower */ {
              // Source: drake/geometry/optimization/cspace_free_box.h
              const char* doc =
R"""(The Lagrangians for s - s_box_lower >= 0.)""";
            } mutable_s_box_lower;
            // Symbol: drake::geometry::optimization::CspaceFreeBox::SeparatingPlaneLagrangians::mutable_s_box_upper
            struct /* mutable_s_box_upper */ {
              // Source: drake/geometry/optimization/cspace_free_box.h
              const char* doc =
R"""(The Lagrangians for s_box_upper - s >= 0.)""";
            } mutable_s_box_upper;
            // Symbol: drake::geometry::optimization::CspaceFreeBox::SeparatingPlaneLagrangians::s_box_lower
            struct /* s_box_lower */ {
              // Source: drake/geometry/optimization/cspace_free_box.h
              const char* doc =
R"""(The Lagrangians for s - s_box_lower >= 0.)""";
            } s_box_lower;
            // Symbol: drake::geometry::optimization::CspaceFreeBox::SeparatingPlaneLagrangians::s_box_upper
            struct /* s_box_upper */ {
              // Source: drake/geometry/optimization/cspace_free_box.h
              const char* doc =
R"""(The Lagrangians for s_box_upper - s >= 0.)""";
            } s_box_upper;
          } SeparatingPlaneLagrangians;
          // Symbol: drake::geometry::optimization::CspaceFreeBox::SeparationCertificate
          struct /* SeparationCertificate */ {
            // Source: drake/geometry/optimization/cspace_free_box.h
            const char* doc =
R"""(This struct stores the necessary information to search for the
separating plane for the polytopic C-space box q_box_lower <= q <=
q_box_upper. We need to impose that N rationals are non-negative in
this C-space box. The denominator of each rational is always positive
hence we need to impose the N numerators are non-negative in this
C-space box. We impose the condition numerator_i(s) - λ_lower(s)ᵀ * (s
- s_lower) -λ_upper(s)ᵀ * (s_upper - s) is sos λ_lower(s) are sos,
λ_upper(s) are sos.)""";
            // Symbol: drake::geometry::optimization::CspaceFreeBox::SeparationCertificate::GetSolution
            struct /* GetSolution */ {
              // Source: drake/geometry/optimization/cspace_free_box.h
              const char* doc = R"""()""";
            } GetSolution;
            // Symbol: drake::geometry::optimization::CspaceFreeBox::SeparationCertificate::SeparationCertificate
            struct /* ctor */ {
              // Source: drake/geometry/optimization/cspace_free_box.h
              const char* doc = R"""()""";
            } ctor;
            // Symbol: drake::geometry::optimization::CspaceFreeBox::SeparationCertificate::mutable_lagrangians
            struct /* mutable_lagrangians */ {
              // Source: drake/geometry/optimization/cspace_free_box.h
              const char* doc = R"""()""";
            } mutable_lagrangians;
            // Symbol: drake::geometry::optimization::CspaceFreeBox::SeparationCertificate::negative_side_rational_lagrangians
            struct /* negative_side_rational_lagrangians */ {
              // Source: drake/geometry/optimization/cspace_free_box.h
              const char* doc = R"""()""";
            } negative_side_rational_lagrangians;
            // Symbol: drake::geometry::optimization::CspaceFreeBox::SeparationCertificate::positive_side_rational_lagrangians
            struct /* positive_side_rational_lagrangians */ {
              // Source: drake/geometry/optimization/cspace_free_box.h
              const char* doc = R"""()""";
            } positive_side_rational_lagrangians;
          } SeparationCertificate;
          // Symbol: drake::geometry::optimization::CspaceFreeBox::SeparationCertificateProgram
          struct /* SeparationCertificateProgram */ {
            // Source: drake/geometry/optimization/cspace_free_box.h
            const char* doc = R"""()""";
            // Symbol: drake::geometry::optimization::CspaceFreeBox::SeparationCertificateProgram::SeparationCertificateProgram
            struct /* ctor */ {
              // Source: drake/geometry/optimization/cspace_free_box.h
              const char* doc = R"""()""";
            } ctor;
            // Symbol: drake::geometry::optimization::CspaceFreeBox::SeparationCertificateProgram::certificate
            struct /* certificate */ {
              // Source: drake/geometry/optimization/cspace_free_box.h
              const char* doc = R"""()""";
            } certificate;
          } SeparationCertificateProgram;
          // Symbol: drake::geometry::optimization::CspaceFreeBox::SeparationCertificateResult
          struct /* SeparationCertificateResult */ {
            // Source: drake/geometry/optimization/cspace_free_box.h
            const char* doc =
R"""(We certify that a pair of geometries is collision free in the C-space
box {q | q_box_lower<=q<=q_box_upper} by finding the separating plane
and the Lagrangian multipliers. This struct contains the certificate,
that the separating plane {x | aᵀx+b=0 } separates the two geometries
in separating_planes()[plane_index] in the C-space box.)""";
            // Symbol: drake::geometry::optimization::CspaceFreeBox::SeparationCertificateResult::SeparationCertificateResult
            struct /* ctor */ {
              // Source: drake/geometry/optimization/cspace_free_box.h
              const char* doc = R"""()""";
            } ctor;
            // Symbol: drake::geometry::optimization::CspaceFreeBox::SeparationCertificateResult::lagrangians
            struct /* lagrangians */ {
              // Source: drake/geometry/optimization/cspace_free_box.h
              const char* doc = R"""()""";
            } lagrangians;
            // Symbol: drake::geometry::optimization::CspaceFreeBox::SeparationCertificateResult::negative_side_rational_lagrangians
            struct /* negative_side_rational_lagrangians */ {
              // Source: drake/geometry/optimization/cspace_free_box.h
              const char* doc = R"""()""";
            } negative_side_rational_lagrangians;
            // Symbol: drake::geometry::optimization::CspaceFreeBox::SeparationCertificateResult::positive_side_rational_lagrangians
            struct /* positive_side_rational_lagrangians */ {
              // Source: drake/geometry/optimization/cspace_free_box.h
              const char* doc = R"""()""";
            } positive_side_rational_lagrangians;
          } SeparationCertificateResult;
        } CspaceFreeBox;
        // Symbol: drake::geometry::optimization::CspaceFreePolytope
        struct /* CspaceFreePolytope */ {
          // Source: drake/geometry/optimization/cspace_free_polytope.h
          const char* doc =
R"""(This class tries to find large convex polytopes in the
tangential-configuration space, such that all configurations in the
convex polytopes is collision free. By tangential-configuration space,
we mean the revolute joint angle θ is replaced by t = tan(θ/2). We
refer to the algorithm as C-IRIS. For more details, refer to the paper

Certified Polyhedral Decomposition of Collision-Free Configuration
Space by Hongkai Dai*, Alexandre Amice*, Peter Werner, Annan Zhang and
Russ Tedrake.

A conference version is published at

Finding and Optimizing Certified, Collision-Free Regions in
Configuration Space for Robot Manipulators by Alexandre Amice*,
Hongkai Dai*, Peter Werner, Annan Zhang and Russ Tedrake.)""";
          // Symbol: drake::geometry::optimization::CspaceFreePolytope::AddCspacePolytopeContainment
          struct /* AddCspacePolytopeContainment */ {
            // Source: drake/geometry/optimization/cspace_free_polytope.h
            const char* doc =
R"""(Adds the constraint that each column of s_inner_pts is in the polytope
{s | C*s<=d}.)""";
          } AddCspacePolytopeContainment;
          // Symbol: drake::geometry::optimization::CspaceFreePolytope::BilinearAlternationOptions
          struct /* BilinearAlternationOptions */ {
            // Source: drake/geometry/optimization/cspace_free_polytope.h
            const char* doc = R"""(Options for bilinear alternation.)""";
            // Symbol: drake::geometry::optimization::CspaceFreePolytope::BilinearAlternationOptions::convergence_tol
            struct /* convergence_tol */ {
              // Source: drake/geometry/optimization/cspace_free_polytope.h
              const char* doc =
R"""(When the change of the cost function between two consecutive
iterations in bilinear alternation is no larger than this number, stop
the bilinear alternation. Must be non-negative.)""";
            } convergence_tol;
            // Symbol: drake::geometry::optimization::CspaceFreePolytope::BilinearAlternationOptions::ellipsoid_scaling
            struct /* ellipsoid_scaling */ {
              // Source: drake/geometry/optimization/cspace_free_polytope.h
              const char* doc =
R"""(After finding the maximal inscribed ellipsoid in C-space polytope {s |
C*s<=d, s_lower<=s<=s_upper}, we scale this ellipsoid by
ellipsoid_scaling, and require the new C-space polytope to contain
this scaled ellipsoid. ellipsoid_scaling=1 corresponds to no scaling.
Must be strictly positive and no greater than 1.)""";
            } ellipsoid_scaling;
            // Symbol: drake::geometry::optimization::CspaceFreePolytope::BilinearAlternationOptions::find_lagrangian_options
            struct /* find_lagrangian_options */ {
              // Source: drake/geometry/optimization/cspace_free_polytope.h
              const char* doc = R"""()""";
            } find_lagrangian_options;
            // Symbol: drake::geometry::optimization::CspaceFreePolytope::BilinearAlternationOptions::find_polytope_options
            struct /* find_polytope_options */ {
              // Source: drake/geometry/optimization/cspace_free_polytope.h
              const char* doc = R"""()""";
            } find_polytope_options;
            // Symbol: drake::geometry::optimization::CspaceFreePolytope::BilinearAlternationOptions::max_iter
            struct /* max_iter */ {
              // Source: drake/geometry/optimization/cspace_free_polytope.h
              const char* doc =
R"""(The maximum number of bilinear alternation iterations. Must be
non-negative.)""";
            } max_iter;
          } BilinearAlternationOptions;
          // Symbol: drake::geometry::optimization::CspaceFreePolytope::BinarySearch
          struct /* BinarySearch */ {
            // Source: drake/geometry/optimization/cspace_free_polytope.h
            const char* doc =
R"""(Binary search on d such that the C-space polytope {s | C*s<=d,
s_lower<=s<=s_upper} is collision free. We scale the polytope {s |
C*s<=d_init} about its center ``s_center`` and search the scaling
factor.

Precondition:
    s_center is in the polytope {s | C*s<=d_init, s_lower<=s<=s_upper})""";
          } BinarySearch;
          // Symbol: drake::geometry::optimization::CspaceFreePolytope::BinarySearchOptions
          struct /* BinarySearchOptions */ {
            // Source: drake/geometry/optimization/cspace_free_polytope.h
            const char* doc = R"""(Options for binary search.)""";
            // Symbol: drake::geometry::optimization::CspaceFreePolytope::BinarySearchOptions::convergence_tol
            struct /* convergence_tol */ {
              // Source: drake/geometry/optimization/cspace_free_polytope.h
              const char* doc =
R"""(When the gap between the upper bound and the lower bound of the
scaling factor is below this ``convergence_tol``, stops the binary
search. Must be strictly positive.)""";
            } convergence_tol;
            // Symbol: drake::geometry::optimization::CspaceFreePolytope::BinarySearchOptions::find_lagrangian_options
            struct /* find_lagrangian_options */ {
              // Source: drake/geometry/optimization/cspace_free_polytope.h
              const char* doc = R"""()""";
            } find_lagrangian_options;
            // Symbol: drake::geometry::optimization::CspaceFreePolytope::BinarySearchOptions::max_iter
            struct /* max_iter */ {
              // Source: drake/geometry/optimization/cspace_free_polytope.h
              const char* doc =
R"""(The maximal number of iterations in binary search. Must be
non-negative.)""";
            } max_iter;
            // Symbol: drake::geometry::optimization::CspaceFreePolytope::BinarySearchOptions::scale_max
            struct /* scale_max */ {
              // Source: drake/geometry/optimization/cspace_free_polytope.h
              const char* doc =
R"""(The maximal value of the scaling factor. Must be finite and no less
than scale_min.)""";
            } scale_max;
            // Symbol: drake::geometry::optimization::CspaceFreePolytope::BinarySearchOptions::scale_min
            struct /* scale_min */ {
              // Source: drake/geometry/optimization/cspace_free_polytope.h
              const char* doc =
R"""(The minimal value of the scaling factor. Must be non-negative.)""";
            } scale_min;
          } BinarySearchOptions;
          // Symbol: drake::geometry::optimization::CspaceFreePolytope::CspaceFreePolytope
          struct /* ctor */ {
            // Source: drake/geometry/optimization/cspace_free_polytope.h
            const char* doc =
R"""(Parameter ``plant``:
    The plant for which we compute the C-space free polytopes. It must
    outlive this CspaceFreePolytope object.

Parameter ``scene_graph``:
    The scene graph that has been connected with ``plant``. It must
    outlive this CspaceFreePolytope object.

Parameter ``plane_order``:
    The order of the polynomials in the plane to separate a pair of
    collision geometries.

Parameter ``q_star``:
    Refer to RationalForwardKinematics for its meaning.

Note:
    CspaceFreePolytope knows nothing about contexts. The plant and
    scene_graph must be fully configured before instantiating this
    class.)""";
          } ctor;
          // Symbol: drake::geometry::optimization::CspaceFreePolytope::EllipsoidMarginCost
          struct /* EllipsoidMarginCost */ {
            // Source: drake/geometry/optimization/cspace_free_polytope.h
            const char* doc =
R"""(The cost used when fixing the Lagrangian multiplier and search for C
and d in the C-space polytope {s | C*s <=d, s_lower<=s<=s_upper}. We
denote δᵢ as the margin between the i'th face C.row(i)<=d(i) to the
inscribed ellipsoid.)""";
            // Symbol: drake::geometry::optimization::CspaceFreePolytope::EllipsoidMarginCost::kGeometricMean
            struct /* kGeometricMean */ {
              // Source: drake/geometry/optimization/cspace_free_polytope.h
              const char* doc =
R"""(Maximize the geometric mean power(∏ᵢ (δᵢ + ε), 1/n) where n is
C.rows(),
ε=FindPolytopeGivenLagrangianOptions.ellipsoid_margin_epsilon.)""";
            } kGeometricMean;
            // Symbol: drake::geometry::optimization::CspaceFreePolytope::EllipsoidMarginCost::kSum
            struct /* kSum */ {
              // Source: drake/geometry/optimization/cspace_free_polytope.h
              const char* doc = R"""(Maximize ∑ᵢδᵢ)""";
            } kSum;
          } EllipsoidMarginCost;
          // Symbol: drake::geometry::optimization::CspaceFreePolytope::FindPolytopeGivenLagrangianOptions
          struct /* FindPolytopeGivenLagrangianOptions */ {
            // Source: drake/geometry/optimization/cspace_free_polytope.h
            const char* doc =
R"""(Options for finding polytope with given Lagrangians.)""";
            // Symbol: drake::geometry::optimization::CspaceFreePolytope::FindPolytopeGivenLagrangianOptions::FindPolytopeGivenLagrangianOptions
            struct /* ctor */ {
              // Source: drake/geometry/optimization/cspace_free_polytope.h
              const char* doc = R"""()""";
            } ctor;
            // Symbol: drake::geometry::optimization::CspaceFreePolytope::FindPolytopeGivenLagrangianOptions::backoff_scale
            struct /* backoff_scale */ {
              // Source: drake/geometry/optimization/cspace_free_polytope.h
              const char* doc = R"""()""";
            } backoff_scale;
            // Symbol: drake::geometry::optimization::CspaceFreePolytope::FindPolytopeGivenLagrangianOptions::ellipsoid_margin_cost
            struct /* ellipsoid_margin_cost */ {
              // Source: drake/geometry/optimization/cspace_free_polytope.h
              const char* doc = R"""(Type of cost on the ellipsoid margin)""";
            } ellipsoid_margin_cost;
            // Symbol: drake::geometry::optimization::CspaceFreePolytope::FindPolytopeGivenLagrangianOptions::ellipsoid_margin_epsilon
            struct /* ellipsoid_margin_epsilon */ {
              // Source: drake/geometry/optimization/cspace_free_polytope.h
              const char* doc =
R"""(We will maximize the cost ∏ᵢ (δᵢ + ε) where δᵢ is the margin from each
face of the polytope {s | Cs<=d} to the inscribed ellipsoid, ε is
ellipsoid_margin_epsilon, a small positive constant to make sure δᵢ +
ε being strictly positive.)""";
            } ellipsoid_margin_epsilon;
            // Symbol: drake::geometry::optimization::CspaceFreePolytope::FindPolytopeGivenLagrangianOptions::s_inner_pts
            struct /* s_inner_pts */ {
              // Source: drake/geometry/optimization/cspace_free_polytope.h
              const char* doc =
R"""(We can constrain the C-space polytope {s | C*s<=d,
s_lower<=s<=s_upper} to contain some sampled s. Each column of
s_inner_pts is a sample of s.)""";
            } s_inner_pts;
            // Symbol: drake::geometry::optimization::CspaceFreePolytope::FindPolytopeGivenLagrangianOptions::search_s_bounds_lagrangians
            struct /* search_s_bounds_lagrangians */ {
              // Source: drake/geometry/optimization/cspace_free_polytope.h
              const char* doc =
R"""(If set to true, then we will also search for the Lagrangian
multipliers for the constraint s_lower <= s <= s_upper; otherwise we
fix the Lagrangian multiplier to the solution found when we fix the
C-space polytope {s | C*s<=d, s_lower<=s<=s_upper}.)""";
            } search_s_bounds_lagrangians;
            // Symbol: drake::geometry::optimization::CspaceFreePolytope::FindPolytopeGivenLagrangianOptions::solver_id
            struct /* solver_id */ {
              // Source: drake/geometry/optimization/cspace_free_polytope.h
              const char* doc = R"""(ID for the solver)""";
            } solver_id;
            // Symbol: drake::geometry::optimization::CspaceFreePolytope::FindPolytopeGivenLagrangianOptions::solver_options
            struct /* solver_options */ {
              // Source: drake/geometry/optimization/cspace_free_polytope.h
              const char* doc =
R"""(options for solving the MathematicalProgram)""";
            } solver_options;
          } FindPolytopeGivenLagrangianOptions;
          // Symbol: drake::geometry::optimization::CspaceFreePolytope::FindSeparationCertificateGivenPolytope
          struct /* FindSeparationCertificateGivenPolytope */ {
            // Source: drake/geometry/optimization/cspace_free_polytope.h
            const char* doc =
R"""(Finds the certificates that the C-space polytope {s | C*s<=d, s_lower
<= s <= s_upper} is collision free.

Parameter ``C``:
    The C-space polytope is {s | C*s<=d, s_lower<=s<=s_upper}

Parameter ``d``:
    The C-space polytope is {s | C*s<=d, s_lower<=s<=s_upper}

Parameter ``ignored_collision_pairs``:
    We will ignore the pair of geometries in
    ``ignored_collision_pairs``.

Parameter ``certificates``:
    Contains the certificate we successfully found for each pair of
    geometries. Notice that depending on ``options``, the program
    could search for the certificate for each geometry pair in
    parallel, and will terminate the search once it fails to find the
    certificate for any pair.

Returns ``success``:
    If true, then we have certified that the C-space polytope {s |
    C*s<=d, s_lower<=s<=s_upper} is collision free. Otherwise
    success=false.)""";
          } FindSeparationCertificateGivenPolytope;
          // Symbol: drake::geometry::optimization::CspaceFreePolytope::FindSeparationCertificateGivenPolytopeOptions
          struct /* FindSeparationCertificateGivenPolytopeOptions */ {
            // Source: drake/geometry/optimization/cspace_free_polytope.h
            const char* doc = R"""()""";
            // Symbol: drake::geometry::optimization::CspaceFreePolytope::FindSeparationCertificateGivenPolytopeOptions::ignore_redundant_C
            struct /* ignore_redundant_C */ {
              // Source: drake/geometry/optimization/cspace_free_polytope.h
              const char* doc = R"""()""";
            } ignore_redundant_C;
          } FindSeparationCertificateGivenPolytopeOptions;
          // Symbol: drake::geometry::optimization::CspaceFreePolytope::InitializePolytopeSearchProgram
          struct /* InitializePolytopeSearchProgram */ {
            // Source: drake/geometry/optimization/cspace_free_polytope.h
            const char* doc =
R"""(Constructs a program to search for the C-space polytope {s | C*s<=d,
s_lower<=s<=s_upper} such that this polytope is collision free. This
program treats C and d as decision variables, and searches for the
separating planes between each pair of geometries. Note that this
program doesn't contain any cost yet.

Parameter ``certificates``:
    The return of FindSeparationCertificateGivenPolytope().

Parameter ``search_s_bounds_lagrangians``:
    Set to true if we search for the Lagrangian multiplier for the
    bounds s_lower <=s<=s_upper.

Parameter ``C``:
    The C-space polytope is parameterized as {s | C*s<=d,
    s_lower<=s<=s_upper}.

Parameter ``d``:
    The C-space polytope is parameterized as {s | C*s<=d,
    s_lower<=s<=s_upper}.

Parameter ``new_certificates``:
    The new certificates to certify the new C-space polytope {s |
    C*s<=d, s_lower<=s<=s_upper} is collision free. If
    new_certificates=nullptr, then we don't update it. This is used
    for testing.)""";
          } InitializePolytopeSearchProgram;
          // Symbol: drake::geometry::optimization::CspaceFreePolytope::MakeIsGeometrySeparableProgram
          struct /* MakeIsGeometrySeparableProgram */ {
            // Source: drake/geometry/optimization/cspace_free_polytope.h
            const char* doc =
R"""(Constructs the MathematicalProgram which searches for a separation
certificate for a pair of geometries for a C-space polytope. Search
for the separation certificate for a pair of geometries for a C-space
polytope {s | C*s<=d, s_lower<=s<=s_upper}.

Raises:
    an error if this ``geometry_pair`` doesn't need separation
    certificate (for example, they are on the same body).)""";
          } MakeIsGeometrySeparableProgram;
          // Symbol: drake::geometry::optimization::CspaceFreePolytope::SearchResult
          struct /* SearchResult */ {
            // Source: drake/geometry/optimization/cspace_free_polytope.h
            const char* doc =
R"""(Result on searching the C-space polytope and separating planes.)""";
            // Symbol: drake::geometry::optimization::CspaceFreePolytope::SearchResult::C
            struct /* C */ {
              // Source: drake/geometry/optimization/cspace_free_polytope.h
              const char* doc = R"""()""";
            } C;
            // Symbol: drake::geometry::optimization::CspaceFreePolytope::SearchResult::SearchResult
            struct /* ctor */ {
              // Source: drake/geometry/optimization/cspace_free_polytope.h
              const char* doc = R"""()""";
            } ctor;
            // Symbol: drake::geometry::optimization::CspaceFreePolytope::SearchResult::a
            struct /* a */ {
              // Source: drake/geometry/optimization/cspace_free_polytope.h
              const char* doc =
R"""(Each plane index is mapped to a vector of polynomials.)""";
            } a;
            // Symbol: drake::geometry::optimization::CspaceFreePolytope::SearchResult::b
            struct /* b */ {
              // Source: drake/geometry/optimization/cspace_free_polytope.h
              const char* doc =
R"""(Each plane index is mapped to a polynomial,)""";
            } b;
            // Symbol: drake::geometry::optimization::CspaceFreePolytope::SearchResult::certified_polytope
            struct /* certified_polytope */ {
              // Source: drake/geometry/optimization/cspace_free_polytope.h
              const char* doc = R"""()""";
            } certified_polytope;
            // Symbol: drake::geometry::optimization::CspaceFreePolytope::SearchResult::d
            struct /* d */ {
              // Source: drake/geometry/optimization/cspace_free_polytope.h
              const char* doc = R"""()""";
            } d;
            // Symbol: drake::geometry::optimization::CspaceFreePolytope::SearchResult::num_iter
            struct /* num_iter */ {
              // Source: drake/geometry/optimization/cspace_free_polytope.h
              const char* doc =
R"""(The number of iterations taken to search for the result.)""";
            } num_iter;
          } SearchResult;
          // Symbol: drake::geometry::optimization::CspaceFreePolytope::SearchWithBilinearAlternation
          struct /* SearchWithBilinearAlternation */ {
            // Source: drake/geometry/optimization/cspace_free_polytope.h
            const char* doc =
R"""(Search for a collision-free C-space polytope. {s | C*s<=d,
s_lower<=s<=s_upper} through bilinear alternation. The goal is to
maximize the volume the C-space polytope. Since we can't compute the
polytope volume in the closed form, we use the volume of the maximal
inscribed ellipsoid as a surrogate function of the polytope volume.

Parameter ``ignored_collision_pairs``:
    The pairs of geometries that we ignore when searching for
    separation certificates.

Parameter ``C_init``:
    The initial value of C.

Parameter ``d_init``:
    The initial value of d.

Parameter ``options``:
    The options for the bilinear alternation.

Returns ``results``:
    Stores the certification result in each iteration of the bilinear
    alternation.)""";
          } SearchWithBilinearAlternation;
          // Symbol: drake::geometry::optimization::CspaceFreePolytope::SeparatingPlaneLagrangians
          struct /* SeparatingPlaneLagrangians */ {
            // Source: drake/geometry/optimization/cspace_free_polytope.h
            const char* doc =
R"""(When searching for the separating plane, we want to certify that the
numerator of a rational is non-negative in the C-space region C*s<=d,
s_lower <= s <= s_upper. Hence for each of the rational we will
introduce Lagrangian multipliers for the polytopic constraint d-C*s >=
0, s - s_lower >= 0, s_upper - s >= 0.)""";
            // Symbol: drake::geometry::optimization::CspaceFreePolytope::SeparatingPlaneLagrangians::GetSolution
            struct /* GetSolution */ {
              // Source: drake/geometry/optimization/cspace_free_polytope.h
              const char* doc =
R"""(Substitutes the decision variables in each Lagrangians with its value
in result, returns the substitution result.)""";
            } GetSolution;
            // Symbol: drake::geometry::optimization::CspaceFreePolytope::SeparatingPlaneLagrangians::SeparatingPlaneLagrangians
            struct /* ctor */ {
              // Source: drake/geometry/optimization/cspace_free_polytope.h
              const char* doc = R"""()""";
            } ctor;
            // Symbol: drake::geometry::optimization::CspaceFreePolytope::SeparatingPlaneLagrangians::mutable_polytope
            struct /* mutable_polytope */ {
              // Source: drake/geometry/optimization/cspace_free_polytope.h
              const char* doc = R"""(The Lagrangians for d - C*s >= 0.)""";
            } mutable_polytope;
            // Symbol: drake::geometry::optimization::CspaceFreePolytope::SeparatingPlaneLagrangians::mutable_s_lower
            struct /* mutable_s_lower */ {
              // Source: drake/geometry/optimization/cspace_free_polytope.h
              const char* doc = R"""(The Lagrangians for s - s_lower >= 0.)""";
            } mutable_s_lower;
            // Symbol: drake::geometry::optimization::CspaceFreePolytope::SeparatingPlaneLagrangians::mutable_s_upper
            struct /* mutable_s_upper */ {
              // Source: drake/geometry/optimization/cspace_free_polytope.h
              const char* doc = R"""(The Lagrangians for s_upper - s >= 0.)""";
            } mutable_s_upper;
            // Symbol: drake::geometry::optimization::CspaceFreePolytope::SeparatingPlaneLagrangians::polytope
            struct /* polytope */ {
              // Source: drake/geometry/optimization/cspace_free_polytope.h
              const char* doc = R"""(The Lagrangians for d - C*s >= 0.)""";
            } polytope;
            // Symbol: drake::geometry::optimization::CspaceFreePolytope::SeparatingPlaneLagrangians::s_lower
            struct /* s_lower */ {
              // Source: drake/geometry/optimization/cspace_free_polytope.h
              const char* doc = R"""(The Lagrangians for s - s_lower >= 0.)""";
            } s_lower;
            // Symbol: drake::geometry::optimization::CspaceFreePolytope::SeparatingPlaneLagrangians::s_upper
            struct /* s_upper */ {
              // Source: drake/geometry/optimization/cspace_free_polytope.h
              const char* doc = R"""(The Lagrangians for s_upper - s >= 0.)""";
            } s_upper;
          } SeparatingPlaneLagrangians;
          // Symbol: drake::geometry::optimization::CspaceFreePolytope::SeparationCertificate
          struct /* SeparationCertificate */ {
            // Source: drake/geometry/optimization/cspace_free_polytope.h
            const char* doc =
R"""(This struct stores the necessary information to search for the
separating plane for the polytopic C-space region C*s <= d, s_lower <=
s <= s_upper. We need to impose that N rationals are non-negative in
this C-space polytope. The denominator of each rational is always
positive hence we need to impose the N numerators are non-negative in
this C-space region. We impose the condition numerator_i(s) - λ(s)ᵀ *
(d - C*s) - λ_lower(s)ᵀ * (s - s_lower) -λ_upper(s)ᵀ * (s_upper - s)
is sos λ(s) are sos, λ_lower(s) are sos, λ_upper(s) are sos.)""";
            // Symbol: drake::geometry::optimization::CspaceFreePolytope::SeparationCertificate::GetSolution
            struct /* GetSolution */ {
              // Source: drake/geometry/optimization/cspace_free_polytope.h
              const char* doc = R"""()""";
            } GetSolution;
            // Symbol: drake::geometry::optimization::CspaceFreePolytope::SeparationCertificate::SeparationCertificate
            struct /* ctor */ {
              // Source: drake/geometry/optimization/cspace_free_polytope.h
              const char* doc = R"""()""";
            } ctor;
            // Symbol: drake::geometry::optimization::CspaceFreePolytope::SeparationCertificate::mutable_lagrangians
            struct /* mutable_lagrangians */ {
              // Source: drake/geometry/optimization/cspace_free_polytope.h
              const char* doc = R"""()""";
            } mutable_lagrangians;
            // Symbol: drake::geometry::optimization::CspaceFreePolytope::SeparationCertificate::negative_side_rational_lagrangians
            struct /* negative_side_rational_lagrangians */ {
              // Source: drake/geometry/optimization/cspace_free_polytope.h
              const char* doc = R"""()""";
            } negative_side_rational_lagrangians;
            // Symbol: drake::geometry::optimization::CspaceFreePolytope::SeparationCertificate::positive_side_rational_lagrangians
            struct /* positive_side_rational_lagrangians */ {
              // Source: drake/geometry/optimization/cspace_free_polytope.h
              const char* doc = R"""()""";
            } positive_side_rational_lagrangians;
          } SeparationCertificate;
          // Symbol: drake::geometry::optimization::CspaceFreePolytope::SeparationCertificateProgram
          struct /* SeparationCertificateProgram */ {
            // Source: drake/geometry/optimization/cspace_free_polytope.h
            const char* doc = R"""()""";
            // Symbol: drake::geometry::optimization::CspaceFreePolytope::SeparationCertificateProgram::SeparationCertificateProgram
            struct /* ctor */ {
              // Source: drake/geometry/optimization/cspace_free_polytope.h
              const char* doc = R"""()""";
            } ctor;
            // Symbol: drake::geometry::optimization::CspaceFreePolytope::SeparationCertificateProgram::certificate
            struct /* certificate */ {
              // Source: drake/geometry/optimization/cspace_free_polytope.h
              const char* doc = R"""()""";
            } certificate;
          } SeparationCertificateProgram;
          // Symbol: drake::geometry::optimization::CspaceFreePolytope::SeparationCertificateResult
          struct /* SeparationCertificateResult */ {
            // Source: drake/geometry/optimization/cspace_free_polytope.h
            const char* doc =
R"""(We certify that a pair of geometries is collision free in the C-space
region {s | Cs<=d, s_lower<=s<=s_upper} by finding the separating
plane and the Lagrangian multipliers. This struct contains the
certificate, that the separating plane {x | aᵀx+b=0 } separates the
two geometries in separating_planes()[plane_index] in the C-space
polytope.)""";
            // Symbol: drake::geometry::optimization::CspaceFreePolytope::SeparationCertificateResult::SeparationCertificateResult
            struct /* ctor */ {
              // Source: drake/geometry/optimization/cspace_free_polytope.h
              const char* doc = R"""()""";
            } ctor;
            // Symbol: drake::geometry::optimization::CspaceFreePolytope::SeparationCertificateResult::lagrangians
            struct /* lagrangians */ {
              // Source: drake/geometry/optimization/cspace_free_polytope.h
              const char* doc = R"""()""";
            } lagrangians;
            // Symbol: drake::geometry::optimization::CspaceFreePolytope::SeparationCertificateResult::negative_side_rational_lagrangians
            struct /* negative_side_rational_lagrangians */ {
              // Source: drake/geometry/optimization/cspace_free_polytope.h
              const char* doc = R"""()""";
            } negative_side_rational_lagrangians;
            // Symbol: drake::geometry::optimization::CspaceFreePolytope::SeparationCertificateResult::positive_side_rational_lagrangians
            struct /* positive_side_rational_lagrangians */ {
              // Source: drake/geometry/optimization/cspace_free_polytope.h
              const char* doc = R"""()""";
            } positive_side_rational_lagrangians;
          } SeparationCertificateResult;
          // Symbol: drake::geometry::optimization::CspaceFreePolytope::SolveSeparationCertificateProgram
          struct /* SolveSeparationCertificateProgram */ {
            // Source: drake/geometry/optimization/cspace_free_polytope.h
            const char* doc =
R"""(Solves a SeparationCertificateProgram with the given options

Returns:
    result If we find the separation certificate, then ``result``
    contains the separation plane and the Lagrangian polynomials;
    otherwise result is empty.)""";
          } SolveSeparationCertificateProgram;
        } CspaceFreePolytope;
        // Symbol: drake::geometry::optimization::CspaceFreePolytopeBase
        struct /* CspaceFreePolytopeBase */ {
          // Source: drake/geometry/optimization/cspace_free_polytope_base.h
          const char* doc =
R"""(This virtual class is the base of CspaceFreePolytope and
CspaceFreeBox. We take the common functionality between these concrete
derived class to this shared parent class.)""";
          // Symbol: drake::geometry::optimization::CspaceFreePolytopeBase::CalcSBoundsPolynomial
          struct /* CalcSBoundsPolynomial */ {
            // Source: drake/geometry/optimization/cspace_free_polytope_base.h
            const char* doc =
R"""(Computes s-s_lower and s_upper - s as polynomials of s.)""";
          } CalcSBoundsPolynomial;
          // Symbol: drake::geometry::optimization::CspaceFreePolytopeBase::CspaceFreePolytopeBase
          struct /* ctor */ {
            // Source: drake/geometry/optimization/cspace_free_polytope_base.h
            const char* doc =
R"""(Constructor. We put the constructor in protected method to make sure
that the user cannot instantiate a CspaceFreePolytopeBase instance.

Precondition:
    plant and scene_graph should be non-null pointers.)""";
          } ctor;
          // Symbol: drake::geometry::optimization::CspaceFreePolytopeBase::GetGramVarSizeForPolytopeSearchProgram
          struct /* GetGramVarSizeForPolytopeSearchProgram */ {
            // Source: drake/geometry/optimization/cspace_free_polytope_base.h
            const char* doc =
R"""(Get the total size of all the decision variables for the Gram
matrices, so as to search for the polytope with given Lagrangian
multipliers.

Parameter ``plane_geometries_vec``:
    This struct contains the information on the rationals that we need
    to certify, so as to prove the existence of separating planes.

Parameter ``ignored_collision_pairs``:
    The collision pairs that we ignore.

Parameter ``count_gram_per_rational``:
    We will impose the sos condition that certain rational is always
    non-negative within a semialgebraic set. This function returns the
    number of variables in the Gram matrices for this rational.)""";
          } GetGramVarSizeForPolytopeSearchProgram;
          // Symbol: drake::geometry::optimization::CspaceFreePolytopeBase::GetSForPlane
          struct /* GetSForPlane */ {
            // Source: drake/geometry/optimization/cspace_free_polytope_base.h
            const char* doc =
R"""(Returns a vector of s variable used in a(s), b(s), which parameterize
the separating plane {x | a(s)ᵀx+b(s) = 0}.)""";
          } GetSForPlane;
          // Symbol: drake::geometry::optimization::CspaceFreePolytopeBase::GetSeparatingPlaneIndex
          struct /* GetSeparatingPlaneIndex */ {
            // Source: drake/geometry/optimization/cspace_free_polytope_base.h
            const char* doc =
R"""(Returns the index of the plane which will separate the geometry pair.
Returns -1 if the pair is not in map_geometries_to_separating_planes_.)""";
          } GetSeparatingPlaneIndex;
          // Symbol: drake::geometry::optimization::CspaceFreePolytopeBase::IgnoredCollisionPairs
          struct /* IgnoredCollisionPairs */ {
            // Source: drake/geometry/optimization/cspace_free_polytope_base.h
            const char* doc = R"""()""";
          } IgnoredCollisionPairs;
          // Symbol: drake::geometry::optimization::CspaceFreePolytopeBase::Options
          struct /* Options */ {
            // Source: drake/geometry/optimization/cspace_free_polytope_base.h
            const char* doc =
R"""(Optional argument for constructing CspaceFreePolytopeBase)""";
            // Symbol: drake::geometry::optimization::CspaceFreePolytopeBase::Options::Options
            struct /* ctor */ {
              // Source: drake/geometry/optimization/cspace_free_polytope_base.h
              const char* doc = R"""()""";
            } ctor;
            // Symbol: drake::geometry::optimization::CspaceFreePolytopeBase::Options::Serialize
            struct /* Serialize */ {
              // Source: drake/geometry/optimization/cspace_free_polytope_base.h
              const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
            } Serialize;
            // Symbol: drake::geometry::optimization::CspaceFreePolytopeBase::Options::with_cross_y
            struct /* with_cross_y */ {
              // Source: drake/geometry/optimization/cspace_free_polytope_base.h
              const char* doc =
R"""(For non-polytopic collision geometries, we will impose a matrix-sos
constraint X(s) being psd, with a slack indeterminates y, such that
the polynomial


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    p(s, y) = ⌈ 1 ⌉ᵀ * X(s) * ⌈ 1 ⌉
    ⌊ y ⌋           ⌊ y ⌋

.. raw:: html

    </details>

is positive. This p(s, y) polynomial doesn't contain the cross term of
y (namely it doesn't have y(i)*y(j), i≠j). When we select the monomial
basis for this polynomial, we can also exclude the cross term of y in
the monomial basis.

To illustrate the idea, let's consider the following toy example: if
we want to certify that a(0) + a(1)*y₀ + a(2)*y₁ + a(3)*y₀² + a(4)*y₁²
is positive (this polynomial doesn't have the cross term y₀*y₁), we
can write it as


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    ⌈ 1⌉ᵀ * A₀ * ⌈ 1⌉ + ⌈ 1⌉ᵀ * A₁ * ⌈ 1⌉
    ⌊y₀⌋         ⌊y₀⌋   ⌊y₁⌋         ⌊y₁⌋

.. raw:: html

    </details>

with two small psd matrices A₀, A₁ Instead of


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    ⌈ 1⌉ᵀ * A * ⌈ 1⌉
    |y₀|        |y₀|
    ⌊y₁⌋        ⌊y₁⌋

.. raw:: html

    </details>

with one large psd matrix A. The first parameterization won't have the
cross term y₀*y₁ by construction, while the second parameterization
requires imposing extra constraints on certain off-diagonal terms in A
so that the cross term vanishes.

If we set with_cross_y = false, then we will use the monomial basis
that doesn't generate cross terms of y, leading to smaller size sos
problems. If we set with_cross_y = true, then we will use the monomial
basis that will generate cross terms of y, causing larger size sos
problems, but possibly able to certify a larger C-space polytope.)""";
            } with_cross_y;
            auto Serialize__fields() const {
              return std::array{
                std::make_pair("with_cross_y", with_cross_y.doc),
              };
            }
          } Options;
          // Symbol: drake::geometry::optimization::CspaceFreePolytopeBase::SForPlane
          struct /* SForPlane */ {
            // Source: drake/geometry/optimization/cspace_free_polytope_base.h
            const char* doc =
R"""(When we set up the separating plane {x | a(s)ᵀx + b(s) = 0} between a
pair of geometries, we need to determine which s are used in a(s) and
b(s).)""";
            // Symbol: drake::geometry::optimization::CspaceFreePolytopeBase::SForPlane::kAll
            struct /* kAll */ {
              // Source: drake/geometry/optimization/cspace_free_polytope_base.h
              const char* doc =
R"""(Use all s in the robot tangent-configuration space.)""";
            } kAll;
            // Symbol: drake::geometry::optimization::CspaceFreePolytopeBase::SForPlane::kOnChain
            struct /* kOnChain */ {
              // Source: drake/geometry/optimization/cspace_free_polytope_base.h
              const char* doc =
R"""(Use s on the kinematics chain between the pair of geometries.)""";
            } kOnChain;
          } SForPlane;
          // Symbol: drake::geometry::optimization::CspaceFreePolytopeBase::SolveCertificationForEachPlaneInParallel
          struct /* SolveCertificationForEachPlaneInParallel */ {
            // Source: drake/geometry/optimization/cspace_free_polytope_base.h
            const char* doc =
R"""(For each pair of geometries, solve the certification problem to find
their separation plane in parallel.

Parameter ``active_plane_indices``:
    We will search for the plane in
    this->separating_planes()[active_plane_indices[i]].

Parameter ``solve_plane_sos``:
    The solve_plane_sos(plane_count) returns the pair (is_success,
    plane_count), where is_success indicates whether the solve for
    this->separating_planes()[active_plane_indices[plane_count]] is
    successful or not. This function returns the input plane_count as
    one of the output. This is because when we access the return value
    of solve_small_sos, we need to know the plane_count, and the
    return value and the input ``plane_count`` live in different part
    of the code due to multi-threading.

Parameter ``parallelism``:
    The number of threads in the parallel solve.

Parameter ``verbose``:
    Whether to print out some messages during the parallel solve.

Parameter ``terminate_at_failure``:
    If set to true, then terminate this function when we failed to
    find a separating plane.)""";
          } SolveCertificationForEachPlaneInParallel;
          // Symbol: drake::geometry::optimization::CspaceFreePolytopeBase::get_s_set
          struct /* get_s_set */ {
            // Source: drake/geometry/optimization/cspace_free_polytope_base.h
            const char* doc = R"""()""";
          } get_s_set;
          // Symbol: drake::geometry::optimization::CspaceFreePolytopeBase::link_geometries
          struct /* link_geometries */ {
            // Source: drake/geometry/optimization/cspace_free_polytope_base.h
            const char* doc = R"""()""";
          } link_geometries;
          // Symbol: drake::geometry::optimization::CspaceFreePolytopeBase::map_body_pair_to_s_on_chain
          struct /* map_body_pair_to_s_on_chain */ {
            // Source: drake/geometry/optimization/cspace_free_polytope_base.h
            const char* doc =
R"""(For a pair of bodies body_pair, returns the indices of all s on the
kinematics chain from body_pair.first() to body_pair.second(). For
each pair of collidable collision geometry (A, B), we denote their
body as (bodyA, bodyB). This keys in this map include all these
(bodyA, bodyB), together with (body_middle, bodyA) and (body_middle,
bodyB), where body_middle is the body in the middle of the kinematics
chain between bodyA and bodyB.)""";
          } map_body_pair_to_s_on_chain;
          // Symbol: drake::geometry::optimization::CspaceFreePolytopeBase::map_body_to_monomial_basis_array
          struct /* map_body_to_monomial_basis_array */ {
            // Source: drake/geometry/optimization/cspace_free_polytope_base.h
            const char* doc =
R"""(Maps a pair of body (body1, body2) to an array of monomial basis
``monomial_basis_array``. monomial_basis_array[0] contains all the
monomials of form ∏ᵢ pow(sᵢ, dᵢ), dᵢ=0 or 1, sᵢ correspond to the
revolute/prismatic joint on the kinematic chain between body1 and
body2. monomial_basis_array[i+1] = y_slack_[i] *
monomial_basis_array[0])""";
          } map_body_to_monomial_basis_array;
          // Symbol: drake::geometry::optimization::CspaceFreePolytopeBase::map_geometries_to_separating_planes
          struct /* map_geometries_to_separating_planes */ {
            // Source: drake/geometry/optimization/cspace_free_polytope_base.h
            const char* doc =
R"""(separating_planes()[map_geometries_to_separating_planes.at(geometry1_id,
geometry2_id)] is the separating plane that separates geometry 1 and
geometry 2.)""";
          } map_geometries_to_separating_planes;
          // Symbol: drake::geometry::optimization::CspaceFreePolytopeBase::plane_order
          struct /* plane_order */ {
            // Source: drake/geometry/optimization/cspace_free_polytope_base.h
            const char* doc = R"""()""";
          } plane_order;
          // Symbol: drake::geometry::optimization::CspaceFreePolytopeBase::rational_forward_kin
          struct /* rational_forward_kin */ {
            // Source: drake/geometry/optimization/cspace_free_polytope_base.h
            const char* doc =
R"""(Getter for the rational forward kinematics object that computes the
forward kinematics as rational functions.)""";
          } rational_forward_kin;
          // Symbol: drake::geometry::optimization::CspaceFreePolytopeBase::scene_graph
          struct /* scene_graph */ {
            // Source: drake/geometry/optimization/cspace_free_polytope_base.h
            const char* doc = R"""()""";
          } scene_graph;
          // Symbol: drake::geometry::optimization::CspaceFreePolytopeBase::separating_planes
          struct /* separating_planes */ {
            // Source: drake/geometry/optimization/cspace_free_polytope_base.h
            const char* doc =
R"""(All the separating planes between each pair of geometries.)""";
          } separating_planes;
          // Symbol: drake::geometry::optimization::CspaceFreePolytopeBase::with_cross_y
          struct /* with_cross_y */ {
            // Source: drake/geometry/optimization/cspace_free_polytope_base.h
            const char* doc =
R"""(Check Options∷with_cross_y for more details.)""";
          } with_cross_y;
          // Symbol: drake::geometry::optimization::CspaceFreePolytopeBase::y_slack
          struct /* y_slack */ {
            // Source: drake/geometry/optimization/cspace_free_polytope_base.h
            const char* doc =
R"""(Get the slack variable used for non-polytopic collision geometries.
Check Options class for more details.)""";
          } y_slack;
        } CspaceFreePolytopeBase;
        // Symbol: drake::geometry::optimization::DistanceToHalfspace
        struct /* DistanceToHalfspace */ {
          // Source: drake/geometry/optimization/c_iris_collision_geometry.h
          const char* doc =
R"""(Computes the signed distance from ``collision_geometry`` to the half
space ℋ, where ℋ = {x | aᵀx+b >= 0} if plane_side=PlaneSide∷kPositive,
and ℋ = {x | aᵀx+b <= 0} if plane_side=PlaneSide∷kNegative. The half
space is measured and expressed in the expressed_body's body frame.
This works for both ``collision_geometry`` separated from the half
space, and ``collision geometry`` in penetration with the halfspace.

Note:
    ``a`` does not need to be a unit length vector (but should be
    non-zero).)""";
        } DistanceToHalfspace;
        // Symbol: drake::geometry::optimization::FindSeparationCertificateOptions
        struct /* FindSeparationCertificateOptions */ {
          // Source: drake/geometry/optimization/cspace_free_structs.h
          const char* doc = R"""()""";
          // Symbol: drake::geometry::optimization::FindSeparationCertificateOptions::FindSeparationCertificateOptions
          struct /* ctor */ {
            // Source: drake/geometry/optimization/cspace_free_structs.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::geometry::optimization::FindSeparationCertificateOptions::parallelism
          struct /* parallelism */ {
            // Source: drake/geometry/optimization/cspace_free_structs.h
            const char* doc = R"""()""";
          } parallelism;
          // Symbol: drake::geometry::optimization::FindSeparationCertificateOptions::solver_id
          struct /* solver_id */ {
            // Source: drake/geometry/optimization/cspace_free_structs.h
            const char* doc = R"""()""";
          } solver_id;
          // Symbol: drake::geometry::optimization::FindSeparationCertificateOptions::solver_options
          struct /* solver_options */ {
            // Source: drake/geometry/optimization/cspace_free_structs.h
            const char* doc = R"""()""";
          } solver_options;
          // Symbol: drake::geometry::optimization::FindSeparationCertificateOptions::terminate_at_failure
          struct /* terminate_at_failure */ {
            // Source: drake/geometry/optimization/cspace_free_structs.h
            const char* doc = R"""()""";
          } terminate_at_failure;
          // Symbol: drake::geometry::optimization::FindSeparationCertificateOptions::verbose
          struct /* verbose */ {
            // Source: drake/geometry/optimization/cspace_free_structs.h
            const char* doc = R"""()""";
          } verbose;
        } FindSeparationCertificateOptions;
        // Symbol: drake::geometry::optimization::GcsGraphvizOptions
        struct /* GcsGraphvizOptions */ {
          // Source: drake/geometry/optimization/graph_of_convex_sets.h
          const char* doc = R"""()""";
          // Symbol: drake::geometry::optimization::GcsGraphvizOptions::Serialize
          struct /* Serialize */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
          } Serialize;
          // Symbol: drake::geometry::optimization::GcsGraphvizOptions::precision
          struct /* precision */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc =
R"""(Sets the floating point precision (how many digits are generated) of
the annotations.)""";
          } precision;
          // Symbol: drake::geometry::optimization::GcsGraphvizOptions::scientific
          struct /* scientific */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc =
R"""(Sets the floating point formatting to scientific (if true) or fixed
(if false).)""";
          } scientific;
          // Symbol: drake::geometry::optimization::GcsGraphvizOptions::show_costs
          struct /* show_costs */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc =
R"""(Determines whether the cost value results are shown. This will show
both edge and vertex costs.)""";
          } show_costs;
          // Symbol: drake::geometry::optimization::GcsGraphvizOptions::show_flows
          struct /* show_flows */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc =
R"""(Determines whether the flow value results are shown. The flow values
are shown both with a numeric value and through the transparency value
on the edge, where a flow of 0.0 will correspond to an (almost)
invisible edge, and a flow of 1.0 will display as a fully black edge.)""";
          } show_flows;
          // Symbol: drake::geometry::optimization::GcsGraphvizOptions::show_slacks
          struct /* show_slacks */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc =
R"""(Determines whether the values of the intermediate (slack) variables
are also displayed in the graph.)""";
          } show_slacks;
          // Symbol: drake::geometry::optimization::GcsGraphvizOptions::show_vars
          struct /* show_vars */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc =
R"""(Determines whether the solution values for decision variables in each
set are shown.)""";
          } show_vars;
          auto Serialize__fields() const {
            return std::array{
              std::make_pair("precision", precision.doc),
              std::make_pair("scientific", scientific.doc),
              std::make_pair("show_costs", show_costs.doc),
              std::make_pair("show_flows", show_flows.doc),
              std::make_pair("show_slacks", show_slacks.doc),
              std::make_pair("show_vars", show_vars.doc),
            };
          }
        } GcsGraphvizOptions;
        // Symbol: drake::geometry::optimization::GetVertices
        struct /* GetVertices */ {
          // Source: drake/geometry/optimization/vpolytope.h
          const char* doc =
R"""(Obtains all the vertices stored in the convex object.

Returns ``vertices``:
    . Each column of ``vertices`` is a vertex. We don't impose any
    specific order on the vertices. The vertices are expressed in the
    convex shape's own frame.)""";
        } GetVertices;
        // Symbol: drake::geometry::optimization::GraphOfConvexSets
        struct /* GraphOfConvexSets */ {
          // Source: drake/geometry/optimization/graph_of_convex_sets.h
          const char* doc =
R"""(GraphOfConvexSets (GCS) implements the design pattern and optimization
problems first introduced in the paper "Shortest Paths in Graphs of
Convex Sets".

"Shortest Paths in Graphs of Convex Sets" by Tobia Marcucci, Jack
Umenberger, Pablo A. Parrilo, Russ Tedrake.
https://arxiv.org/abs/2101.11565

Warning:
    This feature is considered to be **experimental** and may change
    or be removed at any time, without any deprecation notice ahead of
    time.

Each vertex in the graph is associated with a convex set over
continuous variables, edges in the graph contain convex costs and
constraints on these continuous variables. We can then formulate
optimization problems over this graph, such as the shortest path
problem where each visit to a vertex also corresponds to selecting an
element from the convex set subject to the costs and constraints.
Behind the scenes, we construct efficient mixed-integer convex
transcriptions of the graph problem using MathematicalProgram.
However, we provide the option to solve an often tight convex
relaxation of the problem with
GraphOfConvexSetsOptions∷convex_relaxation and employ a cheap rounding
stage which solves the convex restriction along potential paths to
find a feasible solution to the original problem.

Design note: This class avoids providing any direct access to the
MathematicalProgram that it constructs nor to the decision variables /
constraints. The users should be able to write constraints against
"placeholder" decision variables on the vertices and edges, but these
get translated in non-trivial ways to the underlying program.

**Advanced Usage: Guiding Non-convex Optimization with the
GraphOfConvexSets**

Solving a GCS problem using convex relaxation involves two components:
- Convex Relaxation: The relaxation of the binary variables (edge
activations) and perspective operations on the convex cost/constraints
leads to a convex problem that considers the graph as a whole. -
Rounding: After solving the relaxation, a randomized rounding scheme
is applied to obtain a feasible solution for the original problem. We
interpret the relaxed flow variables as edge probabilities to guide
the maximum likelyhood depth first search from the source to target
vertices. Each rounding is calling SolveConvexRestriction.

To handle non-convex constraints, one can provide convex surrogates to
the relaxation and the true non-convex constraints to the rounding
problem. These surrogates approximate the non-convex constraints,
making the relaxation solvable as a convex optimization to guide the
non-convex rounding. This can be controlled by the Transcription enum
in the AddConstraint method. We encourage users to provide a strong
convex surrogate, when possible, to better approximate the original
non-convex problem.

Users can also specify a GCS implicitly, which can be important for
very large or infinite graphs, by deriving from
ImplicitGraphOfConvexSets.)""";
          // Symbol: drake::geometry::optimization::GraphOfConvexSets::AddEdge
          struct /* AddEdge */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc =
R"""(Adds an edge to the graph from Vertex ``u`` to Vertex ``v``. The
vertex references must refer to valid vertices in this graph. If
``name`` is empty then a default name will be provided.

Raises:
    RuntimeError if ``u`` or ``v`` are not valid vertices in this
    graph.)""";
          } AddEdge;
          // Symbol: drake::geometry::optimization::GraphOfConvexSets::AddEdgeFromTemplate
          struct /* AddEdgeFromTemplate */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc =
R"""(Adds an edge to the graph from Vertex ``u`` to Vertex ``v`` (and
assigns a new unique EdgeId), by taking the name, costs, and
constraints from ``template_edge``. `template_edge` does not need to
be registered with this GCS instance; this method can be used to
effectively copy an Edge from another GCS instance into ``this``.

Raises:
    RuntimeError if ``u`` or ``v`` are not valid vertices in this
    graph.

Raises:
    RuntimeError if ``u`` or ``v`` do not match the sizes of the
    ``template_edge.u()`` and ``template_edge.v()`` vertices.

Raises:
    RuntimeError if edges have slack variables. We can add this
    support once it's needed.)""";
          } AddEdgeFromTemplate;
          // Symbol: drake::geometry::optimization::GraphOfConvexSets::AddVertex
          struct /* AddVertex */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc =
R"""(Adds a vertex to the graph. A copy of ``set`` is cloned and stored
inside the graph. If ``name`` is empty then a default name will be
provided.)""";
          } AddVertex;
          // Symbol: drake::geometry::optimization::GraphOfConvexSets::AddVertexFromTemplate
          struct /* AddVertexFromTemplate */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc =
R"""(Adds a new vertex to the graph (and assigns a new unique VertexId) by
taking the name, costs, and constraints (but not any edges) from
``template_vertex``. `template_vertex` does not need to be registered
with this GCS instance; this method can be used to effectively copy a
Vertex from another GCS instance into ``this``.)""";
          } AddVertexFromTemplate;
          // Symbol: drake::geometry::optimization::GraphOfConvexSets::ClearAllPhiConstraints
          struct /* ClearAllPhiConstraints */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc =
R"""(Removes all constraints added to any edge with AddPhiConstraint.)""";
          } ClearAllPhiConstraints;
          // Symbol: drake::geometry::optimization::GraphOfConvexSets::Clone
          struct /* Clone */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc =
R"""(Returns a deep copy of this graph.

Raises:
    RuntimeError if edges have slack variables. We can add this
    support once it's needed.)""";
          } Clone;
          // Symbol: drake::geometry::optimization::GraphOfConvexSets::Edge
          struct /* Edge */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc =
R"""(An edge in the graph connects between vertex ``u`` and vertex ``v``.
The edge also holds a list of cost and constraints associated with the
continuous variables.)""";
            // Symbol: drake::geometry::optimization::GraphOfConvexSets::Edge::AddConstraint
            struct /* AddConstraint */ {
              // Source: drake/geometry/optimization/graph_of_convex_sets.h
              const char* doc_formula =
R"""(Adds a constraint to this edge.

Parameter ``f``:
    must contain *only* elements of xu() and xv() as variables.

Parameter ``use_in_transcription``:
    specifies the components of the problem to which the constraint
    should be added.

Raises:
    RuntimeError if f.GetFreeVariables() is not a subset of xu() ∪
    xv().

Raises:
    RuntimeError if xu() ∪ xv() is empty, i.e., when both vertices
    have an ambient dimension of zero.

Raises:
    RuntimeError if no transcription is specified.)""";
              // Source: drake/geometry/optimization/graph_of_convex_sets.h
              const char* doc_binding =
R"""(Adds a constraint to this edge.

Parameter ``binding``:
    must contain *only* elements of xu() and xv() as variables.

Parameter ``use_in_transcription``:
    specifies the components of the problem to which the constraint
    should be added.

Raises:
    RuntimeError if binding.variables() is not a subset of xu() ∪
    xv().

Raises:
    RuntimeError if xu() ∪ xv() is empty, i.e., when both vertices
    have an ambient dimension of zero.

Raises:
    RuntimeError if no transcription is specified.)""";
            } AddConstraint;
            // Symbol: drake::geometry::optimization::GraphOfConvexSets::Edge::AddCost
            struct /* AddCost */ {
              // Source: drake/geometry/optimization/graph_of_convex_sets.h
              const char* doc_expression =
R"""(Adds a cost to this edge, described by a symbolic∷Expression ``e``
containing *only* elements of xu() and xv() as variables. For
technical reasons relating to being able to "turn-off" the cost on
inactive edges, all costs are eventually implemented with a slack
variable and a constraint:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    min g(xu, xv) ⇒ min ℓ, s.t. ℓ ≥ g(xu,xv)

.. raw:: html

    </details>

You must use GetSolutionCost() to retrieve the cost of the solution,
rather than evaluating the cost directly, in order to get consistent
behavior when solving with the different GCS transcriptions.

Parameter ``use_in_transcription``:
    specifies the components of the problem to which the constraint
    should be added.

Note:
    Linear costs lead to negative costs if decision variables are not
    properly constrained. Users may want to check that the solution
    does not contain negative costs.

Returns:
    the added cost, g(xu, xv).

Raises:
    RuntimeError if e.GetVariables() is not a subset of xu() ∪ xv().

Raises:
    RuntimeError if no transcription is specified.)""";
              // Source: drake/geometry/optimization/graph_of_convex_sets.h
              const char* doc_binding =
R"""(Adds a cost to this edge. ``binding`` must contain *only* elements of
xu() and xv() as variables. For technical reasons relating to being
able to "turn-off" the cost on inactive edges, all costs are
eventually implemented with a slack variable and a constraint:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    min g(xu, xv) ⇒ min ℓ, s.t. ℓ ≥ g(xu,xv)

.. raw:: html

    </details>

You must use GetSolutionCost() to retrieve the cost of the solution,
rather than evaluating the cost directly, in order to get consistent
behavior when solving with the different GCS transcriptions.

Parameter ``use_in_transcription``:
    specifies the components of the problem to which the constraint
    should be added.

Note:
    Linear costs lead to negative costs if decision variables are not
    properly constrained. Users may want to check that the solution
    does not contain negative costs.

Returns:
    the added cost, g(xu, xv).

Raises:
    RuntimeError if binding.variables() is not a subset of xu() ∪
    xv().

Raises:
    RuntimeError if no transcription is specified.)""";
            } AddCost;
            // Symbol: drake::geometry::optimization::GraphOfConvexSets::Edge::AddPhiConstraint
            struct /* AddPhiConstraint */ {
              // Source: drake/geometry/optimization/graph_of_convex_sets.h
              const char* doc =
R"""(Adds a constraint on the binary variable associated with this edge.

Note:
    We intentionally do not return a binding to the constraint created
    by this call, as that would allow the caller to make nonsensical
    modifications to its bounds (i.e. requiring phi == 0.5).)""";
            } AddPhiConstraint;
            // Symbol: drake::geometry::optimization::GraphOfConvexSets::Edge::ClearPhiConstraints
            struct /* ClearPhiConstraints */ {
              // Source: drake/geometry/optimization/graph_of_convex_sets.h
              const char* doc =
R"""(Removes any constraints added with AddPhiConstraint.)""";
            } ClearPhiConstraints;
            // Symbol: drake::geometry::optimization::GraphOfConvexSets::Edge::Edge
            struct /* ctor */ {
              // Source: drake/geometry/optimization/graph_of_convex_sets.h
              const char* doc = R"""()""";
            } ctor;
            // Symbol: drake::geometry::optimization::GraphOfConvexSets::Edge::GetConstraints
            struct /* GetConstraints */ {
              // Source: drake/geometry/optimization/graph_of_convex_sets.h
              const char* doc =
R"""(Returns constraints on this edge.

Parameter ``used_in_transcription``:
    specifies the components of the problem from which the constraint
    should be retrieved.

Raises:
    RuntimeError if no transcription is specified.)""";
            } GetConstraints;
            // Symbol: drake::geometry::optimization::GraphOfConvexSets::Edge::GetCosts
            struct /* GetCosts */ {
              // Source: drake/geometry/optimization/graph_of_convex_sets.h
              const char* doc =
R"""(Returns costs on this edge.

Parameter ``used_in_transcription``:
    specifies the components of the problem from which the constraint
    should be retrieved.

Raises:
    RuntimeError if no transcription is specified.)""";
            } GetCosts;
            // Symbol: drake::geometry::optimization::GraphOfConvexSets::Edge::GetSolutionCost
            struct /* GetSolutionCost */ {
              // Source: drake/geometry/optimization/graph_of_convex_sets.h
              const char* doc_1args =
R"""(Returns the sum of the costs associated with this edge in ``result``,
or std∷nullopt if no solution for this edge is available.)""";
              // Source: drake/geometry/optimization/graph_of_convex_sets.h
              const char* doc_2args =
R"""(Returns the cost associated with the ``cost`` binding on this edge in
``result``, or std∷nullopt if no solution for this edge is available.

Raises:
    RuntimeError if cost is not associated with this edge.)""";
            } GetSolutionCost;
            // Symbol: drake::geometry::optimization::GraphOfConvexSets::Edge::GetSolutionPhiXu
            struct /* GetSolutionPhiXu */ {
              // Source: drake/geometry/optimization/graph_of_convex_sets.h
              const char* doc =
R"""(Returns the vector value of the slack variables associated with ϕxᵤ in
``result``, or std∷nullopt if no solution for this edge is available.
This can obtain a different value than the Vertex∷GetSolution(), e.g.
from ``edge->xu().GetSolution(result)``. First, a deactivated edge
(defined by Phi ~= 0) will return the zero vector here, while
Vertex∷GetSolution() will return std∷nullopt (rather than divide by
zero to recover Xu). Second, in the case of a loose convex relaxation,
the vertex version will return the averaged* value of the edge slacks
for all non-zero-flow edges.)""";
            } GetSolutionPhiXu;
            // Symbol: drake::geometry::optimization::GraphOfConvexSets::Edge::GetSolutionPhiXv
            struct /* GetSolutionPhiXv */ {
              // Source: drake/geometry/optimization/graph_of_convex_sets.h
              const char* doc =
R"""(Returns the vector value of the slack variables associated with ϕxᵥ in
``result``, or std∷nullopt if no solution for this edge is available.
See GetSolutionPhiXu() for more details.)""";
            } GetSolutionPhiXv;
            // Symbol: drake::geometry::optimization::GraphOfConvexSets::Edge::NewSlackVariables
            struct /* NewSlackVariables */ {
              // Source: drake/geometry/optimization/graph_of_convex_sets.h
              const char* doc =
R"""(Creates continuous slack variables for this edge, appending them to an
internal vector of existing slack variables. These slack variables can
be used in any cost or constraint on this edge only, and allows for
modeling more complex costs and constraints.)""";
            } NewSlackVariables;
            // Symbol: drake::geometry::optimization::GraphOfConvexSets::Edge::id
            struct /* id */ {
              // Source: drake/geometry/optimization/graph_of_convex_sets.h
              const char* doc =
R"""(Returns the unique identifier associated with this Edge.)""";
            } id;
            // Symbol: drake::geometry::optimization::GraphOfConvexSets::Edge::name
            struct /* name */ {
              // Source: drake/geometry/optimization/graph_of_convex_sets.h
              const char* doc =
R"""(Returns the string name associated with this edge.)""";
            } name;
            // Symbol: drake::geometry::optimization::GraphOfConvexSets::Edge::phi
            struct /* phi */ {
              // Source: drake/geometry/optimization/graph_of_convex_sets.h
              const char* doc =
R"""(Returns the binary variable associated with this edge. It can be used
to determine whether this edge was active in the solution to an
optimization problem, by calling GetSolution(phi()) on a returned
MathematicalProgramResult.)""";
            } phi;
            // Symbol: drake::geometry::optimization::GraphOfConvexSets::Edge::u
            struct /* u */ {
              // Source: drake/geometry/optimization/graph_of_convex_sets.h
              const char* doc_0args_const =
R"""(Returns a const reference to the "left" Vertex that this edge connects
to.)""";
              // Source: drake/geometry/optimization/graph_of_convex_sets.h
              const char* doc_0args_nonconst =
R"""(Returns a mutable reference to the "left" Vertex that this edge
connects to.)""";
            } u;
            // Symbol: drake::geometry::optimization::GraphOfConvexSets::Edge::v
            struct /* v */ {
              // Source: drake/geometry/optimization/graph_of_convex_sets.h
              const char* doc_0args_const =
R"""(Returns a const reference to the "right" Vertex that this edge
connects to.)""";
              // Source: drake/geometry/optimization/graph_of_convex_sets.h
              const char* doc_0args_nonconst =
R"""(Returns a mutable reference to the "right" Vertex that this edge
connects to.)""";
            } v;
            // Symbol: drake::geometry::optimization::GraphOfConvexSets::Edge::xu
            struct /* xu */ {
              // Source: drake/geometry/optimization/graph_of_convex_sets.h
              const char* doc =
R"""(Returns the continuous decision variables associated with vertex
``u``. This can be used for constructing symbolic∷Expression costs and
constraints.

See also GetSolutionPhiXu(); using ``result.GetSolution(xu())`` may
not be what you want.)""";
            } xu;
            // Symbol: drake::geometry::optimization::GraphOfConvexSets::Edge::xv
            struct /* xv */ {
              // Source: drake/geometry/optimization/graph_of_convex_sets.h
              const char* doc =
R"""(Returns the continuous decision variables associated with vertex
``v``. This can be used for constructing symbolic∷Expression costs and
constraints.

See also GetSolutionPhiXv(); using ``result.GetSolution(xv())`` may
not be what you want.)""";
            } xv;
          } Edge;
          // Symbol: drake::geometry::optimization::GraphOfConvexSets::EdgeId
          struct /* EdgeId */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc = R"""()""";
          } EdgeId;
          // Symbol: drake::geometry::optimization::GraphOfConvexSets::Edges
          struct /* Edges */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc =
R"""(Returns mutable pointers to the edges stored in the graph.)""";
          } Edges;
          // Symbol: drake::geometry::optimization::GraphOfConvexSets::GetEdgeByName
          struct /* GetEdgeByName */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc =
R"""(Returns the first edge (by the order added to ``this``) with the given
name, or nullptr if no such edge exists.)""";
          } GetEdgeByName;
          // Symbol: drake::geometry::optimization::GraphOfConvexSets::GetGraphvizString
          struct /* GetGraphvizString */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc =
R"""(Returns a Graphviz string describing the graph vertices and edges. If
``result`` is supplied, then the graph will be annotated with the
solution values, according to ``options``.

Parameter ``result``:
    the optional result from a solver.

Parameter ``options``:
    the struct containing various options for visualization.

Parameter ``active_path``:
    optionally highlights a given path in the graph. The path is
    displayed as dashed edges in red, displayed in addition to the
    original graph edges.)""";
          } GetGraphvizString;
          // Symbol: drake::geometry::optimization::GraphOfConvexSets::GetMutableEdgeByName
          struct /* GetMutableEdgeByName */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc =
R"""(Returns the first edge (by the order added to ``this``) with the given
name, or nullptr if no such edge exists.)""";
          } GetMutableEdgeByName;
          // Symbol: drake::geometry::optimization::GraphOfConvexSets::GetMutableVertexByName
          struct /* GetMutableVertexByName */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc =
R"""(Returns the first vertex (by the order added to ``this``) with the
given name, or nullptr if no such vertex exists.)""";
          } GetMutableVertexByName;
          // Symbol: drake::geometry::optimization::GraphOfConvexSets::GetSolutionPath
          struct /* GetSolutionPath */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc =
R"""(Extracts a path from ``source`` to ``target`` described by the
``result`` returned by SolveShortestPath(), via depth-first search
following the largest values of the edge binary variables.

Parameter ``tolerance``:
    defines the threshold for checking the integrality conditions of
    the binary variables for each edge. ``tolerance`` = 0 would demand
    that the binary variables are exactly 1 for the edges on the path.
    ``tolerance`` = 1 would allow the binary variables to be any value
    in [0, 1]. The default value is 1e-3.

Raises:
    RuntimeError if !result.is_success() or no path from ``source`` to
    ``target`` can be found in the solution.)""";
          } GetSolutionPath;
          // Symbol: drake::geometry::optimization::GraphOfConvexSets::GetVertexByName
          struct /* GetVertexByName */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc =
R"""(Returns the first vertex (by the order added to ``this``) with the
given name, or nullptr if no such vertex exists.)""";
          } GetVertexByName;
          // Symbol: drake::geometry::optimization::GraphOfConvexSets::GraphOfConvexSets
          struct /* ctor */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc = R"""(Constructs an empty graph.)""";
          } ctor;
          // Symbol: drake::geometry::optimization::GraphOfConvexSets::IsValid
          struct /* IsValid */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc_vertex =
R"""(Returns true iff ``v`` is registered as a vertex with ``this``.)""";
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc_edge =
R"""(Returns true iff ``e`` is registered as an edge with ``this``.)""";
          } IsValid;
          // Symbol: drake::geometry::optimization::GraphOfConvexSets::RemoveEdge
          struct /* RemoveEdge */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc =
R"""(Removes edge ``edge`` from the graph.

Precondition:
    The edge must be part of the graph.)""";
          } RemoveEdge;
          // Symbol: drake::geometry::optimization::GraphOfConvexSets::RemoveVertex
          struct /* RemoveVertex */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc =
R"""(Removes vertex ``vertex`` from the graph as well as any edges from or
to the vertex. Runtime is O(nₑ) where nₑ is the number of edges
connected to ``vertex``

Precondition:
    The vertex must be part of the graph.)""";
          } RemoveVertex;
          // Symbol: drake::geometry::optimization::GraphOfConvexSets::SamplePaths
          struct /* SamplePaths */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc_flows =
R"""(Samples a collection of unique paths from ``source`` to ``target``,
where the flow values (the relaxed binary variables associated with
each ``Edge``) `flows` are interpreted as the probabilities of
transitioning an edge. The returned paths are guaranteed to be unique,
and the number of returned paths can be 0 if no paths are found. This
function implements the first part of the rounding scheme put forth in
Section 4.2 of "Motion Planning around Obstacles with Convex
Optimization": https://arxiv.org/abs/2205.04422

Parameter ``source``:
    specifies the source vertex.

Parameter ``target``:
    specifies the target vertex.

Parameter ``flows``:
    specifies the edge flows, which are interprested as the
    probability of transition an edge. Edge flows that are not
    specified are taken to be zero.

Parameter ``options``:
    include all settings for sampling the paths. Specifically, the
    behavior of this function is determined through
    ``options.rounding_seed``, `options.max_rounded_paths`,
    ``options.max_rounding_trials``, and ``options.flow_tolerance``,
    as described in ``GraphOfConvexSetsOptions``.

Returns:
    A vector of paths, where each path is a vector of `Edge`s.

Raises:
    RuntimeError if options.max_rounded_path < 1.)""";
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc_result =
R"""(Samples a collection of unique paths from ``source`` to ``target``,
where the flow values (the relaxed binary variables associated with
each ``Edge``) in ``result`` are interpreted as the probabilities of
transitioning an edge. The returned paths are guaranteed to be unique,
and the number of returned paths can be 0 if no paths are found. This
function implements the first part of the rounding scheme put forth in
Section 4.2 of "Motion Planning around Obstacles with Convex
Optimization": https://arxiv.org/abs/2205.04422

Parameter ``source``:
    specifies the source vertex.

Parameter ``target``:
    specifies the target vertex.

Parameter ``options``:
    include all settings for sampling the paths. Specifically, the
    behavior of this function is determined through
    ``options.rounding_seed``, `options.max_rounded_paths`,
    ``options.max_rounding_trials``, and ``options.flow_tolerance``,
    as described in ``GraphOfConvexSetsOptions``.

Returns:
    A vector of paths, where each path is a vector of `Edge`s.

Raises:
    RuntimeError if options.max_rounded_path < 1.)""";
          } SamplePaths;
          // Symbol: drake::geometry::optimization::GraphOfConvexSets::SolveConvexRestriction
          struct /* SolveConvexRestriction */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc =
R"""(The non-convexity in a GCS problem comes from the binary variables
(phi) associated with the edges being active or inactive in the
solution. If those binary variables are fixed, then the problem is
convex -- this is a so-called "convex restriction" of the original
problem.

The convex restriction can often be solved much more efficiently than
solving the full GCS problem with additional constraints to fix the
binaries; it can be written using less decision variables, and needs
only to include the vertices associated with at least one of the
active edges. Decision variables for all other convex sets will be set
to NaN.

Note that one can specify additional non-convex constraints, which may
be not supported by all solvers. In this case, the provided solver
will throw an exception.

If an ``initial_guess`` is provided, the solution inside this result
will be used to set the initial guess for the convex restriction.
Typically, this will be the result obtained by solving the convex
relaxation.

Raises:
    RuntimeError if the ``initial_guess`` does not contain solutions
    for the decision variables required in this convex restriction.)""";
          } SolveConvexRestriction;
          // Symbol: drake::geometry::optimization::GraphOfConvexSets::SolveShortestPath
          struct /* SolveShortestPath */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc =
R"""(Formulates and solves the mixed-integer convex formulation of the
shortest path problem on the graph, as discussed in detail in

"Shortest Paths in Graphs of Convex Sets" by Tobia Marcucci, Jack
Umenberger, Pablo A. Parrilo, Russ Tedrake.
https://arxiv.org/abs/2101.11565

Parameter ``source``:
    specifies the source set. The solver will choose any point in that
    set; to start at a particular continuous state consider adding a
    Point set to the graph and using that as the source.

Parameter ``target``:
    specifies the target set. The solver will choose any point in that
    set.

Parameter ``options``:
    include all settings for solving the shortest path problem. See
    ``GraphOfConvexSetsOptions`` for further details. The following
    default options will be used if they are not provided in
    ``options``: - `options.convex_relaxation = false`, -
    ``options.max_rounded_paths = 0``, - `options.preprocessing =
    false`.

Raises:
    RuntimeError if any of the costs or constraints in the graph are
    incompatible with the shortest path formulation or otherwise
    unsupported. All costs must be non-negative for all values of the
    continuous variables.)""";
          } SolveShortestPath;
          // Symbol: drake::geometry::optimization::GraphOfConvexSets::Transcription
          struct /* Transcription */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc =
R"""(Specify the transcription of the optimization problem to which a
constraint or cost should be added, or from which they should be
retrieved.)""";
            // Symbol: drake::geometry::optimization::GraphOfConvexSets::Transcription::kMIP
            struct /* kMIP */ {
              // Source: drake/geometry/optimization/graph_of_convex_sets.h
              const char* doc =
R"""(The mixed integer formulation of the GCS problem.)""";
            } kMIP;
            // Symbol: drake::geometry::optimization::GraphOfConvexSets::Transcription::kRelaxation
            struct /* kRelaxation */ {
              // Source: drake/geometry/optimization/graph_of_convex_sets.h
              const char* doc = R"""(The relaxation of the GCS problem.)""";
            } kRelaxation;
            // Symbol: drake::geometry::optimization::GraphOfConvexSets::Transcription::kRestriction
            struct /* kRestriction */ {
              // Source: drake/geometry/optimization/graph_of_convex_sets.h
              const char* doc =
R"""(The restrction of the GCS problem where the path is fixed.)""";
            } kRestriction;
          } Transcription;
          // Symbol: drake::geometry::optimization::GraphOfConvexSets::Vertex
          struct /* Vertex */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc =
R"""(Each vertex in the graph has a corresponding ConvexSet, and a
std∷string name.)""";
            // Symbol: drake::geometry::optimization::GraphOfConvexSets::Vertex::AddConstraint
            struct /* AddConstraint */ {
              // Source: drake/geometry/optimization/graph_of_convex_sets.h
              const char* doc_formula =
R"""(Adds a constraint to this vertex.

Parameter ``f``:
    must contain *only* elements of x() as variables.

Parameter ``use_in_transcription``:
    specifies the components of the problem to which the constraint
    should be added.

Raises:
    RuntimeError if f.GetFreeVariables() is not a subset of x().

Raises:
    RuntimeError if ambient_dimension() == 0.

Raises:
    RuntimeError if no transcription is specified.)""";
              // Source: drake/geometry/optimization/graph_of_convex_sets.h
              const char* doc_binding =
R"""(Adds a constraint to this vertex.

Parameter ``binding``:
    must contain *only* elements of x() as variables.

Parameter ``use_in_transcription``:
    specifies the components of the problem to which the constraint
    should be added.

Raises:
    RuntimeError if binding.variables() is not a subset of x().

Raises:
    RuntimeError if ambient_dimension() == 0.

Raises:
    RuntimeError if no transcription is specified.)""";
            } AddConstraint;
            // Symbol: drake::geometry::optimization::GraphOfConvexSets::Vertex::AddCost
            struct /* AddCost */ {
              // Source: drake/geometry/optimization/graph_of_convex_sets.h
              const char* doc_expression =
R"""(Adds a cost to this vertex, described by a symbolic∷Expression ``e``
containing *only* elements of x() as variables. For technical reasons
relating to being able to "turn-off" the cost on inactive vertices,
all costs are eventually implemented with a slack variable and a
constraint:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    min g(x) ⇒ min ℓ, s.t. ℓ ≥ g(x).

.. raw:: html

    </details>

You must use GetSolutionCost() to retrieve the cost of the solution,
rather than evaluating the cost directly, in order to get consistent
behavior when solving with the different GCS transcriptions.

Parameter ``use_in_transcription``:
    specifies the components of the problem to which the constraint
    should be added.

Note:
    Linear costs lead to negative costs if decision variables are not
    properly constrained. Users may want to check that the solution
    does not contain negative costs.

Returns:
    the added cost, g(x).

Raises:
    RuntimeError if e.GetVariables() is not a subset of x().

Raises:
    RuntimeError if no transcription is specified.)""";
              // Source: drake/geometry/optimization/graph_of_convex_sets.h
              const char* doc_binding =
R"""(Adds a cost to this vertex. ``binding`` must contain *only* elements
of x() as variables. For technical reasons relating to being able to
"turn-off" the cost on inactive vertices, all costs are eventually
implemented with a slack variable and a constraint:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    min g(x) ⇒ min ℓ, s.t. ℓ ≥ g(x).

.. raw:: html

    </details>

You must use GetSolutionCost() to retrieve the cost of the solution,
rather than evaluating the cost directly, in order to get consistent
behavior when solving with the different GCS transcriptions.

Parameter ``use_in_transcription``:
    specifies the components of the problem to which the constraint
    should be added.

Note:
    Linear costs lead to negative costs if decision variables are not
    properly constrained. Users may want to check that the solution
    does not contain negative costs.

Returns:
    the added cost, g(x).

Raises:
    RuntimeError if binding.variables() is not a subset of x().

Raises:
    RuntimeError if no transcription is specified.)""";
            } AddCost;
            // Symbol: drake::geometry::optimization::GraphOfConvexSets::Vertex::GetConstraints
            struct /* GetConstraints */ {
              // Source: drake/geometry/optimization/graph_of_convex_sets.h
              const char* doc =
R"""(Returns constraints on this vertex.

Parameter ``used_in_transcription``:
    specifies the components of the problem from which the constraint
    should be retrieved.

Raises:
    RuntimeError if no transcription is specified.)""";
            } GetConstraints;
            // Symbol: drake::geometry::optimization::GraphOfConvexSets::Vertex::GetCosts
            struct /* GetCosts */ {
              // Source: drake/geometry/optimization/graph_of_convex_sets.h
              const char* doc =
R"""(Returns costs on this vertex.

Parameter ``used_in_transcription``:
    specifies the components of the problem from which the constraint
    should be retrieved.

Raises:
    RuntimeError if no transcription is specified.)""";
            } GetCosts;
            // Symbol: drake::geometry::optimization::GraphOfConvexSets::Vertex::GetSolution
            struct /* GetSolution */ {
              // Source: drake/geometry/optimization/graph_of_convex_sets.h
              const char* doc =
R"""(Returns the solution of x() in ``result``, or std∷nullopt if no
solution for this vertex is available. std∷nullopt can happen if the
vertex is deactivated (e.g. not in the shorest path) in the solution.)""";
            } GetSolution;
            // Symbol: drake::geometry::optimization::GraphOfConvexSets::Vertex::GetSolutionCost
            struct /* GetSolutionCost */ {
              // Source: drake/geometry/optimization/graph_of_convex_sets.h
              const char* doc_1args =
R"""(Returns the sum of the costs associated with this vertex in
``result``, or std∷nullopt if no solution for this vertex is
available.)""";
              // Source: drake/geometry/optimization/graph_of_convex_sets.h
              const char* doc_2args =
R"""(Returns the cost associated with the ``cost`` binding on this vertex
in ``result``, or std∷nullopt if no solution for this vertex is
available.

Raises:
    RuntimeError if cost is not associated with this vertex.)""";
            } GetSolutionCost;
            // Symbol: drake::geometry::optimization::GraphOfConvexSets::Vertex::Vertex
            struct /* ctor */ {
              // Source: drake/geometry/optimization/graph_of_convex_sets.h
              const char* doc = R"""()""";
            } ctor;
            // Symbol: drake::geometry::optimization::GraphOfConvexSets::Vertex::ambient_dimension
            struct /* ambient_dimension */ {
              // Source: drake/geometry/optimization/graph_of_convex_sets.h
              const char* doc =
R"""(Returns the ambient dimension of the ConvexSet.)""";
            } ambient_dimension;
            // Symbol: drake::geometry::optimization::GraphOfConvexSets::Vertex::id
            struct /* id */ {
              // Source: drake/geometry/optimization/graph_of_convex_sets.h
              const char* doc =
R"""(Returns the unique identifier associated with this Vertex.)""";
            } id;
            // Symbol: drake::geometry::optimization::GraphOfConvexSets::Vertex::incoming_edges
            struct /* incoming_edges */ {
              // Source: drake/geometry/optimization/graph_of_convex_sets.h
              const char* doc = R"""()""";
            } incoming_edges;
            // Symbol: drake::geometry::optimization::GraphOfConvexSets::Vertex::name
            struct /* name */ {
              // Source: drake/geometry/optimization/graph_of_convex_sets.h
              const char* doc = R"""(Returns the name of the vertex.)""";
            } name;
            // Symbol: drake::geometry::optimization::GraphOfConvexSets::Vertex::outgoing_edges
            struct /* outgoing_edges */ {
              // Source: drake/geometry/optimization/graph_of_convex_sets.h
              const char* doc = R"""()""";
            } outgoing_edges;
            // Symbol: drake::geometry::optimization::GraphOfConvexSets::Vertex::set
            struct /* set */ {
              // Source: drake/geometry/optimization/graph_of_convex_sets.h
              const char* doc =
R"""(Returns a const reference to the underlying ConvexSet.)""";
            } set;
            // Symbol: drake::geometry::optimization::GraphOfConvexSets::Vertex::x
            struct /* x */ {
              // Source: drake/geometry/optimization/graph_of_convex_sets.h
              const char* doc =
R"""(Returns a decision variable corresponding to an element of the
ConvexSet, which can be used for constructing symbolic∷Expression
costs and constraints.)""";
            } x;
          } Vertex;
          // Symbol: drake::geometry::optimization::GraphOfConvexSets::VertexId
          struct /* VertexId */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc = R"""()""";
          } VertexId;
          // Symbol: drake::geometry::optimization::GraphOfConvexSets::Vertices
          struct /* Vertices */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc =
R"""(Returns mutable pointers to the vertices stored in the graph.)""";
          } Vertices;
          // Symbol: drake::geometry::optimization::GraphOfConvexSets::num_edges
          struct /* num_edges */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc = R"""()""";
          } num_edges;
          // Symbol: drake::geometry::optimization::GraphOfConvexSets::num_vertices
          struct /* num_vertices */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc = R"""()""";
          } num_vertices;
        } GraphOfConvexSets;
        // Symbol: drake::geometry::optimization::GraphOfConvexSetsOptions
        struct /* GraphOfConvexSetsOptions */ {
          // Source: drake/geometry/optimization/graph_of_convex_sets.h
          const char* doc = R"""()""";
          // Symbol: drake::geometry::optimization::GraphOfConvexSetsOptions::Serialize
          struct /* Serialize */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background. Note: This only serializes options that
are YAML built-in types.)""";
          } Serialize;
          // Symbol: drake::geometry::optimization::GraphOfConvexSetsOptions::convex_relaxation
          struct /* convex_relaxation */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc =
R"""(Flag to solve the relaxed version of the problem. As discussed in the
paper, we know that this relaxation cannot solve the original NP-hard
problem for all instances, but there are also many instances for which
the convex relaxation is tight. If convex_relaxation=nullopt, then
each GCS method is free to choose an appropriate default.)""";
          } convex_relaxation;
          // Symbol: drake::geometry::optimization::GraphOfConvexSetsOptions::flow_tolerance
          struct /* flow_tolerance */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc =
R"""(Tolerance for ignoring flow along a given edge during random rounding.
If convex_relaxation is false or max_rounded_paths is less than or
equal to zero, this option is ignored.)""";
          } flow_tolerance;
          // Symbol: drake::geometry::optimization::GraphOfConvexSetsOptions::max_rounded_paths
          struct /* max_rounded_paths */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc =
R"""(Maximum number of distinct paths to compare during random rounding;
only the lowest cost path is returned. If convex_relaxation is false
or this is less than or equal to zero, rounding is not performed. If
max_rounded_paths=nullopt, then each GCS method is free to choose an
appropriate default.)""";
          } max_rounded_paths;
          // Symbol: drake::geometry::optimization::GraphOfConvexSetsOptions::max_rounding_trials
          struct /* max_rounding_trials */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc =
R"""(Maximum number of trials to find a novel path during random rounding.
If convex_relaxation is false or max_rounded_paths is less than or
equal to zero, this option is ignored.)""";
          } max_rounding_trials;
          // Symbol: drake::geometry::optimization::GraphOfConvexSetsOptions::parallelism
          struct /* parallelism */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc =
R"""(Some steps in GCS can be parallelized. This is the maximum number of
threads used in all places in the algorithm.

Note:
    Some solvers will choose their own level of parallelization,
    independent of this setting. To limit the number of threads, add
    solvers∷CommonSolverOption∷kMaxThreads to the solver_options.)""";
          } parallelism;
          // Symbol: drake::geometry::optimization::GraphOfConvexSetsOptions::preprocessing
          struct /* preprocessing */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc =
R"""(Performs a preprocessing step to remove edges that cannot lie on the
path from source to target. In most cases, preprocessing causes a net
reduction in computation by reducing the size of the optimization
solved. Note that this preprocessing is not exact. There may be edges
that cannot lie on the path from source to target that this does not
detect. If preprocessing=nullopt, then each GCS method is free to
choose an appropriate default.)""";
          } preprocessing;
          // Symbol: drake::geometry::optimization::GraphOfConvexSetsOptions::preprocessing_solver
          struct /* preprocessing_solver */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc =
R"""(Optimizer to be used in the preprocessing stage of GCS, which is
performed when SolveShortestPath is called when the ``preprocessing``
setting has been set to true. If not set, the interface at .solver
will be used, if provided, otherwise the best solver for the given
problem is selected. Note that if the solver cannot handle the type of
optimization problem generated, then calling the
solvers∷SolverInterface∷Solve() method will throw.)""";
          } preprocessing_solver;
          // Symbol: drake::geometry::optimization::GraphOfConvexSetsOptions::preprocessing_solver_options
          struct /* preprocessing_solver_options */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc =
R"""(Optional solver options to be used by preprocessing_solver in the
preprocessing stage of GCS, which is used in SolveShortestPath. If
preprocessing_solver is set but this parameter is not then
solver_options is used. For instance, one might want to print solver
logs for the main optimization, but not from the many smaller
preprocessing optimizations.)""";
          } preprocessing_solver_options;
          // Symbol: drake::geometry::optimization::GraphOfConvexSetsOptions::restriction_solver
          struct /* restriction_solver */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc =
R"""(Optimizer to be used in SolveConvexRestriction(), which is also called
during the rounding stage of SolveShortestPath() given the relaxation.
If not set, the interface at .solver will be used, if provided,
otherwise the best solver for the given problem is selected. Note that
if the solver cannot handle the type of optimization problem
generated, then calling the solvers∷SolverInterface∷Solve() method
will throw.)""";
          } restriction_solver;
          // Symbol: drake::geometry::optimization::GraphOfConvexSetsOptions::restriction_solver_options
          struct /* restriction_solver_options */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc =
R"""(Optional solver options to be used in SolveConvexRestriction(), which
is also used during the rounding stage of SolveShortestPath() given
the relaxation. If not set, solver_options is used. For instance, one
might want to set tighter (i.e., lower) tolerances for running the
relaxed problem and looser (i.e., higher) tolerances for final solves
during rounding.)""";
          } restriction_solver_options;
          // Symbol: drake::geometry::optimization::GraphOfConvexSetsOptions::rounding_seed
          struct /* rounding_seed */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc =
R"""(Random seed to use for random rounding. If convex_relaxation is false
or max_rounded_paths is less than or equal to zero, this option is
ignored.)""";
          } rounding_seed;
          // Symbol: drake::geometry::optimization::GraphOfConvexSetsOptions::solver
          struct /* solver */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc =
R"""(Optimizer to be used to solve the MIP, the relaxation of the shortest
path optimization problem and the convex restriction if no
restriction_solver is provided. If not set, the best solver for the
given problem is selected. Note that if the solver cannot handle the
type of optimization problem generated, the calling
solvers∷SolverInterface∷Solve() method will throw.)""";
          } solver;
          // Symbol: drake::geometry::optimization::GraphOfConvexSetsOptions::solver_options
          struct /* solver_options */ {
            // Source: drake/geometry/optimization/graph_of_convex_sets.h
            const char* doc =
R"""(Options passed to the solver when solving the generated problem.)""";
          } solver_options;
          auto Serialize__fields() const {
            return std::array{
              std::make_pair("convex_relaxation", convex_relaxation.doc),
              std::make_pair("flow_tolerance", flow_tolerance.doc),
              std::make_pair("max_rounded_paths", max_rounded_paths.doc),
              std::make_pair("max_rounding_trials", max_rounding_trials.doc),
              std::make_pair("parallelism", parallelism.doc),
              std::make_pair("preprocessing", preprocessing.doc),
              std::make_pair("preprocessing_solver", preprocessing_solver.doc),
              std::make_pair("preprocessing_solver_options", preprocessing_solver_options.doc),
              std::make_pair("restriction_solver", restriction_solver.doc),
              std::make_pair("restriction_solver_options", restriction_solver_options.doc),
              std::make_pair("rounding_seed", rounding_seed.doc),
              std::make_pair("solver", solver.doc),
              std::make_pair("solver_options", solver_options.doc),
            };
          }
        } GraphOfConvexSetsOptions;
        // Symbol: drake::geometry::optimization::HPolyhedron
        struct /* HPolyhedron */ {
          // Source: drake/geometry/optimization/hpolyhedron.h
          const char* doc =
R"""(Implements a polyhedral convex set using the half-space
representation: ``{x| A x ≤ b}``. Note: This set may be unbounded.

By convention, we treat a zero-dimensional HPolyhedron as nonempty.)""";
          // Symbol: drake::geometry::optimization::HPolyhedron::A
          struct /* A */ {
            // Source: drake/geometry/optimization/hpolyhedron.h
            const char* doc =
R"""(Returns the half-space representation matrix A.)""";
          } A;
          // Symbol: drake::geometry::optimization::HPolyhedron::CartesianPower
          struct /* CartesianPower */ {
            // Source: drake/geometry/optimization/hpolyhedron.h
            const char* doc =
R"""(Returns the ``n``-ary Cartesian power of ``this``. The n-ary Cartesian
power of a set H is the set H ⨉ H ⨉ ... ⨉ H, where H is repeated n
times.)""";
          } CartesianPower;
          // Symbol: drake::geometry::optimization::HPolyhedron::CartesianProduct
          struct /* CartesianProduct */ {
            // Source: drake/geometry/optimization/hpolyhedron.h
            const char* doc =
R"""(Returns the Cartesian product of ``this`` and ``other``.)""";
          } CartesianProduct;
          // Symbol: drake::geometry::optimization::HPolyhedron::ChebyshevCenter
          struct /* ChebyshevCenter */ {
            // Source: drake/geometry/optimization/hpolyhedron.h
            const char* doc =
R"""(Solves a linear program to compute the center of the largest inscribed
ball in the polyhedron. This is often the recommended way to find some
interior point of the set, for example, as a step towards computing
the convex hull or a vertex-representation of the set.

Note that the Chebyshev center is not necessarily unique, and may not
conform to the point that one might consider the "visual center" of
the set. For example, for a long thin rectangle, any point in the
center line segment illustrated below would be a valid center point.
The solver may return any point on that line segment.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    ┌──────────────────────────────────┐
    │                                  │
    │   ────────────────────────────   │
    │                                  │
    └──────────────────────────────────┘

.. raw:: html

    </details>

To find the visual center, consider using the more expensive
MaximumVolumeInscribedEllipsoid() method, and then taking the center
of the returned Hyperellipsoid.

Raises:
    RuntimeError if the solver fails to solve the problem.)""";
          } ChebyshevCenter;
          // Symbol: drake::geometry::optimization::HPolyhedron::ContainedIn
          struct /* ContainedIn */ {
            // Source: drake/geometry/optimization/hpolyhedron.h
            const char* doc =
R"""(Returns true iff this HPolyhedron is entirely contained in the
HPolyhedron other. This is done by checking whether every inequality
in ``other`` is redundant when added to this.

Parameter ``tol``:
    We check if this polyhedron is contained in
    other.A().row(i).dot(x) <= other.b()(i) + tol. The larger tol
    value is, the more relaxation we add to the containment. If tol is
    negative, then we check if a shrinked ``other`` contains this
    polyheron.)""";
          } ContainedIn;
          // Symbol: drake::geometry::optimization::HPolyhedron::FindRedundant
          struct /* FindRedundant */ {
            // Source: drake/geometry/optimization/hpolyhedron.h
            const char* doc =
R"""(Finds the redundant inequalities in this polyhedron. Returns a set ℑ,
such that if we remove the rows of A * x <= b in ℑ, the remaining
inequalities still define the same polyhedron, namely {x | A*x<=b} =
{x | A.row(i)*x<=b(i), ∀i ∉ ℑ}. This function solves a series of
linear programs. We say the jᵗʰ row A.row(j)*x <= b(j) is redundant,
if {x | A.row(i) * x <= b(i), ∀i ∉ ℑ} implies that A.row(j) * x <=
b(j) + tol. Note that we do NOT guarantee that we find all the
redundant rows.)""";
          } FindRedundant;
          // Symbol: drake::geometry::optimization::HPolyhedron::HPolyhedron
          struct /* ctor */ {
            // Source: drake/geometry/optimization/hpolyhedron.h
            const char* doc_0args =
R"""(Constructs a default (zero-dimensional, nonempty) polyhedron.)""";
            // Source: drake/geometry/optimization/hpolyhedron.h
            const char* doc_2args_A_b =
R"""(Constructs the polyhedron.

Precondition:
    A.rows() == b.size().)""";
            // Source: drake/geometry/optimization/hpolyhedron.h
            const char* doc_3args_query_object_geometry_id_reference_frame =
R"""(Constructs a new HPolyhedron from a SceneGraph geometry and pose in
the ``reference_frame`` frame, obtained via the QueryObject. If
``reference_frame`` frame is std∷nullopt, then it will be expressed in
the world frame.

Raises:
    RuntimeError the geometry is not a convex polytope.)""";
            // Source: drake/geometry/optimization/hpolyhedron.h
            const char* doc_2args_vpoly_tol =
R"""(Constructs a new HPolyedron from a VPolytope object. This function
will use qhull. If the VPolytope is empty, then the HPolyhedron will
also be empty. If the HPolyhedron is not full-dimensional, we perform
computations in a coordinate system of its affine hull. ``tol``
specifies the numerical tolerance used in the computation of the
affine hull. (See the documentation of AffineSubspace.) A tighter
tolerance can be used with commercial solvers (e.g. Gurobi and Mosek).

Raises:
    RuntimeError if vpoly is empty and zero dimensional.)""";
            // Source: drake/geometry/optimization/hpolyhedron.h
            const char* doc_1args_prog =
R"""(Constructs a new HPolyhedron describing the feasible set of a linear
program ``prog``. The ``i`th dimension in this representation
corresponds to the `i`th decision variable of `prog``. Note that if
``prog`` is infeasible, then the constructed HPolyhedron will be
empty.

Raises:
    RuntimeError if prog has constraints which are not of type linear
    inequality, linear equality, or bounding box.)""";
          } ctor;
          // Symbol: drake::geometry::optimization::HPolyhedron::Intersection
          struct /* Intersection */ {
            // Source: drake/geometry/optimization/hpolyhedron.h
            const char* doc =
R"""(Constructs the intersection of two HPolyhedron by adding the rows of
inequalities from ``other``. If ``check_for_redundancy`` is true then
only adds the rows of ``other`` other.A().row(i).dot(x)<=other.b()(i)
to this HPolyhedron if the inequality
other.A().row(i).dot(x)<=other.b()(i)+tol is not implied by the
inequalities from this HPolyhedron. A positive tol means it is more
likely to deem a constraint being redundant and remove it. A negative
tol means it is less likely to remove a constraint.)""";
          } Intersection;
          // Symbol: drake::geometry::optimization::HPolyhedron::MakeBox
          struct /* MakeBox */ {
            // Source: drake/geometry/optimization/hpolyhedron.h
            const char* doc =
R"""(Constructs a polyhedron as an axis-aligned box from the lower and
upper corners.)""";
          } MakeBox;
          // Symbol: drake::geometry::optimization::HPolyhedron::MakeL1Ball
          struct /* MakeL1Ball */ {
            // Source: drake/geometry/optimization/hpolyhedron.h
            const char* doc =
R"""(Constructs the L1-norm unit ball in ``dim`` dimensions, {x | |x|₁ <= 1
}. This set is also known as the cross-polytope and is described by
the 2ᵈⁱᵐ signed unit vectors.)""";
          } MakeL1Ball;
          // Symbol: drake::geometry::optimization::HPolyhedron::MakeUnitBox
          struct /* MakeUnitBox */ {
            // Source: drake/geometry/optimization/hpolyhedron.h
            const char* doc =
R"""(Constructs the L∞-norm unit box in ``dim`` dimensions, {x | |x|∞ <= 1
}. This is an axis-aligned box, centered at the origin, with edge
length 2.)""";
          } MakeUnitBox;
          // Symbol: drake::geometry::optimization::HPolyhedron::MaximumVolumeInscribedAffineTransformation
          struct /* MaximumVolumeInscribedAffineTransformation */ {
            // Source: drake/geometry/optimization/hpolyhedron.h
            const char* doc =
R"""(Solves a semi-definite program to compute the maximum-volume affine
transformation of ``this``, subject to being a subset of
``circumbody``, and subject to the transformation matrix being
positive semi-definite. The latter condition is necessary for
convexity of the program. We use the containment condition stated in
Lemma 1 of "Linear Encodings for Polytope Containment Problems" by
Sadra Sadraddini and Russ Tedrake, extended to apply to the affine
transformation of ``this``. We solve


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    max_{T,t} log det (T)
    s.t. T ≽ 0
    t + TX ⊆ Y

.. raw:: html

    </details>

where X is ``this``, and Y is ``circumbody``.

Returns:
    the transformed polyhedron, t + TX.

Parameter ``circumbody``:
    is an HPolyhedron that must contain the returned inbody.

Precondition:
    ``this`` is bounded. If ``check_bounded`` is true, this condition
    is checked and an exception is thrown if it is not satisfied. If
    ``check_bounded`` is set to false, then it is the user's
    responsibility to ensure that ``this`` is bounded and the result
    is not necessarily to be trusted if the precondition is not
    satisfied.

Raises:
    RuntimeError if the solver fails to solve the problem.)""";
          } MaximumVolumeInscribedAffineTransformation;
          // Symbol: drake::geometry::optimization::HPolyhedron::MaximumVolumeInscribedEllipsoid
          struct /* MaximumVolumeInscribedEllipsoid */ {
            // Source: drake/geometry/optimization/hpolyhedron.h
            const char* doc =
R"""(Solves a semi-definite program to compute the inscribed ellipsoid.
This is also known as the inner Löwner-John ellipsoid. From Section
8.4.2 in Boyd and Vandenberghe, 2004, we solve


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    max_{C,d} log det (C)
    s.t. |aᵢC|₂ ≤ bᵢ - aᵢd, ∀i
    C ≽ 0

.. raw:: html

    </details>

where aᵢ and bᵢ denote the ith row. This defines the ellipsoid E = {
Cx + d | |x|₂ ≤ 1}.

Precondition:
    the HPolyhedron is bounded.

Raises:
    RuntimeError if the solver fails to solve the problem.)""";
          } MaximumVolumeInscribedEllipsoid;
          // Symbol: drake::geometry::optimization::HPolyhedron::PontryaginDifference
          struct /* PontryaginDifference */ {
            // Source: drake/geometry/optimization/hpolyhedron.h
            const char* doc =
R"""(Returns the Pontryagin (Minkowski) Difference of ``this`` and
``other``. This is the set A ⊖ B = { a|a+ B ⊆ A }. The result is an
HPolyhedron with the same number of inequalities as A. Requires that
``this`` and ``other`` both be bounded and have the same ambient
dimension. This method may throw a runtime error if ``this`` or
``other`` are ill-conditioned.)""";
          } PontryaginDifference;
          // Symbol: drake::geometry::optimization::HPolyhedron::ReduceInequalities
          struct /* ReduceInequalities */ {
            // Source: drake/geometry/optimization/hpolyhedron.h
            const char* doc =
R"""(Reduces some (not necessarily all) redundant inequalities in the
HPolyhedron. This is not guaranteed to give the minimal representation
of the polyhedron but is a relatively fast way to reduce the number of
inequalities.

Parameter ``tol``:
    For a constraint c'x<=d, if the halfspace c'x<=d + tol contains
    the hpolyhedron generated by the rest of the constraints, then we
    remove this inequality. A positive tol means it is more likely to
    remove a constraint, a negative tol means it is less likely to
    remote a constraint.)""";
          } ReduceInequalities;
          // Symbol: drake::geometry::optimization::HPolyhedron::Scale
          struct /* Scale */ {
            // Source: drake/geometry/optimization/hpolyhedron.h
            const char* doc =
R"""(Results a new HPolyhedron that is a scaled version of ``this``, by
scaling the distance from each face to the ``center`` by a factor of
``pow(scale, 1/ambient_dimension())``, to have units of volume: -
``scale = 0`` will result in a point, - ``0 < scale < 1`` shrinks the
region, - ``scale = 1`` returns a copy of the ``this``, and - ``1 <
scale`` grows the region.

If ``center`` is not provided, then the value returned by
ChebyshevCenter() will be used.

``this`` does not need to be bounded, nor have volume. ``center`` does
not need to be in the set.

Precondition:
    ``scale`` >= 0.

Precondition:
    ``center`` has size equal to the ambient dimension.)""";
          } Scale;
          // Symbol: drake::geometry::optimization::HPolyhedron::Serialize
          struct /* Serialize */ {
            // Source: drake/geometry/optimization/hpolyhedron.h
            const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
          } Serialize;
          // Symbol: drake::geometry::optimization::HPolyhedron::SimplifyByIncrementalFaceTranslation
          struct /* SimplifyByIncrementalFaceTranslation */ {
            // Source: drake/geometry/optimization/hpolyhedron.h
            const char* doc =
R"""(Returns an inner approximation of ``this``, aiming to use fewer faces.
Proceeds by incrementally translating faces inward and removing other
faces that become redundant upon doing so.

Parameter ``min_volume_ratio``:
    is a lower bound for the ratio of the volume of the returned
    inbody and the volume of ``this``.

Parameter ``do_affine_transformation``:
    specifies whether to call
    MaximumVolumeInscribedAffineTransformation(), to take an affine
    transformation of the inner approximation to maximize its volume.
    The affine transformation is reverted if the resulting inner
    approximation violates conditions related to ``points_to_contain``
    or ``intersecting_polytopes``.

Parameter ``max_iterations``:
    is the maximum number of times to loop through all faces.

Parameter ``points_to_contain``:
    is an optional matrix whose columns are points that must be
    contained in the returned inbody.

Parameter ``intersecting_polytopes``:
    is an optional list of HPolyhedrons that must intersect with the
    returned inbody.

Parameter ``keep_whole_intersection``:
    specifies whether the face translation step of the algorithm is
    prohibited from reducing the intersections with the HPolyhedrons
    in ``intersecting_polytopes``. Regardless of the value of this
    parameter, the intersections may be reduced by the affine
    transformation step if ``do_affine_transformation`` is true.

Parameter ``intersection_padding``:
    limits how much the intersection between the inbody and each
    polytope in ``intersection_polytopes`` can be reduced. For each
    polytope in ``intersecting_polytopes``, there is a ball fully
    contained in the inbody, of radius ``intersection_padding``, whose
    center is contained in the intersecting polytope. In the case
    where ``keep_whole_intersection`` is false, using a non-zero value
    for this parameter prevents intersections from being single
    points.

Parameter ``random_seed``:
    is a seed for a random number generator used to shuffle the
    ordering of hyperplanes in between iterations.

Precondition:
    ``min_volume_ratio`` > 0.

Precondition:
    ``max_iterations`` > 0.

Precondition:
    ``intersection_padding`` >= 0.

Precondition:
    All columns of ``points_to_contain`` are points contained within
    ``this``.

Precondition:
    All elements of ``intersecting_polytopes`` intersect with
    ``this``.)""";
          } SimplifyByIncrementalFaceTranslation;
          // Symbol: drake::geometry::optimization::HPolyhedron::UniformSample
          struct /* UniformSample */ {
            // Source: drake/geometry/optimization/hpolyhedron.h
            const char* doc_5args =
R"""(Draw an (approximately) uniform sample from the set using the hit and
run Markov-chain Monte-Carlo strategy described in
https://link.springer.com/article/10.1007/s101070050099. To draw many
samples from the uniform distribution, pass the output of one
iteration in as the ``previous_sample`` to the next, with
``mixing_steps`` set to a relatively low number. When drawing a single
sample, ``mixing_steps`` should be set relatively high in order to
obtain an approximately uniformly random point. The distribution of
samples will converge to the true uniform distribution at a geometric
rate in the total number of hit-and-run steps which is
``mixing_steps`` * the number of times this function is called. If a
``subspace`` is provided, the random samples are constrained to lie in
the affine subspace through ``previous_sample``, spanned by the
columns of ``subspace``. To obtain uniform samples, subspace should
have orthonormal, columns. This enables drawing uniform samples from
an HPolyhedron which is not full-dimensional -- one can pass the basis
of the affine hull of the HPolyhedron, which can be computed with the
AffineSubspace class. ``tol`` is a numerical tolerance for checking if
any halfspaces in the given HPolyhedron are implied by the
``subspace`` definition (and therefore can be ignored by the
hit-and-run sampler).

Precondition:
    subspace.rows() == ambient_dimension().

Raises:
    RuntimeError if previous_sample is not in the set.)""";
            // Source: drake/geometry/optimization/hpolyhedron.h
            const char* doc_4args =
R"""(Variant of UniformSample that uses the ChebyshevCenter() as the
previous_sample as a feasible point to start the Markov chain
sampling.)""";
          } UniformSample;
          // Symbol: drake::geometry::optimization::HPolyhedron::b
          struct /* b */ {
            // Source: drake/geometry/optimization/hpolyhedron.h
            const char* doc =
R"""(Returns the half-space representation vector b.)""";
          } b;
        } HPolyhedron;
        // Symbol: drake::geometry::optimization::Hyperellipsoid
        struct /* Hyperellipsoid */ {
          // Source: drake/geometry/optimization/hyperellipsoid.h
          const char* doc =
R"""(Implements an ellipsoidal convex set represented by the quadratic form
``{x | (x-center)ᵀAᵀA(x-center) ≤ 1}``. Note that ``A`` need not be
square; we require only that the matrix AᵀA is positive semi-definite.

Compare this with an alternative (very useful) parameterization of the
ellipsoid: ``{Bu + center | |u|₂ ≤ 1}``, which is an affine scaling of
the unit ball. This is related to the quadratic form by ``B = A⁻¹``,
when ``A`` is invertible, but the quadratic form can also represent
unbounded sets. The affine scaling of the unit ball representation is
available via the AffineBall class.

Note: the name Hyperellipsoid was taken here to avoid conflicting with
geometry∷Ellipsoid and to distinguish that this class supports N
dimensions.

A hyperellipsoid can never be empty -- it always contains its center.
This includes the zero-dimensional case.)""";
          // Symbol: drake::geometry::optimization::Hyperellipsoid::A
          struct /* A */ {
            // Source: drake/geometry/optimization/hyperellipsoid.h
            const char* doc = R"""(Returns the quadratic form matrix A.)""";
          } A;
          // Symbol: drake::geometry::optimization::Hyperellipsoid::Hyperellipsoid
          struct /* ctor */ {
            // Source: drake/geometry/optimization/hyperellipsoid.h
            const char* doc_0args =
R"""(Constructs a default (zero-dimensional, nonempty) set.)""";
            // Source: drake/geometry/optimization/hyperellipsoid.h
            const char* doc_2args =
R"""(Constructs the ellipsoid.

Precondition:
    A.cols() == center.size().)""";
            // Source: drake/geometry/optimization/hyperellipsoid.h
            const char* doc_3args =
R"""(Constructs a Hyperellipsoid from a SceneGraph geometry and pose in the
``reference_frame`` frame, obtained via the QueryObject. If
``reference_frame`` frame is std∷nullopt, then it will be expressed in
the world frame.

Raises:
    RuntimeError if geometry_id does not represent a shape that can be
    described as an Hyperellipsoid.)""";
            // Source: drake/geometry/optimization/hyperellipsoid.h
            const char* doc_1args =
R"""(Constructs a Hyperellipsoid from an AffineBall.

Precondition:
    ellipsoid.B() is invertible.)""";
          } ctor;
          // Symbol: drake::geometry::optimization::Hyperellipsoid::MakeAxisAligned
          struct /* MakeAxisAligned */ {
            // Source: drake/geometry/optimization/hyperellipsoid.h
            const char* doc =
R"""(Constructs the an axis-aligned Hyperellipsoid with the implicit form
(x₀-c₀)²/r₀² + (x₁-c₁)²/r₁² + ... + (x_N - c_N)²/r_N² ≤ 1, where c is
shorthand for ``center`` and r is shorthand for ``radius``.)""";
          } MakeAxisAligned;
          // Symbol: drake::geometry::optimization::Hyperellipsoid::MakeHypersphere
          struct /* MakeHypersphere */ {
            // Source: drake/geometry/optimization/hyperellipsoid.h
            const char* doc =
R"""(Constructs a hypersphere with ``radius`` and ``center``.)""";
          } MakeHypersphere;
          // Symbol: drake::geometry::optimization::Hyperellipsoid::MakeUnitBall
          struct /* MakeUnitBall */ {
            // Source: drake/geometry/optimization/hyperellipsoid.h
            const char* doc =
R"""(Constructs the L₂-norm unit ball in ``dim`` dimensions, {x | |x|₂ <= 1
}.)""";
          } MakeUnitBall;
          // Symbol: drake::geometry::optimization::Hyperellipsoid::MinimumUniformScalingToTouch
          struct /* MinimumUniformScalingToTouch */ {
            // Source: drake/geometry/optimization/hyperellipsoid.h
            const char* doc =
R"""(Computes the smallest uniform scaling of this ellipsoid for which it
still intersects ``other``. √ minₓ (x-center)ᵀAᵀA(x-center) s.t. x ∈
other. Note that if center ∈ other, then we expect scaling = 0 and x =
center (up to precision).

Precondition:
    ``other`` must have the same ambient_dimension as this.

Returns:
    the minimal scaling and the witness point, x, on other.

Raises:
    RuntimeError if ``other`` is empty.

Raises:
    RuntimeError if ambient_dimension() == 0)""";
          } MinimumUniformScalingToTouch;
          // Symbol: drake::geometry::optimization::Hyperellipsoid::MinimumVolumeCircumscribedEllipsoid
          struct /* MinimumVolumeCircumscribedEllipsoid */ {
            // Source: drake/geometry/optimization/hyperellipsoid.h
            const char* doc =
R"""(Constructs the minimum-volume ellipsoid which contains all of the
``points``. This is commonly referred to as the outer Löwner-John
ellipsoid.

Parameter ``points``:
    is a d-by-n matrix, where d is the ambient dimension and each
    column represents one point.

Parameter ``rank_tol``:
    the singular values of the data matrix will be considered non-zero
    if they are strictly greater than ``rank_tol`` *
    ``max_singular_value``. The default is 1e-6 to be compatible with
    common solver tolerances. This is used to detect if the data lies
    on a lower-dimensional affine space than the ambient dimension of
    the ellipsoid. If this is the case, then use
    AffineBall∷MinimumVolumeCircumscribedEllipsoid instead.

Raises:
    RuntimeError if the MathematicalProgram fails to solve. If this
    were to happen (due to numerical issues), then increasing
    ``rank_tol`` should provide a mitigation.

Raises:
    RuntimeError if points includes NaNs or infinite values.

Raises:
    RuntimeError if the numerical data rank of points is less than d.)""";
          } MinimumVolumeCircumscribedEllipsoid;
          // Symbol: drake::geometry::optimization::Hyperellipsoid::Scale
          struct /* Scale */ {
            // Source: drake/geometry/optimization/hyperellipsoid.h
            const char* doc =
R"""(Results a new Hyperellipsoid that is a scaled version of ``this``
about the center. Any point on the boundary of the ellipsoid, x, is
now translated to a new point, x*, such that ||x* - center|| = ||x -
center|| * pow(scale, 1.0/ambient_dimension()). The volume of the
resulting shape is scaled up by 'scale'.

Precondition:
    ``scale`` > 0.)""";
          } Scale;
          // Symbol: drake::geometry::optimization::Hyperellipsoid::Serialize
          struct /* Serialize */ {
            // Source: drake/geometry/optimization/hyperellipsoid.h
            const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
          } Serialize;
          // Symbol: drake::geometry::optimization::Hyperellipsoid::Volume
          struct /* Volume */ {
            // Source: drake/geometry/optimization/hyperellipsoid.h
            const char* doc =
R"""(Computes the volume for the hyperellipsoid set.)""";
          } Volume;
          // Symbol: drake::geometry::optimization::Hyperellipsoid::center
          struct /* center */ {
            // Source: drake/geometry/optimization/hyperellipsoid.h
            const char* doc = R"""(Returns the center of the ellipsoid.)""";
          } center;
        } Hyperellipsoid;
        // Symbol: drake::geometry::optimization::Hyperrectangle
        struct /* Hyperrectangle */ {
          // Source: drake/geometry/optimization/hyperrectangle.h
          const char* doc =
R"""(Axis-aligned hyperrectangle in Rᵈ defined by its lower bounds and
upper bounds as {x| lb ≤ x ≤ ub})""";
          // Symbol: drake::geometry::optimization::Hyperrectangle::Center
          struct /* Center */ {
            // Source: drake/geometry/optimization/hyperrectangle.h
            const char* doc = R"""(Get the center of the hyperrectangle.)""";
          } Center;
          // Symbol: drake::geometry::optimization::Hyperrectangle::Hyperrectangle
          struct /* ctor */ {
            // Source: drake/geometry/optimization/hyperrectangle.h
            const char* doc_0args =
R"""(Constructs a default (zero-dimensional, nonempty) hyperrectangle.)""";
            // Source: drake/geometry/optimization/hyperrectangle.h
            const char* doc_2args =
R"""(Constructs a hyperrectangle from its lower and upper bounds.

Precondition:
    lb.size() == ub.size()

Precondition:
    lb and ub are finite.

Precondition:
    lb(i) <= ub(i) for all i)""";
          } ctor;
          // Symbol: drake::geometry::optimization::Hyperrectangle::MakeHPolyhedron
          struct /* MakeHPolyhedron */ {
            // Source: drake/geometry/optimization/hyperrectangle.h
            const char* doc =
R"""(Helper to convert this hyperrectangle to an HPolyhedron.)""";
          } MakeHPolyhedron;
          // Symbol: drake::geometry::optimization::Hyperrectangle::MaybeCalcAxisAlignedBoundingBox
          struct /* MaybeCalcAxisAlignedBoundingBox */ {
            // Source: drake/geometry/optimization/hyperrectangle.h
            const char* doc =
R"""(Returns the minimum axis-aligned bounding box of a convex set, for
sets with finite volume. (std∷nullopt otherwise).)""";
          } MaybeCalcAxisAlignedBoundingBox;
          // Symbol: drake::geometry::optimization::Hyperrectangle::MaybeGetIntersection
          struct /* MaybeGetIntersection */ {
            // Source: drake/geometry/optimization/hyperrectangle.h
            const char* doc =
R"""(Constructs the intersection of two Hyperrectangle by taking the
pointwise maximum of the lower bounds and the pointwise minimums of
the upper bounds. Returns std∷nullopt if the intersection is empty.

Precondition:
    this and other need to have the same ambient dimension.)""";
          } MaybeGetIntersection;
          // Symbol: drake::geometry::optimization::Hyperrectangle::Serialize
          struct /* Serialize */ {
            // Source: drake/geometry/optimization/hyperrectangle.h
            const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
          } Serialize;
          // Symbol: drake::geometry::optimization::Hyperrectangle::UniformSample
          struct /* UniformSample */ {
            // Source: drake/geometry/optimization/hyperrectangle.h
            const char* doc = R"""(Draws a uniform sample from the set.)""";
          } UniformSample;
          // Symbol: drake::geometry::optimization::Hyperrectangle::lb
          struct /* lb */ {
            // Source: drake/geometry/optimization/hyperrectangle.h
            const char* doc =
R"""(Get the lower bounds of the hyperrectangle.)""";
          } lb;
          // Symbol: drake::geometry::optimization::Hyperrectangle::ub
          struct /* ub */ {
            // Source: drake/geometry/optimization/hyperrectangle.h
            const char* doc =
R"""(Get the upper bounds of the hyperrectangle.)""";
          } ub;
        } Hyperrectangle;
        // Symbol: drake::geometry::optimization::ImplicitGraphOfConvexSets
        struct /* ImplicitGraphOfConvexSets */ {
          // Source: drake/geometry/optimization/implicit_graph_of_convex_sets.h
          const char* doc =
R"""(A base class to define the interface to an implicit graph of convex
sets.

Implementations of this class must implement DoSuccesors() and provide
some method of accessing at least one vertex in the graph.

Warning:
    This feature is considered to be **experimental** and may change
    or be removed at any time, without any deprecation notice ahead of
    time.)""";
          // Symbol: drake::geometry::optimization::ImplicitGraphOfConvexSets::Expand
          struct /* Expand */ {
            // Source: drake/geometry/optimization/implicit_graph_of_convex_sets.h
            const char* doc =
R"""(Expands a vertex ``v`` by adding its outgoing edges (and the vertices
that they point to) to the mutable_gcs(), calling
mutable_gcs().AddVertex() and mutable_gcs().AddEdge() as needed.

Due to a caching mechanism, implementations can assume that Expand(v)
will only be called once for each ``v``.)""";
          } Expand;
          // Symbol: drake::geometry::optimization::ImplicitGraphOfConvexSets::ExpandRecursively
          struct /* ExpandRecursively */ {
            // Source: drake/geometry/optimization/implicit_graph_of_convex_sets.h
            const char* doc =
R"""(Makes repeated recursive calls to Successors() until no new vertices
will be added to the graph, or ``max_successor_calls`` has been
reached.

Note: ``v`` is mutable because expanding a vertex requires changes to
the underlying vertex object.

Raises:
    RuntimeError if ``v`` is not already registered with the graph.)""";
          } ExpandRecursively;
          // Symbol: drake::geometry::optimization::ImplicitGraphOfConvexSets::ImplicitGraphOfConvexSets
          struct /* ctor */ {
            // Source: drake/geometry/optimization/implicit_graph_of_convex_sets.h
            const char* doc = R"""(Constructs the (empty) implicit GCS.)""";
          } ctor;
          // Symbol: drake::geometry::optimization::ImplicitGraphOfConvexSets::Successors
          struct /* Successors */ {
            // Source: drake/geometry/optimization/implicit_graph_of_convex_sets.h
            const char* doc =
R"""(Returns the outgoing edges from ``v``, which defines the "successors"
of ``v`` in the common notation of implicit graph search. The internal
gcs() object is expanded as needed to include the edges (and the
vertices they point to) that are returned.

Note: The input arguments are mutable because expanding a vertex
requires changes to the underlying vertex object. Similarly, the
output is mutable because callers will need to get mutable vertex
pointers from the returned edges to expand them further.

Raises:
    RuntimeError if ``v`` is not already registered with the graph.)""";
          } Successors;
          // Symbol: drake::geometry::optimization::ImplicitGraphOfConvexSets::gcs
          struct /* gcs */ {
            // Source: drake/geometry/optimization/implicit_graph_of_convex_sets.h
            const char* doc = R"""()""";
          } gcs;
          // Symbol: drake::geometry::optimization::ImplicitGraphOfConvexSets::mutable_gcs
          struct /* mutable_gcs */ {
            // Source: drake/geometry/optimization/implicit_graph_of_convex_sets.h
            const char* doc = R"""()""";
          } mutable_gcs;
        } ImplicitGraphOfConvexSets;
        // Symbol: drake::geometry::optimization::ImplicitGraphOfConvexSetsFromExplicit
        struct /* ImplicitGraphOfConvexSetsFromExplicit */ {
          // Source: drake/geometry/optimization/implicit_graph_of_convex_sets.h
          const char* doc =
R"""(Provides an implicit GCS interface given an explicit GCS. Vertices and
edges are cloned into the implicit GCS as they are expanded.)""";
          // Symbol: drake::geometry::optimization::ImplicitGraphOfConvexSetsFromExplicit::Expand
          struct /* Expand */ {
            // Source: drake/geometry/optimization/implicit_graph_of_convex_sets.h
            const char* doc = R"""()""";
          } Expand;
          // Symbol: drake::geometry::optimization::ImplicitGraphOfConvexSetsFromExplicit::ImplicitGraphOfConvexSetsFromExplicit
          struct /* ctor */ {
            // Source: drake/geometry/optimization/implicit_graph_of_convex_sets.h
            const char* doc =
R"""(Constructs an implicit GCS from an explicit GCS. ``gcs`` must remain
valid for the lifetime of this object.)""";
          } ctor;
          // Symbol: drake::geometry::optimization::ImplicitGraphOfConvexSetsFromExplicit::ImplicitVertexFromExplicit
          struct /* ImplicitVertexFromExplicit */ {
            // Source: drake/geometry/optimization/implicit_graph_of_convex_sets.h
            const char* doc =
R"""(Looks up the implicit vertex corresponding to ``v``. If ``v`` is not
already in the implicit GCS, it is added.

Raises:
    RuntimeError if ``v`` is not registered with the explicit GCS
    passed in the constructor.)""";
          } ImplicitVertexFromExplicit;
        } ImplicitGraphOfConvexSetsFromExplicit;
        // Symbol: drake::geometry::optimization::Intersection
        struct /* Intersection */ {
          // Source: drake/geometry/optimization/intersection.h
          const char* doc =
R"""(A convex set that represents the intersection of multiple sets: S = X₁
∩ X₂ ∩ ... ∩ Xₙ = {x | x ∈ X₁, x ∈ X₂, ..., x ∈ Xₙ}

Special behavior for IsEmpty: The intersection of zero sets (i.e. when
we have sets_.size() == 0) is always nonempty. This includes the
zero-dimensional case, which we treat as being {0}, the unique
zero-dimensional vector space.)""";
          // Symbol: drake::geometry::optimization::Intersection::Intersection
          struct /* ctor */ {
            // Source: drake/geometry/optimization/intersection.h
            const char* doc_0args =
R"""(Constructs a default (zero-dimensional, nonempty) set.)""";
            // Source: drake/geometry/optimization/intersection.h
            const char* doc_1args =
R"""(Constructs the intersection from a vector of convex sets.)""";
            // Source: drake/geometry/optimization/intersection.h
            const char* doc_2args =
R"""(Constructs the intersection from a pair of convex sets.)""";
          } ctor;
          // Symbol: drake::geometry::optimization::Intersection::element
          struct /* element */ {
            // Source: drake/geometry/optimization/intersection.h
            const char* doc =
R"""(Returns a reference to the ConvexSet defining the ``index`` element in
the intersection.)""";
          } element;
          // Symbol: drake::geometry::optimization::Intersection::num_elements
          struct /* num_elements */ {
            // Source: drake/geometry/optimization/intersection.h
            const char* doc =
R"""(The number of elements (or sets) used in the intersection.)""";
          } num_elements;
        } Intersection;
        // Symbol: drake::geometry::optimization::Iris
        struct /* Iris */ {
          // Source: drake/geometry/optimization/iris.h
          const char* doc =
R"""(The IRIS (Iterative Region Inflation by Semidefinite programming)
algorithm, as described in

R. L. H. Deits and R. Tedrake, “Computing large convex regions of
obstacle-free space through semidefinite programming,” Workshop on the
Algorithmic Fundamentals of Robotics, Istanbul, Aug. 2014.
http://groups.csail.mit.edu/robotics-center/public_papers/Deits14.pdf

This algorithm attempts to locally maximize the volume of a convex
polytope representing obstacle-free space given a sample point and
list of convex obstacles. Rather than compute the volume of the
polytope directly, the algorithm maximizes the volume of an inscribed
ellipsoid. It alternates between finding separating hyperplanes
between the ellipsoid and the obstacles and then finding a new
maximum-volume inscribed ellipsoid.

Parameter ``obstacles``:
    is a vector of convex sets representing the occupied space.

Parameter ``sample``:
    provides a point in the space; the algorithm is initialized using
    a tiny sphere around this point. The algorithm is only guaranteed
    to succeed if this sample point is collision free (outside of all
    obstacles), but in practice the algorithm can often escape bad
    initialization (assuming the require_sample_point_is_contained
    option is false).

Parameter ``domain``:
    describes the total region of interest; computed IRIS regions will
    be inside this domain. It must be bounded, and is typically a
    simple bounding box (e.g. from HPolyhedron∷MakeBox).

The ``obstacles``, ``sample``, and the ``domain`` must describe
elements in the same ambient dimension (but that dimension can be any
positive integer).

Note:
    Some members of ``options`` are only applicable to IrisNp. The
    members relevant for this function are starting_ellipse,
    termination_func, bounding_region, verify_domain_boundedness,
    require_sample_point_is_contained, iteration_limit,
    termination_threshold, relative_termination_threshold.)""";
        } Iris;
        // Symbol: drake::geometry::optimization::IrisNp
        struct /* IrisNp */ {
          // Source: drake/geometry/optimization/iris.h
          const char* doc =
R"""(A variation of the Iris (Iterative Region Inflation by Semidefinite
programming) algorithm which finds collision-free regions in the
*configuration space* of ``plant``.

See also:
    Iris for details on the original algorithm. This variant uses
    nonlinear optimization (instead of convex optimization) to find
    collisions in configuration space; each potential collision is
    probabilistically "certified" by restarting the nonlinear
    optimization from random initial seeds inside the candidate IRIS
    region until it fails to find a collision in
    ``options.num_collision_infeasible_samples`` consecutive attempts.

This method constructs a single Iris region in the configuration space
of ``plant``.

See also:
    planning∷IrisInConfigurationSpaceFromCliqueCover for a method to
    automatically cover the configuration space with multiple Iris
    regions.

Parameter ``plant``:
    describes the kinematics of configuration space. It must be
    connected to a SceneGraph in a systems∷Diagram.

Parameter ``context``:
    is a context of the ``plant``. The context must have the positions
    of the plant set to the initialIRIS seed configuration.

Parameter ``options``:
    provides additional configuration options. In particular,
    increasing ``options.num_collision_infeasible_samples`` increases
    the chances that the IRIS regions are collision free but can also
    significantly increase the run-time of the algorithm. The same
    goes for
    ``options.num_additional_constraints_infeasible_samples``.

Raises:
    RuntimeError if the sample configuration in ``context`` is
    infeasible.

Raises:
    RuntimeError if termination_func is invalid on the domain. See
    IrisOptions.termination_func for more details.)""";
        } IrisNp;
        // Symbol: drake::geometry::optimization::IrisOptions
        struct /* IrisOptions */ {
          // Source: drake/geometry/optimization/iris.h
          const char* doc =
R"""(Configuration options for the IRIS algorithm.)""";
          // Symbol: drake::geometry::optimization::IrisOptions::Serialize
          struct /* Serialize */ {
            // Source: drake/geometry/optimization/iris.h
            const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background. Note: This only serializes options that
are YAML built-in types.)""";
          } Serialize;
          // Symbol: drake::geometry::optimization::IrisOptions::bounding_region
          struct /* bounding_region */ {
            // Source: drake/geometry/optimization/iris.h
            const char* doc =
R"""(Optionally allows the caller to restrict the space within which IRIS
regions are allowed to grow. By default, IRIS regions are bounded by
the ``domain`` argument in the case of ``Iris`` or the joint limits of
the input ``plant`` in the case of ``IrisNp``. If this option is
specified, IRIS regions will be confined to the intersection between
the domain and ``bounding_region``.)""";
          } bounding_region;
          // Symbol: drake::geometry::optimization::IrisOptions::configuration_obstacles
          struct /* configuration_obstacles */ {
            // Source: drake/geometry/optimization/iris.h
            const char* doc =
R"""(For IrisNp, it can be beneficial to not only specify task-space
obstacles (passed in through the plant) but also obstacles that are
defined by convex sets in the configuration space. This option can be
used to pass in such configuration space obstacles.)""";
          } configuration_obstacles;
          // Symbol: drake::geometry::optimization::IrisOptions::configuration_space_margin
          struct /* configuration_space_margin */ {
            // Source: drake/geometry/optimization/iris.h
            const char* doc =
R"""(For IrisNp, we retreat by this margin from each C-space obstacle in
order to avoid the possibility of requiring an infinite number of
faces to approximate a curved boundary.)""";
          } configuration_space_margin;
          // Symbol: drake::geometry::optimization::IrisOptions::convexity_radius_stepback
          struct /* convexity_radius_stepback */ {
            // Source: drake/geometry/optimization/iris.h
            const char* doc =
R"""(Artificial joint limits are added to continuous revolute joints and
planar joints with an unbounded revolute degree-of-freedom on a
per-region basis. If the seed point value for that joint is θ, then
the limits are θ - π/2 + convexity_radius_stepback and θ + π/2 -
convexity_radius_stepback. Setting this to a negative number allows
growing larger regions, but those regions must then be partitioned to
be used with GcsTrajectoryOptimization. See
geometry_optimization_geodesic_convexity for more details. IrisNp
throws if this value is not smaller than π/2.)""";
          } convexity_radius_stepback;
          // Symbol: drake::geometry::optimization::IrisOptions::iteration_limit
          struct /* iteration_limit */ {
            // Source: drake/geometry/optimization/iris.h
            const char* doc = R"""(Maximum number of iterations.)""";
          } iteration_limit;
          // Symbol: drake::geometry::optimization::IrisOptions::meshcat
          struct /* meshcat */ {
            // Source: drake/geometry/optimization/iris.h
            const char* doc =
R"""(Passing a meshcat instance may enable debugging visualizations; this
currently only happens in IrisNp and when the configuration space is
<= 3 dimensional.)""";
          } meshcat;
          // Symbol: drake::geometry::optimization::IrisOptions::mixing_steps
          struct /* mixing_steps */ {
            // Source: drake/geometry/optimization/iris.h
            const char* doc =
R"""(The ``mixing_steps`` parameters is passed to HPolyhedron∷UniformSample
to control the total number of hit-and-run steps taken for each new
random sample.)""";
          } mixing_steps;
          // Symbol: drake::geometry::optimization::IrisOptions::num_additional_constraint_infeasible_samples
          struct /* num_additional_constraint_infeasible_samples */ {
            // Source: drake/geometry/optimization/iris.h
            const char* doc =
R"""(For each constraint in ``prog_with_additional_constraints``, IRIS will
search for a counter-example by formulating a (likely nonconvex)
optimization problem. The initial guess for this optimization is taken
by sampling uniformly inside the current IRIS region. This option
controls the termination condition for that counter-example search,
defining the number of consecutive failures to find a counter-example
requested before moving on to the next constraint.)""";
          } num_additional_constraint_infeasible_samples;
          // Symbol: drake::geometry::optimization::IrisOptions::num_collision_infeasible_samples
          struct /* num_collision_infeasible_samples */ {
            // Source: drake/geometry/optimization/iris.h
            const char* doc =
R"""(For each possible collision, IRIS will search for a counter-example by
formulating a (likely nonconvex) optimization problem. The initial
guess for this optimization is taken by sampling uniformly inside the
current IRIS region. This option controls the termination condition
for that counter-example search, defining the number of consecutive
failures to find a counter-example requested before moving on to the
next constraint.)""";
          } num_collision_infeasible_samples;
          // Symbol: drake::geometry::optimization::IrisOptions::prog_with_additional_constraints
          struct /* prog_with_additional_constraints */ {
            // Source: drake/geometry/optimization/iris.h
            const char* doc =
R"""(By default, IrisNp certifies regions for collision avoidance
constraints and joint limits. This option can be used to pass
additional constraints that should be satisfied by the IRIS region. We
accept these in the form of a MathematicalProgram:

find q subject to g(q) ≤ 0.

The decision_variables() for the program are taken to define ``q``.
IRIS will silently ignore any costs in
``prog_with_additional_constraints``, and will throw RuntimeError if
it contains any unsupported constraints.

For example, one could create an InverseKinematics problem with rich
kinematic constraints, and then pass ``InverseKinematics∷prog()`` into
this option.)""";
          } prog_with_additional_constraints;
          // Symbol: drake::geometry::optimization::IrisOptions::random_seed
          struct /* random_seed */ {
            // Source: drake/geometry/optimization/iris.h
            const char* doc =
R"""(The only randomization in IRIS is the random sampling done to find
counter-examples for the additional constraints using in IrisNp. Use
this option to set the initial seed.)""";
          } random_seed;
          // Symbol: drake::geometry::optimization::IrisOptions::relative_termination_threshold
          struct /* relative_termination_threshold */ {
            // Source: drake/geometry/optimization/iris.h
            const char* doc =
R"""(IRIS will terminate if the change in the *volume* of the
hyperellipsoid between iterations is less that this percent of the
previous best volume. This termination condition can be disabled by
setting to a negative value.)""";
          } relative_termination_threshold;
          // Symbol: drake::geometry::optimization::IrisOptions::require_sample_point_is_contained
          struct /* require_sample_point_is_contained */ {
            // Source: drake/geometry/optimization/iris.h
            const char* doc =
R"""(The initial polytope is guaranteed to contain the point if that point
is collision-free. However, the IRIS alternation objectives do not
include (and can not easily include) a constraint that the original
sample point is contained. Therefore, the IRIS paper recommends that
if containment is a requirement, then the algorithm should simply
terminate early if alternations would ever cause the set to not
contain the point.)""";
          } require_sample_point_is_contained;
          // Symbol: drake::geometry::optimization::IrisOptions::solver_options
          struct /* solver_options */ {
            // Source: drake/geometry/optimization/iris.h
            const char* doc =
R"""(The SolverOptions used in the optimization program.)""";
          } solver_options;
          // Symbol: drake::geometry::optimization::IrisOptions::starting_ellipse
          struct /* starting_ellipse */ {
            // Source: drake/geometry/optimization/iris.h
            const char* doc =
R"""(The initial hyperellipsoid that IRIS will use for calculating
hyperplanes in the first iteration. If no hyperellipsoid is provided,
a small hypershpere centered at the given sample will be used.)""";
          } starting_ellipse;
          // Symbol: drake::geometry::optimization::IrisOptions::termination_func
          struct /* termination_func */ {
            // Source: drake/geometry/optimization/iris.h
            const char* doc =
R"""(A user-defined termination function to determine whether the
iterations should stop. This function is called after computing each
hyperplane at every IRIS iteration. If the function returns true, then
the computations will stop and the last step region will be returned.
Therefore, it is highly recommended that the termination function
possesses a monotonic property such that for any two HPolyhedrons A
and B such that B ⊆ A, we have if termination(A) -> termination(B).
For example, a valid termination function is to check whether if the
region does not contain any of a set of desired points.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    auto termination_func = [](const HPolyhedron& set) {
    for (const VectorXd& point : desired_points) {
    if (!set.PointInSet(point)) {
    return true;
    }
    }
    return false;
    };

.. raw:: html

    </details>

The algorithm will stop when as soon as the region leaves one of the
desired points, in a similar way to how
``require_sample_point_is_contained`` is enforced.)""";
          } termination_func;
          // Symbol: drake::geometry::optimization::IrisOptions::termination_threshold
          struct /* termination_threshold */ {
            // Source: drake/geometry/optimization/iris.h
            const char* doc =
R"""(IRIS will terminate if the change in the *volume* of the
hyperellipsoid between iterations is less that this threshold. This
termination condition can be disabled by setting to a negative value.)""";
          } termination_threshold;
          // Symbol: drake::geometry::optimization::IrisOptions::verify_domain_boundedness
          struct /* verify_domain_boundedness */ {
            // Source: drake/geometry/optimization/iris.h
            const char* doc =
R"""(If the user knows the intersection of bounding_region and the domain
(for IRIS) or plant joint limits (for IrisNp) is bounded, setting this
flag to ``False`` will skip the boundedness check that IRIS and IrisNp
perform (leading to a small speedup, as checking boundedness requires
solving optimization problems). If the intersection turns out to be
unbounded, this will lead to undefined behavior.)""";
          } verify_domain_boundedness;
          auto Serialize__fields() const {
            return std::array{
              std::make_pair("bounding_region", bounding_region.doc),
              std::make_pair("configuration_obstacles", configuration_obstacles.doc),
              std::make_pair("configuration_space_margin", configuration_space_margin.doc),
              std::make_pair("convexity_radius_stepback", convexity_radius_stepback.doc),
              std::make_pair("iteration_limit", iteration_limit.doc),
              std::make_pair("meshcat", meshcat.doc),
              std::make_pair("mixing_steps", mixing_steps.doc),
              std::make_pair("num_additional_constraint_infeasible_samples", num_additional_constraint_infeasible_samples.doc),
              std::make_pair("num_collision_infeasible_samples", num_collision_infeasible_samples.doc),
              std::make_pair("prog_with_additional_constraints", prog_with_additional_constraints.doc),
              std::make_pair("random_seed", random_seed.doc),
              std::make_pair("relative_termination_threshold", relative_termination_threshold.doc),
              std::make_pair("require_sample_point_is_contained", require_sample_point_is_contained.doc),
              std::make_pair("solver_options", solver_options.doc),
              std::make_pair("starting_ellipse", starting_ellipse.doc),
              std::make_pair("termination_func", termination_func.doc),
              std::make_pair("termination_threshold", termination_threshold.doc),
              std::make_pair("verify_domain_boundedness", verify_domain_boundedness.doc),
            };
          }
        } IrisOptions;
        // Symbol: drake::geometry::optimization::IrisRegions
        struct /* IrisRegions */ {
          // Source: drake/geometry/optimization/iris.h
          const char* doc =
R"""(Defines a standardized representation for (named) IrisRegions, which
can be serialized in both C++ and Python.)""";
        } IrisRegions;
        // Symbol: drake::geometry::optimization::MakeConvexSets
        struct /* MakeConvexSets */ {
          // Source: drake/geometry/optimization/convex_set.h
          const char* doc =
R"""(Helper function that allows the ConvexSets to be initialized from
arguments containing ConvexSet references, or unique_ptr<ConvexSet>
instances, or any object that can be assigned to
ConvexSets∷value_type.)""";
        } MakeConvexSets;
        // Symbol: drake::geometry::optimization::MakeIrisObstacles
        struct /* MakeIrisObstacles */ {
          // Source: drake/geometry/optimization/iris.h
          const char* doc =
R"""(Constructs ConvexSet representations of obstacles for IRIS in 3D using
the geometry from a SceneGraph QueryObject. All geometry in the scene
with a proximity role, both anchored and dynamic, are consider to be
*fixed* obstacles frozen in the poses captured in the context used to
create the QueryObject.

When multiple representations are available for a particular geometry
(e.g. a Box can be represented as either an HPolyhedron or a
VPolytope), then this method will prioritize the representation that
we expect is most performant for the current implementation of the
IRIS algorithm.)""";
        } MakeIrisObstacles;
        // Symbol: drake::geometry::optimization::MinkowskiSum
        struct /* MinkowskiSum */ {
          // Source: drake/geometry/optimization/minkowski_sum.h
          const char* doc =
R"""(A convex set that represents the Minkowski sum of multiple sets: S =
X₁ ⨁ X₂ ⨁ ... ⨁ Xₙ = {x₁ + x₂ + ... + xₙ | x₁ ∈ X₁, x₂ ∈ X₂, ..., xₙ ∈
Xₙ}

Special behavior for IsEmpty: The Minkowski sum of zero sets (i.e.
when we have sets_.size() == 0) is treated as the singleton {0}, which
is nonempty. This includes the zero-dimensional case.)""";
          // Symbol: drake::geometry::optimization::MinkowskiSum::MinkowskiSum
          struct /* ctor */ {
            // Source: drake/geometry/optimization/minkowski_sum.h
            const char* doc_0args =
R"""(Constructs a default (zero-dimensional, nonempty) set.)""";
            // Source: drake/geometry/optimization/minkowski_sum.h
            const char* doc_1args =
R"""(Constructs the sum from a vector of convex sets.)""";
            // Source: drake/geometry/optimization/minkowski_sum.h
            const char* doc_2args =
R"""(Constructs the sum from a pair of convex sets.)""";
            // Source: drake/geometry/optimization/minkowski_sum.h
            const char* doc_3args =
R"""(Constructs a MinkowskiSum from a SceneGraph geometry and pose in the
``reference_frame`` frame, obtained via the QueryObject. If
``reference_frame`` frame is std∷nullopt, then it will be expressed in
the world frame.

Although in principle a MinkowskiSum can represent any ConvexSet as
the sum of a single set, here we only support Capsule geometry, which
will be represented as the (non-trivial) Minkowski sum of a sphere
with a line segment. Most SceneGraph geometry types are supported by
at least one of the ConvexSet class constructors.

Raises:
    RuntimeError if geometry_id does not correspond to a Capsule.)""";
          } ctor;
          // Symbol: drake::geometry::optimization::MinkowskiSum::num_terms
          struct /* num_terms */ {
            // Source: drake/geometry/optimization/minkowski_sum.h
            const char* doc =
R"""(The number of terms (or sets) used in the sum.)""";
          } num_terms;
          // Symbol: drake::geometry::optimization::MinkowskiSum::term
          struct /* term */ {
            // Source: drake/geometry/optimization/minkowski_sum.h
            const char* doc =
R"""(Returns a reference to the ConvexSet defining the ``index`` term in
the sum.)""";
          } term;
        } MinkowskiSum;
        // Symbol: drake::geometry::optimization::PartitionConvexSet
        struct /* PartitionConvexSet */ {
          // Source: drake/geometry/optimization/geodesic_convexity.h
          const char* doc_3args_convex_set_continuous_revolute_joints_epsilon =
R"""(Partitions a convex set into (smaller) convex sets whose union is the
original set and that each respect the convexity radius as in
CheckIfSatisfiesConvexityRadius. In practice, this is implemented as
partitioning sets into pieces whose width is less than or equal to
π-ϵ. Each entry in continuous_revolute_joints must be non-negative,
less than num_positions, and unique.

Parameter ``epsilon``:
    is the ϵ value used for the convexity radius inequality. The
    partitioned sets are made by intersecting convex_set with
    axis-aligned bounding boxes that respect the convexity radius.
    These boxes are made to overlap by ϵ radians along each dimension,
    for numerical purposes.

Returns:
    the vector of convex sets that each respect convexity radius.

Raises:
    RuntimeError if ϵ <= 0 or ϵ >= π.

Raises:
    RuntimeError if the input convex set is unbounded along dimensions
    corresponding to continuous revolute joints.

Raises:
    RuntimeError if continuous_revolute_joints has repeated entries,
    or if any entry is outside the interval [0,
    convex_set.ambient_dimension()).)""";
          // Source: drake/geometry/optimization/geodesic_convexity.h
          const char* doc_3args_convex_sets_continuous_revolute_joints_epsilon =
R"""(Function overload to take in a list of convex sets, and partition all
so as to respect the convexity radius. Every set must be bounded and
have the same ambient dimension. Each entry in
continuous_revolute_joints must be non-negative, less than
num_positions, and unique.

Raises:
    RuntimeError unless every ConvexSet in convex_sets has the same
    ambient_dimension.

Raises:
    RuntimeError if ϵ <= 0 or ϵ >= π.

Raises:
    RuntimeError if any input convex set is unbounded along dimensions
    corresponding to continuous revolute joints.

Raises:
    RuntimeError if continuous_revolute_joints has repeated entries,
    or if any entry is outside the interval [0, ambient_dimension).

Raises:
    if any entry of ``convex_sets`` is a nullptr.)""";
        } PartitionConvexSet;
        // Symbol: drake::geometry::optimization::PlaneSeparatesGeometries
        struct /* PlaneSeparatesGeometries */ {
          // Source: drake/geometry/optimization/cspace_free_structs.h
          const char* doc =
R"""(Contains the information to enforce a pair of geometries are separated
by a plane. The conditions are that certain rational functions should
be always positive.)""";
          // Symbol: drake::geometry::optimization::PlaneSeparatesGeometries::PlaneSeparatesGeometries
          struct /* ctor */ {
            // Source: drake/geometry/optimization/cspace_free_structs.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::geometry::optimization::PlaneSeparatesGeometries::negative_side_rationals
          struct /* negative_side_rationals */ {
            // Source: drake/geometry/optimization/cspace_free_structs.h
            const char* doc = R"""()""";
          } negative_side_rationals;
          // Symbol: drake::geometry::optimization::PlaneSeparatesGeometries::plane_index
          struct /* plane_index */ {
            // Source: drake/geometry/optimization/cspace_free_structs.h
            const char* doc = R"""()""";
          } plane_index;
          // Symbol: drake::geometry::optimization::PlaneSeparatesGeometries::positive_side_rationals
          struct /* positive_side_rationals */ {
            // Source: drake/geometry/optimization/cspace_free_structs.h
            const char* doc = R"""()""";
          } positive_side_rationals;
          // Symbol: drake::geometry::optimization::PlaneSeparatesGeometries::rationals
          struct /* rationals */ {
            // Source: drake/geometry/optimization/cspace_free_structs.h
            const char* doc = R"""()""";
          } rationals;
        } PlaneSeparatesGeometries;
        // Symbol: drake::geometry::optimization::PlaneSide
        struct /* PlaneSide */ {
          // Source: drake/geometry/optimization/c_iris_collision_geometry.h
          const char* doc = R"""()""";
          // Symbol: drake::geometry::optimization::PlaneSide::kNegative
          struct /* kNegative */ {
            // Source: drake/geometry/optimization/c_iris_collision_geometry.h
            const char* doc = R"""()""";
          } kNegative;
          // Symbol: drake::geometry::optimization::PlaneSide::kPositive
          struct /* kPositive */ {
            // Source: drake/geometry/optimization/c_iris_collision_geometry.h
            const char* doc = R"""()""";
          } kPositive;
        } PlaneSide;
        // Symbol: drake::geometry::optimization::Point
        struct /* Point */ {
          // Source: drake/geometry/optimization/point.h
          const char* doc =
R"""(A convex set that contains exactly one element. Also known as a
singleton or unit set.

This set is always nonempty, even in the zero-dimensional case.)""";
          // Symbol: drake::geometry::optimization::Point::Point
          struct /* ctor */ {
            // Source: drake/geometry/optimization/point.h
            const char* doc_0args =
R"""(Constructs a default (zero-dimensional, nonempty) set.)""";
            // Source: drake/geometry/optimization/point.h
            const char* doc_1args = R"""(Constructs a Point.)""";
            // Source: drake/geometry/optimization/point.h
            const char* doc_4args =
R"""(Constructs a Point from a SceneGraph geometry and pose in the
``reference_frame`` frame, obtained via the QueryObject. If
``reference_frame`` frame is std∷nullopt, then it will be expressed in
the world frame.

Raises:
    RuntimeError if geometry_id does not correspond to a Sphere or if
    the Sphere has radius greater than ``maximum_allowable_radius``.)""";
          } ctor;
          // Symbol: drake::geometry::optimization::Point::set_x
          struct /* set_x */ {
            // Source: drake/geometry/optimization/point.h
            const char* doc =
R"""(Changes the element ``x`` describing the set.

Precondition:
    x must be of size ambient_dimension().)""";
          } set_x;
          // Symbol: drake::geometry::optimization::Point::x
          struct /* x */ {
            // Source: drake/geometry/optimization/point.h
            const char* doc = R"""(Retrieves the point.)""";
          } x;
        } Point;
        // Symbol: drake::geometry::optimization::SampledVolume
        struct /* SampledVolume */ {
          // Source: drake/geometry/optimization/convex_set.h
          const char* doc =
R"""(The result of a volume calculation from CalcVolumeViaSampling().)""";
          // Symbol: drake::geometry::optimization::SampledVolume::num_samples
          struct /* num_samples */ {
            // Source: drake/geometry/optimization/convex_set.h
            const char* doc =
R"""(The number of samples used to compute the volume estimate.)""";
          } num_samples;
          // Symbol: drake::geometry::optimization::SampledVolume::rel_accuracy
          struct /* rel_accuracy */ {
            // Source: drake/geometry/optimization/convex_set.h
            const char* doc =
R"""(An upper bound for the relative accuracy of the volume estimate. When
not evaluated, this value is NaN.)""";
          } rel_accuracy;
          // Symbol: drake::geometry::optimization::SampledVolume::volume
          struct /* volume */ {
            // Source: drake/geometry/optimization/convex_set.h
            const char* doc = R"""(The estimated volume of the set.)""";
          } volume;
        } SampledVolume;
        // Symbol: drake::geometry::optimization::SeparatingPlaneOrder
        struct /* SeparatingPlaneOrder */ {
          // Source: drake/geometry/optimization/cspace_separating_plane.h
          const char* doc =
R"""(The separating plane aᵀx + b ≥ δ, aᵀx + b ≤ −δ has parameters a and b.
These parameterize a polynomial function of ``s_for_plane`` with the
specified order. ``s_for_plane`` is a sub set of the
configuration-space variable ``s``, please refer to the
RationalForwardKinematics class or the paper above on the meaning of
s.)""";
          // Symbol: drake::geometry::optimization::SeparatingPlaneOrder::kAffine
          struct /* kAffine */ {
            // Source: drake/geometry/optimization/cspace_separating_plane.h
            const char* doc = R"""(a and b are affine functions of s.)""";
          } kAffine;
        } SeparatingPlaneOrder;
        // Symbol: drake::geometry::optimization::SeparationCertificateProgramBase
        struct /* SeparationCertificateProgramBase */ {
          // Source: drake/geometry/optimization/cspace_free_structs.h
          const char* doc = R"""()""";
          // Symbol: drake::geometry::optimization::SeparationCertificateProgramBase::SeparationCertificateProgramBase
          struct /* ctor */ {
            // Source: drake/geometry/optimization/cspace_free_structs.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::geometry::optimization::SeparationCertificateProgramBase::plane_index
          struct /* plane_index */ {
            // Source: drake/geometry/optimization/cspace_free_structs.h
            const char* doc = R"""()""";
          } plane_index;
          // Symbol: drake::geometry::optimization::SeparationCertificateProgramBase::prog
          struct /* prog */ {
            // Source: drake/geometry/optimization/cspace_free_structs.h
            const char* doc =
R"""(The program that stores all the constraints to search for the
separating plane and Lagrangian multipliers as certificate.)""";
          } prog;
        } SeparationCertificateProgramBase;
        // Symbol: drake::geometry::optimization::SeparationCertificateResultBase
        struct /* SeparationCertificateResultBase */ {
          // Source: drake/geometry/optimization/cspace_free_structs.h
          const char* doc =
R"""(We certify that a pair of geometries is collision free by finding the
separating plane over a range of configuration. The Lagrangian
multipliers used for certifying this condition will differ in derived
classes. This struct contains the the separating plane {x | aᵀx+b=0 }
and derived classes may store the Lagrangians certifying that the
plane separates the two geometries in separating_planes()[plane_index]
in the C-space region.)""";
          // Symbol: drake::geometry::optimization::SeparationCertificateResultBase::SeparationCertificateResultBase
          struct /* ctor */ {
            // Source: drake/geometry/optimization/cspace_free_structs.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::geometry::optimization::SeparationCertificateResultBase::a
          struct /* a */ {
            // Source: drake/geometry/optimization/cspace_free_structs.h
            const char* doc = R"""(The separating plane is { x | aᵀx+b=0 })""";
          } a;
          // Symbol: drake::geometry::optimization::SeparationCertificateResultBase::b
          struct /* b */ {
            // Source: drake/geometry/optimization/cspace_free_structs.h
            const char* doc = R"""()""";
          } b;
          // Symbol: drake::geometry::optimization::SeparationCertificateResultBase::plane_decision_var_vals
          struct /* plane_decision_var_vals */ {
            // Source: drake/geometry/optimization/cspace_free_structs.h
            const char* doc = R"""()""";
          } plane_decision_var_vals;
          // Symbol: drake::geometry::optimization::SeparationCertificateResultBase::plane_index
          struct /* plane_index */ {
            // Source: drake/geometry/optimization/cspace_free_structs.h
            const char* doc = R"""()""";
          } plane_index;
          // Symbol: drake::geometry::optimization::SeparationCertificateResultBase::result
          struct /* result */ {
            // Source: drake/geometry/optimization/cspace_free_structs.h
            const char* doc = R"""()""";
          } result;
        } SeparationCertificateResultBase;
        // Symbol: drake::geometry::optimization::SetEdgeContainmentTerminationCondition
        struct /* SetEdgeContainmentTerminationCondition */ {
          // Source: drake/geometry/optimization/iris.h
          const char* doc =
R"""(Modifies the ``iris_options`` to facilitate finding a region that
contains the edge between x_1 and x_2. It sets
``iris_options``.starting_ellipse to be a hyperellipsoid that contains
the edge, is centered at the midpoint of the edge and extends in other
directions by epsilon. It also sets ``iris_options``.termination_func
such that IRIS iterations terminate when the edge is no longer
contained in the IRIS region with tolerance tol.

Raises:
    RuntimeError if x_1.size() != x_2.size().

Raises:
    RuntimeError if epsilon <= 0. This is due to the fact that the
    hyperellipsoid for ``iris_options``.starting_ellipse must have
    non-zero volume.)""";
        } SetEdgeContainmentTerminationCondition;
        // Symbol: drake::geometry::optimization::Spectrahedron
        struct /* Spectrahedron */ {
          // Source: drake/geometry/optimization/spectrahedron.h
          const char* doc =
R"""(Implements a spectrahedron (the feasible set of a semidefinite
program). The ambient dimension of the set is N(N+1)/2; the number of
variables required to describe the N-by-N semidefinite matrix.

By convention, a zero-dimensional spectrahedron is considered
nonempty.)""";
          // Symbol: drake::geometry::optimization::Spectrahedron::Spectrahedron
          struct /* ctor */ {
            // Source: drake/geometry/optimization/spectrahedron.h
            const char* doc_0args =
R"""(Default constructor (yields the zero-dimensional nonempty set).)""";
            // Source: drake/geometry/optimization/spectrahedron.h
            const char* doc_1args =
R"""(Constructs the spectrahedron from a MathematicalProgram.

Raises:
    RuntimeError if ``prog.required_capabilities()`` is not a subset
    of supported_attributes().)""";
          } ctor;
          // Symbol: drake::geometry::optimization::Spectrahedron::supported_attributes
          struct /* supported_attributes */ {
            // Source: drake/geometry/optimization/spectrahedron.h
            const char* doc =
R"""(Returns the list of solvers∷ProgramAttributes supported by this class.)""";
          } supported_attributes;
        } Spectrahedron;
        // Symbol: drake::geometry::optimization::ToPlaneDegree
        struct /* ToPlaneDegree */ {
          // Source: drake/geometry/optimization/cspace_separating_plane.h
          const char* doc =
R"""(Convert SeparatingPlaneOrder to an integer degree.)""";
        } ToPlaneDegree;
        // Symbol: drake::geometry::optimization::ToPlaneOrder
        struct /* ToPlaneOrder */ {
          // Source: drake/geometry/optimization/cspace_separating_plane.h
          const char* doc =
R"""(Convert an integer degree to the SeparatingPlaneOrder)""";
        } ToPlaneOrder;
        // Symbol: drake::geometry::optimization::VPolytope
        struct /* VPolytope */ {
          // Source: drake/geometry/optimization/vpolytope.h
          const char* doc =
R"""(A polytope described using the vertex representation. The set is
defined as the convex hull of the vertices. The vertices are not
guaranteed to be in any particular order, nor to be minimal (some
vertices could be strictly in the interior of the set).

Note: Unlike the half-space representation, this definition means the
set is always bounded (hence the name polytope, instead of
polyhedron).

A VPolytope is empty if and only if it is composed of zero vertices,
i.e., if vertices_.cols() == 0. This includes the zero-dimensional
case. If vertices_.rows() == 0 but vertices_.cols() > 0, we treat this
as having one or more copies of 0 in the zero-dimensional vector space
{0}. If vertices_.rows() and vertices_.cols() are zero, we treat this
as no points in {0}, which is empty.)""";
          // Symbol: drake::geometry::optimization::VPolytope::GetMinimalRepresentation
          struct /* GetMinimalRepresentation */ {
            // Source: drake/geometry/optimization/vpolytope.h
            const char* doc =
R"""(Creates a new VPolytope whose vertices are guaranteed to be minimal,
i.e., if we remove any point from its vertices, then the convex hull
of the remaining vertices is a strict subset of the polytope. In the
2D case the vertices of the new VPolytope are ordered
counter-clockwise from the negative X axis. For all other cases an
order is not guaranteed. If the VPolytope is not full-dimensional, we
perform computations in a coordinate system of its affine hull.
``tol`` specifies the numerical tolerance used in the computation of
the affine hull.)""";
          } GetMinimalRepresentation;
          // Symbol: drake::geometry::optimization::VPolytope::MakeBox
          struct /* MakeBox */ {
            // Source: drake/geometry/optimization/vpolytope.h
            const char* doc =
R"""(Constructs a polyhedron as an axis-aligned box from the lower and
upper corners.)""";
          } MakeBox;
          // Symbol: drake::geometry::optimization::VPolytope::MakeUnitBox
          struct /* MakeUnitBox */ {
            // Source: drake/geometry/optimization/vpolytope.h
            const char* doc =
R"""(Constructs the L∞-norm unit box in ``dim`` dimensions, {x | |x|∞ <= 1
}. This is an axis-aligned box, centered at the origin, with edge
length 2.)""";
          } MakeUnitBox;
          // Symbol: drake::geometry::optimization::VPolytope::ToShapeConvex
          struct /* ToShapeConvex */ {
            // Source: drake/geometry/optimization/vpolytope.h
            const char* doc =
R"""(Creates a geometry∷Convex shape using the vertices of this VPolytope.
The convex_label is passed as the 'label' of the Convex object.

Precondition:
    ambient_dimension() == 3.)""";
          } ToShapeConvex;
          // Symbol: drake::geometry::optimization::VPolytope::VPolytope
          struct /* ctor */ {
            // Source: drake/geometry/optimization/vpolytope.h
            const char* doc =
R"""(Constructs a set with no vertices in the zero-dimensional space, which
is empty (by convention).)""";
            // Source: drake/geometry/optimization/vpolytope.h
            const char* doc_vertices =
R"""(Constructs the polytope from a d-by-n matrix, where d is the ambient
dimension, and n is the number of vertices. The vertices do not have
to be ordered, nor minimal (they can contain points inside their
convex hull).)""";
            // Source: drake/geometry/optimization/vpolytope.h
            const char* doc_hpolyhedron =
R"""(Constructs the polytope from a bounded polyhedron (using Qhull). If
the HPolyhedron is not full-dimensional, we perform computations in a
coordinate system of its affine hull. ``tol`` specifies the numerical
tolerance used in the computation of the affine hull. See the
documentation of AffineSubspace for more details. A loose tolerance is
necessary for the built-in solvers, but a tighter tolerance can be
used with commercial solvers (e.g. Gurobi and Mosek).

Raises:
    RuntimeError if H is unbounded or if Qhull terminates with an
    error.)""";
            // Source: drake/geometry/optimization/vpolytope.h
            const char* doc_scenegraph =
R"""(Constructs the polytope from a SceneGraph geometry.)""";
          } ctor;
          // Symbol: drake::geometry::optimization::VPolytope::WriteObj
          struct /* WriteObj */ {
            // Source: drake/geometry/optimization/vpolytope.h
            const char* doc =
R"""(Uses qhull to compute the Delaunay triangulation and then writes the
vertices and faces to ``filename`` in the Wavefront Obj format. Note
that the extension ``.obj`` is not automatically added to the
``filename``.

Precondition:
    ambient_dimension() == 3.)""";
          } WriteObj;
          // Symbol: drake::geometry::optimization::VPolytope::vertices
          struct /* vertices */ {
            // Source: drake/geometry/optimization/vpolytope.h
            const char* doc =
R"""(Returns the vertices in a d-by-n matrix, where d is the ambient
dimension, and n is the number of vertices.)""";
          } vertices;
        } VPolytope;
      } optimization;
    } geometry;
  } drake;
} pydrake_doc_geometry_optimization;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
