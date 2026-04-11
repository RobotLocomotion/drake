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

// #include "drake/geometry/proximity/aabb.h"
// #include "drake/geometry/proximity/boxes_overlap.h"
// #include "drake/geometry/proximity/bvh.h"
// #include "drake/geometry/proximity/bvh_updater.h"
// #include "drake/geometry/proximity/calc_obb.h"
// #include "drake/geometry/proximity/collision_filter.h"
// #include "drake/geometry/proximity/contact_surface_utility.h"
// #include "drake/geometry/proximity/deformable_contact_geometries.h"
// #include "drake/geometry/proximity/deformable_contact_internal.h"
// #include "drake/geometry/proximity/deformable_field_intersection.h"
// #include "drake/geometry/proximity/deformable_mesh_intersection.h"
// #include "drake/geometry/proximity/detect_zero_simplex.h"
// #include "drake/geometry/proximity/field_intersection.h"
// #include "drake/geometry/proximity/hydroelastic_internal.h"
// #include "drake/geometry/proximity/inflate_mesh.h"
// #include "drake/geometry/proximity/make_box_field.h"
// #include "drake/geometry/proximity/make_box_mesh.h"
// #include "drake/geometry/proximity/make_capsule_field.h"
// #include "drake/geometry/proximity/make_capsule_mesh.h"
// #include "drake/geometry/proximity/make_convex_field.h"
// #include "drake/geometry/proximity/make_convex_hull_mesh.h"
// #include "drake/geometry/proximity/make_convex_mesh.h"
// #include "drake/geometry/proximity/make_cylinder_field.h"
// #include "drake/geometry/proximity/make_cylinder_mesh.h"
// #include "drake/geometry/proximity/make_ellipsoid_field.h"
// #include "drake/geometry/proximity/make_ellipsoid_mesh.h"
// #include "drake/geometry/proximity/make_mesh_field.h"
// #include "drake/geometry/proximity/make_mesh_from_vtk.h"
// #include "drake/geometry/proximity/make_sphere_field.h"
// #include "drake/geometry/proximity/make_sphere_mesh.h"
// #include "drake/geometry/proximity/mesh_field_linear.h"
// #include "drake/geometry/proximity/mesh_half_space_intersection.h"
// #include "drake/geometry/proximity/mesh_intersection.h"
// #include "drake/geometry/proximity/mesh_plane_intersection.h"
// #include "drake/geometry/proximity/mesh_to_vtk.h"
// #include "drake/geometry/proximity/mesh_traits.h"
// #include "drake/geometry/proximity/meshing_utilities.h"
// #include "drake/geometry/proximity/obb.h"
// #include "drake/geometry/proximity/obj_to_surface_mesh.h"
// #include "drake/geometry/proximity/plane.h"
// #include "drake/geometry/proximity/polygon_surface_mesh.h"
// #include "drake/geometry/proximity/polygon_surface_mesh_field.h"
// #include "drake/geometry/proximity/polygon_to_triangle_mesh.h"
// #include "drake/geometry/proximity/posed_half_space.h"
// #include "drake/geometry/proximity/sorted_triplet.h"
// #include "drake/geometry/proximity/tessellation_strategy.h"
// #include "drake/geometry/proximity/triangle_surface_mesh.h"
// #include "drake/geometry/proximity/triangle_surface_mesh_field.h"
// #include "drake/geometry/proximity/volume_mesh.h"
// #include "drake/geometry/proximity/volume_mesh_field.h"
// #include "drake/geometry/proximity/volume_mesh_refiner.h"
// #include "drake/geometry/proximity/volume_mesh_topology.h"
// #include "drake/geometry/proximity/volume_to_surface_mesh.h"
// #include "drake/geometry/proximity/vtk_to_volume_mesh.h"

// Symbol: pydrake_doc_geometry_proximity
constexpr struct /* pydrake_doc_geometry_proximity */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::geometry
    struct /* geometry */ {
      // Symbol: drake::geometry::Aabb
      struct /* Aabb */ {
        // Source: drake/geometry/proximity/aabb.h
        const char* doc =
R"""(Axis-aligned bounding box. The box is defined in a canonical frame B
such that it is centered on Bo and its extents are aligned with B's
axes. However, the box is posed in a hierarchical frame H. Because
this is an *axis-aligned* bounding box, ``R_HB = I``. Therefore the
pose of the box is completely captured with p_HoBo_H (see center()).

Because of this, an instance of Aabb is a frame-dependent quantity and
should be expressed that way. For example, for a mesh measured and
expressed in frame M, the bounding boxes on its triangles will be
measured and expressed in the same frame.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    auto mesh_M = ...;
    Aabb bv_M = ...;  // A bounding volume for mesh_M in the same frame.

.. raw:: html

    </details>)""";
        // Symbol: drake::geometry::Aabb::Aabb
        struct /* ctor */ {
          // Source: drake/geometry/proximity/aabb.h
          const char* doc =
R"""(Constructs an axis-aligned bounding box measured and expressed in
frame H.

Parameter ``p_HoBo``:
    The position vector from the hierarchy frame's origin to the box's
    canonical origin, expressed in frame H. The box is centered on Bo
    and aligned with Bx, By, and Bz.

Parameter ``half_width``:
    The *half* measures of the box in each of the Bx, By, and Bz
    directions. (Also the half measures in the Hx, Hy, and Hz
    directions because R_HB = I.)

Precondition:
    half_width.x(), half_width.y(), half_width.z() are not negative.)""";
        } ctor;
        // Symbol: drake::geometry::Aabb::CalcVolume
        struct /* CalcVolume */ {
          // Source: drake/geometry/proximity/aabb.h
          const char* doc =
R"""(Returns:
    Volume of the bounding box.)""";
        } CalcVolume;
        // Symbol: drake::geometry::Aabb::Equal
        struct /* Equal */ {
          // Source: drake/geometry/proximity/aabb.h
          const char* doc =
R"""(Compares the values of the two Aabb instances for exact equality down
to the last bit. Assumes that the quantities are measured and
expressed in the same frame.)""";
        } Equal;
        // Symbol: drake::geometry::Aabb::HasOverlap
        struct /* HasOverlap */ {
          // Source: drake/geometry/proximity/aabb.h
          const char* doc_aabb_aabb =
R"""(Reports whether the two axis-aligned bounding boxes ``a_G`` and
``b_H`` intersect. The poses of ``a_G`` and ``b_H`` are defined in
their corresponding hierarchy frames G and H, respectively.

Parameter ``a_G``:
    The first axis-aligned box.

Parameter ``b_H``:
    The second axis-aligned box.

Parameter ``X_GH``:
    The relative pose between hierarchy frame G and hierarchy frame H.

Returns:
    ``True`` if the boxes intersect.)""";
          // Source: drake/geometry/proximity/aabb.h
          const char* doc_aabb_obb =
R"""(Reports whether axis-aligned bounding box ``aabb_G`` intersects the
given oriented bounding box ``obb_H``. The poses of ``aabb_G`` and
``obb_H`` are defined in their corresponding hierarchy frames G and H,
respectively.

Parameter ``aabb_G``:
    The axis-aligned box.

Parameter ``obb_H``:
    The oriented box.

Parameter ``X_GH``:
    The relative pose between the aabb hierarchy frame G and the obb
    hierarchy frame H.

Returns:
    ``True`` if the boxes intersect.)""";
          // Source: drake/geometry/proximity/aabb.h
          const char* doc_aabb_plane =
R"""(Checks whether bounding volume ``bv`` intersects the given plane. The
bounding volume is centered on its canonical frame B, and B is posed
in the corresponding hierarchy frame H. The plane is defined in frame
P.

The box and plane intersect if *any* point within the bounding volume
has zero height (see CalcHeight()).

Parameter ``bv_H``:
    The bounding box to test.

Parameter ``plane_P``:
    The plane to test against the ``bv``. The plane is expressed in
    frame P, therefore, to evaluate the height of a point with respect
    to it, that point must be measured and expressed in P.

Parameter ``X_PH``:
    The relative pose between the hierarchy frame H and the plane
    frame P.

Returns:
    ``True`` if the plane intersects the box.)""";
          // Source: drake/geometry/proximity/aabb.h
          const char* doc_aabb_halfspace =
R"""(Checks whether bounding volume ``bv`` intersects the given half space.
The bounding volume is centered on its canonical frame B, and B is
posed in the corresponding hierarchy frame H. The half space is
defined in its canonical frame C (such that the boundary plane of the
half space is perpendicular to Cz and Co lies on the boundary plane).

The box and halfspace intersect if *any* point within the bounding
volume has a height less than or equal to zero.

Parameter ``bv_H``:
    The bounding box to test.

Parameter ``hs_C``:
    The half space to test against the ``bv``. The half space is
    expressed in Frame C, therefore, to evaluate the signed distance
    of a point with respect to it, that point must be measured and
    expressed in C.

Parameter ``X_CH``:
    The relative pose between the hierarchy halfspace canonical frame
    C and the box frame B.

Returns:
    ``True`` if the half space intersects the box.)""";
        } HasOverlap;
        // Symbol: drake::geometry::Aabb::center
        struct /* center */ {
          // Source: drake/geometry/proximity/aabb.h
          const char* doc =
R"""(Returns the center of the box -- equivalent to the position vector
from the hierarchy frame's origin Ho to ``this`` box's origin Bo:
``p_HoBo_H``.)""";
        } center;
        // Symbol: drake::geometry::Aabb::half_width
        struct /* half_width */ {
          // Source: drake/geometry/proximity/aabb.h
          const char* doc = R"""(Returns the half_width.)""";
        } half_width;
        // Symbol: drake::geometry::Aabb::lower
        struct /* lower */ {
          // Source: drake/geometry/proximity/aabb.h
          const char* doc =
R"""(The point on the axis-aligned box with the smallest measures along the
Bx-, By-, and Bz-directions.)""";
        } lower;
        // Symbol: drake::geometry::Aabb::pose
        struct /* pose */ {
          // Source: drake/geometry/proximity/aabb.h
          const char* doc =
R"""(Returns the pose X_HB of the box frame B in the hierarchy frame H.)""";
        } pose;
        // Symbol: drake::geometry::Aabb::upper
        struct /* upper */ {
          // Source: drake/geometry/proximity/aabb.h
          const char* doc =
R"""(The point on the axis-aligned box with the largest measures along the
Bx-, By-, and Bz-directions.)""";
        } upper;
      } Aabb;
      // Symbol: drake::geometry::AabbMaker
      struct /* AabbMaker */ {
        // Source: drake/geometry/proximity/aabb.h
        const char* doc =
R"""(AabbMaker implements the logic to fit an Aabb to a collection of
points. The points are the position of a subset of vertices in a mesh.
The Aabb will be measured and expressed in the same frame as the mesh.

This serves as the interface to Bvh, allowing the Bvh to fit volumes
to geometry without knowing the details of the bounding volume types.

Template parameter ``MeshType``:
    is either TriangleSurfaceMesh<T> or VolumeMesh<T>, where T is
    double or AutoDiffXd.)""";
        // Symbol: drake::geometry::AabbMaker::AabbMaker<type-parameter-0-0>
        struct /* ctor */ {
          // Source: drake/geometry/proximity/aabb.h
          const char* doc =
R"""(Constructs the maker with the reference mesh and the subset of
vertices to fit (indicated by corresponding index).

Parameter ``mesh_M``:
    The mesh frame M.

Parameter ``vertices``:
    The subset of vertices to fit.

Precondition:
    ``vertices`` is not empty, and each of its entry is in the range
    [0, mesh_M.num_vertices()).)""";
        } ctor;
        // Symbol: drake::geometry::AabbMaker::Compute
        struct /* Compute */ {
          // Source: drake/geometry/proximity/aabb.h
          const char* doc =
R"""(Computes the bounding volume of the vertices specified in the
constructor.

Returns ``aabb_M``:
    The axis-aligned bounding box posed in mesh frame M.)""";
        } Compute;
      } AabbMaker;
      // Symbol: drake::geometry::CalcObb
      struct /* CalcObb */ {
        // Source: drake/geometry/proximity/calc_obb.h
        const char* doc =
R"""(Calculates the oriented bounding box (OBB) for the Shape in its
canonical frame. Returns ``std∷nullopt`` if the Shape is HalfSpace
which doesn't have a bounding box.

Raises:
    RuntimeError if a referenced file cannot be opened.)""";
      } CalcObb;
      // Symbol: drake::geometry::ConvertVolumeToSurfaceMesh
      struct /* ConvertVolumeToSurfaceMesh */ {
        // Source: drake/geometry/proximity/volume_to_surface_mesh.h
        const char* doc =
R"""(Converts a tetrahedral volume mesh to a triangulated surface mesh of
the boundary surface of the volume.

Parameter ``volume``:
    The tetrahedral volume mesh, whose vertex positions are measured
    and expressed in some frame E.

Returns:
    The triangulated surface mesh, whose vertex positions are measured
    and expressed in the same frame E of the volume mesh.

Precondition:
    The vertices of the volume mesh are unique. Adjacent tetrahedra
    share the same vertices, instead of repeating the vertices with
    the same coordinates. Otherwise, the returned surface mesh will
    have extra triangles in addition to the boundary triangles of the
    volume.)""";
      } ConvertVolumeToSurfaceMesh;
      // Symbol: drake::geometry::MeshFieldLinear
      struct /* MeshFieldLinear */ {
        // Source: drake/geometry/proximity/mesh_field_linear.h
        const char* doc =
R"""(MeshFieldLinear represents a continuous piecewise-linear scalar field
``f`` defined on a (triangular or tetrahedral) mesh; the field value
changes linearly within each element E (triangle or tetrahedron), and
the gradient ∇f is constant within each element. The field is
continuous across adjacent elements, but its gradient is discontinuous
from one element to the other.

To represent a piecewise linear field f, we store one field value per
vertex of the mesh. Each element E (triangle or tetrahedron) has (d+1)
vertices, where d is the dimension of the element. For triangle, d =
2, and for tetrahedron, d = 3.

On each element E, we define a linear function fᵉ:ℝ³→ℝ using the field
values at vertices of E. The gradient ∇fᵉ:ℝ³→ℝ³ is a constant map, so
we write ∇fᵉ for the constant gradient vector on E as well. For a
point Q in element E, we have:

f(Q) = fᵉ(Q) for Q ∈ E, ∇f(Q) = ∇fᵉ for Q ∈ E.

Notice that the domain of fᵉ is the entire space of ℝ³, while the
domain of f is the underlying space of the mesh.

The following sections are details for interested readers.

** Barycentric coordinate **

For a linear triangle or tetrahedron element E in 3-D, we use
barycentric coordinate:

(b₀, b₁, b₂) for triangle, (b₀, b₁, b₂, b₃) for tetrahedron, ∑bᵢ = 1,
bᵢ ≥ 0,

to identify a point Q that lies in the simplicial element E. The
coefficient bᵢ is the weight of vertex Vᵉᵢ of the element E, where the
index i is a local index within the element E, not the global index of
the entire mesh. In other words, vertex Vᵉᵢ is the iᵗʰ vertex of E,
not the iᵗʰ vertex among all vertices in the mesh. The point Q in E
can be expressed as:

Q = ∑bᵉᵢ(Q)Vᵉᵢ,

where we indicate the barycentric coordinate of a point Q on an
element E as bᵉᵢ(Q).

** Field value from barycentric coordinates **

At a point Q in element E, the piecewise linear field f has value:

f(Q) = fᵉ(Q) = ∑bᵉᵢ(Q)Fᵉᵢ

where Fᵉᵢ is the field value at the iᵗʰ vertex of element E.

** Frame dependency **

A MeshFieldLinear is a frame-dependent quantity. Instances of a field
should be named, as with any other frame-dependent quantity, with a
trailing _F indicating the field's frame F. The field's frame is
implicitly defined to be the same as the mesh's frame on which the
field is instantiated. The field's frame affects two APIs:

- The gradients reported by EvaluateGradient() are expressed in the field's
frame.
- The cartesian point passed to EvaluateCartesian() must be measured and
expressed in the field's frame.

The field (along with its corresponding mesh) can be transformed into
a new frame by invoking the TransformVertices() method on the mesh and
Transform() on the field, passing the same math∷RigidTransform to
both.

** Gradient **

Consider each bᵉᵢ:ℝ³→ℝ as a linear function, its gradient ∇bᵉᵢ:ℝ³→ℝ³
is a constant map, and we write ∇bᵉᵢ for the constant gradient vector.
The gradient of the piecewise linear field f at a point Q in an
element E is:

∇f(Q) = ∇fᵉ = ∑Fᵉᵢ∇bᵉᵢ.

** Field value from Cartesian coordinates **

At a point Q in element E, the piecewise linear field f has value:

f(Q) = ∇fᵉ⋅Q + fᵉ(0,0,0).

Notice that (0,0,0) may or may not lie in element E.

Template parameter ``T``:
    a valid Eigen scalar for field values.

Template parameter ``MeshType``:
    the type of the meshes: TriangleSurfaceMesh or VolumeMesh.)""";
        // Symbol: drake::geometry::MeshFieldLinear::CloneAndSetMesh
        struct /* CloneAndSetMesh */ {
          // Source: drake/geometry/proximity/mesh_field_linear.h
          const char* doc =
R"""(Copy to a new MeshFieldLinear and set the new MeshFieldLinear to use a
new compatible mesh. MeshFieldLinear needs a mesh to operate; however,
MeshFieldLinear does not own the mesh. In fact, several
MeshFieldLinear objects can use the same mesh.)""";
        } CloneAndSetMesh;
        // Symbol: drake::geometry::MeshFieldLinear::Equal
        struct /* Equal */ {
          // Source: drake/geometry/proximity/mesh_field_linear.h
          const char* doc =
R"""(Checks to see whether the given MeshFieldLinear object is equal via
deep exact comparison. The name of the objects are exempt from this
comparison. NaNs are treated as not equal as per the IEEE standard.

Parameter ``field``:
    The field for comparison.

Returns:
    ``True`` if the given field is equal.)""";
        } Equal;
        // Symbol: drake::geometry::MeshFieldLinear::Evaluate
        struct /* Evaluate */ {
          // Source: drake/geometry/proximity/mesh_field_linear.h
          const char* doc =
R"""(Evaluates the field value at a location on an element.

The return type depends on both the field's scalar type ``T`` and the
Barycentric coordinate type ``B``. See
drake∷geometry∷promoted_numerical "promoted_numerical_t" for details.

Warning:
    This can only be evaluated if the underlying MeshType itself
    supports barycentric evaluation (e.g., compare TriangleSurfaceMesh
    with PolygonSurfaceMesh).

Parameter ``e``:
    The index of the element.

Parameter ``b``:
    The barycentric coordinates.

Raises:
    RuntimeError if MeshType doesn't support Barycentric coordinates.

Template parameter ``B``:
    The scalar type for the barycentric coordinate.)""";
        } Evaluate;
        // Symbol: drake::geometry::MeshFieldLinear::EvaluateAtMo
        struct /* EvaluateAtMo */ {
          // Source: drake/geometry/proximity/mesh_field_linear.h
          const char* doc =
R"""((Advanced) Evaluates the linear function associated with element e at
the mesh origin Mo.

Parameter ``e``:
    The index of the element.

Precondition:
    e ∈ [0, this->mesh().num_elements()).

Precondition:
    The field has valid gradients.)""";
        } EvaluateAtMo;
        // Symbol: drake::geometry::MeshFieldLinear::EvaluateAtVertex
        struct /* EvaluateAtVertex */ {
          // Source: drake/geometry/proximity/mesh_field_linear.h
          const char* doc =
R"""(Evaluates the field value at a vertex.

Parameter ``v``:
    The index of the vertex.

Precondition:
    v ∈ [0, this->mesh().num_vertices()).)""";
        } EvaluateAtVertex;
        // Symbol: drake::geometry::MeshFieldLinear::EvaluateCartesian
        struct /* EvaluateCartesian */ {
          // Source: drake/geometry/proximity/mesh_field_linear.h
          const char* doc =
R"""(Evaluates the field at a point Qp on an element. If the element is a
tetrahedron, Qp is the input point Q. If the element is a triangle, Qp
is the projection of Q on the triangle's plane.

If gradients have been calculated, it evaluates the field value
directly. Otherwise, it converts Cartesian coordinates to barycentric
coordinates for barycentric interpolation.

The return type depends on both the field's scalar type ``T`` and the
Cartesian coordinate type ``C``. See drake∷geometry∷promoted_numerical
"promoted_numerical_t" for details.

Parameter ``e``:
    The index of the element.

Parameter ``p_MQ``:
    The position of point Q expressed in frame M, in Cartesian
    coordinates. M is the frame of the mesh.

Raises:
    RuntimeError if the field does not have gradients defined *and*
    the MeshType doesn't support Barycentric coordinates.

Template parameter ``C``:
    must be either ``double`` or ``AutoDiffXd``.)""";
        } EvaluateCartesian;
        // Symbol: drake::geometry::MeshFieldLinear::EvaluateGradient
        struct /* EvaluateGradient */ {
          // Source: drake/geometry/proximity/mesh_field_linear.h
          const char* doc =
R"""(Evaluates the gradient in the domain of the element indicated by
``e``. The gradient is a vector in R³ expressed in frame M. For
surface meshes, it will particularly lie parallel to the plane of the
corresponding triangle.

Raises:
    RuntimeError if the gradient vector was not calculated.

Raises:
    RuntimeError if the gradient field is marked degenerate.)""";
        } EvaluateGradient;
        // Symbol: drake::geometry::MeshFieldLinear::EvaluateMax
        struct /* EvaluateMax */ {
          // Source: drake/geometry/proximity/mesh_field_linear.h
          const char* doc =
R"""((Advanced) Evaluates the maximum field value on an element.

Parameter ``e``:
    The index of the element.

Precondition:
    e ∈ [0, this->mesh().num_elements()).)""";
        } EvaluateMax;
        // Symbol: drake::geometry::MeshFieldLinear::EvaluateMin
        struct /* EvaluateMin */ {
          // Source: drake/geometry/proximity/mesh_field_linear.h
          const char* doc =
R"""((Advanced) Evaluates the minimum field value on an element.

Parameter ``e``:
    The index of the element.

Precondition:
    e ∈ [0, this->mesh().num_elements()).)""";
        } EvaluateMin;
        // Symbol: drake::geometry::MeshFieldLinear::MeshFieldLinear<T, MeshType>
        struct /* ctor */ {
          // Source: drake/geometry/proximity/mesh_field_linear.h
          const char* doc_3args_values_mesh_gradient_mode =
R"""(Constructs a MeshFieldLinear.

Parameter ``values``:
    The field value at each vertex of the mesh.

Parameter ``mesh``:
    The mesh M to which this field refers.

Parameter ``gradient_mode``:
    Whether to calculate gradient field, and how to report failures.
    Calculating gradient allows EvaluateCartesian() to evaluate the
    field directly instead of converting Cartesian coordinates to
    barycentric coordinates first. If no gradient is calculated,
    EvaluateCartesian() will be slower. On the other hand, calculating
    gradient requires certain quality from mesh elements. If the mesh
    quality is very poor, calculating gradient may either throw or
    mark the gradient field as degenerate. See
    is_gradient_field_degenerate(). The default is to succeed or
    throw.

You can use the parameter ``gradient_mode`` to trade time and space of
this constructor for speed of EvaluateCartesian(). For
``gradient_mode`` != ``kNone`` (`kOkOrThrow` by default, or
``kOkOrMarkDegenerate`` similarly) and good mesh quality, this
constructor will take longer time to compute and will store one
field-gradient vector for each element in the mesh, but the
interpolation by EvaluateCartesian() will be faster because we will
use a dot product with the Cartesian coordinates directly, instead of
solving a linear system to convert Cartesian coordinates to
barycentric coordinates first.

When ``gradient_mode`` != ``kNone`` and gradient calculation succeeds,
EvaluateGradient() on a mesh element will be available. Otherwise,
EvaluateGradient() will ``throw``.

The following features are independent of the choice of
``gradient_mode``.

- Evaluating the field at a vertex.
- Evaluating the field at a user-given barycentric coordinate.

Note:
    When ``gradient_mode`` != ``kNone``, a poor quality element can
    cause numerical errors in calculating field gradients. A poor
    quality element is defined as having an extremely large aspect
    ratio R=E/h, where E is the longest edge length and h is the
    shortest height. A height of a triangular element is the distance
    between a vertex and its opposite edge. A height of a tetrahedral
    element is the distance between a vertex and its opposite
    triangular face. For example, an extremely skinny triangle has
    poor quality, and a tetrahedron with four vertices almost
    co-planar also has poor quality. The exact threshold of the
    acceptable aspect ratio depends on many factors including the
    underlying scalar type and the exact shape and size of the
    element; however, a rough conservative estimation is 1e12.

Precondition:
    The ``mesh`` is non-null, and the number of entries in ``values``
    is the same as the number of vertices of the mesh.)""";
          // Source: drake/geometry/proximity/mesh_field_linear.h
          const char* doc_3args_values_mesh_gradients =
R"""((Advanced) Constructor variant which receives the pre-computed,
per-element gradients of the field. ``gradients[i]`` is the gradient
of the linear function defined on the ith element of ``mesh``.

The caller is responsible for making sure that the gradients are
consistent with the field ``values`` defined at the vertices. Failure
to do so will lead to nonsensical results when evaluating the field
*near* a mesh vertex as opposed to *at* the vertex.

As with the other constructor, ``mesh`` must remain alive at least as
long as this field instance.

Precondition:
    gradients.size() == mesh.num_elements().)""";
        } ctor;
        // Symbol: drake::geometry::MeshFieldLinear::Transform
        struct /* Transform */ {
          // Source: drake/geometry/proximity/mesh_field_linear.h
          const char* doc =
R"""((Advanced) Transforms this mesh field to be measured and expressed in
frame N (from its original frame M). See the class documentation for
further details.

Warning:
    This method should always be invoked in tandem with the
    transformation of the underlying mesh into the same frame
    (TransformVertices()). To be safe, the mesh should be transformed
    first.)""";
        } Transform;
        // Symbol: drake::geometry::MeshFieldLinear::is_gradient_field_degenerate
        struct /* is_gradient_field_degenerate */ {
          // Source: drake/geometry/proximity/mesh_field_linear.h
          const char* doc =
R"""(Returns:
    true iff the gradient field could not be computed, and the mesh
    was constructed with MeshGradientMode∷kOkOrMarkDegenerate.)""";
        } is_gradient_field_degenerate;
        // Symbol: drake::geometry::MeshFieldLinear::max_values
        struct /* max_values */ {
          // Source: drake/geometry/proximity/mesh_field_linear.h
          const char* doc =
R"""(The maximum field value on each element.)""";
        } max_values;
        // Symbol: drake::geometry::MeshFieldLinear::mesh
        struct /* mesh */ {
          // Source: drake/geometry/proximity/mesh_field_linear.h
          const char* doc = R"""(The mesh M to which this field refers.)""";
        } mesh;
        // Symbol: drake::geometry::MeshFieldLinear::min_values
        struct /* min_values */ {
          // Source: drake/geometry/proximity/mesh_field_linear.h
          const char* doc =
R"""(The minimum field value on each element.)""";
        } min_values;
        // Symbol: drake::geometry::MeshFieldLinear::values
        struct /* values */ {
          // Source: drake/geometry/proximity/mesh_field_linear.h
          const char* doc = R"""(The field value at each vertex.)""";
        } values;
      } MeshFieldLinear;
      // Symbol: drake::geometry::MeshGradientMode
      struct /* MeshGradientMode */ {
        // Source: drake/geometry/proximity/mesh_field_linear.h
        const char* doc =
R"""(Specify whether to generate gradients, and how to handle numerical
failures.)""";
        // Symbol: drake::geometry::MeshGradientMode::kNone
        struct /* kNone */ {
          // Source: drake/geometry/proximity/mesh_field_linear.h
          const char* doc = R"""(Don't compute gradients at all.)""";
        } kNone;
        // Symbol: drake::geometry::MeshGradientMode::kOkOrMarkDegenerate
        struct /* kOkOrMarkDegenerate */ {
          // Source: drake/geometry/proximity/mesh_field_linear.h
          const char* doc =
R"""(If gradient computation fails, mark it degenerate. See
MeshFieldLinear∷is_gradient_field_degenerate().)""";
        } kOkOrMarkDegenerate;
        // Symbol: drake::geometry::MeshGradientMode::kOkOrThrow
        struct /* kOkOrThrow */ {
          // Source: drake/geometry/proximity/mesh_field_linear.h
          const char* doc =
R"""(If gradient computation fails, throw an exception.)""";
        } kOkOrThrow;
      } MeshGradientMode;
      // Symbol: drake::geometry::Obb
      struct /* Obb */ {
        // Source: drake/geometry/proximity/obb.h
        const char* doc = R"""()""";
        // Symbol: drake::geometry::Obb::CalcVolume
        struct /* CalcVolume */ {
          // Source: drake/geometry/proximity/obb.h
          const char* doc =
R"""(Returns:
    Volume of the bounding box.)""";
        } CalcVolume;
        // Symbol: drake::geometry::Obb::Equal
        struct /* Equal */ {
          // Source: drake/geometry/proximity/obb.h
          const char* doc =
R"""(Compares the values of the two Obb instances for exact equality down
to the last bit. Assumes that the quantities are measured and
expressed in the same frame.)""";
        } Equal;
        // Symbol: drake::geometry::Obb::HasOverlap
        struct /* HasOverlap */ {
          // Source: drake/geometry/proximity/obb.h
          const char* doc_obb_obb =
R"""(Reports whether the two oriented bounding boxes ``a_G`` and ``b_H``
intersect. The poses of ``a_G`` and ``b_H`` are defined in their
corresponding hierarchy frames G and H, respectively.

Parameter ``a_G``:
    The first oriented box.

Parameter ``b_H``:
    The second oriented box.

Parameter ``X_GH``:
    The relative pose between hierarchy frame G and hierarchy frame H.

Returns:
    ``True`` if the boxes intersect.)""";
          // Source: drake/geometry/proximity/obb.h
          const char* doc_obb_aabb =
R"""(Reports whether oriented bounding box ``obb_G`` intersects the given
axis-aligned bounding box ``aabb_H``. The poses of ``obb_G`` and
``aabb_H`` are defined in their corresponding hierarchy frames G and
H, respectively.

Parameter ``obb_G``:
    The oriented box.

Parameter ``aabb_H``:
    The axis-aligned box.

Parameter ``X_GH``:
    The relative pose between the obb hierarchy frame G and the aabb
    hierarchy frame H.

Returns:
    ``True`` if the boxes intersect.)""";
          // Source: drake/geometry/proximity/obb.h
          const char* doc_obb_plane =
R"""(Checks whether bounding volume ``bv`` intersects the given plane. The
bounding volume is centered on its canonical frame B, and B is posed
in the corresponding hierarchy frame H. The plane is defined in frame
P.

The box and plane intersect if *any* point within the bounding volume
has zero height.

Parameter ``bv_H``:
    The bounding box to test.

Parameter ``plane_P``:
    The plane to test against the ``bv``. The plane is expressed in
    frame P, therefore, to evaluate the height of a point with respect
    to it, that point must be measured and expressed in P.

Parameter ``X_PH``:
    The relative pose between the hierarchy frame H and the plane
    frame P.

Returns:
    ``True`` if the plane intersects the box.)""";
          // Source: drake/geometry/proximity/obb.h
          const char* doc_obb_halfspace =
R"""(Checks whether bounding volume ``bv`` intersects the given half space.
The bounding volume is centered on its canonical frame B, and B is
posed in the corresponding hierarchy frame H. The half space is
defined in its canonical frame C (such that the boundary plane of the
half space is perpendicular to Cz and Co lies on the boundary plane).

The box and halfspace intersect if *any* point within the bounding
volume has a height less than or equal to zero.

Parameter ``bv_H``:
    The bounding box to test.

Parameter ``hs_C``:
    The half space to test against the ``bv``. The half space is
    expressed in Frame C, therefore, to evaluate the signed distance
    of a point with respect to it, that point must be measured and
    expressed in C.

Parameter ``X_CH``:
    The relative pose between the hierarchy frame H and the half space
    canonical frame C.

Returns:
    ``True`` if the half space intersects the box.)""";
        } HasOverlap;
        // Symbol: drake::geometry::Obb::Obb
        struct /* ctor */ {
          // Source: drake/geometry/proximity/obb.h
          const char* doc =
R"""(Constructs an oriented bounding box measured and expressed in frame H.

Parameter ``X_HB``:
    The pose of the box in the hierarchy frame H. The box is centered
    on Bo and aligned with Bx, By, and Bz.

Parameter ``half_width``:
    The *half* measures of the box in each of the Bx, By, and Bz
    directions.

Precondition:
    half_width.x(), half_width.y(), half_width.z() are not negative.)""";
        } ctor;
        // Symbol: drake::geometry::Obb::center
        struct /* center */ {
          // Source: drake/geometry/proximity/obb.h
          const char* doc =
R"""(Returns the center of the box -- equivalent to the position vector
from the hierarchy frame's origin Ho to ``this`` box's origin Bo:
``p_HoBo_H``.)""";
        } center;
        // Symbol: drake::geometry::Obb::half_width
        struct /* half_width */ {
          // Source: drake/geometry/proximity/obb.h
          const char* doc =
R"""(Returns the half_width -- equivalent to the position vector from the
box's center Bo to the box's first octant (+,+,+) corner U expressed
in the box's frame B: ``p_BoU_B``.)""";
        } half_width;
        // Symbol: drake::geometry::Obb::pose
        struct /* pose */ {
          // Source: drake/geometry/proximity/obb.h
          const char* doc =
R"""(Returns the pose X_HB of the box frame B in the hierarchy frame H)""";
        } pose;
      } Obb;
      // Symbol: drake::geometry::ObbMaker
      struct /* ObbMaker */ {
        // Source: drake/geometry/proximity/obb.h
        const char* doc =
R"""(ObbMaker performs an algorithm to create an oriented bounding box that
fits a specified set of vertices in a mesh.

Template parameter ``MeshType``:
    is TriangleSurfaceMesh<T>, VolumeMesh<T>, PolygonSurfaceMesh<T>,
    where T is double or AutoDiffXd.)""";
        // Symbol: drake::geometry::ObbMaker::Compute
        struct /* Compute */ {
          // Source: drake/geometry/proximity/obb.h
          const char* doc =
R"""(Computes the bounding volume of the vertices specified in the
constructor.

Returns ``obb_M``:
    The oriented bounding box posed in frame M.)""";
        } Compute;
        // Symbol: drake::geometry::ObbMaker::ObbMaker<type-parameter-0-0>
        struct /* ctor */ {
          // Source: drake/geometry/proximity/obb.h
          const char* doc =
R"""(Specifies the input mesh with frame M and a set of vertices to fit.

Parameter ``mesh_M``:
    The mesh that owns the vertices expressed in frame M.

Parameter ``vertices``:
    The vertices to fit.

Precondition:
    ``vertices`` is not empty, and each of its entry is in the range
    [0, V), where V is mesh_M.num_vertices().)""";
        } ctor;
      } ObbMaker;
      // Symbol: drake::geometry::Plane
      struct /* Plane */ {
        // Source: drake/geometry/proximity/plane.h
        const char* doc =
R"""(The definition of a plane in ℜ³, posed in an arbitrary frame. The
plane normal implicitly defines "above" and "below" directions
relative to the plane. The "height" of a point relative to the plane
can be queried.

It is defined with the implicit equation: ``P(x⃗ = n̂⋅x⃗- d = 0``. A
particular instance is measured and expressed in a particular frame,
such that only points measured and expressed in that same frame can be
meaningfully compared to the plane. E.g.,


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    const Vector3<T> nhat_F = ...;
    const Vector3<T> p_FP = ...;  // P is a point on the plane.
    const Plane<T> plane_F(nhat_F, p_FP);  // Plane in frame F.
    const double distance_Q = plane_F.CalcHeight(p_FQ);  // valid!
    const double distance_R = plane_F.CalcHeight(p_GR);  // invalid!

.. raw:: html

    </details>)""";
        // Symbol: drake::geometry::Plane::BoxOverlaps
        struct /* BoxOverlaps */ {
          // Source: drake/geometry/proximity/plane.h
          const char* doc =
R"""(Reports if the given box intersects this plane. The plane is specified
in a frame P (the plane normal is not necessarily aligned with Pz).
The box is specified in generic terms. It is an box whose axes are
aligned to frame B, centered on Bo, and posed in the plane's frame P.

Parameter ``half_width``:
    The half-width extents of the box along its local axes.

Parameter ``box_center_in_plane``:
    The center of the box measured and expressed in the plane's frame:
    p_PBo.

Parameter ``box_orientation_in_plane``:
    The orientation of the box expressed in the plane's frame. The ith
    column goes with the ith half width: R_PB.)""";
        } BoxOverlaps;
        // Symbol: drake::geometry::Plane::CalcHeight
        struct /* CalcHeight */ {
          // Source: drake/geometry/proximity/plane.h
          const char* doc =
R"""(Computes the height of Point Q relative to the plane. A positive
height indicates the point lies *above* the plane; negative height
indicates *below*. The point must be measured and expressed in the
same frame as the plane.

The return type depends on both the plane's scalar type ``T`` and the
given query point's scalar type ``U``. See
drake∷geometry∷promoted_numerical "promoted_numerical_t" for details.

Parameter ``point``:
    The quantity p_FQ (query point Q measured and expressed in the
    plane's frame F).)""";
        } CalcHeight;
        // Symbol: drake::geometry::Plane::Plane<T>
        struct /* ctor */ {
          // Source: drake/geometry/proximity/plane.h
          const char* doc =
R"""(Constructs a Plane in frame F which is normal to ``normal`` and passes
through the point ``point_on_plane``.

Parameter ``normal``:
    A (possibly unit-length) vector perpendicular to the plane
    expressed in Frame F (the ``n̂`` in the implicit equation). By
    default, the vector will be normalized before being stored (see
    below), becoming the nhat_F documented above.

Parameter ``point_on_plane``:
    A point on the plane measured and expressed in Frame F, p_FP. The
    ``d`` in the implicit equation is derived from this quantity.

Parameter ``already_normalized``:
    (Advanced) If ``True``, the ``normal`` will be treated as if it
    has already been normalized by the caller. It should still
    essentially have unit length. This function reserves the right to
    validate this property in debug build up to an arbitrary
    tolerance. When in doubt, allow the plane to normalize the normal
    vector.

Precondition:
    If ``already_normalized`` is ``False``, `normal` must have
    magnitude ≥ 1e-10.)""";
        } ctor;
        // Symbol: drake::geometry::Plane::reference_point
        struct /* reference_point */ {
          // Source: drake/geometry/proximity/plane.h
          const char* doc =
R"""(Returns a point on the plane, measured and expressed in frame F. This
is not necessarily the same point used to construct the plane.)""";
        } reference_point;
        // Symbol: drake::geometry::Plane::unit_normal
        struct /* unit_normal */ {
          // Source: drake/geometry/proximity/plane.h
          const char* doc =
R"""(Gets the plane's unit normal expressed in frame F.)""";
        } unit_normal;
      } Plane;
      // Symbol: drake::geometry::PolygonSurfaceMesh
      struct /* PolygonSurfaceMesh */ {
        // Source: drake/geometry/proximity/polygon_surface_mesh.h
        const char* doc =
R"""(PolygonSurfaceMesh represents a surface comprised of *polygonal*
elements (three or more sides).)""";
        // Symbol: drake::geometry::PolygonSurfaceMesh::CalcBarycentric
        struct /* CalcBarycentric */ {
          // Source: drake/geometry/proximity/polygon_surface_mesh.h
          const char* doc =
R"""(See TriangleSurfaceMesh∷CalcBaryCentric(). This implementation is
provided to maintain compatibility with MeshFieldLinear. However, it
only throws. PolygonSurfaceMesh does not support barycentric
coordinates.

Template parameter ``C``:
    must be either ``double`` or ``AutoDiffXd``.)""";
        } CalcBarycentric;
        // Symbol: drake::geometry::PolygonSurfaceMesh::CalcBoundingBox
        struct /* CalcBoundingBox */ {
          // Source: drake/geometry/proximity/polygon_surface_mesh.h
          const char* doc =
R"""(Calculates the axis-aligned bounding box of this surface mesh M.

Returns:
    the center and the size vector of the box expressed in M's frame.)""";
        } CalcBoundingBox;
        // Symbol: drake::geometry::PolygonSurfaceMesh::CalcGradientVectorOfLinearField
        struct /* CalcGradientVectorOfLinearField */ {
          // Source: drake/geometry/proximity/polygon_surface_mesh.h
          const char* doc =
R"""(This is a stub method. It is provided so that PolygonSurfaceMesh
provides a sufficient API to compile against MeshFieldLinear. However,
we expect that the gradients of the field will always be provided when
defining a MeshFieldLinear with a PolygonSurfaceMesh. Failure to
provide those gradients will cause *this* method to be invoked which
will, in turn, throw.)""";
        } CalcGradientVectorOfLinearField;
        // Symbol: drake::geometry::PolygonSurfaceMesh::Equal
        struct /* Equal */ {
          // Source: drake/geometry/proximity/polygon_surface_mesh.h
          const char* doc =
R"""(Checks to see whether the given PolygonSurfaceMesh object is equal via
deep exact comparison. NaNs are treated as not equal as per the IEEE
standard.

Parameter ``mesh``:
    The mesh for comparison.

Returns:
    ``True`` if the given mesh is equal.)""";
        } Equal;
        // Symbol: drake::geometry::PolygonSurfaceMesh::MaybeCalcGradientVectorOfLinearField
        struct /* MaybeCalcGradientVectorOfLinearField */ {
          // Source: drake/geometry/proximity/polygon_surface_mesh.h
          const char* doc =
R"""(Like CalcGradientVectorOfLinearField above, this is a stub method,
provided for compatibility with MeshFieldLinear. The empty return
value here will cause the caller to report errors.)""";
        } MaybeCalcGradientVectorOfLinearField;
        // Symbol: drake::geometry::PolygonSurfaceMesh::PolygonSurfaceMesh<T>
        struct /* ctor */ {
          // Source: drake/geometry/proximity/polygon_surface_mesh.h
          const char* doc_0args =
R"""(Advanced() Constructs an *empty* mesh. This enables compatibility with
STL container types and facilitates some unit tests. Otherwise, it
shouldn't be used.)""";
          // Source: drake/geometry/proximity/polygon_surface_mesh.h
          const char* doc_2args =
R"""(Constructs a mesh from specified vertex and mesh data.

The vertices are simply a vector of position vectors (interpreted as
being measured and expressed in the mesh's frame M).

The polygon data is more complex. Syntactically, it is a sequence of
integers which *encodes* P polygons. Each polygon can have an
arbitrary number of vertices. The encoding of the P polygons is as
follows:

|c₁|v₁₀|v₁₁|...|cᵢ|cᵢ₀|cᵢ₁|...|cₚ|cₚ₀|cₚ₁|...|

Each polygon is defined in sequence. The definition consists of an
integer indicating the *number* of vertices in that polygon (c₁, cᵢ,
and cₘ in the illustration above). The next cᵢ integers in the
sequence are zero-based indices into the vector of vertex positions
(indicating which vertices the polygon spans). The vertex indices are
sorted such that the plane normal found by applying the right-handed
rule is used as the face normal.

This implies the following: Polygon one: Located at index i₁ = 0 in
``face_data``. c₁ = face_data[i₁] is the number of vertices in polygon
one. Polygon two: Located at index i₂ = i₁ + c₁ + 1 in ``face_data``.
c₂ = face_data[i₂] is the number of vertices in polygon zero. Polygon
j: Located at index iⱼ = iⱼ₋₁ + cⱼ₋₁ + 1 cⱼ = face_data[iⱼ]

The polygons must all be planar and convex.

Parameter ``face_data``:
    The sequence of counts and indices which encode the faces of the
    mesh (see above).

Parameter ``vertices``:
    The vertex positions, measured and expressed in this mesh's frame.

Precondition:
    The indices in ``face_data`` all refer to valid indices into
    ``vertices``.

Note:
    If ``face_data`` includes a zero-area polygon, that polygon will
    have a non-NaN centroid chosen arbitrarily. For hydroelastics,
    this is acceptable because its zero area will neutralize its
    contribution to computation of contact wrench. If all polygons
    have zero area, the mesh's centroid will be chosen arbitrarily as
    well.)""";
        } ctor;
        // Symbol: drake::geometry::PolygonSurfaceMesh::ReverseFaceWinding
        struct /* ReverseFaceWinding */ {
          // Source: drake/geometry/proximity/polygon_surface_mesh.h
          const char* doc =
R"""((Internal use only) Reverses the ordering of all the faces' indices.)""";
        } ReverseFaceWinding;
        // Symbol: drake::geometry::PolygonSurfaceMesh::ScalarType
        struct /* ScalarType */ {
          // Source: drake/geometry/proximity/polygon_surface_mesh.h
          const char* doc = R"""()""";
        } ScalarType;
        // Symbol: drake::geometry::PolygonSurfaceMesh::SetAllPositions
        struct /* SetAllPositions */ {
          // Source: drake/geometry/proximity/polygon_surface_mesh.h
          const char* doc =
R"""(Updates the position of all vertices in the mesh. Each sequential
triple in p_MVs (e.g., 3i, 3i + 1, 3i + 2), i ∈ ℤ, is interpreted as a
position vector associated with the iᵗʰ vertex. The position values
are interpreted to be measured and expressed in the same frame as the
mesh to be deformed.

Parameter ``p_MVs``:
    Vertex positions for the mesh's N vertices flattened into a vector
    (where each position vector is measured and expressed in the
    mesh's original frame).

Raises:
    RuntimeError if p_MVs.size() != 3 * num_vertices())""";
        } SetAllPositions;
        // Symbol: drake::geometry::PolygonSurfaceMesh::TransformVertices
        struct /* TransformVertices */ {
          // Source: drake/geometry/proximity/polygon_surface_mesh.h
          const char* doc =
R"""((Internal use only) Transforms the vertices of this mesh from its
initial frame M to the new frame N.)""";
        } TransformVertices;
        // Symbol: drake::geometry::PolygonSurfaceMesh::area
        struct /* area */ {
          // Source: drake/geometry/proximity/polygon_surface_mesh.h
          const char* doc =
R"""(Returns area of a polygonal element.

Precondition:
    f ∈ {0, 1, 2, ..., num_faces()-1}.)""";
        } area;
        // Symbol: drake::geometry::PolygonSurfaceMesh::centroid
        struct /* centroid */ {
          // Source: drake/geometry/proximity/polygon_surface_mesh.h
          const char* doc =
R"""(Returns the geometric centroid of this mesh measured and expressed in
the mesh's frame M. (M is the frame in which this mesh's vertices are
measured and expressed.) Note that the centroid is not necessarily a
point on the surface. If the total mesh area is exactly zero, we
define the centroid to be (0,0,0).)""";
        } centroid;
        // Symbol: drake::geometry::PolygonSurfaceMesh::element
        struct /* element */ {
          // Source: drake/geometry/proximity/polygon_surface_mesh.h
          const char* doc =
R"""(Returns the polygonal element identified by the given index ``e``.

Precondition:
    e ∈ {0, 1, 2, ..., num_faces()-1}.)""";
        } element;
        // Symbol: drake::geometry::PolygonSurfaceMesh::element_centroid
        struct /* element_centroid */ {
          // Source: drake/geometry/proximity/polygon_surface_mesh.h
          const char* doc =
R"""(Returns the geometric centroid of the element indicated be index
``e``, measured and expressed in the mesh's frame M.

Precondition:
    f ∈ {0, 1, 2, ..., num_faces()-1}.)""";
        } element_centroid;
        // Symbol: drake::geometry::PolygonSurfaceMesh::face_data
        struct /* face_data */ {
          // Source: drake/geometry/proximity/polygon_surface_mesh.h
          const char* doc = R"""()""";
        } face_data;
        // Symbol: drake::geometry::PolygonSurfaceMesh::face_normal
        struct /* face_normal */ {
          // Source: drake/geometry/proximity/polygon_surface_mesh.h
          const char* doc =
R"""(Returns the unit face normal vector of a polygon. It respects the
right-handed normal rule. A near-zero-area triangle may get an
unreliable normal vector. A zero-area triangle will get a zero vector.

Precondition:
    f ∈ {0, 1, 2, ..., num_faces()-1}.)""";
        } face_normal;
        // Symbol: drake::geometry::PolygonSurfaceMesh::num_elements
        struct /* num_elements */ {
          // Source: drake/geometry/proximity/polygon_surface_mesh.h
          const char* doc =
R"""(Returns the number of elements in the mesh. For PolygonSurfaceMesh, an
element is a polygon. Returns the same number as num_faces() and
enables mesh consumers to be templated on mesh type.)""";
        } num_elements;
        // Symbol: drake::geometry::PolygonSurfaceMesh::num_faces
        struct /* num_faces */ {
          // Source: drake/geometry/proximity/polygon_surface_mesh.h
          const char* doc =
R"""(Returns the number of polygonal elements in the mesh.)""";
        } num_faces;
        // Symbol: drake::geometry::PolygonSurfaceMesh::num_vertices
        struct /* num_vertices */ {
          // Source: drake/geometry/proximity/polygon_surface_mesh.h
          const char* doc =
R"""(Returns the number of vertices in the mesh.)""";
        } num_vertices;
        // Symbol: drake::geometry::PolygonSurfaceMesh::total_area
        struct /* total_area */ {
          // Source: drake/geometry/proximity/polygon_surface_mesh.h
          const char* doc =
R"""(Returns the total area of all the faces of this surface mesh.)""";
        } total_area;
        // Symbol: drake::geometry::PolygonSurfaceMesh::vertex
        struct /* vertex */ {
          // Source: drake/geometry/proximity/polygon_surface_mesh.h
          const char* doc =
R"""(Returns the vertex identified by the given index ``v``.

Precondition:
    v ∈ {0, 1, 2, ..., num_vertices()-1}.)""";
        } vertex;
      } PolygonSurfaceMesh;
      // Symbol: drake::geometry::ReadObjToTriangleSurfaceMesh
      struct /* ReadObjToTriangleSurfaceMesh */ {
        // Source: drake/geometry/proximity/obj_to_surface_mesh.h
        const char* doc_3args_filename_scale3_on_warning =
R"""(Constructs a surface mesh from a Wavefront .obj file and optionally
scales coordinates by the given scale factor. Polygons will be
triangulated if they are not triangles already. All objects in the
.obj file will be merged into the surface mesh. See
https://en.wikipedia.org/wiki/Wavefront_.obj_file for the file format.

Parameter ``filename``:
    A valid file name with absolute path or relative path.

Parameter ``scale3``:
    A scale to coordinates.

Parameter ``on_warning``:
    An optional callback that will receive warning message(s)
    encountered while reading the mesh. When not provided, drake∷log()
    will be used.

Raises:
    RuntimeError if there is an error reading the mesh data.

Returns:
    surface mesh)""";
        // Source: drake/geometry/proximity/obj_to_surface_mesh.h
        const char* doc_3args_filename_scale_on_warning =
R"""(Variant that allows defining uniform scaling from a single scalar
value.)""";
        // Source: drake/geometry/proximity/obj_to_surface_mesh.h
        const char* doc_3args_mesh_source_scale3_on_warning =
R"""(Overload of ReadObjToTriangleSurfaceMesh(const std∷filesystem∷path&,
double) with the Wavefront .obj in a Mesh shape specification.)""";
      } ReadObjToTriangleSurfaceMesh;
      // Symbol: drake::geometry::RefineVolumeMesh
      struct /* RefineVolumeMesh */ {
        // Source: drake/geometry/proximity/volume_mesh_refiner.h
        const char* doc =
R"""(Refines a tetrahedral mesh to eliminate problematic simplices.

Parameter ``mesh``:
    The mesh to refine.

Returns:
    The refined mesh, or a copy of the input mesh if no refinement was
    needed.

Raises:
    RuntimeError if ``mesh`` is not a valid tetrahedral mesh.)""";
      } RefineVolumeMesh;
      // Symbol: drake::geometry::RefineVolumeMeshIntoVtkFileContents
      struct /* RefineVolumeMeshIntoVtkFileContents */ {
        // Source: drake/geometry/proximity/volume_mesh_refiner.h
        const char* doc =
R"""(Refines a tetrahedral mesh to eliminate problematic simplices.

Parameter ``mesh_source``:
    The mesh to refine.

Returns:
    A valid, ASCII VTK file defining the refined tetrahedral mesh.
    Represents the input mesh if no refinement was needed.

Raises:
    RuntimeError if ``mesh_source`` does not reference a valid
    VTK-formatted tetrahedral mesh.)""";
      } RefineVolumeMeshIntoVtkFileContents;
      // Symbol: drake::geometry::SurfacePolygon
      struct /* SurfacePolygon */ {
        // Source: drake/geometry/proximity/polygon_surface_mesh.h
        const char* doc =
R"""(Representation of a polygonal face in a SurfacePolygon.)""";
        // Symbol: drake::geometry::SurfacePolygon::SurfacePolygon
        struct /* ctor */ {
          // Source: drake/geometry/proximity/polygon_surface_mesh.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::geometry::SurfacePolygon::copy_to_unique
        struct /* copy_to_unique */ {
          // Source: drake/geometry/proximity/polygon_surface_mesh.h
          const char* doc =
R"""((Internal use only) Returns a copy of this, wrapped in a unique_ptr.
This function is only intended for use by Drake's Python bindings.)""";
        } copy_to_unique;
        // Symbol: drake::geometry::SurfacePolygon::num_vertices
        struct /* num_vertices */ {
          // Source: drake/geometry/proximity/polygon_surface_mesh.h
          const char* doc =
R"""(Returns the number of vertices in this face.)""";
        } num_vertices;
        // Symbol: drake::geometry::SurfacePolygon::vertex
        struct /* vertex */ {
          // Source: drake/geometry/proximity/polygon_surface_mesh.h
          const char* doc =
R"""(Returns the vertex index in PolygonSurfaceMesh of the i-th vertex of
this face.

Parameter ``i``:
    The local index of the vertex in this face.

Precondition:
    0 <= i < num_vertices())""";
        } vertex;
      } SurfacePolygon;
      // Symbol: drake::geometry::SurfaceTriangle
      struct /* SurfaceTriangle */ {
        // Source: drake/geometry/proximity/triangle_surface_mesh.h
        const char* doc =
R"""(%SurfaceTriangle represents a triangular face in a
TriangleSurfaceMesh.)""";
        // Symbol: drake::geometry::SurfaceTriangle::ReverseWinding
        struct /* ReverseWinding */ {
          // Source: drake/geometry/proximity/triangle_surface_mesh.h
          const char* doc =
R"""(Reverses the order of the vertex indices -- this essentially flips the
triangle normal based on the right-handed normal rule.)""";
        } ReverseWinding;
        // Symbol: drake::geometry::SurfaceTriangle::SurfaceTriangle
        struct /* ctor */ {
          // Source: drake/geometry/proximity/triangle_surface_mesh.h
          const char* doc_3args =
R"""(Constructs SurfaceTriangle.

Parameter ``v0``:
    Index of the first vertex in TriangleSurfaceMesh.

Parameter ``v1``:
    Index of the second vertex in TriangleSurfaceMesh.

Parameter ``v2``:
    Index of the last vertex in TriangleSurfaceMesh.

Precondition:
    index values are non-negative.)""";
          // Source: drake/geometry/proximity/triangle_surface_mesh.h
          const char* doc_1args =
R"""(Constructs SurfaceTriangle.

Parameter ``v``:
    array of three integer indices of the vertices of the triangle in
    TriangleSurfaceMesh.

Precondition:
    index values are non-negative.)""";
        } ctor;
        // Symbol: drake::geometry::SurfaceTriangle::num_vertices
        struct /* num_vertices */ {
          // Source: drake/geometry/proximity/triangle_surface_mesh.h
          const char* doc =
R"""(Returns the number of vertices in this face.)""";
        } num_vertices;
        // Symbol: drake::geometry::SurfaceTriangle::vertex
        struct /* vertex */ {
          // Source: drake/geometry/proximity/triangle_surface_mesh.h
          const char* doc =
R"""(Returns the vertex index in TriangleSurfaceMesh of the i-th vertex of
this triangle.

Parameter ``i``:
    The local index of the vertex in this triangle.

Precondition:
    0 <= i < 3)""";
        } vertex;
      } SurfaceTriangle;
      // Symbol: drake::geometry::TriangleSurfaceMesh
      struct /* TriangleSurfaceMesh */ {
        // Source: drake/geometry/proximity/triangle_surface_mesh.h
        const char* doc =
R"""(TriangleSurfaceMesh represents a union of triangles. The surface is
not necessarily continuous.)""";
        // Symbol: drake::geometry::TriangleSurfaceMesh::CalcBarycentric
        struct /* CalcBarycentric */ {
          // Source: drake/geometry/proximity/triangle_surface_mesh.h
          const char* doc =
R"""(Calculate barycentric coordinates with respect to the triangle ``t``
of the point Q'. Q' is the projection of the provided point Q on the
plane of triangle ``t``. If Q lies on the plane, Q = Q'. This
operation is expensive compared with going from barycentric to
Cartesian.

The return type depends on both the mesh's vertex position scalar type
``T`` and the Cartesian coordinate type ``C`` of the query point. See
drake∷geometry∷promoted_numerical "promoted_numerical_t" for details.

Parameter ``p_MQ``:
    The position of point Q measured and expressed in the mesh's frame
    M.

Parameter ``t``:
    The index of a triangle.

Returns ``b_Q``:
    ' The barycentric coordinates of Q' (projection of Q onto ``t`'s
    plane) relative to triangle t.

Note:
    If Q' is outside the triangle, the barycentric coordinates (b₀,
    b₁, b₂) still satisfy b₀ + b₁ + b₂ = 1; however, some bᵢ will be
    negative.

Precondition:
    t ∈ {0, 1, 2,..., num_triangles()-1}.

Template parameter ``C``:
    must be either `double`` or ``AutoDiffXd``.)""";
        } CalcBarycentric;
        // Symbol: drake::geometry::TriangleSurfaceMesh::CalcBoundingBox
        struct /* CalcBoundingBox */ {
          // Source: drake/geometry/proximity/triangle_surface_mesh.h
          const char* doc =
R"""(Calculates the axis-aligned bounding box of this surface mesh M.

Returns:
    the center and the size vector of the box expressed in M's frame.)""";
        } CalcBoundingBox;
        // Symbol: drake::geometry::TriangleSurfaceMesh::CalcCartesianFromBarycentric
        struct /* CalcCartesianFromBarycentric */ {
          // Source: drake/geometry/proximity/triangle_surface_mesh.h
          const char* doc =
R"""(Maps the barycentric coordinates ``Q_barycentric`` of a point Q in
``element_index`` to its position vector p_MQ.

The return type depends on both the mesh's vertex position scalar type
``T`` and the Barycentric coordinate type ``B`` of the query point.
See drake∷geometry∷promoted_numerical "promoted_numerical_t" for
details.

Precondition:
    ``element_index`` ∈ {0, 1, 2,..., num_triangles()-1}.)""";
        } CalcCartesianFromBarycentric;
        // Symbol: drake::geometry::TriangleSurfaceMesh::CalcGradientVectorOfLinearField
        struct /* CalcGradientVectorOfLinearField */ {
          // Source: drake/geometry/proximity/triangle_surface_mesh.h
          const char* doc =
R"""(Calculates the gradient ∇u of a linear field u on the triangle ``t``.
Field u is defined by the three field values ``field_value[i]`` at the
i-th vertex of the triangle. The gradient ∇u is expressed in the
coordinates frame of this mesh M.)""";
        } CalcGradientVectorOfLinearField;
        // Symbol: drake::geometry::TriangleSurfaceMesh::Equal
        struct /* Equal */ {
          // Source: drake/geometry/proximity/triangle_surface_mesh.h
          const char* doc =
R"""(Checks to see whether the given TriangleSurfaceMesh object is equal
via deep exact comparison. NaNs are treated as not equal as per the
IEEE standard.

Parameter ``mesh``:
    The mesh for comparison.

Returns:
    ``True`` if the given mesh is equal.)""";
        } Equal;
        // Symbol: drake::geometry::TriangleSurfaceMesh::MaybeCalcGradientVectorOfLinearField
        struct /* MaybeCalcGradientVectorOfLinearField */ {
          // Source: drake/geometry/proximity/triangle_surface_mesh.h
          const char* doc =
R"""(Calculates the gradient ∇u of a linear field u on the triangle ``t``.
Field u is defined by the three field values ``field_value[i]`` at the
i-th vertex of the triangle. The gradient ∇u is expressed in the
coordinates frame of this mesh M.)""";
        } MaybeCalcGradientVectorOfLinearField;
        // Symbol: drake::geometry::TriangleSurfaceMesh::ReverseFaceWinding
        struct /* ReverseFaceWinding */ {
          // Source: drake/geometry/proximity/triangle_surface_mesh.h
          const char* doc =
R"""((Internal use only) Reverses the ordering of all the triangles'
indices -- see SurfaceTriangle∷ReverseWinding().)""";
        } ReverseFaceWinding;
        // Symbol: drake::geometry::TriangleSurfaceMesh::ScalarType
        struct /* ScalarType */ {
          // Source: drake/geometry/proximity/triangle_surface_mesh.h
          const char* doc = R"""()""";
        } ScalarType;
        // Symbol: drake::geometry::TriangleSurfaceMesh::SetAllPositions
        struct /* SetAllPositions */ {
          // Source: drake/geometry/proximity/triangle_surface_mesh.h
          const char* doc =
R"""(Updates the position of all vertices in the mesh. Each sequential
triple in p_MVs (e.g., 3i, 3i + 1, 3i + 2), i ∈ ℤ, is interpreted as a
position vector associated with the iᵗʰ vertex. The position values
are interpreted to be measured and expressed in the same frame as the
mesh to be deformed.

Parameter ``p_MVs``:
    Vertex positions for the mesh's N vertices flattened into a vector
    (where each position vector is measured and expressed in the
    mesh's original frame).

Raises:
    RuntimeError if p_MVs.size() != 3 * num_vertices())""";
        } SetAllPositions;
        // Symbol: drake::geometry::TriangleSurfaceMesh::TransformVertices
        struct /* TransformVertices */ {
          // Source: drake/geometry/proximity/triangle_surface_mesh.h
          const char* doc =
R"""((Internal use only) Transforms the vertices of this mesh from its
initial frame M to the new frame N.)""";
        } TransformVertices;
        // Symbol: drake::geometry::TriangleSurfaceMesh::TriangleSurfaceMesh<T>
        struct /* ctor */ {
          // Source: drake/geometry/proximity/triangle_surface_mesh.h
          const char* doc =
R"""(Constructs a TriangleSurfaceMesh from triangles and vertices.

Parameter ``triangles``:
    The triangular triangles.

Parameter ``vertices``:
    The vertices.)""";
        } ctor;
        // Symbol: drake::geometry::TriangleSurfaceMesh::area
        struct /* area */ {
          // Source: drake/geometry/proximity/triangle_surface_mesh.h
          const char* doc =
R"""(Returns area of triangle ``t``.

Precondition:
    t ∈ {0, 1, 2,..., num_triangles()-1}.)""";
        } area;
        // Symbol: drake::geometry::TriangleSurfaceMesh::centroid
        struct /* centroid */ {
          // Source: drake/geometry/proximity/triangle_surface_mesh.h
          const char* doc =
R"""(Returns the area-weighted geometric centroid of this surface mesh. The
returned value is the position vector p_MSc from M's origin to the
centroid Sc, expressed in frame M. (M is the frame in which this
mesh's vertices are measured and expressed.) Note that the centroid is
not necessarily a point on the surface. If the total mesh area is
exactly zero, we define the centroid to be (0,0,0).

The centroid location is calculated *per face* not *per vertex* so is
insensitive to whether vertices are shared by triangles.)""";
        } centroid;
        // Symbol: drake::geometry::TriangleSurfaceMesh::element
        struct /* element */ {
          // Source: drake/geometry/proximity/triangle_surface_mesh.h
          const char* doc =
R"""(Returns the triangular element identified by a given index.

Parameter ``e``:
    The index of the triangular element.

Precondition:
    e ∈ {0, 1, 2,..., num_triangles()-1}.)""";
        } element;
        // Symbol: drake::geometry::TriangleSurfaceMesh::element_centroid
        struct /* element_centroid */ {
          // Source: drake/geometry/proximity/triangle_surface_mesh.h
          const char* doc =
R"""(Returns the centroid of a triangle measured and expressed in the
mesh's frame.)""";
        } element_centroid;
        // Symbol: drake::geometry::TriangleSurfaceMesh::face_normal
        struct /* face_normal */ {
          // Source: drake/geometry/proximity/triangle_surface_mesh.h
          const char* doc =
R"""(Returns the unit face normal vector of a triangle. It respects the
right-handed normal rule. A near-zero-area triangle may get an
unreliable normal vector. A zero-area triangle will get a zero vector.

Precondition:
    t ∈ {0, 1, 2,..., num_triangles()-1}.)""";
        } face_normal;
        // Symbol: drake::geometry::TriangleSurfaceMesh::num_elements
        struct /* num_elements */ {
          // Source: drake/geometry/proximity/triangle_surface_mesh.h
          const char* doc =
R"""(Returns the number of triangles in the mesh. For TriangleSurfaceMesh,
an element is a triangle. Returns the same number as num_triangles()
and enables mesh consumers to be templated on mesh type.)""";
        } num_elements;
        // Symbol: drake::geometry::TriangleSurfaceMesh::num_triangles
        struct /* num_triangles */ {
          // Source: drake/geometry/proximity/triangle_surface_mesh.h
          const char* doc =
R"""(Returns the number of triangles in the mesh.)""";
        } num_triangles;
        // Symbol: drake::geometry::TriangleSurfaceMesh::num_vertices
        struct /* num_vertices */ {
          // Source: drake/geometry/proximity/triangle_surface_mesh.h
          const char* doc =
R"""(Returns the number of vertices in the mesh.)""";
        } num_vertices;
        // Symbol: drake::geometry::TriangleSurfaceMesh::total_area
        struct /* total_area */ {
          // Source: drake/geometry/proximity/triangle_surface_mesh.h
          const char* doc =
R"""(Returns the total area of all the triangles of this surface mesh.)""";
        } total_area;
        // Symbol: drake::geometry::TriangleSurfaceMesh::triangles
        struct /* triangles */ {
          // Source: drake/geometry/proximity/triangle_surface_mesh.h
          const char* doc = R"""(Returns the triangles.)""";
        } triangles;
        // Symbol: drake::geometry::TriangleSurfaceMesh::vertex
        struct /* vertex */ {
          // Source: drake/geometry/proximity/triangle_surface_mesh.h
          const char* doc =
R"""(Returns the vertex identified by a given index.

Parameter ``v``:
    The index of the vertex.

Precondition:
    v ∈ {0, 1, 2,...,num_vertices()-1}.)""";
        } vertex;
        // Symbol: drake::geometry::TriangleSurfaceMesh::vertices
        struct /* vertices */ {
          // Source: drake/geometry/proximity/triangle_surface_mesh.h
          const char* doc = R"""(Returns the vertices.)""";
        } vertices;
      } TriangleSurfaceMesh;
      // Symbol: drake::geometry::VolumeElement
      struct /* VolumeElement */ {
        // Source: drake/geometry/proximity/volume_mesh.h
        const char* doc =
R"""(VolumeElement represents a tetrahedral element in a VolumeMesh. It is
a topological entity in the sense that it only knows the indices of
its vertices but not their coordinates.)""";
        // Symbol: drake::geometry::VolumeElement::Equal
        struct /* Equal */ {
          // Source: drake/geometry/proximity/volume_mesh.h
          const char* doc =
R"""(Checks to see whether the given VolumeElement use the same four
indices in the same order. We check for equality to the last bit
consistently with VolumeMesh∷Equal(). Two permutations of the four
vertex indices of a tetrahedron are considered different tetrahedra
even though they span the same space.)""";
        } Equal;
        // Symbol: drake::geometry::VolumeElement::VolumeElement
        struct /* ctor */ {
          // Source: drake/geometry/proximity/volume_mesh.h
          const char* doc_4args =
R"""(Constructs VolumeElement. We follow the convention that the first
three vertices define a triangle with its right-handed normal pointing
inwards. The fourth vertex is then on the positive side of this first
triangle.

Warning:
    This class does not enforce our convention for the ordering of the
    vertices.

Parameter ``v0``:
    Index of the first vertex in VolumeMesh.

Parameter ``v1``:
    Index of the second vertex in VolumeMesh.

Parameter ``v2``:
    Index of the third vertex in VolumeMesh.

Parameter ``v3``:
    Index of the last vertex in VolumeMesh.

Precondition:
    All indices are non-negative.)""";
          // Source: drake/geometry/proximity/volume_mesh.h
          const char* doc_1args =
R"""(Constructs VolumeElement.

Parameter ``v``:
    Array of four integer indices of the vertices of the element in
    VolumeMesh.

Precondition:
    All indices are non-negative.)""";
        } ctor;
        // Symbol: drake::geometry::VolumeElement::num_vertices
        struct /* num_vertices */ {
          // Source: drake/geometry/proximity/volume_mesh.h
          const char* doc =
R"""(Returns the number of vertices in this element.)""";
        } num_vertices;
        // Symbol: drake::geometry::VolumeElement::vertex
        struct /* vertex */ {
          // Source: drake/geometry/proximity/volume_mesh.h
          const char* doc =
R"""(Returns the vertex index in VolumeMesh of the i-th vertex of this
element.

Parameter ``i``:
    The local index of the vertex in this element.

Precondition:
    0 <= i < 4)""";
        } vertex;
      } VolumeElement;
      // Symbol: drake::geometry::VolumeMesh
      struct /* VolumeMesh */ {
        // Source: drake/geometry/proximity/volume_mesh.h
        const char* doc =
R"""(VolumeMesh represents a tetrahedral volume mesh.

Template parameter ``T``:
    The underlying scalar type for coordinates, e.g., double or
    AutoDiffXd. Must be a valid Eigen scalar.)""";
        // Symbol: drake::geometry::VolumeMesh::CalcBarycentric
        struct /* CalcBarycentric */ {
          // Source: drake/geometry/proximity/volume_mesh.h
          const char* doc =
R"""(Calculate barycentric coordinates with respect to the tetrahedron
``e`` of the point Q. This operation is expensive compared with going
from barycentric to Cartesian.

The return type depends on both the mesh's vertex position scalar type
``T`` and the Cartesian coordinate type ``C`` of the query point. See
drake∷geometry∷promoted_numerical "promoted_numerical_t" for details.

Parameter ``p_MQ``:
    A position expressed in the frame M of the mesh.

Parameter ``e``:
    The index of a tetrahedral element.

Note:
    If p_MQ is outside the tetrahedral element, the barycentric
    coordinates (b₀, b₁, b₂, b₃) still satisfy b₀ + b₁ + b₂ + b₃ = 1;
    however, some bᵢ will be negative.

Template parameter ``C``:
    must be either ``double`` or ``AutoDiffXd``.)""";
        } CalcBarycentric;
        // Symbol: drake::geometry::VolumeMesh::CalcGradientVectorOfLinearField
        struct /* CalcGradientVectorOfLinearField */ {
          // Source: drake/geometry/proximity/volume_mesh.h
          const char* doc =
R"""(Like MaybeCalcGradientVectorOfLinearField, but throws if the geometry
is degenerate.

Raises:
    RuntimeError if the gradient could not be computed.)""";
        } CalcGradientVectorOfLinearField;
        // Symbol: drake::geometry::VolumeMesh::CalcTetrahedronVolume
        struct /* CalcTetrahedronVolume */ {
          // Source: drake/geometry/proximity/volume_mesh.h
          const char* doc =
R"""(Calculates volume of a tetrahedral element. It is a signed volume,
i.e., it can be negative depending on the order of the four vertices
of the tetrahedron.

Precondition:
    ``e ∈ [0, num_elements())``.)""";
        } CalcTetrahedronVolume;
        // Symbol: drake::geometry::VolumeMesh::CalcVolume
        struct /* CalcVolume */ {
          // Source: drake/geometry/proximity/volume_mesh.h
          const char* doc =
R"""(Calculates the volume of ``this`` mesh by taking the sum of the volume
of each tetrahedral element.)""";
        } CalcVolume;
        // Symbol: drake::geometry::VolumeMesh::Equal
        struct /* Equal */ {
          // Source: drake/geometry/proximity/volume_mesh.h
          const char* doc =
R"""(Checks to see whether the given VolumeMesh object is equal via deep
comparison (up to a tolerance). NaNs are treated as not equal as per
the IEEE standard. The tolerance is applied to corresponding vertex
positions; the ith vertex in each mesh can have a distance of no more
than ``vertex_tolerance``.

Parameter ``mesh``:
    The mesh for comparison.

Parameter ``vertex_tolerance``:
    The maximum distance allowed between two vertices to be considered
    equal.

Returns:
    ``True`` if the given mesh is equal.)""";
        } Equal;
        // Symbol: drake::geometry::VolumeMesh::MaybeCalcGradientVectorOfLinearField
        struct /* MaybeCalcGradientVectorOfLinearField */ {
          // Source: drake/geometry/proximity/volume_mesh.h
          const char* doc =
R"""(Calculates the gradient ∇u of a linear field u on the tetrahedron
``e``. Field u is defined by the four field values ``field_value[i]``
at the i-th vertex of the tetrahedron. The gradient ∇u is expressed in
the coordinates frame of this mesh M.

If the return value is std∷nullopt, the tetrahedron is degenerate, and
no reliable gradient could be computed.

The return type depends on both the mesh's vertex position scalar type
``T`` and the given field's scalar type ``FieldValue``. See
drake∷geometry∷promoted_numerical "promoted_numerical_t" for details.)""";
        } MaybeCalcGradientVectorOfLinearField;
        // Symbol: drake::geometry::VolumeMesh::ScalarType
        struct /* ScalarType */ {
          // Source: drake/geometry/proximity/volume_mesh.h
          const char* doc = R"""()""";
        } ScalarType;
        // Symbol: drake::geometry::VolumeMesh::SetAllPositions
        struct /* SetAllPositions */ {
          // Source: drake/geometry/proximity/volume_mesh.h
          const char* doc =
R"""(Updates the position of all vertices in the mesh. Each sequential
triple in p_MVs (e.g., 3i, 3i + 1, 3i + 2), i ∈ ℤ, is interpreted as a
position vector associated with the iᵗʰ vertex. The position values
are interpreted to be measured and expressed in the same frame as the
mesh to be deformed.

Parameter ``p_MVs``:
    Vertex positions for the mesh's N vertices flattened into a vector
    (where each position vector is measured and expressed in the
    mesh's original frame).

Raises:
    RuntimeError if p_MVs.size() != 3 * num_vertices())""";
        } SetAllPositions;
        // Symbol: drake::geometry::VolumeMesh::TransformVertices
        struct /* TransformVertices */ {
          // Source: drake/geometry/proximity/volume_mesh.h
          const char* doc =
R"""(Transforms the vertices of this mesh from its initial frame M to the
new frame N.

Parameter ``transform``:
    The transform X_NM relating the mesh in frame M to the new frame
    N.)""";
        } TransformVertices;
        // Symbol: drake::geometry::VolumeMesh::VolumeMesh<T>
        struct /* ctor */ {
          // Source: drake/geometry/proximity/volume_mesh.h
          const char* doc =
R"""(Constructor from a vector of vertices and from a vector of elements.
Each element must be a valid VolumeElement following the vertex
ordering convention documented in the VolumeElement class. This class
however does not enforce this convention and it is thus the
responsibility of the user.)""";
        } ctor;
        // Symbol: drake::geometry::VolumeMesh::edge_vector
        struct /* edge_vector */ {
          // Source: drake/geometry/proximity/volume_mesh.h
          const char* doc =
R"""(Returns p_AB_M, the position vector from vertex A to vertex B in M,
where A and B are specified by the element local indices a and b of
element e.

Parameter ``e``:
    The index of the element.

Parameter ``a``:
    The element local index of vertex A.

Parameter ``b``:
    The element local index of vertex B.

Precondition:
    e ∈ [0, num_elements())

Precondition:
    a ∈ [0, 4)

Precondition:
    b ∈ [0, 4)

Precondition:
    a < b)""";
        } edge_vector;
        // Symbol: drake::geometry::VolumeMesh::element
        struct /* element */ {
          // Source: drake/geometry/proximity/volume_mesh.h
          const char* doc = R"""()""";
        } element;
        // Symbol: drake::geometry::VolumeMesh::inward_normal
        struct /* inward_normal */ {
          // Source: drake/geometry/proximity/volume_mesh.h
          const char* doc =
R"""(Returns the inward facing normal of face f of element e.

Parameter ``e``:
    The index of the element.

Parameter ``f``:
    The index of the triangular face of the tetrahedral element e
    formed by the vertices [(f + 1) % 4, (f + 2) % 4, (f + 3) % 4].

Precondition:
    e ∈ [0, num_elements())

Precondition:
    f ∈ [0, 4))""";
        } inward_normal;
        // Symbol: drake::geometry::VolumeMesh::num_elements
        struct /* num_elements */ {
          // Source: drake/geometry/proximity/volume_mesh.h
          const char* doc =
R"""(Returns the number of tetrahedral elements in the mesh.)""";
        } num_elements;
        // Symbol: drake::geometry::VolumeMesh::num_vertices
        struct /* num_vertices */ {
          // Source: drake/geometry/proximity/volume_mesh.h
          const char* doc =
R"""(Returns the number of vertices in the mesh.)""";
        } num_vertices;
        // Symbol: drake::geometry::VolumeMesh::tetrahedra
        struct /* tetrahedra */ {
          // Source: drake/geometry/proximity/volume_mesh.h
          const char* doc = R"""()""";
        } tetrahedra;
        // Symbol: drake::geometry::VolumeMesh::vertex
        struct /* vertex */ {
          // Source: drake/geometry/proximity/volume_mesh.h
          const char* doc =
R"""(Returns the vertex identified by a given index.

Parameter ``v``:
    The index of the vertex.

Precondition:
    v ∈ {0, 1, 2,...,num_vertices()-1}.)""";
        } vertex;
        // Symbol: drake::geometry::VolumeMesh::vertices
        struct /* vertices */ {
          // Source: drake/geometry/proximity/volume_mesh.h
          const char* doc = R"""()""";
        } vertices;
      } VolumeMesh;
      // Symbol: drake::geometry::operator!=
      struct /* operator_ne */ {
        // Source: drake/geometry/proximity/volume_mesh.h
        const char* doc = R"""()""";
      } operator_ne;
      // Symbol: drake::geometry::promoted_numerical
      struct /* promoted_numerical */ {
        // Source: drake/geometry/proximity/mesh_traits.h
        const char* doc =
R"""(Given the two scalar types U and T, returns the most "promoted" type.
The scalars must be either ``double`` or ``AutoDiffXd``.

This trait implicitly encodes the logic: if either B or T are
AutoDiffXd, return AutoDiffXd. The logic is illustrated with this
truth table:

| T | U | Result | | :----------: | :----------: | :----------: | |
AutoDiffXd | AutoDiffXd | AutoDiffXd | | double | AutoDiffXd |
AutoDiffXd | | AutoDiffXd | double | AutoDiffXd | | double | double |
double |

This also includes the helper type:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    template <typename T, typename U>
    using promoted_numerical_t = typename promoted_numerical<T, U>∷type;

.. raw:: html

    </details>)""";
        // Symbol: drake::geometry::promoted_numerical::type
        struct /* type */ {
          // Source: drake/geometry/proximity/mesh_traits.h
          const char* doc = R"""()""";
        } type;
      } promoted_numerical;
    } geometry;
  } drake;
} pydrake_doc_geometry_proximity;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
