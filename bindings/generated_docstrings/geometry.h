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

// #include "drake/geometry/collision_filter_declaration.h"
// #include "drake/geometry/collision_filter_manager.h"
// #include "drake/geometry/deformable_mesh_with_bvh.h"
// #include "drake/geometry/drake_visualizer.h"
// #include "drake/geometry/drake_visualizer_params.h"
// #include "drake/geometry/geometry_frame.h"
// #include "drake/geometry/geometry_ids.h"
// #include "drake/geometry/geometry_instance.h"
// #include "drake/geometry/geometry_properties.h"
// #include "drake/geometry/geometry_roles.h"
// #include "drake/geometry/geometry_set.h"
// #include "drake/geometry/geometry_state.h"
// #include "drake/geometry/geometry_version.h"
// #include "drake/geometry/in_memory_mesh.h"
// #include "drake/geometry/internal_frame.h"
// #include "drake/geometry/internal_geometry.h"
// #include "drake/geometry/kinematics_vector.h"
// #include "drake/geometry/make_mesh_for_deformable.h"
// #include "drake/geometry/mesh_deformation_interpolator.h"
// #include "drake/geometry/mesh_source.h"
// #include "drake/geometry/meshcat.h"
// #include "drake/geometry/meshcat_animation.h"
// #include "drake/geometry/meshcat_file_storage_internal.h"
// #include "drake/geometry/meshcat_graphviz.h"
// #include "drake/geometry/meshcat_params.h"
// #include "drake/geometry/meshcat_point_cloud_visualizer.h"
// #include "drake/geometry/meshcat_visualizer.h"
// #include "drake/geometry/meshcat_visualizer_params.h"
// #include "drake/geometry/poisson_disk.h"
// #include "drake/geometry/proximity_engine.h"
// #include "drake/geometry/proximity_properties.h"
// #include "drake/geometry/query_object.h"
// #include "drake/geometry/read_gltf_to_memory.h"
// #include "drake/geometry/read_obj.h"
// #include "drake/geometry/rgba.h"
// #include "drake/geometry/scene_graph.h"
// #include "drake/geometry/scene_graph_config.h"
// #include "drake/geometry/scene_graph_inspector.h"
// #include "drake/geometry/shape_specification.h"
// #include "drake/geometry/utilities.h"

// Symbol: pydrake_doc_geometry
constexpr struct /* pydrake_doc_geometry */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::geometry
    struct /* geometry */ {
      // Symbol: drake::geometry::AddCompliantHydroelasticProperties
      struct /* AddCompliantHydroelasticProperties */ {
        // Source: drake/geometry/proximity_properties.h
        const char* doc =
R"""(Adds properties to the given set of proximity properties sufficient to
cause the associated geometry to generate a compliant hydroelastic
representation. The geometry's pressure field will be the function
p(e) = Ee, where E is the hydroelastic modulus stored in the given
``properties``.

Parameter ``resolution_hint``:
    If the geometry is to be tessellated, it is the parameter that
    guides the level of mesh refinement. It has length units (in
    meters) and roughly corresponds to a typical edge length in the
    resulting mesh. See hug_properties. This will be ignored for
    geometry types that don't require tessellation.

Parameter ``hydroelastic_modulus``:
    A multiplier that maps penetration to pressure. See
    hug_properties.

Parameter ``properties``:
    The properties will be added to this property set.

Raises:
    RuntimeError If ``properties`` already has properties with the
    names that this function would need to add.

Precondition:
    0 < ``resolution_hint`` < ∞, 0 < ``hydroelastic_modulus``, and
    ``properties`` is not nullptr.)""";
      } AddCompliantHydroelasticProperties;
      // Symbol: drake::geometry::AddCompliantHydroelasticPropertiesForHalfSpace
      struct /* AddCompliantHydroelasticPropertiesForHalfSpace */ {
        // Source: drake/geometry/proximity_properties.h
        const char* doc =
R"""(Compliant half spaces are handled as a special case; they do not get
tessellated. Instead, they are treated as infinite slabs with a finite
thickness. This variant is required for hydroelastic half spaces.

Parameter ``slab_thickness``:
    The distance from the half space boundary to its rigid core (this
    helps define the extent field of the half space).

Parameter ``hydroelastic_modulus``:
    A multiplier that maps penetration to pressure. See
    hug_properties.

Parameter ``properties``:
    The properties will be added to this property set.

Raises:
    RuntimeError If ``properties`` already has properties with the
    names that this function would need to add.

Precondition:
    0 < ``slab_thickness`` < ∞, 0 < ``hydroelastic_modulus``, and
    ``properties`` is not nullptr.)""";
      } AddCompliantHydroelasticPropertiesForHalfSpace;
      // Symbol: drake::geometry::AddContactMaterial
      struct /* AddContactMaterial */ {
        // Source: drake/geometry/proximity_properties.h
        const char* doc =
R"""(AddContactMaterial() adds general contact material properties to the
given set of proximity ``properties``. These are the properties
required by the default point contact model. However, other contact
models can opt to use these properties as well. Only the parameters
that carry values will be added to the given set of ``properties``; no
default values will be provided. Downstream consumers of the contact
materials can optionally provide defaults for missing properties.

Raises:
    RuntimeError if ``dissipation`` is negative, ``point_stiffness``
    is not positive, of any of the contact material properties have
    already been defined in ``properties``.

Precondition:
    ``properties`` is not nullptr.)""";
      } AddContactMaterial;
      // Symbol: drake::geometry::AddRigidHydroelasticProperties
      struct /* AddRigidHydroelasticProperties */ {
        // Source: drake/geometry/proximity_properties.h
        const char* doc_2args =
R"""(Adds properties to the given set of proximity properties sufficient to
cause the associated geometry to generate a rigid hydroelastic
representation.

Parameter ``resolution_hint``:
    If the geometry is to be tessellated, it is the parameter that
    guides the level of mesh refinement. It has length units (in
    meters) and roughly corresponds to a typical edge length in the
    resulting mesh. See hug_properties. This will be ignored for
    geometry types that don't require tessellation.

Parameter ``properties``:
    The properties will be added to this property set.

Raises:
    RuntimeError If ``properties`` already has properties with the
    names that this function would need to add.

Precondition:
    0 < ``resolution_hint`` < ∞ and ``properties`` is not nullptr.)""";
        // Source: drake/geometry/proximity_properties.h
        const char* doc_1args =
R"""(Overload, intended for shapes that don't get tessellated in their
hydroelastic representation (e.g., HalfSpace and Mesh). See
hug_properties.)""";
      } AddRigidHydroelasticProperties;
      // Symbol: drake::geometry::Box
      struct /* Box */ {
        // Source: drake/geometry/shape_specification.h
        const char* doc =
R"""(Definition of a box. The box is centered on the origin of its
canonical frame with its dimensions aligned with the frame's axes. The
size of the box is given by three sizes.)""";
        // Symbol: drake::geometry::Box::Box
        struct /* ctor */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc_3args =
R"""(Constructs a box with the given ``width``, `depth`, and ``height``,
which specify the box's dimension along the canonical x-, y-, and
z-axes, respectively.

Raises:
    RuntimeError if any measure is not finite positive.)""";
          // Source: drake/geometry/shape_specification.h
          const char* doc_1args =
R"""(Constructs a box with a vector of measures: width, depth, and height
-- the box's dimensions along the canonical x-, y-, and z-axes,
respectively.

Raises:
    RuntimeError if any measure is not finite positive.)""";
        } ctor;
        // Symbol: drake::geometry::Box::MakeCube
        struct /* MakeCube */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc =
R"""(Constructs a cube with the given ``edge_size`` for its width, depth,
and height.

Raises:
    RuntimeError if edge_size is not finite positive.)""";
        } MakeCube;
        // Symbol: drake::geometry::Box::depth
        struct /* depth */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc =
R"""(Returns the box's dimension along the y axis.)""";
        } depth;
        // Symbol: drake::geometry::Box::height
        struct /* height */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc =
R"""(Returns the box's dimension along the z axis.)""";
        } height;
        // Symbol: drake::geometry::Box::size
        struct /* size */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc = R"""(Returns the box's dimensions.)""";
        } size;
        // Symbol: drake::geometry::Box::width
        struct /* width */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc =
R"""(Returns the box's dimension along the x axis.)""";
        } width;
      } Box;
      // Symbol: drake::geometry::CalcVolume
      struct /* CalcVolume */ {
        // Source: drake/geometry/shape_specification.h
        const char* doc =
R"""(Calculates the volume (in meters^3) for the Shape. For convex and mesh
geometries, the algorithm only supports ".obj" files and only produces
meaningful results for "closed" shapes.

Raises:
    RuntimeError if the derived type hasn't overloaded this
    implementation (yet), if a filetype is unsupported, or if a
    referenced file cannot be opened.)""";
      } CalcVolume;
      // Symbol: drake::geometry::Capsule
      struct /* Capsule */ {
        // Source: drake/geometry/shape_specification.h
        const char* doc =
R"""(Definition of a capsule. The capsule can be thought of as a cylinder
with spherical caps attached. The capsule's length refers to the
length of the cylindrical region, and the radius applies to both the
cylinder and spherical caps. A capsule with zero length is a sphere of
the given radius. And a capsule with zero radius is a line segment
with the given length. The capsule is defined in its canonical frame
C, centered on the frame origin and with the length of the capsule
parallel with the frame's z-axis.)""";
        // Symbol: drake::geometry::Capsule::Capsule
        struct /* ctor */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc_2args =
R"""(Constructs a capsule with the given ``radius`` and ``length``.

Raises:
    RuntimeError if any measure is not finite positive.)""";
          // Source: drake/geometry/shape_specification.h
          const char* doc_1args =
R"""(Constructs a capsule with a vector of measures: radius and length.

Raises:
    RuntimeError if any measure is not finite positive.)""";
        } ctor;
        // Symbol: drake::geometry::Capsule::length
        struct /* length */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc = R"""()""";
        } length;
        // Symbol: drake::geometry::Capsule::radius
        struct /* radius */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc = R"""()""";
        } radius;
      } Capsule;
      // Symbol: drake::geometry::CollisionFilterDeclaration
      struct /* CollisionFilterDeclaration */ {
        // Source: drake/geometry/collision_filter_declaration.h
        const char* doc =
R"""(Class for articulating changes to the configuration of SceneGraph's
"collision filters"; collision filters limit the scope of various
proximity queries.

This class provides the basis for *declaring* what pairs should or
should not be included in the set of proximity query candidates C (see
documentation for CollisionFilterManager for details on the set C).

A declaration consists of zero or more *statements*. Each statement
can declare, for example, that the pair (g₁, g₂) should be excluded
from C (also known as "filtering the pair"). That statement is added
to the declaration by invoking the corresponding statement method (see
below), and the result of the invocation, is that the statement is
appended to the declaration.

Each statement method returns a reference to the declaration instance,
so a number of statements can be chained together, i.e.,


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    collision_filter_manager.Apply(
    CollisionFilterDeclaration()
    .ExcludeWithin(some_geometry_set)
    .ExcludeBetween(set_A, set_B));

.. raw:: html

    </details>

It's worth noting, that the statements are evaluated in *invocation*
order such that a later statement can partially or completely undo the
effect of an earlier statement. The full declaration is evaluated by
CollisionFilterManager∷Apply().)""";
        // Symbol: drake::geometry::CollisionFilterDeclaration::AllowBetween
        struct /* AllowBetween */ {
          // Source: drake/geometry/collision_filter_declaration.h
          const char* doc =
R"""(Allows geometry pairs in proximity evaluation by updating the
candidate pair set ``C ← C ⋃ P*``, where ``P = {(a, b)}, ∀ a ∈ A, b ∈
B, a ≠ b`` and ``A = {a₀, a₁, ..., aₘ}`` and ``B = {b₀, b₁, ..., bₙ}``
are the input sets of geometries ``set_A`` and ``set_B``,
respectively. Where ``P* = P - (Aₚ × Aₚ) - Fₚ - Iₚ``, the set of pairs
after we remove the SceneGraph-mandated invariants (see
CollisionFilterManager for details on those invariants).

To be explicit, in contrast to AllowWithin, this does *not* clear
filters between members of the *same* set (e.g., ``(aᵢ, aⱼ)`` or
``(bᵢ, bⱼ)``).)""";
        } AllowBetween;
        // Symbol: drake::geometry::CollisionFilterDeclaration::AllowWithin
        struct /* AllowWithin */ {
          // Source: drake/geometry/collision_filter_declaration.h
          const char* doc =
R"""(Allows geometry pairs in proximity evaluation by updating the
candidate pair set ``C ← C ⋃ P*``, where ``P = {(gᵢ, gⱼ)}, ∀ gᵢ, gⱼ ∈
G, i ≠ j`` and ``G = {g₀, g₁, ..., gₘ}`` is the input ``geometry_set``
of geometries. Where ``P* = P - (Aₚ × Aₚ) - Fₚ - Iₚ``, the set of
pairs after we remove the SceneGraph-mandated invariants (see
CollisionFilterManager for details on those invariants).)""";
        } AllowWithin;
        // Symbol: drake::geometry::CollisionFilterDeclaration::CollisionFilterDeclaration
        struct /* ctor */ {
          // Source: drake/geometry/collision_filter_declaration.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::geometry::CollisionFilterDeclaration::ExcludeBetween
        struct /* ExcludeBetween */ {
          // Source: drake/geometry/collision_filter_declaration.h
          const char* doc =
R"""(Excludes geometry pairs from proximity evaluation by updating the
candidate pair set ``C ← C - P``, where ``P = {(a, b)}, ∀ a ∈ A, b ∈
B`` and ``A = {a₀, a₁, ..., aₘ}`` and ``B = {b₀, b₁, ..., bₙ}`` are
the input sets of geometries ``set_A`` and ``set_B``, respectively.)""";
        } ExcludeBetween;
        // Symbol: drake::geometry::CollisionFilterDeclaration::ExcludeWithin
        struct /* ExcludeWithin */ {
          // Source: drake/geometry/collision_filter_declaration.h
          const char* doc =
R"""(Excludes geometry pairs from proximity evaluation by updating the
candidate pair set ``C ← C - P``, where ``P = {(gᵢ, gⱼ)}, ∀ gᵢ, gⱼ ∈
G`` and ``G = {g₀, g₁, ..., gₘ}`` is the input ``geometry_set`` of
geometries.)""";
        } ExcludeWithin;
      } CollisionFilterDeclaration;
      // Symbol: drake::geometry::CollisionFilterManager
      struct /* CollisionFilterManager */ {
        // Source: drake/geometry/collision_filter_manager.h
        const char* doc =
R"""(Class for configuring "collision filters"; collision filters limit the
scope of various proximity queries.

The sole source of CollisionFilterManager instances is SceneGraph. See
scene_graph_collision_filter_manager "SceneGraph's documentation" for
details on acquiring an instance.

A SceneGraph instance contains the set of geometry ``G = D ⋃ A = {g₀,
g₁, ..., gₙ}``, where D is the set of dynamic geometries and A is the
set of anchored geometries (by definition ``D ⋂ A = ∅``). `Gₚ ⊂ G` is
the subset of geometries that have a proximity role (with an analogous
interpretation of ``Dₚ`` and ``Aₚ``). Many proximity queries operate
on pairs of geometries (e.g., (gᵢ, gⱼ)). The set of proximity
candidate pairs for such queries is initially defined as ``C = (Gₚ ×
Gₚ) - (Aₚ × Aₚ) - Fₚ - Iₚ``, where:

- ``Gₚ × Gₚ = {(gᵢ, gⱼ)}, ∀ gᵢ, gⱼ ∈ Gₚ`` is the Cartesian product of the set
of SceneGraph proximity geometries.
- ``Aₚ × Aₚ`` represents all pairs consisting only of anchored geometries;
an anchored geometry is never tested against another anchored geometry.
- ``Fₚ = {(gᵢ, gⱼ)} ∀ i, j``, such that ``gᵢ, gⱼ ∈ Dₚ`` and
``frame(gᵢ) == frame(gⱼ)``; the pairs where both geometries are rigidly
affixed to the same frame.
- ``Iₚ = {(g, g)}, ∀ g ∈ Gₚ`` is the set of all pairs consisting of a
geometry with itself; there is no meaningful proximity query on a
geometry with itself.

Only pairs contained in C will be included in pairwise proximity
operations.

The manager provides an interface to modify the set C. Changes to C
are articulated with CollisionFilterDeclaration. Once a change has
been *declared* it is applied via the manager's API to change the
configuration of C.

There are limits to how C can be modified.

- ``∀ (gᵢ, gⱼ) ∈ C``, both gᵢ and gⱼ must be registered with SceneGraph; you
can't inject arbitrary ids. Attempting to do so will result in an error.
- No pairs in ``Aₚ × Aₚ``, `Fₚ`, or ``Iₚ`` can ever be added to C. Excluding
those pairs is a SceneGraph invariant. Attempts to do so will be ignored.

The current configuration of C depends on the sequence of filter
declarations that have been applied in the manager. Changing the order
can change the end result.

A CollisionFilterManager is a view into geometry data (either that
owned by a SceneGraph instance or a SceneGraph context). The manager
instance can be copied or moved and the resulting instance is a view
into the same data. For both the original and the copy (or just the
target when moving the manager), the source data must stay alive for
at least as long as the manager instance.

Warning:
    The effect of applying a declaration is based on the state of
    SceneGraph's geometry data at the time of application. More
    concretely: - For a particular FrameId in a GeometrySet instance,
    only those geometries attached to the identified frame with the
    proximity role assigned at the time of the call will be included
    in the filter. If geometries are subsequently added or assigned
    the proximity role, they will not be retroactively added to the
    user-declared filter. - If the geometry set in a declaration
    statement includes geometries which have *not* been assigned a
    proximity role, those geometries will be ignored. If a proximity
    role is subsequently assigned, those geometries will *still* not
    be part of any user-declared collision filters. - In general,
    adding collisions and assigning proximity roles should happen
    prior to collision filter configuration.

**Transient vs Persistent changes**

Collision filtering is all about defining which geometry pairs are in
the set C. The simplest way to modify the set C is through
*persistent* modifications to a "base configuration" (via calls to
Apply()). However, declarations can also be applied in a *transient*
manner. Transient declarations form a history. The current definition
of the set C is the result of applying the history of collision filter
declarations to the persistent base configuration in order. That
history can be modified:

- New transient declarations can be appended.
- Arbitrary transient declarations can be removed from the sequence.
- The current configuration (based on a history with an arbitrary number
of transient declarations) can be "flattened" into the persistent base
(with no history).

The table below illustrates a sequence of operations and their effect
on C. We define C's initial configuration as ``C = {P₁, P₂, ..., Pₙ}``
(for ``n`` geometry pairs). Each action is described as either
persistent or transient.

| Line | C | Action | | :--: | :------------------ |
:-------------------------------------------------- | | 1 | ``{P₁, P₂,
..., Pₙ}`` | Initial condition of the persistent base | | 2 | ``{P₂,
..., Pₙ}`` | Remove P₁ from the persistent base | | 3 | ``{P₂, ...,
Pₙ₋₁}`` | Remove Pₙ from the persistent base | | 4 | ``{P₂}`` | Remove
all pairs except P₂ from the persistent base | | 5 | ``{P₂, P₄, P₅}``
| Transient declaration #1 puts P₄ and P₅ into C | | 6 | ``{P₂, P₃,
P₄, P₅}`` | Transient declaration #2 puts P₃ and P₄ into C | | 7 |
``{P₂, P₃, P₄}`` | Remove declaration #1. | | 8 | ``{P₂, P₃, P₄}`` |
Configuration flattened; #2 no longer exists | **Table 1**: An example
sequence of operations on collision filters.

Notes: - lines 2 - 4 represent a series of *persistent* operations,
filtering pairs of geometry (aka removing them from C by calling
Apply()). - line 5: the first transient filter declaration which is
assigned the id value #1 (via a call to ApplyTransient()). - line 6:
Adds a new transient filter declaration to the sequence (assigned id
#2). Note, that it redundantly declares that P₄ is a member of C just
as declaration #1 did. This redundancy is fine. In fact, it may be
very important (see below). - line 7: We remove declaration #1.
Although #1 added both P₄ and P₅ to C, the result of removing this
declaration from the sequence is that P₅ is no longer a member of C
but P₄ *is*. This is because #2's declaration that P₄ is* in C
preserves its membership. - line 8: the current configuration is
"flattened" into the persistent base. All history is thrown out and
any filter identifiers are rendered invalid.

This example workflow above illustrates some key ideas in using
transient declarations.

- **The persistent configuration can only be modified when there is no
transient history.** Applying a filter declaration should have the effect
of realizing that declaration. If a pair is *declared* to be in C, the
result of the declaration is that the pair is in C (assuming that the pair
can* be in C). If we allowed modifying the persistent configuration with
an active transient history, there might be no discernible change in the
resultant configuration state because a subsequent transient declaration
may supplant it. This would lead to inscrutable bugs. Therefore, it's
simply not allowed.
- **When defining a transient declaration, the declaration should include all
critical pairs *explicitly*.** This includes those pairs that should and
should not be in C. Any pair *not* explicitly accounted for should be one
whose filter status is immaterial.
It *might* seems desirable (from an optimization perspective) to examine
the *current* configuration of C and apply the minimum change to put it
into a desired state (i.e., if a pair I need filtered is already filtered,
I would omit it from the declaration). This approach is fraught with peril.
If the current configuration is the result of previous transient
declarations, the removal of any of those declarations could invalidate the
attempted difference calculation and my declaration would no longer be
sufficient to guarantee the set C required.
- **Anyone at any time can add to the history.** Some code can modify C to
suit its needs. However, it can also make subsequent calls into code that
also modifies C with no guarantees that the called code will undo those
changes. Code that modifies collision filters configuration should document
that it does so. It should also have a clear protocol for cleaning up its
own changes. Finally, code that applies declarations should not take for
granted that its transient declarations are necessarily the last
declarations in the history.

*Making transient filter declarations*

There is a custom API for applying a filter declaration as a
*transient* declaration. It returns the id for the transient API (used
to remove the declaration from the history sequence).

Attempting to change the persistent configuration when there are
active transient declarations in the history will throw an exception.)""";
        // Symbol: drake::geometry::CollisionFilterManager::Apply
        struct /* Apply */ {
          // Source: drake/geometry/collision_filter_manager.h
          const char* doc =
R"""(Applies the given ``declaration`` to the geometry state managed by
``this`` instance.

The process of *applying* the collision filter data also validates it.
The following polices are implemented during application:

- Referencing an invalid id (FrameId or GeometryId): throws.
- Declaring a filtered pair that is already filtered: no discernible
change.
- Attempts to "allow" collision between a pair that is strictly excluded
(e.g., between two anchored geometries) will be ignored.

Raises:
    RuntimeError if the ``declaration`` references invalid ids or
    there is an active history.)""";
        } Apply;
        // Symbol: drake::geometry::CollisionFilterManager::ApplyTransient
        struct /* ApplyTransient */ {
          // Source: drake/geometry/collision_filter_manager.h
          const char* doc =
R"""(Applies the declaration as the newest *transient* modification to the
collision filter configuration. The declaration must be considered
"valid", as defined for Apply().)""";
        } ApplyTransient;
        // Symbol: drake::geometry::CollisionFilterManager::CollisionFilterManager
        struct /* ctor */ {
          // Source: drake/geometry/collision_filter_manager.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::geometry::CollisionFilterManager::IsActive
        struct /* IsActive */ {
          // Source: drake/geometry/collision_filter_manager.h
          const char* doc =
R"""(Reports if the transient collision filter declaration indicated by the
given ``filter_id`` is part of the history.)""";
        } IsActive;
        // Symbol: drake::geometry::CollisionFilterManager::RemoveDeclaration
        struct /* RemoveDeclaration */ {
          // Source: drake/geometry/collision_filter_manager.h
          const char* doc =
R"""(Attempts to remove the transient declaration from the history for the
declaration associated with the given ``filter_id``.

Parameter ``filter_id``:
    The id of the filter declaration to remove.

Returns:
    ``True`` iff ``is_active(filter_id)`` returns ``True`` before
    calling this method (i.e., ``filter_id`` refers to an existent
    filter that has successfully been removed).)""";
        } RemoveDeclaration;
        // Symbol: drake::geometry::CollisionFilterManager::has_transient_history
        struct /* has_transient_history */ {
          // Source: drake/geometry/collision_filter_manager.h
          const char* doc =
R"""(Reports if there are any active transient filter declarations.)""";
        } has_transient_history;
      } CollisionFilterManager;
      // Symbol: drake::geometry::CollisionFilterScope
      struct /* CollisionFilterScope */ {
        // Source: drake/geometry/collision_filter_declaration.h
        const char* doc =
R"""(Enum that defines the scope of the geometries that are affected by the
collision filtering mechanism.)""";
        // Symbol: drake::geometry::CollisionFilterScope::kAll
        struct /* kAll */ {
          // Source: drake/geometry/collision_filter_declaration.h
          const char* doc =
R"""(All geometries are considered when collision filters are applied.)""";
        } kAll;
        // Symbol: drake::geometry::CollisionFilterScope::kOmitDeformable
        struct /* kOmitDeformable */ {
          // Source: drake/geometry/collision_filter_declaration.h
          const char* doc =
R"""(Deformable geometries are omitted when applying collision filters.
That means that all deformable geometries are not affected by the
collision filter declaration even if they are included in the
GeometrySet when the filter is declared.)""";
        } kOmitDeformable;
      } CollisionFilterScope;
      // Symbol: drake::geometry::Convex
      struct /* Convex */ {
        // Source: drake/geometry/shape_specification.h
        const char* doc =
R"""(Definition of a *convex* surface mesh.

This shape is *not* the mesh contained in the file named by
``filename``. It is the convex hull of that mesh. As such, the only
contents of the mesh file that matter are the vertex positions. All
other data is ignored. This includes materials, textures, normals etc.
This is true for *all* roles. Its appearance in an illustration or
perception role, will be a faceted polytope whose color is defined by
its assigned properties (or by the geometry consumer's defaults).

Because Drake computes the convex hull, the named mesh file need not
be convex.

The mesh is defined in a canonical frame C, implicit in the file
parsed. Upon loading it in SceneGraph it can be scaled around the
origin of C by a given ``scale`` amount.)""";
        // Symbol: drake::geometry::Convex::Convex
        struct /* ctor */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc_2args_filename_scale =
R"""(Constructs a convex shape specification from the file located at the
given file path. Optionally uniformly scaled by the given scale
factor.

The mesh file referenced can be an .obj, .gltf, or a tetrahedral .vtk.

Parameter ``filename``:
    The file name; if it is not absolute, it will be interpreted
    relative to the current working directory.

Parameter ``scale``:
    An optional scale to coordinates.

Raises:
    RuntimeError if |scale| < 1e-8. Note that a negative scale is
    considered valid. We want to preclude scales near zero but
    recognise that scale is a convenience tool for "tweaking" models.
    8 orders of magnitude should be plenty without considering
    revisiting the model itself.)""";
          // Source: drake/geometry/shape_specification.h
          const char* doc_2args_filename_scale3 =
R"""(File variant that allows for specification of non-uniform scale.)""";
          // Source: drake/geometry/shape_specification.h
          const char* doc_2args_mesh_data_scale =
R"""(Constructs a convex shape specification from the contents of a
Drake-supported mesh file type.

The mesh is defined by the contents of a supported_file_types "mesh
file format supported by Drake". Those contents are passed in as
``mesh_data``. For Convex, the only supporting files required are
those necessary to define vertex positions (e.g., a glTF's .bin file);
material and texture files, if provided, are ignored and are therefore
unnecessary.

Parameter ``mesh_data``:
    The in-memory file contents that define the vertex data for this
    shape.

Parameter ``scale``:
    An optional scale to coordinates.)""";
          // Source: drake/geometry/shape_specification.h
          const char* doc_2args_mesh_data_scale3 =
R"""(Mesh-contents variant that allows for specification of non-uniform
scale.)""";
          // Source: drake/geometry/shape_specification.h
          const char* doc_2args_source_scale =
R"""(Constructs a convex shape specification from the given ``source``.

Parameter ``source``:
    The source for the mesh data.

Parameter ``scale``:
    An optional scale to coordinates.)""";
          // Source: drake/geometry/shape_specification.h
          const char* doc_2args_source_scale3 =
R"""(Mesh-source variant that allows for specification of non-uniform
scale.)""";
          // Source: drake/geometry/shape_specification.h
          const char* doc_3args_points_label_scale =
R"""(Constructs an in-memory convex shape specification from the given
points.

The convex hull is computed from the points provided. The points are
expected to be in the canonical frame of the shape. Optionally
uniformly scaled by the given scale factor.

Parameter ``points``:
    The points whose convex hull define the shape.

Parameter ``label``:
    A label for the object. The label is used for warning and error
    messages. Otherwise, the label has no other functional purpose. It
    must consist of a single line.

Parameter ``scale``:
    An optional scale to coordinates.

Raises:
    RuntimeError if label contains newlines.

Raises:
    RuntimeError if |scale| < 1e-8.)""";
          // Source: drake/geometry/shape_specification.h
          const char* doc_3args_points_label_scale3 =
R"""(Point variant that allows for specification of non-uniform scale.)""";
        } ctor;
        // Symbol: drake::geometry::Convex::GetConvexHull
        struct /* GetConvexHull */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc =
R"""(Reports the convex hull of the named mesh.

Note: the convex hull is computed on demand on the first invocation.
All subsequent invocations should have an O(1) cost.

Raises:
    if the referenced mesh data cannot be read or is degenerate
    (insufficient number of vertices, co-linear or coincident
    vertices, etc.) All of the vertices lying on a plane is *not*
    considered degenerate.)""";
        } GetConvexHull;
        // Symbol: drake::geometry::Convex::extension
        struct /* extension */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc =
R"""(Returns the extension of the underlying input mesh -- all lower case
and including the dot. If ``this`` is constructed from a file path,
the extension is extracted from the path. I.e., /foo/bar/mesh.obj and
/foo/bar/mesh.OBJ would both report the ".obj" extension. The
"extension" portion of the filename is defined as in
std∷filesystem∷path∷extension().

If ``this`` is constructed using in-memory file contents, it is the
extension of the MemoryFile passed to the constructor.)""";
        } extension;
        // Symbol: drake::geometry::Convex::scale
        struct /* scale */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc =
R"""(Returns a single scale representing the *uniform* scale factor.

Raises:
    if the scale is not uniform in all directions.)""";
        } scale;
        // Symbol: drake::geometry::Convex::scale3
        struct /* scale3 */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc =
R"""(Returns general scale factors for this mesh.)""";
        } scale3;
        // Symbol: drake::geometry::Convex::source
        struct /* source */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc =
R"""(Returns the source for this specification's mesh data. When working
with Convex, this API should only be used for introspection. The
contract for Convex is that the convex hull is always used in place of
whatever underlying mesh declaration is provided. For all functional
geometric usage, exclusively use the convex hull returned by
GetConvexHull().)""";
        } source;
      } Convex;
      // Symbol: drake::geometry::Cylinder
      struct /* Cylinder */ {
        // Source: drake/geometry/shape_specification.h
        const char* doc =
R"""(Definition of a cylinder. It is centered in its canonical frame with
the length of the cylinder parallel with the frame's z-axis.)""";
        // Symbol: drake::geometry::Cylinder::Cylinder
        struct /* ctor */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc_2args =
R"""(Constructs a cylinder with the given ``radius`` and ``length``.

Raises:
    RuntimeError if any measure is not finite positive.)""";
          // Source: drake/geometry/shape_specification.h
          const char* doc_1args =
R"""(Constructs a cylinder with a vector of measures: radius and length.

Raises:
    RuntimeError if any measure is not finite positive.)""";
        } ctor;
        // Symbol: drake::geometry::Cylinder::length
        struct /* length */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc = R"""()""";
        } length;
        // Symbol: drake::geometry::Cylinder::radius
        struct /* radius */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc = R"""()""";
        } radius;
      } Cylinder;
      // Symbol: drake::geometry::DefaultProximityProperties
      struct /* DefaultProximityProperties */ {
        // Source: drake/geometry/scene_graph_config.h
        const char* doc =
R"""(These properties will be used as defaults when the geometry as added
via API calls or parsed from model files doesn't say anything more
specific.

See also:
    compliant_contact, hydroelastic_user_guide, friction_model and
    subsections therein.)""";
        // Symbol: drake::geometry::DefaultProximityProperties::Serialize
        struct /* Serialize */ {
          // Source: drake/geometry/scene_graph_config.h
          const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
        } Serialize;
        // Symbol: drake::geometry::DefaultProximityProperties::ValidateOrThrow
        struct /* ValidateOrThrow */ {
          // Source: drake/geometry/scene_graph_config.h
          const char* doc = R"""(Throws if the values are inconsistent.)""";
        } ValidateOrThrow;
        // Symbol: drake::geometry::DefaultProximityProperties::compliance_type
        struct /* compliance_type */ {
          // Source: drake/geometry/scene_graph_config.h
          const char* doc =
R"""(@name Hydroelastic Contact Properties

These properties affect hydroelastic contact only. For more detail,
including limits of the numeric parameters,

See also:
    geometry∷AddRigidHydroelasticProperties,
    geometry∷AddCompliantHydroelasticProperties,
    geometry∷AddCompliantHydroelasticPropertiesForHalfSpace.

For more context,

See also:
    hug_properties. There are three valid options for
    ``compliance_type``: - "undefined": hydroelastic contact will not
    be used. - "rigid": the default hydroelastic compliance type will
    be rigid; note that rigid-rigid contact is not supported by the
    hydroelastic contact model, but is supported by point contact
    (multibody∷ContactModel∷kPoint or
    multibody∷ContactModel∷kHydroelasticWithFallback). - "compliant":
    the default hydroelastic compliance type will be compliant.)""";
        } compliance_type;
        // Symbol: drake::geometry::DefaultProximityProperties::dynamic_friction
        struct /* dynamic_friction */ {
          // Source: drake/geometry/scene_graph_config.h
          const char* doc =
R"""(@name General Contact Properties

These properties affect contact in general. For more detail, including
limits of the numeric parameters,

See also:
    geometry∷AddContactMaterial, multibody∷CoulombFriction,
    mbp_contact_modeling, mbp_dissipation_model. To be valid, either
    both friction values must be populated, or neither. Friction
    quantities are unitless.)""";
        } dynamic_friction;
        // Symbol: drake::geometry::DefaultProximityProperties::hunt_crossley_dissipation
        struct /* hunt_crossley_dissipation */ {
          // Source: drake/geometry/scene_graph_config.h
          const char* doc =
R"""(Controls energy dissipation from contact, for contact approximations
other than* multibody∷DiscreteContactApproximation∷kSap. Units are
seconds per meter.

If a non-deformable geometry is missing a value for dissipation,
MultibodyPlant will generate a default value (based on
multibody∷MultibodyPlantConfig∷penetration_allowance). However, this
behavior will be going away. Therefore, we recommend guaranteeing that
every geometry has a dissipation value either by assigning the
property directly to the geometry or by providing a non-null value
here.

Please refer to contact_defaults "Default Contact Parameters" for more
details on Drake's defaults along with guidelines on how to estimate
parameters specific to your model.)""";
        } hunt_crossley_dissipation;
        // Symbol: drake::geometry::DefaultProximityProperties::hydroelastic_modulus
        struct /* hydroelastic_modulus */ {
          // Source: drake/geometry/scene_graph_config.h
          const char* doc =
R"""(A measure of material stiffness, in units of Pascals.)""";
        } hydroelastic_modulus;
        // Symbol: drake::geometry::DefaultProximityProperties::margin
        struct /* margin */ {
          // Source: drake/geometry/scene_graph_config.h
          const char* doc =
R"""((Advanced) Specifies a thin layer of thickness "margin" (in meters)
around each geometry. Two bodies with margins δ₁ and δ₂ are considered
for contact resolution whenever their distance is within δ₁ + δ₂. That
is, (speculative) contact constraints are added for objects at a
distance smaller than δ₁+δ₂.

Refer to hydro_margin for further details, including theory, examples,
recommended margin values and limitations.

There will only be *contact* if the two zero level sets intersect.
Unless the zero level sets of both geometries overlap, there is no
contact and no contact force. However, (speculative) contact
constraints will be added allowing our discrete contact solvers to
predict if a contact "will" occur at the next time step. This leads to
additional time coherence and stability. Analytical studies of
stability show that a value of 0.1 mm to 1.0 mm is more than enough
for most robotics applications. This is not an "action-at-a-distance"
trick, there is no contact when the thin margin layers of two objects
overlap. The margin is simply a "cheap" mechanism to avoid
significantly more complex and costly strategies such as Continuous
Collision Detection.

Note:
    Inflating the geometries does not appreciably change the domain of
    contact. In the original mesh, the contact pressure is zero on the
    mesh's boundary surface. When inflating the meshes, we redefine
    the pressure field so that its zero level set field is a good
    approximation to the surface of the original geometry. When
    visualizing hydroelastic proximity geometry, the rendered geometry
    will include the inflation.

Note:
    Currently margin only applies to *compliant* hydroelastic contact
    and it does not affect point contact.)""";
        } margin;
        // Symbol: drake::geometry::DefaultProximityProperties::point_stiffness
        struct /* point_stiffness */ {
          // Source: drake/geometry/scene_graph_config.h
          const char* doc =
R"""(A measure of material stiffness, in units of Newtons per meter.

If a non-deformable geometry is missing a value for stiffness,
MultibodyPlant will generate a default value (based on
multibody∷MultibodyPlantConfig∷penetration_allowance). However, this
behavior will be going away. Therefore, we recommend guaranteeing that
every geometry has a stiffness value either by assigning the property
directly to the geometry or by providing a non-null value here.

Please refer to contact_defaults "Default Contact Parameters" for more
details on Drake's defaults along with guidelines on how to estimate
parameters specific to your model.)""";
        } point_stiffness;
        // Symbol: drake::geometry::DefaultProximityProperties::relaxation_time
        struct /* relaxation_time */ {
          // Source: drake/geometry/scene_graph_config.h
          const char* doc =
R"""(Controls energy damping from contact, *only for*
multibody∷DiscreteContactApproximation∷kSap. Units are seconds.)""";
        } relaxation_time;
        // Symbol: drake::geometry::DefaultProximityProperties::resolution_hint
        struct /* resolution_hint */ {
          // Source: drake/geometry/scene_graph_config.h
          const char* doc =
R"""(Controls how finely primitive geometries are tessellated, units of
meters.

While no single value is universally appropriate, this value was
selected based on the following idea. We're attempting to make
introducing novel manipulands as easy as possible. Considering a
simple soup can as a representative manipuland, we've picked a value
that would result in a tessellated cylinder with enough faces to be
appropriate for contact with a compliant gripper.)""";
        } resolution_hint;
        // Symbol: drake::geometry::DefaultProximityProperties::slab_thickness
        struct /* slab_thickness */ {
          // Source: drake/geometry/scene_graph_config.h
          const char* doc =
R"""(For a halfspace, the thickness of compliant material to model, in
units of meters.)""";
        } slab_thickness;
        // Symbol: drake::geometry::DefaultProximityProperties::static_friction
        struct /* static_friction */ {
          // Source: drake/geometry/scene_graph_config.h
          const char* doc =
R"""(See also:
    dynamic_friction.)""";
        } static_friction;
        auto Serialize__fields() const {
          return std::array{
            std::make_pair("compliance_type", compliance_type.doc),
            std::make_pair("dynamic_friction", dynamic_friction.doc),
            std::make_pair("hunt_crossley_dissipation", hunt_crossley_dissipation.doc),
            std::make_pair("hydroelastic_modulus", hydroelastic_modulus.doc),
            std::make_pair("margin", margin.doc),
            std::make_pair("point_stiffness", point_stiffness.doc),
            std::make_pair("relaxation_time", relaxation_time.doc),
            std::make_pair("resolution_hint", resolution_hint.doc),
            std::make_pair("slab_thickness", slab_thickness.doc),
            std::make_pair("static_friction", static_friction.doc),
          };
        }
      } DefaultProximityProperties;
      // Symbol: drake::geometry::DrakeVisualizer
      struct /* DrakeVisualizer */ {
        // Source: drake/geometry/drake_visualizer.h
        const char* doc =
R"""(A system that publishes LCM messages representing the current state of
a SceneGraph instance (whose QueryObject-valued output port is
connected to this system's input port).

The messages are compatible with `Meldis
</pydrake/pydrake.visualization.meldis.html>`_.

.. pydrake_system::

    name: DrakeVisualizer
    input_ports:
    - query_object

The DrakeVisualizer system broadcasts three kinds of LCM messages:

- a message that defines the non-deformable geometries in the world on the
lcm channel named "DRAKE_VIEWER_LOAD_ROBOT"
- a message that updates the poses of those non-deformable geometries on the
lcm channel named "DRAKE_VIEWER_DRAW",
- a message that sets the world space vertex positions of the deformable
geometries on the lcm channel named "DRAKE_VIEWER_DEFORMABLE"

If requested in DrakeVisualizerParams, the above channel names are
modified according to the role specified. This allows simultaneous
availability of geometry from multiple roles, by using multiple
DrakeVisualizer instances.

- kIllustration: channel names gain a "_ILLUSTRATION" suffix.
- kProximity: channel names gain a "_PROXIMITY" suffix.
- kPerception: channel names gain a "_PERCEPTION" suffix.

The system uses the versioning mechanism provided by SceneGraph to
detect changes to the geometry so that a change in SceneGraph's data
will propagate to the message receiver.

**Visualization by Role**

By default, DrakeVisualizer visualizes geometries with the
illustration role (see geometry_roles for more details). It can be
configured to visualize geometries with other roles (see
DrakeVisualizerParams). Only one role can be specified.

The appearance of the geometry in the visualizer is typically defined
by the geometry's properties for the visualized role.

- For the visualized role, if a geometry has the ("phong", "diffuse")
property described in the table below, that value is used.
- Otherwise, if the geometry *also* has the illustration properties, those
properties are likewise tested for the ("phong", "diffuse") property. This
rule only has significance is the visualized role is *not* the illustration
role.
- Otherwise, the configured default color will be applied (see
DrakeVisualizerParams).

| Group name | Required | Property Name | Property Type | Property
Description | | :--------: | :------: | :-----------: |
:-------------: | :------------------- | | phong | no | diffuse | Rgba
| The rgba value of the object surface. |

*Appearance of OBJ files for non-deformable geometries*

Meshes represented by OBJ are special. The OBJ file can reference a
material file (.mtl). If the mtl file is found by the receiving
application, values in the .mtl will take precedence over the
("phong", "diffuse") geometry property.

It's worth emphasizing that these rules permits control over the
appearance of collision geometry on a per-geometry basis by assigning
an explicit Rgba value to the ("phong", "diffuse") property in the
geometry's ProximityProperties.

Note:
    If collision geometries are added to SceneGraph by parsing
    URDF/SDF files, they will not have diffuse values. Even if
    elements were added to the specification files, they would not be
    parsed. They must be added to the geometries after parsing.

**Effective visualization**

The best visualization is when draw messages have been preceded by a
compatible load message (i.e., a "coherent" message sequence). While
LCM doesn't guarantee that messages will be received/processed in the
same order as they are broadcast, your results will be best if
DrakeVisualizer is allowed to broadcast coherent messages. Practices
that interfere with that will likely produce undesirable results.
E.g.,

- Evaluating a single instance of DrakeVisualizer across several threads,
such that the data in the per-thread systems∷Context varies.
- Evaluating multiple instances of DrakeVisualizer in a single thread that
share the same lcm∷DrakeLcmInterface.

**Mesh support**

DrakeVisualizer is mesh file format agnostic. A Mesh or Convex shape
that references arbitrary mesh files will simply be packaged in the
broadcast LCM message. It defers to the message *receiver* on whether
that particular file format is supported.

**Scalar support and conversion**

DrakeVisualizer is templated on ``T`` and can be used in a ``double``-
or AutoDiffXd-valued Diagram. However, the diagram can only be
converted from one scalar type to another if the DrakeVisualizer
*owns* its lcm∷DrakeLcmInterface instance. Attempts to scalar convert
the system otherwise will throw an exception.)""";
        // Symbol: drake::geometry::DrakeVisualizer::AddToBuilder
        struct /* AddToBuilder */ {
          // Source: drake/geometry/drake_visualizer.h
          const char* doc_4args_builder_scene_graph_lcm_params =
R"""(Connects the newly added DrakeVisualizer to the given SceneGraph's
QueryObject-valued output port. The DrakeVisualizer's name (see
systems∷SystemBase∷set_name) will be set to a sensible default value,
unless the default name was already in use by another system.)""";
          // Source: drake/geometry/drake_visualizer.h
          const char* doc_4args_builder_query_object_port_lcm_params =
R"""(Connects the newly added DrakeVisualizer to the given
QueryObject-valued output port. The DrakeVisualizer's name (see
systems∷SystemBase∷set_name) will be set to a sensible default value,
unless the default name was already in use by another system.)""";
        } AddToBuilder;
        // Symbol: drake::geometry::DrakeVisualizer::DispatchLoadMessage
        struct /* DispatchLoadMessage */ {
          // Source: drake/geometry/drake_visualizer.h
          const char* doc =
R"""((Advanced) Dispatches a load message built on the *model* geometry for
the given SceneGraph instance. This should be used sparingly. When we
have a starightforward method for binding lcmtypes in python, this
will be replaced with an API that will simply generate the lcm
*messages* that the caller can then do whatever they like with.

Precondition:
    ``lcm != nullptr``.)""";
        } DispatchLoadMessage;
        // Symbol: drake::geometry::DrakeVisualizer::DrakeVisualizer<T>
        struct /* ctor */ {
          // Source: drake/geometry/drake_visualizer.h
          const char* doc =
R"""(Creates an instance of DrakeVisualizer.

Parameter ``lcm``:
    An optional LCM interface. If none is provided, this system will
    allocate its own instance. If one is provided it must remain valid
    for the lifetime of this object.

Parameter ``params``:
    The set of parameters to control this system's behavior.

Raises:
    RuntimeError if ``params.publish_period <= 0``.

Raises:
    RuntimeError if ``params.role == Role∷kUnassigned``.)""";
          // Source: drake/geometry/drake_visualizer.h
          const char* doc_copyconvert =
R"""(Scalar-converting copy constructor. See system_scalar_conversion. It
should only be used to convert *from* double *to* other scalar types.

Raises:
    RuntimeError if ``other`` does not *own* its
    lcm∷DrakeLcmInterface.)""";
        } ctor;
        // Symbol: drake::geometry::DrakeVisualizer::query_object_input_port
        struct /* query_object_input_port */ {
          // Source: drake/geometry/drake_visualizer.h
          const char* doc =
R"""(Returns the QueryObject-valued input port. It should be connected to
SceneGraph's QueryObject-valued output port. Failure to do so will
cause a runtime error when attempting to broadcast messages.)""";
        } query_object_input_port;
      } DrakeVisualizer;
      // Symbol: drake::geometry::DrakeVisualizerParams
      struct /* DrakeVisualizerParams */ {
        // Source: drake/geometry/drake_visualizer_params.h
        const char* doc =
R"""(The set of parameters for configuring DrakeVisualizer.)""";
        // Symbol: drake::geometry::DrakeVisualizerParams::Serialize
        struct /* Serialize */ {
          // Source: drake/geometry/drake_visualizer_params.h
          const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
        } Serialize;
        // Symbol: drake::geometry::DrakeVisualizerParams::default_color
        struct /* default_color */ {
          // Source: drake/geometry/drake_visualizer_params.h
          const char* doc =
R"""(The color to apply to any geometry that hasn't defined one.)""";
        } default_color;
        // Symbol: drake::geometry::DrakeVisualizerParams::publish_period
        struct /* publish_period */ {
          // Source: drake/geometry/drake_visualizer_params.h
          const char* doc =
R"""(The duration (in seconds) between published LCM messages that update
the poses of the scene's geometry. (To help avoid small simulation
time steps, we use a default period that has an exact representation
in binary floating point; see drake#15021 for details.))""";
        } publish_period;
        // Symbol: drake::geometry::DrakeVisualizerParams::role
        struct /* role */ {
          // Source: drake/geometry/drake_visualizer_params.h
          const char* doc =
R"""(The role of the geometries to be sent to the visualizer.)""";
        } role;
        // Symbol: drake::geometry::DrakeVisualizerParams::show_hydroelastic
        struct /* show_hydroelastic */ {
          // Source: drake/geometry/drake_visualizer_params.h
          const char* doc =
R"""(When using the hydroelastic contact model, collision geometries that
are *declared* as geometric primitives are frequently represented by
some discretely tessellated mesh when computing contact. It can be
quite helpful in assessing contact behavior to visualize these
discrete meshes (in place of the idealized primitives).

To visualize these representations it is necessary to request
visualization of geometries with the Role∷kProximity role (see the
role field). It is further necessary to explicitly request the
hydroelastic meshes where available (setting show_hydroelastic to
``True``).

Setting this ``show_hydroelastic`` to ``True`` will have no apparent
effect if none of the collision meshes have a hydroelastic mesh
associated with them.)""";
        } show_hydroelastic;
        // Symbol: drake::geometry::DrakeVisualizerParams::use_role_channel_suffix
        struct /* use_role_channel_suffix */ {
          // Source: drake/geometry/drake_visualizer_params.h
          const char* doc =
R"""(Setting this to ``True`` will cause LCM channel names to have a suffix
appended, allowing simultaneous transmission of multiple geometry
roles via multiple DrakeVisualizer instances. See DrakeVisualizer for
details.)""";
        } use_role_channel_suffix;
        auto Serialize__fields() const {
          return std::array{
            std::make_pair("default_color", default_color.doc),
            std::make_pair("publish_period", publish_period.doc),
            std::make_pair("role", role.doc),
            std::make_pair("show_hydroelastic", show_hydroelastic.doc),
            std::make_pair("use_role_channel_suffix", use_role_channel_suffix.doc),
          };
        }
      } DrakeVisualizerParams;
      // Symbol: drake::geometry::DrakeVisualizerd
      struct /* DrakeVisualizerd */ {
        // Source: drake/geometry/drake_visualizer.h
        const char* doc =
R"""(A convenient alias for the DrakeVisualizer class when using the
``double`` scalar type.)""";
      } DrakeVisualizerd;
      // Symbol: drake::geometry::Ellipsoid
      struct /* Ellipsoid */ {
        // Source: drake/geometry/shape_specification.h
        const char* doc =
R"""(Definition of an ellipsoid. It is centered on the origin of its
canonical frame with its dimensions aligned with the frame's axes. The
standard equation for the ellipsoid is:

x²/a² + y²/b² + z²/c² = 1,

where a,b,c are the lengths of the principal semi-axes of the
ellipsoid. The bounding box of the ellipsoid is [-a,a]x[-b,b]x[-c,c].)""";
        // Symbol: drake::geometry::Ellipsoid::Ellipsoid
        struct /* ctor */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc_3args =
R"""(Constructs an ellipsoid with the given lengths of its principal
semi-axes, with a, b, and c measured along the x-, y-, and z- axes of
the canonical frame, respectively.

Raises:
    RuntimeError if any measure is not finite positive.)""";
          // Source: drake/geometry/shape_specification.h
          const char* doc_1args =
R"""(Constructs an ellipsoid with a vector of measures: the lengths of its
principal semi-axes, with a, b, and c measured along the x-, y-, and
z- axes of the canonical frame, respectively.

Raises:
    RuntimeError if any measure is not finite positive.)""";
        } ctor;
        // Symbol: drake::geometry::Ellipsoid::a
        struct /* a */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc = R"""()""";
        } a;
        // Symbol: drake::geometry::Ellipsoid::b
        struct /* b */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc = R"""()""";
        } b;
        // Symbol: drake::geometry::Ellipsoid::c
        struct /* c */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc = R"""()""";
        } c;
      } Ellipsoid;
      // Symbol: drake::geometry::FilterId
      struct /* FilterId */ {
        // Source: drake/geometry/geometry_ids.h
        const char* doc =
R"""(Type used to identify transient collision filter declarations in
SceneGraph.)""";
      } FilterId;
      // Symbol: drake::geometry::FrameId
      struct /* FrameId */ {
        // Source: drake/geometry/geometry_ids.h
        const char* doc =
R"""(Type used to identify geometry frames in SceneGraph.)""";
      } FrameId;
      // Symbol: drake::geometry::FrameIdSet
      struct /* FrameIdSet */ {
        // Source: drake/geometry/geometry_state.h
        const char* doc = R"""(Collection of unique frame ids.)""";
      } FrameIdSet;
      // Symbol: drake::geometry::GeometryFrame
      struct /* GeometryFrame */ {
        // Source: drake/geometry/geometry_frame.h
        const char* doc =
R"""(This simple class carries the definition of a frame used in the
SceneGraph. To register moving frames with SceneGraph (see
SceneGraph∷RegisterFrame()), a geometry source (see
SceneGraph∷RegisterSource()) instantiates a frame and passes ownership
over to SceneGraph.

A frame is defined by two pieces of information:

- the name, which must be unique within a single geometry source and
- the "frame group", an integer identifier that can be used to group frames
together within a geometry source.

The "frame group" is intended as a generic synonym for the model
instance id defined by the RigidBodyTree.

See also:
    SceneGraph)""";
        // Symbol: drake::geometry::GeometryFrame::GeometryFrame
        struct /* ctor */ {
          // Source: drake/geometry/geometry_frame.h
          const char* doc =
R"""(Constructor.

Parameter ``frame_name``:
    The name of the frame.

Parameter ``frame_group_id``:
    The optional frame group identifier. If unspecified, defaults to
    the common, 0 group. Must be non-negative.)""";
        } ctor;
        // Symbol: drake::geometry::GeometryFrame::frame_group
        struct /* frame_group */ {
          // Source: drake/geometry/geometry_frame.h
          const char* doc = R"""()""";
        } frame_group;
        // Symbol: drake::geometry::GeometryFrame::id
        struct /* id */ {
          // Source: drake/geometry/geometry_frame.h
          const char* doc =
R"""(Returns the globally unique id for this geometry specification. Every
instantiation of FrameInstance will contain a unique id value. The id
value is preserved across copies. After successfully registering this
FrameInstance, this id will serve as the identifier for the registered
representation as well.)""";
        } id;
        // Symbol: drake::geometry::GeometryFrame::name
        struct /* name */ {
          // Source: drake/geometry/geometry_frame.h
          const char* doc = R"""()""";
        } name;
      } GeometryFrame;
      // Symbol: drake::geometry::GeometryId
      struct /* GeometryId */ {
        // Source: drake/geometry/geometry_ids.h
        const char* doc =
R"""(Type used to identify geometry instances in SceneGraph.)""";
        // Symbol: drake::geometry::GeometryId::GeometryId
        struct /* ctor */ {
          // Source: drake/geometry/geometry_ids.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::geometry::GeometryId::get_new_id
        struct /* get_new_id */ {
          // Source: drake/geometry/geometry_ids.h
          const char* doc = R"""()""";
        } get_new_id;
      } GeometryId;
      // Symbol: drake::geometry::GeometryIdSet
      struct /* GeometryIdSet */ {
        // Source: drake/geometry/geometry_state.h
        const char* doc = R"""(Collection of unique geometry ids.)""";
      } GeometryIdSet;
      // Symbol: drake::geometry::GeometryInstance
      struct /* GeometryInstance */ {
        // Source: drake/geometry/geometry_instance.h
        const char* doc =
R"""(A geometry instance combines a geometry definition (i.e., a shape of
some sort), a pose (relative to a parent "frame" P), material
information, and an opaque collection of metadata. The parent frame
can be a registered frame or another registered geometry.

Every GeometryInstance must be named. The naming convention mirrors
that of valid names in SDF files. Specifically, any user-specified
name will have all leading and trailing space and tab characters
trimmed off. The trimmed name will have to satisfy the following
requirements:

- cannot be empty, and
- the name should be unique in the scope of its frame and role. For example,
two GeometryInstances can both be called "ball" as long as they are
affixed to different frames or if one is a collision geometry and the
other is an illustration geometry. This is enforced when a role is assigned
to the geometry.

If valid, the trimmed name will be assigned to the instance.

Names *can* have internal whitespace (e.g., "my geometry name").

**Canonicalized names**

The silent transformation of a user-defined name to canonical name
mirrors that of specifying geometry names in an SDF file. Consider the
following SDF snippet:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    {xml}
    ...
    <visual name="  visual">
    <geometry>
    <sphere>
    <radius>1.0</radius>
    </sphere>
    </geometry>
    </visual>
    ...

.. raw:: html

    </details>

The name has two leading whitespace characters. The parsing process
will consider this name as equivalent to "visual" and tests for
uniqueness and non-emptiness will be applied to that trimmed result.
The following code has an analogous effect:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    scene_graph->RegisterGeometry(
    source_id, frame_id,
    make_unique<GeometryInstance>(pose, make_unique<Sphere>(1.0), "  visual"));

.. raw:: html

    </details>

The specified name includes leading whitespace. That name will be
trimmed and the *result* will be stored in the GeometryInstance (to be
later validated by SceneGraph as part of geometry registration).
Querying the instance of its name will return this *canonicalized*
name.)""";
        // Symbol: drake::geometry::GeometryInstance::GeometryInstance
        struct /* ctor */ {
          // Source: drake/geometry/geometry_instance.h
          const char* doc =
R"""(Constructs a geometry instance specification.

Parameter ``X_PG``:
    The pose of this geometry (``G``) in its parent's frame (``P``).

Parameter ``shape``:
    The underlying shape for this geometry instance.

Parameter ``name``:
    The name of the geometry (must satisfy the name requirements).

Raises:
    RuntimeError if the canonicalized version of ``name`` is empty.)""";
        } ctor;
        // Symbol: drake::geometry::GeometryInstance::id
        struct /* id */ {
          // Source: drake/geometry/geometry_instance.h
          const char* doc =
R"""(Returns the globally unique id for this geometry specification. Every
instantiation of GeometryInstance will contain a unique id value. The
id value is preserved across copies. After successfully registering
this GeometryInstance, this id will serve as the identifier for the
registered representation as well.)""";
        } id;
        // Symbol: drake::geometry::GeometryInstance::illustration_properties
        struct /* illustration_properties */ {
          // Source: drake/geometry/geometry_instance.h
          const char* doc =
R"""(Returns a pointer to the geometry's const illustration properties (if
they are defined). Nullptr otherwise.)""";
        } illustration_properties;
        // Symbol: drake::geometry::GeometryInstance::mutable_illustration_properties
        struct /* mutable_illustration_properties */ {
          // Source: drake/geometry/geometry_instance.h
          const char* doc =
R"""(Returns a pointer to the geometry's mutable illustration properties
(if they are defined). Nullptr otherwise.)""";
        } mutable_illustration_properties;
        // Symbol: drake::geometry::GeometryInstance::mutable_perception_properties
        struct /* mutable_perception_properties */ {
          // Source: drake/geometry/geometry_instance.h
          const char* doc =
R"""(Returns a pointer to the geometry's mutable perception properties (if
they are defined). Nullptr otherwise.)""";
        } mutable_perception_properties;
        // Symbol: drake::geometry::GeometryInstance::mutable_proximity_properties
        struct /* mutable_proximity_properties */ {
          // Source: drake/geometry/geometry_instance.h
          const char* doc =
R"""(Returns a pointer to the geometry's mutable proximity properties (if
they are defined). Nullptr otherwise.)""";
        } mutable_proximity_properties;
        // Symbol: drake::geometry::GeometryInstance::name
        struct /* name */ {
          // Source: drake/geometry/geometry_instance.h
          const char* doc =
R"""(Returns the *canonicalized* name for the instance.

See also:
    canonicalized_geometry_names "Canonicalized names")""";
        } name;
        // Symbol: drake::geometry::GeometryInstance::perception_properties
        struct /* perception_properties */ {
          // Source: drake/geometry/geometry_instance.h
          const char* doc =
R"""(Returns a pointer to the geometry's const perception properties (if
they are defined). Nullptr otherwise.)""";
        } perception_properties;
        // Symbol: drake::geometry::GeometryInstance::pose
        struct /* pose */ {
          // Source: drake/geometry/geometry_instance.h
          const char* doc =
R"""(Returns the instance geometry's pose in its parent frame.)""";
        } pose;
        // Symbol: drake::geometry::GeometryInstance::proximity_properties
        struct /* proximity_properties */ {
          // Source: drake/geometry/geometry_instance.h
          const char* doc =
R"""(Returns a pointer to the geometry's const proximity properties (if
they are defined). Nullptr otherwise.)""";
        } proximity_properties;
        // Symbol: drake::geometry::GeometryInstance::release_shape
        struct /* release_shape */ {
          // Source: drake/geometry/geometry_instance.h
          const char* doc =
R"""((Advanced) Transfers ownership of this geometry instance's underlying
shape specification to the caller.

Precondition:
    release_shape() has not been called nor has this been moved-from.)""";
        } release_shape;
        // Symbol: drake::geometry::GeometryInstance::set_illustration_properties
        struct /* set_illustration_properties */ {
          // Source: drake/geometry/geometry_instance.h
          const char* doc =
R"""(Sets the illustration properties for the given instance.)""";
        } set_illustration_properties;
        // Symbol: drake::geometry::GeometryInstance::set_name
        struct /* set_name */ {
          // Source: drake/geometry/geometry_instance.h
          const char* doc =
R"""(Sets the *canonicalized* name for the instance.

See also:
    canonicalized_geometry_names "Canonicalized names")""";
        } set_name;
        // Symbol: drake::geometry::GeometryInstance::set_perception_properties
        struct /* set_perception_properties */ {
          // Source: drake/geometry/geometry_instance.h
          const char* doc =
R"""(Sets the perception properties for the given instance.)""";
        } set_perception_properties;
        // Symbol: drake::geometry::GeometryInstance::set_pose
        struct /* set_pose */ {
          // Source: drake/geometry/geometry_instance.h
          const char* doc =
R"""(Sets the pose of this instance in its parent's frame.)""";
        } set_pose;
        // Symbol: drake::geometry::GeometryInstance::set_proximity_properties
        struct /* set_proximity_properties */ {
          // Source: drake/geometry/geometry_instance.h
          const char* doc =
R"""(Sets the proximity properties for the given instance.)""";
        } set_proximity_properties;
        // Symbol: drake::geometry::GeometryInstance::shape
        struct /* shape */ {
          // Source: drake/geometry/geometry_instance.h
          const char* doc =
R"""(Returns the underlying shape specification for this geometry instance.

Precondition:
    release_shape() has not been called nor has this been moved-from.)""";
        } shape;
      } GeometryInstance;
      // Symbol: drake::geometry::GeometryProperties
      struct /* GeometryProperties */ {
        // Source: drake/geometry/geometry_properties.h
        const char* doc =
R"""(The base class for defining a set of geometry properties.

Each property consists of a ``(group, property)`` name-pair and a
typed value. The name pair allows for reuse of common property names
(e.g., "diffuse") to be differentiated in interpretation by
associating them with different groups. The only restriction on the
value type is that it must be either cloneable or copy-constructible.

A set of geometry property values are defined when geometry is
registered with SceneGraph by an instantiator and accessed by some
downstream consumer entity. Each consumer specifies what properties it
expects to find and what default values (if any) it provides. For
example, the consumer could document that a particular property is
always required and its absence would throw an exception.
Alternatively, it could indicate that a property is optional and a
default value will be used in its absence. It is the responsibility of
the instantiator to make sure that the geometry property values are
*correctly* defined according to the expected consumer's
specification. Correctness includes such issues as key-value pairs
placed into a *correctly*-spelled group, property keys being likewise
correctly spelled, and values of the expected type. Correct spelling
includes correct case. The instantiator uses the AddProperty() method
to add new properties to the set.

To read the property (``some_group``, `some_property`) from a property
set:

1. Optionally test to see if the property exists by confirming the group
``some_group`` is in the set via HasGroup() and that the property
``some_property`` is in ``some_group`` via HasProperty(). Attempting to access
a property with a non-existent (group, property) pair may lead to an
exception (see API documentation below).
2. Acquire a property value via the GetProperty() or GetPropertyOrDefault()
methods.
NOTE: Reading a property requires a compile-time declaration of the *type*
of value being read. If the stored value is of a different type, an
exception will be thrown.

Common workflows
----------------

The following examples outline a number of ways to create and consume
geometry properties. By design, GeometryProperties cannot be
constructed, copied, or moved directly. Only derived classes can do
so. This facilitates *strongly typed* sets of properties associated
with particular geometry roles. So, for these examples we'll exercise
the derived class associated with proximity queries:
ProximityProperties.

The string-based structure of GeometryProperties provides a great deal
of flexibility at the cost of spelling sensitivity. It would be easy
to introduce typos that would then "hide" property values in some
group a consumer wouldn't look. In these examples, we avoid using
string literals as group or property names (at least in the cases
where the same name is used multiple times) to help avoid the
possibility of typo-induced errors. That is not required and certainly
not the only way to avoid such bugs.

**Creating properties**

*Creating properties in a new group*

This is a simple example in which a single group is added with
properties of various types.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    const std∷string group_name("my_group");
    ProximityProperties properties;
    // This first invocation implicitly creates the group "my_group".
    properties.AddProperty(group_name, "count", 7);     // int type
    properties.AddProperty(group_name, "length", 7.);   // double type
    properties.AddProperty(group_name, "name", "7");    // std∷string type

.. raw:: html

    </details>

*Creating properties in the default group*

Similar to the previous examples, the properties are added to the
default group. Just be aware that if multiple sites in your code add
properties to the default group, the possibility that names get
repeated increases. Property names *must* be unique within a single
group, including the default group.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    ProximityProperties properties;
    properties.AddProperty(ProximityProperties∷default_group_name(), "count", 7);
    properties.AddProperty(ProximityProperties∷default_group_name(), "width", 7.);
    properties.AddProperty(ProximityProperties∷default_group_name(), "name", "7");

.. raw:: html

    </details>

*Aggregate properties in a struct*

In some cases, there is a set of values that will *always* be accessed
together (specified with coordinated semantics). In these cases, it
makes sense to aggregate them into a struct and store that as a single
value. This reduces the number of lookups required.

It's worth noting, that if the data value is a struct, calls to
GetPropertyOrDefault() still operate as an "all-or-nothing" basis. If
the property *struct* exists, it will be returned, if it's missing the
default struct will be returned. There is no concept of a "partial"
struct in which some undefined values in the struct will be replaced
with their corresponding values in the default struct.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    struct MyData {
    int i{};
    double d{};
    std∷string s;
    };
    
    ProximityProperties properties;
    const std∷string group_name("my_group");
    MyData data{7, 7., "7"};
    properties.AddProperty(group_name, "data1", data);
    // These alternate forms are also acceptable (but not in succession, as the
    // property name has already been used by the first invocation).
    properties.AddProperty(group_name, "data2", MyData{6, 6., "6"});
    properties.AddProperty<MyData>(group_name, "data2", {6, 6., "6"});

.. raw:: html

    </details>

**Reading properties**

This section describes how to read properties under several different
scenarios: (a) when specific properties are required, (b) when the
consumer provides a default value for missing properties, and (c) when
the consumer needs to inspect what properties are available.

*Look up specific, *required* properties*

In this case, the consumer of the properties is looking for one or
more specific properties. It will ignore any other properties. More
particularly, if those properties are missing, it is considered a
runtime error and an exception is thrown.

The error can be handled in one of two ways: simply let the generic
exception generated by GeometryProperties propagate upward, or detect
the missing property and throw an exception with a custom message. The
example below shows both approaches.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    const IllustrationProperties& properties = FunctionThatReturnsProperties();
    // Looking for a Rgba of rgba colors named "rgba" - send generic error that
    // the property set is missing the required property.
    const Rgba rgba =
    properties.GetProperty<Rgba>("MyGroup", "rgba");
    
    // Explicitly detect missing property and throw exception with custom message.
    if (!properties.HasProperty("MyGroup", "rgba")) {
    throw RuntimeError(
    "ThisClass: Missing the necessary 'rgba' property; the object cannot be "
    "rendered");
    }
    // Otherwise acquire value, confident that no exception will be thrown.
    const Rgba rgba =
    properties.GetProperty<Rgba>("MyGroup", "rgba");

.. raw:: html

    </details>

Note:
    calls to ``GetProperty()`` always require the return type template
    value (e.g., ``Rgba``) to be specified in the call.

*Look up specific properties with default property values*

As with the previous case, the consumer is looking for one or more
specific properties. However, in this case, the consumer provides a
default value to use in case the target property is not defined. In
this invocation, the template parameter need not be explicitly
declared -- the inferred return type will be the same as the default
value.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    const IllustrationProperties& properties = FunctionThatReturnsProperties();
    // Looking for a Rgba of rgba colors named "rgba".
    const Rgba default_color{0.9, 0.9, 0.9};
    const Rgba rgba =
    properties.GetPropertyOrDefault("MyGroup", "rgba", default_color);

.. raw:: html

    </details>

Alternatively, the default value can be provided in one of the
following forms:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    properties.GetPropertyOrDefault("MyGroup", "rgba",
    Rgba{0.9, 0.9, 0.9});
    properties.GetPropertyOrDefault<Rgba>("MyGroup", "rgba",
    {0.9, 0.9, 0.9});

.. raw:: html

    </details>

*Iterating through provided properties*

Another alternative is to iterate through the properties that *have*
been provided. This might be done for several reasons, e.g.:

- the consumer wants to validate the set of properties, giving the user
feedback if an unsupported property has been provided, and/or
- the consumer has a default value for every property and allows the
registering code to define only those properties that deviate from the
specified default.

Working with properties in this manner requires knowledge of how to
work with AbstractValue.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    const IllustrationProperties& properties = FunctionThatReturnsProperties();
    for (const auto& pair : properties.GetGroupProperties("MyGroup") {
    const std∷string& name = pair.first;
    if (name == "rgba") {
    // Throws an exception if the named parameter is of the wrong type.
    const Rgba& rgba =
    pair.second->GetValueOrThrow<Rgba>();
    }
    }

.. raw:: html

    </details>)""";
        // Symbol: drake::geometry::GeometryProperties::AddProperty
        struct /* AddProperty */ {
          // Source: drake/geometry/geometry_properties.h
          const char* doc =
R"""(Adds the named property (``group_name``, `name`) with the given
``value``. Adds the group if it doesn't already exist.

Parameter ``group_name``:
    The group name.

Parameter ``name``:
    The name of the property -- must be unique in the group.

Parameter ``value``:
    The value to assign to the property.

Raises:
    RuntimeError if the property already exists.

Template parameter ``ValueType``:
    The type of data to store with the attribute -- must be copy
    constructible or cloneable (see Value).)""";
        } AddProperty;
        // Symbol: drake::geometry::GeometryProperties::AddPropertyAbstract
        struct /* AddPropertyAbstract */ {
          // Source: drake/geometry/geometry_properties.h
          const char* doc =
R"""(Adds the named property (``group_name``, `name`) with the given
type-erased ``value``. Adds the group if it doesn't already exist.

Parameter ``group_name``:
    The group name.

Parameter ``name``:
    The name of the property -- must be unique in the group.

Parameter ``value``:
    The value to assign to the property.

Raises:
    RuntimeError if the property already exists.)""";
        } AddPropertyAbstract;
        // Symbol: drake::geometry::GeometryProperties::GeometryProperties
        struct /* ctor */ {
          // Source: drake/geometry/geometry_properties.h
          const char* doc =
R"""(Constructs a property set with the default group. Only invoked by
final subclasses.)""";
        } ctor;
        // Symbol: drake::geometry::GeometryProperties::GetGroupNames
        struct /* GetGroupNames */ {
          // Source: drake/geometry/geometry_properties.h
          const char* doc = R"""(Returns all of the defined group names.)""";
        } GetGroupNames;
        // Symbol: drake::geometry::GeometryProperties::GetPropertiesInGroup
        struct /* GetPropertiesInGroup */ {
          // Source: drake/geometry/geometry_properties.h
          const char* doc =
R"""(Retrieves the indicated property group. The returned group is valid
for as long as this instance.

Raises:
    RuntimeError if there is no group with the given name.)""";
        } GetPropertiesInGroup;
        // Symbol: drake::geometry::GeometryProperties::GetProperty
        struct /* GetProperty */ {
          // Source: drake/geometry/geometry_properties.h
          const char* doc =
R"""(Retrieves the typed value for the property (``group_name``, `name`)
from this set of properties.

Parameter ``group_name``:
    The name of the group to which the property belongs.

Parameter ``name``:
    The name of the desired property.

Raises:
    RuntimeError if a) the group name is invalid, b) the property name
    is invalid, or c) the property type is not that specified.

Template parameter ``ValueType``:
    The expected type of the desired property.

Returns:
    const ValueType& of stored value. If ValueType is Eigen∷Vector4d,
    the return type will be a copy translated from Rgba.)""";
        } GetProperty;
        // Symbol: drake::geometry::GeometryProperties::GetPropertyAbstract
        struct /* GetPropertyAbstract */ {
          // Source: drake/geometry/geometry_properties.h
          const char* doc =
R"""(Retrieves the type-erased value for the property (``group_name``,
`name`) from this set of properties.

Parameter ``group_name``:
    The name of the group to which the property belongs.

Parameter ``name``:
    The name of the desired property.

Raises:
    RuntimeError if a) the group name is invalid, or b) the property
    name is invalid.)""";
        } GetPropertyAbstract;
        // Symbol: drake::geometry::GeometryProperties::GetPropertyOrDefault
        struct /* GetPropertyOrDefault */ {
          // Source: drake/geometry/geometry_properties.h
          const char* doc =
R"""(Retrieves the typed value for the property (``group_name``, `name`)
from the set of properties (if it exists), otherwise returns the given
default value. The given ``default_value`` is returned only if the
property is missing. If the property exists and is of a *different*
type, an exception will be thrown. If it is of the expected type, the
stored value will be returned.

Generally, it is unnecessary to explicitly declare the ``ValueType``
of the property value; it will be inferred from the provided default
value. Sometimes it is convenient to provide the default value in a
form that can be implicitly converted to the final type. In that case,
it is necessary to explicitly declare the desired ``ValueType`` so the
compiler does not infer the wrong type, e.g.:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    // Note the _integer_ value as default value.
    const double my_value = properties.GetPropertyOrDefault<double>("g", "p", 2);

.. raw:: html

    </details>

Parameter ``group_name``:
    The name of the group to which the property belongs.

Parameter ``name``:
    The name of the desired property.

Parameter ``default_value``:
    The alternate value to return if the property cannot be acquired.

Raises:
    RuntimeError if a property of the given name exists but is not of
    ``ValueType``.)""";
        } GetPropertyOrDefault;
        // Symbol: drake::geometry::GeometryProperties::Group
        struct /* Group */ {
          // Source: drake/geometry/geometry_properties.h
          const char* doc =
R"""(The properties for a single group as a property name-value map.)""";
        } Group;
        // Symbol: drake::geometry::GeometryProperties::HasGroup
        struct /* HasGroup */ {
          // Source: drake/geometry/geometry_properties.h
          const char* doc =
R"""(Reports if the given named group is part of this property set.)""";
        } HasGroup;
        // Symbol: drake::geometry::GeometryProperties::HasProperty
        struct /* HasProperty */ {
          // Source: drake/geometry/geometry_properties.h
          const char* doc =
R"""(Reports if the property (``group_name``, `name`) exists in the group.

Parameter ``group_name``:
    The name of the group to which the tested property should belong.

Parameter ``name``:
    The name of the property under question.

Returns:
    true iff the group exists and a property with the given ``name``
    exists in that group.)""";
        } HasProperty;
        // Symbol: drake::geometry::GeometryProperties::RemoveProperty
        struct /* RemoveProperty */ {
          // Source: drake/geometry/geometry_properties.h
          const char* doc =
R"""(Removes the (``group_name``, `name`) property (if it exists). Upon
completion the property will not be in the set.

Returns:
    ``True`` if the property existed prior to the call.)""";
        } RemoveProperty;
        // Symbol: drake::geometry::GeometryProperties::UpdateProperty
        struct /* UpdateProperty */ {
          // Source: drake/geometry/geometry_properties.h
          const char* doc =
R"""(Updates the named property (``group_name``, `name`) with the given
``value``. If the property doesn't already exist, it is equivalent to
calling ``AddProperty``. If the property does exist, its value (which
must have the same type as ``value``) will be replaced.

Parameter ``group_name``:
    The group name.

Parameter ``name``:
    The name of the property -- must be unique in the group.

Parameter ``value``:
    The value to assign to the property.

Raises:
    RuntimeError if the property exists with a different type.

Template parameter ``ValueType``:
    The type of data to store with the attribute -- must be copy
    constructible or cloneable (see Value).)""";
        } UpdateProperty;
        // Symbol: drake::geometry::GeometryProperties::UpdatePropertyAbstract
        struct /* UpdatePropertyAbstract */ {
          // Source: drake/geometry/geometry_properties.h
          const char* doc =
R"""(Updates the named property (``group_name``, `name`) with the given
type-erased ``value``. If the property doesn't already exist, it is
equivalent to calling ``AddPropertyAbstract``. If the property does
exist, its value (which must have the same type as ``value``) will be
replaced.

Parameter ``group_name``:
    The group name.

Parameter ``name``:
    The name of the property -- must be unique in the group.

Parameter ``value``:
    The value to assign to the property.

Raises:
    RuntimeError if the property exists with a different type.)""";
        } UpdatePropertyAbstract;
        // Symbol: drake::geometry::GeometryProperties::default_group_name
        struct /* default_group_name */ {
          // Source: drake/geometry/geometry_properties.h
          const char* doc =
R"""(Returns the default group name. There is no guarantee as to *what*
string corresponds to the default group. Therefore it should always be
accessed via this method.)""";
        } default_group_name;
        // Symbol: drake::geometry::GeometryProperties::num_groups
        struct /* num_groups */ {
          // Source: drake/geometry/geometry_properties.h
          const char* doc =
R"""(Reports the number of property groups in this set.)""";
        } num_groups;
        // Symbol: drake::geometry::GeometryProperties::to_string
        struct /* to_string */ {
          // Source: drake/geometry/geometry_properties.h
          const char* doc =
R"""(Converts the GeometryProperties to a string representation.)""";
        } to_string;
      } GeometryProperties;
      // Symbol: drake::geometry::GeometrySet
      struct /* GeometrySet */ {
        // Source: drake/geometry/geometry_set.h
        const char* doc =
R"""(The GeometrySet, as its name implies, is a convenience class for
defining a set of geometries. What makes it unique from a simple
``std∷set<GeometryId>`` instance is that membership doesn't require
explicit GeometryId enumeration; GeometryId values can be added to the
set by adding the ``FrameId`` for the frame to which the geometries
are rigidly affixed.

This class does no validation; it is a simple collection. Ultimately,
it serves as the operand of various geometry operations (e.g.,
CollisionFilterDeclaration and CollisionFilterManager∷Apply(). If the
*operation* has a particular prerequisite on the members of a
GeometrySet, it is the operation's responsibility to enforce that
requirement.

More formally, the SceneGraph consists of a set of geometries, each
associated with a unique identifier. As such, we can consider the set
of all identifiers ``SG = {g₀, g₁, ..., gₙ}`` that belong to a
SceneGraph. A GeometrySet should represent a subset of those
identifiers, ``Gₛ ⊆ SG``. The convenience of the GeometrySet class is
*how* the subset is defined. Given a set of frame ids ``F = {f₀, f₁,
..., fₙ}`` and geometry ids ``G = {g₀, g₁, ..., gₘ}``, `Gₛ = G ⋃
geometry(f₀) ⋃ ... ⋃ geometry(fₙ)` (where ``geometry(f)`` is the set
of geometries rigidly affixed to frame f).)""";
        // Symbol: drake::geometry::GeometrySet::Add
        struct /* Add */ {
          // Source: drake/geometry/geometry_set.h
          const char* doc = R"""()""";
        } Add;
        // Symbol: drake::geometry::GeometrySet::GeometrySet
        struct /* ctor */ {
          // Source: drake/geometry/geometry_set.h
          const char* doc = R"""()""";
        } ctor;
      } GeometrySet;
      // Symbol: drake::geometry::GeometryState
      struct /* GeometryState */ {
        // Source: drake/geometry/geometry_state.h
        const char* doc =
R"""(The context-dependent state of SceneGraph. This serves as an
AbstractValue in the context. SceneGraph's time-dependent state
includes more than just values; objects can be added to or removed
from the world over time. Therefore, SceneGraph's context-dependent
state includes values (the poses/configurations) and structure (the
topology of the world).

Note:
    This is intended as an internal class only.)""";
        // Symbol: drake::geometry::GeometryState::AddRenderer
        struct /* AddRenderer */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of SceneGraph∷AddRenderer().)""";
        } AddRenderer;
        // Symbol: drake::geometry::GeometryState::ApplyProximityDefaults
        struct /* ApplyProximityDefaults */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc_1args =
R"""(Applies the default proximity values in ``defaults`` to the proximity
properties of every currently registered geometry that has a proximity
role. For detailed semantics, see the 2-argument overload.)""";
          // Source: drake/geometry/geometry_state.h
          const char* doc_2args =
R"""(Applies the default proximity values in ``defaults`` to the proximity
properties of the geometry with the given geometry_id as appropriate.
For a given property, no value will be written if (a) ``defaults``
contains no value for it, or (b) a value has previously been set for
that property.

Precondition:
    geometry_id indicates a geometry with an assigned proximity role.)""";
        } ApplyProximityDefaults;
        // Symbol: drake::geometry::GeometryState::AssignRole
        struct /* AssignRole */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc_was_unable_to_choose_unambiguous_names =
R"""(Implementation of SceneGraph∷AssignRole(SourceId, GeometryId,
ProximityProperties) "SceneGraph∷AssignRole()".)""";
        } AssignRole;
        // Symbol: drake::geometry::GeometryState::BelongsToSource
        struct /* BelongsToSource */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc_2args_frame_id_source_id =
R"""(Implementation of SceneGraphInspector∷BelongsToSource(FrameId,
SourceId) const.)""";
          // Source: drake/geometry/geometry_state.h
          const char* doc_2args_geometry_id_source_id =
R"""(Implementation of SceneGraphInspector∷BelongsToSource(GeometryId,
SourceId) const.)""";
        } BelongsToSource;
        // Symbol: drake::geometry::GeometryState::ChangeShape
        struct /* ChangeShape */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of SceneGraph∷ChangeShape().)""";
        } ChangeShape;
        // Symbol: drake::geometry::GeometryState::CollisionFiltered
        struct /* CollisionFiltered */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of SceneGraphInspector∷CollisionFiltered().)""";
        } CollisionFiltered;
        // Symbol: drake::geometry::GeometryState::ComputeAabbInWorld
        struct /* ComputeAabbInWorld */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of QueryObject∷ComputeAabbInWorld(GeometryId).)""";
        } ComputeAabbInWorld;
        // Symbol: drake::geometry::GeometryState::ComputeContactSurfaces
        struct /* ComputeContactSurfaces */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of QueryObject∷ComputeContactSurfaces().)""";
        } ComputeContactSurfaces;
        // Symbol: drake::geometry::GeometryState::ComputeContactSurfacesWithFallback
        struct /* ComputeContactSurfacesWithFallback */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of QueryObject∷ComputeContactSurfacesWithFallback().)""";
        } ComputeContactSurfacesWithFallback;
        // Symbol: drake::geometry::GeometryState::ComputeDeformableContact
        struct /* ComputeDeformableContact */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of QueryObject∷ComputeDeformableContact().)""";
        } ComputeDeformableContact;
        // Symbol: drake::geometry::GeometryState::ComputeObbInWorld
        struct /* ComputeObbInWorld */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of QueryObject∷ComputeObbInWorld(GeometryId).)""";
        } ComputeObbInWorld;
        // Symbol: drake::geometry::GeometryState::ComputePointPairPenetration
        struct /* ComputePointPairPenetration */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of QueryObject∷ComputePointPairPenetration().)""";
        } ComputePointPairPenetration;
        // Symbol: drake::geometry::GeometryState::ComputeSignedDistanceGeometryToPoint
        struct /* ComputeSignedDistanceGeometryToPoint */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of QueryObject∷ComputeSignedDistanceGeometryToPoint().)""";
        } ComputeSignedDistanceGeometryToPoint;
        // Symbol: drake::geometry::GeometryState::ComputeSignedDistancePairClosestPoints
        struct /* ComputeSignedDistancePairClosestPoints */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of
QueryObject∷ComputeSignedDistancePairClosestPoints().)""";
        } ComputeSignedDistancePairClosestPoints;
        // Symbol: drake::geometry::GeometryState::ComputeSignedDistancePairwiseClosestPoints
        struct /* ComputeSignedDistancePairwiseClosestPoints */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of
QueryObject∷ComputeSignedDistancePairwiseClosestPoints().)""";
        } ComputeSignedDistancePairwiseClosestPoints;
        // Symbol: drake::geometry::GeometryState::ComputeSignedDistanceToPoint
        struct /* ComputeSignedDistanceToPoint */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of QueryObject∷ComputeSignedDistanceToPoint().)""";
        } ComputeSignedDistanceToPoint;
        // Symbol: drake::geometry::GeometryState::FindCollisionCandidates
        struct /* FindCollisionCandidates */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of QueryObject∷FindCollisionCandidates().)""";
        } FindCollisionCandidates;
        // Symbol: drake::geometry::GeometryState::FrameIdRange
        struct /* FrameIdRange */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(An object that represents the range of FrameId values in the state. It
is used in range-based for loops to iterate through registered frames.)""";
        } FrameIdRange;
        // Symbol: drake::geometry::GeometryState::FramesForSource
        struct /* FramesForSource */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of SceneGraphInspector∷FramesForSource().)""";
        } FramesForSource;
        // Symbol: drake::geometry::GeometryState::GeometryState<T>
        struct /* ctor */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc = R"""(Default constructor.)""";
        } ctor;
        // Symbol: drake::geometry::GeometryState::GetAllDeformableGeometryIds
        struct /* GetAllDeformableGeometryIds */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of SceneGraphInspector∷GetAllDeformableGeometryIds().)""";
        } GetAllDeformableGeometryIds;
        // Symbol: drake::geometry::GeometryState::GetAllGeometryIds
        struct /* GetAllGeometryIds */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of SceneGraphInspector∷GetAllGeometryIds().)""";
        } GetAllGeometryIds;
        // Symbol: drake::geometry::GeometryState::GetAllSourceIds
        struct /* GetAllSourceIds */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Returns all of the source ids in the scene graph. The order is
guaranteed to be stable and consistent. The first element is the
SceneGraph-internal source.)""";
        } GetAllSourceIds;
        // Symbol: drake::geometry::GeometryState::GetCollisionCandidates
        struct /* GetCollisionCandidates */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of SceneGraphInspector∷GetCollisionCandidates().)""";
        } GetCollisionCandidates;
        // Symbol: drake::geometry::GeometryState::GetConvexHull
        struct /* GetConvexHull */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of SceneGraphInspector∷GetConvexHull().)""";
        } GetConvexHull;
        // Symbol: drake::geometry::GeometryState::GetDrivenMeshConfigurationsInWorld
        struct /* GetDrivenMeshConfigurationsInWorld */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of QueryObject∷GetDrivenMeshConfigurationsInWorld().)""";
        } GetDrivenMeshConfigurationsInWorld;
        // Symbol: drake::geometry::GeometryState::GetDrivenRenderMeshes
        struct /* GetDrivenRenderMeshes */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of SceneGraphInspector∷GetDrivenRenderMeshes().)""";
        } GetDrivenRenderMeshes;
        // Symbol: drake::geometry::GeometryState::GetFrameGroup
        struct /* GetFrameGroup */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of SceneGraphInspector∷GetFrameGroup().)""";
        } GetFrameGroup;
        // Symbol: drake::geometry::GeometryState::GetFrameId
        struct /* GetFrameId */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of SceneGraphInspector∷GetFrameId().)""";
        } GetFrameId;
        // Symbol: drake::geometry::GeometryState::GetGeometries
        struct /* GetGeometries */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of SceneGraphInspector∷GetGeometries.)""";
        } GetGeometries;
        // Symbol: drake::geometry::GeometryState::GetGeometryIdByName
        struct /* GetGeometryIdByName */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of SceneGraphInspector∷GetGeometryIdByName().)""";
        } GetGeometryIdByName;
        // Symbol: drake::geometry::GeometryState::GetGeometryIds
        struct /* GetGeometryIds */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of SceneGraphInspector∷GetGeometryIds().)""";
        } GetGeometryIds;
        // Symbol: drake::geometry::GeometryState::GetIllustrationProperties
        struct /* GetIllustrationProperties */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of SceneGraphInspector∷GetIllustrationProperties().)""";
        } GetIllustrationProperties;
        // Symbol: drake::geometry::GeometryState::GetName
        struct /* GetName */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc_1args_id =
R"""(Implementation of SceneGraphInspector∷GetName().)""";
          // Source: drake/geometry/geometry_state.h
          const char* doc_1args_frame_id =
R"""(Implementation of SceneGraphInspector∷GetName(FrameId) const.)""";
          // Source: drake/geometry/geometry_state.h
          const char* doc_1args_geometry_id =
R"""(Implementation of SceneGraphInspector∷GetName(GeometryId) const.)""";
        } GetName;
        // Symbol: drake::geometry::GeometryState::GetObbInGeometryFrame
        struct /* GetObbInGeometryFrame */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of SceneGraphInspector∷GetObbInGeometryFrame().)""";
        } GetObbInGeometryFrame;
        // Symbol: drake::geometry::GeometryState::GetOwningSourceName
        struct /* GetOwningSourceName */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc_was_unable_to_choose_unambiguous_names =
R"""(Implementation of SceneGraphInspector∷GetOwningSourceName(FrameId)
const.)""";
        } GetOwningSourceName;
        // Symbol: drake::geometry::GeometryState::GetParentFrame
        struct /* GetParentFrame */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of SceneGraphInspector∷GetParentFrame(FrameId) const.)""";
        } GetParentFrame;
        // Symbol: drake::geometry::GeometryState::GetPerceptionProperties
        struct /* GetPerceptionProperties */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of SceneGraphInspector∷GetPerceptionProperties().)""";
        } GetPerceptionProperties;
        // Symbol: drake::geometry::GeometryState::GetPoseInFrame
        struct /* GetPoseInFrame */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of SceneGraphInspector∷X_FG().)""";
        } GetPoseInFrame;
        // Symbol: drake::geometry::GeometryState::GetProximityProperties
        struct /* GetProximityProperties */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of SceneGraphInspector∷GetProximityProperties().)""";
        } GetProximityProperties;
        // Symbol: drake::geometry::GeometryState::GetReferenceMesh
        struct /* GetReferenceMesh */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of SceneGraphInspector∷GetReferenceMesh().)""";
        } GetReferenceMesh;
        // Symbol: drake::geometry::GeometryState::GetRenderEngineByName
        struct /* GetRenderEngineByName */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of QueryObject∷GetRenderEngineByName.)""";
        } GetRenderEngineByName;
        // Symbol: drake::geometry::GeometryState::GetShape
        struct /* GetShape */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Support for SceneGraphInspector∷Reify().)""";
        } GetShape;
        // Symbol: drake::geometry::GeometryState::HasCollisions
        struct /* HasCollisions */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of QueryObject∷HasCollisions().)""";
        } HasCollisions;
        // Symbol: drake::geometry::GeometryState::HasRenderer
        struct /* HasRenderer */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of SceneGraph∷HasRenderer().)""";
        } HasRenderer;
        // Symbol: drake::geometry::GeometryState::IsDeformableGeometry
        struct /* IsDeformableGeometry */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of SceneGraphInspector∷IsDeformableGeometry().)""";
        } IsDeformableGeometry;
        // Symbol: drake::geometry::GeometryState::IsValidGeometryName
        struct /* IsValidGeometryName */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Reports whether the canonicalized version of the given candidate
geometry name is considered valid. This tests the requirements
described in the documentation of canonicalized_geometry_names
"GeometryInstance". When adding a geometry to a frame, if there is
doubt if a proposed name is valid, the name can be tested prior to
registering the geometry.

Parameter ``frame_id``:
    The id of the frame to which the geometry would be assigned.

Parameter ``role``:
    The role for the candidate name.

Parameter ``candidate_name``:
    The name to validate.

Returns:
    true if the ``candidate_name`` can be given to a
    ``GeometryInstance`` assigned to the indicated frame with the
    indicated role.

Raises:
    RuntimeError if ``frame_id`` does not refer to a valid frame.)""";
        } IsValidGeometryName;
        // Symbol: drake::geometry::GeometryState::NumAnchoredGeometries
        struct /* NumAnchoredGeometries */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of SceneGraphInspector∷NumAnchoredGeometries().)""";
        } NumAnchoredGeometries;
        // Symbol: drake::geometry::GeometryState::NumDeformableGeometries
        struct /* NumDeformableGeometries */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Returns the total number of registered deformable geometries. All
deformable geometries are dynamic and *not* anchored.)""";
        } NumDeformableGeometries;
        // Symbol: drake::geometry::GeometryState::NumDeformableGeometriesWithRole
        struct /* NumDeformableGeometriesWithRole */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of
SceneGraphInspector∷NumDeformableGeometriesWithRole().)""";
        } NumDeformableGeometriesWithRole;
        // Symbol: drake::geometry::GeometryState::NumDynamicGeometries
        struct /* NumDynamicGeometries */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of SceneGraphInspector∷NumDynamicGeometries().)""";
        } NumDynamicGeometries;
        // Symbol: drake::geometry::GeometryState::NumDynamicNonDeformableGeometries
        struct /* NumDynamicNonDeformableGeometries */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Returns the total number of registered dynamic non-deformable
geometries.)""";
        } NumDynamicNonDeformableGeometries;
        // Symbol: drake::geometry::GeometryState::NumFramesForSource
        struct /* NumFramesForSource */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of SceneGraphInspector∷NumFramesForSource().)""";
        } NumFramesForSource;
        // Symbol: drake::geometry::GeometryState::NumGeometriesForFrame
        struct /* NumGeometriesForFrame */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of SceneGraphInspector∷NumGeometriesForFrame().)""";
        } NumGeometriesForFrame;
        // Symbol: drake::geometry::GeometryState::NumGeometriesForFrameWithRole
        struct /* NumGeometriesForFrameWithRole */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of SceneGraphInspector∷NumGeometriesForFrameWithRole().)""";
        } NumGeometriesForFrameWithRole;
        // Symbol: drake::geometry::GeometryState::NumGeometriesWithRole
        struct /* NumGeometriesWithRole */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc_1args =
R"""(Implementation of SceneGraphInspector∷NumGeometriesWithRole().)""";
          // Source: drake/geometry/geometry_state.h
          const char* doc_2args =
R"""(Reports the number of child geometries for this frame that have the
indicated role assigned. This only includes the immediate child
geometries of this* frame, and not those of child frames.

Raises:
    RuntimeError if the ``frame_id`` does not map to a valid frame.)""";
        } NumGeometriesWithRole;
        // Symbol: drake::geometry::GeometryState::RegisterAnchoredGeometry
        struct /* RegisterAnchoredGeometry */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of SceneGraph∷RegisterAnchoredGeometry().)""";
        } RegisterAnchoredGeometry;
        // Symbol: drake::geometry::GeometryState::RegisterDeformableGeometry
        struct /* RegisterDeformableGeometry */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of SceneGraph∷RegisterDeformableGeometry())""";
        } RegisterDeformableGeometry;
        // Symbol: drake::geometry::GeometryState::RegisterFrame
        struct /* RegisterFrame */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc_2args =
R"""(Implementation of SceneGraph∷RegisterFrame().)""";
          // Source: drake/geometry/geometry_state.h
          const char* doc_3args =
R"""(Implementation of SceneGraph∷RegisterFrame(SourceId,FrameId,const
GeometryFrame&) "SceneGraph∷RegisterFrame()" with parent FrameId.)""";
        } RegisterFrame;
        // Symbol: drake::geometry::GeometryState::RegisterGeometry
        struct /* RegisterGeometry */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of SceneGraph∷RegisterGeometry(SourceId,FrameId,
std∷unique_ptr<GeometryInstance>) "SceneGraph∷RegisterGeometry()" with
parent FrameId.)""";
        } RegisterGeometry;
        // Symbol: drake::geometry::GeometryState::RegisterNewSource
        struct /* RegisterNewSource */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of SceneGraph∷RegisterSource(). The default logic is to
define name as "Source_##" where the number is the value of the
returned SourceId.)""";
        } RegisterNewSource;
        // Symbol: drake::geometry::GeometryState::RegisteredRendererNames
        struct /* RegisteredRendererNames */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of SceneGraph∷RegisteredRendererNames().)""";
        } RegisteredRendererNames;
        // Symbol: drake::geometry::GeometryState::RemoveFromRenderer
        struct /* RemoveFromRenderer */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc_3args_renderer_name_source_id_frame_id =
R"""(For every geometry directly registered to the frame with the given
``frame_id``, if it has been added to the renderer with the given
``renderer_name`` it is removed from that renderer.

Returns:
    The number of geometries affected by the removal.

Raises:
    RuntimeError if a) ``source_id`` does not map to a registered
    source, b) ``frame_id`` does not map to a registered frame, c)
    ``frame_id`` does not belong to ``source_id`` (unless ``frame_id``
    is the world frame id), or d) the context has already been
    allocated.)""";
          // Source: drake/geometry/geometry_state.h
          const char* doc_3args_renderer_name_source_id_geometry_id =
R"""(Removes the geometry with the given ``geometry_id`` from the renderer
with the given ``renderer_name``, *if* it has previously been added.

Returns:
    The number of geometries affected by the removal (0 or 1).

Raises:
    RuntimeError if a) ``source_id`` does not map to a registered
    source, b) ``geometry_id`` does not map to a registered geometry,
    c) ``geometry_id`` does not belong to ``source_id``, or d) the
    context has already been allocated.)""";
        } RemoveFromRenderer;
        // Symbol: drake::geometry::GeometryState::RemoveGeometry
        struct /* RemoveGeometry */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of SceneGraph∷RemoveGeometry().)""";
        } RemoveGeometry;
        // Symbol: drake::geometry::GeometryState::RemoveRenderer
        struct /* RemoveRenderer */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of SceneGraph∷RemoveRenderer().)""";
        } RemoveRenderer;
        // Symbol: drake::geometry::GeometryState::RemoveRole
        struct /* RemoveRole */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc_3args_source_id_frame_id_role =
R"""(Implementation of SceneGraph∷RemoveRole(SourceId, FrameId, Role)
"SceneGraph∷RemoveRole()".)""";
          // Source: drake/geometry/geometry_state.h
          const char* doc_3args_source_id_geometry_id_role =
R"""(Implementation of SceneGraph∷RemoveRole(SourceId, GeometryId, Role)
"SceneGraph∷RemoveRole()".)""";
        } RemoveRole;
        // Symbol: drake::geometry::GeometryState::RenameFrame
        struct /* RenameFrame */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of SceneGraph∷RenameFrame().)""";
        } RenameFrame;
        // Symbol: drake::geometry::GeometryState::RenameGeometry
        struct /* RenameGeometry */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of SceneGraph∷RenameGeometry().)""";
        } RenameGeometry;
        // Symbol: drake::geometry::GeometryState::RenderColorImage
        struct /* RenderColorImage */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of QueryObject∷RenderColorImage().

Precondition:
    All poses have already been updated.)""";
        } RenderColorImage;
        // Symbol: drake::geometry::GeometryState::RenderDepthImage
        struct /* RenderDepthImage */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of QueryObject∷RenderDepthImage().

Precondition:
    All poses have already been updated.)""";
        } RenderDepthImage;
        // Symbol: drake::geometry::GeometryState::RenderLabelImage
        struct /* RenderLabelImage */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of QueryObject∷RenderLabelImage().

Precondition:
    All poses have already been updated.)""";
        } RenderLabelImage;
        // Symbol: drake::geometry::GeometryState::RendererCount
        struct /* RendererCount */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of SceneGraph∷RendererCount().)""";
        } RendererCount;
        // Symbol: drake::geometry::GeometryState::SourceIsRegistered
        struct /* SourceIsRegistered */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of SceneGraphInspector∷SourceIsRegistered().)""";
        } SourceIsRegistered;
        // Symbol: drake::geometry::GeometryState::ToAutoDiffXd
        struct /* ToAutoDiffXd */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Returns a deep copy of this state using the AutoDiffXd scalar with all
scalar values initialized from the current values. If this is invoked
on an instance already instantiated on AutoDiffXd, it is equivalent to
cloning the instance.)""";
        } ToAutoDiffXd;
        // Symbol: drake::geometry::GeometryState::collision_filter_manager
        struct /* collision_filter_manager */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of SceneGraph∷collision_filter_manager().)""";
        } collision_filter_manager;
        // Symbol: drake::geometry::GeometryState::geometry_version
        struct /* geometry_version */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of SceneGraphInspector∷GetGeometryVersion().)""";
        } geometry_version;
        // Symbol: drake::geometry::GeometryState::get_configurations_in_world
        struct /* get_configurations_in_world */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of QueryObject∷GetConfigurationsInWorld().)""";
        } get_configurations_in_world;
        // Symbol: drake::geometry::GeometryState::get_frame_ids
        struct /* get_frame_ids */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of SceneGraphInspector∷GetAllFrameIds().)""";
        } get_frame_ids;
        // Symbol: drake::geometry::GeometryState::get_num_frames
        struct /* get_num_frames */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of SceneGraphInspector∷num_frames().)""";
        } get_num_frames;
        // Symbol: drake::geometry::GeometryState::get_num_geometries
        struct /* get_num_geometries */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of SceneGraphInspector∷num_geometries().)""";
        } get_num_geometries;
        // Symbol: drake::geometry::GeometryState::get_num_sources
        struct /* get_num_sources */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of SceneGraphInspector∷num_sources().)""";
        } get_num_sources;
        // Symbol: drake::geometry::GeometryState::get_pose_in_parent
        struct /* get_pose_in_parent */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of QueryObject∷GetPoseInParent().)""";
        } get_pose_in_parent;
        // Symbol: drake::geometry::GeometryState::get_pose_in_world
        struct /* get_pose_in_world */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc_1args_frame_id =
R"""(Implementation of QueryObject∷GetPoseInWorld(FrameId).)""";
          // Source: drake/geometry/geometry_state.h
          const char* doc_1args_geometry_id =
R"""(Implementation of QueryObject∷GetPoseInWorld(GeometryId).)""";
        } get_pose_in_world;
        // Symbol: drake::geometry::GeometryState::maybe_get_hydroelastic_mesh
        struct /* maybe_get_hydroelastic_mesh */ {
          // Source: drake/geometry/geometry_state.h
          const char* doc =
R"""(Implementation of SceneGraphInspector∷maybe_get_hydroelastic_mesh().)""";
        } maybe_get_hydroelastic_mesh;
      } GeometryState;
      // Symbol: drake::geometry::GeometryVersion
      struct /* GeometryVersion */ {
        // Source: drake/geometry/geometry_version.h
        const char* doc =
R"""(A version numbering class that reports revisions of SceneGraph's
geometric data.

Other Systems can use this version number to perform updates when they
detect changes to the geometric data they consume. The version of the
geometry data is made available through SceneGraphInspector.

The geometry data is partitioned by geometric role and have
independent role version values. Some of SceneGraph's API (as
variously documented) will cause one or more role versions to change.
This class provides the API ``IsSameAs`` that takes another
GeometryVersion as well as a Role to help detect whether the provided
role of the geometries may have changed. For example:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    // Downstream system holds an instance of GeometryVersion `old_version` as a
    // reference to compare against.
    // Get the version under test from SceneGraphInspector.
    const GeometryVersion& test_version = scene_graph_inspector.geometry_version();
    // Determine if two versions have the same proximity data.
    bool same_proximity = old_version.IsSameAs(test_version, Role∷kProximity);

.. raw:: html

    </details>)""";
        // Symbol: drake::geometry::GeometryVersion::GeometryVersion
        struct /* ctor */ {
          // Source: drake/geometry/geometry_version.h
          const char* doc =
R"""(Constructs a default-initialized instance; guaranteed to be different
from every other instance.)""";
        } ctor;
        // Symbol: drake::geometry::GeometryVersion::IsSameAs
        struct /* IsSameAs */ {
          // Source: drake/geometry/geometry_version.h
          const char* doc =
R"""(Returns true if ``this`` GeometryVersion has the same ``role`` version
as the ``other`` GeometryVersion.)""";
        } IsSameAs;
      } GeometryVersion;
      // Symbol: drake::geometry::HalfSpace
      struct /* HalfSpace */ {
        // Source: drake/geometry/shape_specification.h
        const char* doc =
R"""(Definition of a half space. In its canonical frame, the plane defining
the boundary of the half space is that frame's z = 0 plane. By
implication, the plane's normal points in the +z direction and the
origin lies on the plane. Other shapes are considered to be
penetrating the half space if there exists a point on the test shape
that lies on the side of the plane opposite the normal.)""";
        // Symbol: drake::geometry::HalfSpace::HalfSpace
        struct /* ctor */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::geometry::HalfSpace::MakePose
        struct /* MakePose */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc =
R"""(Creates the pose of a canonical half space in frame F. The half
space's normal is aligned to the positive z-axis of its canonical
frame H. Given a vector that points in the same direction, measured in
the F frame (Hz_dir_F) and a position vector to a point on the half
space's boundary* expressed in the same frame, ``p_FB``, creates the
pose of the half space in frame F: ``X_FH``.

Parameter ``Hz_dir_F``:
    A vector in the direction of the positive z-axis of the canonical
    frame expressed in frame F. It must be a non-zero vector but need
    not be unit length.

Parameter ``p_FB``:
    A point B lying on the half space's boundary measured and
    expressed in frame F.

Returns ``X_FH``:
    The pose of the canonical half-space in frame F.

Raises:
    RuntimeError if the normal is *close* to a zero-vector (e.g.,
    ‖normal_F‖₂ < ε).)""";
        } MakePose;
      } HalfSpace;
      // Symbol: drake::geometry::IllustrationProperties
      struct /* IllustrationProperties */ {
        // Source: drake/geometry/geometry_roles.h
        const char* doc =
R"""(The set of properties for geometry used in an "illustration" role.

Examples of functionality that depends on the illustration role: -
drake_visualizer_role_consumer "drake∷geometry∷DrakeVisualizer")""";
        // Symbol: drake::geometry::IllustrationProperties::IllustrationProperties
        struct /* ctor */ {
          // Source: drake/geometry/geometry_roles.h
          const char* doc = R"""()""";
        } ctor;
      } IllustrationProperties;
      // Symbol: drake::geometry::InMemoryMesh
      struct /* InMemoryMesh */ {
        // Source: drake/geometry/in_memory_mesh.h
        const char* doc =
R"""(Representation of a mesh file stored in memory. At a minimum it
includes the contents of a mesh file. If that mesh file makes
reference to additional files (e.g., a .obj file can reference a .mtl
which in turn references a .png file), then those files *can* be
included in ``this`` InMemoryMesh's supporting files. Each supporting
file is represented by a key-value pair. The key is the
file-referencing string as it appears in the referencing file and the
value is a FileSource. All supporting files can be located on disk;
only the main mesh file is strictly required to be in memory. Failure
to provide the supporting files may or may not lead to errors; it
depends on the context in which the mesh data is used.)""";
        // Symbol: drake::geometry::InMemoryMesh::mesh_file
        struct /* mesh_file */ {
          // Source: drake/geometry/in_memory_mesh.h
          const char* doc =
R"""(The main mesh file's contents (e.g., a .obj or .gltf file, but not a
.mtl or .bin).)""";
        } mesh_file;
        // Symbol: drake::geometry::InMemoryMesh::supporting_files
        struct /* supporting_files */ {
          // Source: drake/geometry/in_memory_mesh.h
          const char* doc =
R"""(The optional collection of supporting files.)""";
        } supporting_files;
        // Symbol: drake::geometry::InMemoryMesh::to_string
        struct /* to_string */ {
          // Source: drake/geometry/in_memory_mesh.h
          const char* doc = R"""(Returns a string representation.)""";
        } to_string;
      } InMemoryMesh;
      // Symbol: drake::geometry::KinematicsVector
      struct /* KinematicsVector */ {
        // Source: drake/geometry/kinematics_vector.h
        const char* doc =
R"""(A KinematicsVector is a container class used to report kinematics data
for registered frames and geometries (keyed by unique
FrameId/GeometryId values) to SceneGraph where the set of keys
(FrameId/GeometryId) is usually constant and the values (kinematics
data) are varying. It is an internal class and one should never
interact with it directly. The template aliases FramePoseVector and
GeometryConfigurationVector that instantiate KinematicsVector should
be used instead.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    template <typename T>
    class MySystem : public LeafSystem<T> {
    public:
    MySystem() {
    ...
    this->DeclareAbstractOutputPort(
    &AllocInConstructorSystem∷CalcFramePoseOutput);
    ...
    }
    
    private:
    void CalcFramePoseOutput(const Context<T>& context,
    geometry∷FramePoseVector<T>* poses) const {
    poses->clear();
    for (int i = 0; i < static_cast<int>(frame_ids_.size()); ++i) {
    poses->set_value(frame_ids_[i], poses_[i]);
    }
    }
    
    std∷vector<FrameId> frame_ids_;
    std∷vector<RigidTransform<T>> poses_;
    };

.. raw:: html

    </details>

If a System only ever emits a single frame/geometry (or
small-constant-number of frames/geometries), then there's a shorter
alternative way to write a Calc method, using an initializer_list:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    void CalcFramePoseOutput(const Context<T>& context,
    geometry∷FramePoseVector<T>* poses) const {
    const RigidTransform<T>& pose = ...;
    poses = {{frame_id_, pose}};
    }

.. raw:: html

    </details>

N.B. When the systems framework calls the ``Calc`` method, the value
pointed to by ``poses`` is in an unspecified state. The implementation
of ``Calc`` must always ensure that ``poses`` contains the correct
value upon return, no matter what value it started with. The easy ways
to do this are to call either ``poses->clear()`` or the assignment
operator ``*poses = ...``.

Template parameter ``Id``:
    The key used to locate the kinematics data. Can be FrameId or
    GeometryId.

Template parameter ``KinematicsValue``:
    The underlying data type of the kinematics data (e.g., pose,
    configuration, or velocity).

The FramePoseVector and GeometryConfigurationVector classes are
aliases of the KinematicsVector instantiated on specific data types
(RigidTransform and VectorX respectively). Each of these data types
are templated on Eigen scalars. All supported combinations of data
type and scalar type are already available to link against in the
containing library. No other values for KinematicsValue are supported.

Currently, the following data types with the following scalar types
are supported:

Alias | Instantiation | Scalar types
-------------------------------------|--------------------------------------------------|-------------
FramePoseVector<Scalar> |
KinematicsVector<FrameId,RigidTransform<Scalar>> |
double/AutoDiffXd/Expression GeometryConfigurationVector<Scalar> |
KinematicsVector<GeometryId, VectorX<Scalar>> |
double/AutoDiffXd/Expression)""";
        // Symbol: drake::geometry::KinematicsVector::IsFinite
        struct /* IsFinite */ {
          // Source: drake/geometry/kinematics_vector.h
          const char* doc = R"""(Reports if *all* values are finite.)""";
        } IsFinite;
        // Symbol: drake::geometry::KinematicsVector::KinematicsVector<Id, KinematicsValue>
        struct /* ctor */ {
          // Source: drake/geometry/kinematics_vector.h
          const char* doc_0args = R"""(Initializes the vector with no data .)""";
          // Source: drake/geometry/kinematics_vector.h
          const char* doc_1args =
R"""(Initializes the vector using the given the keys and their
corresponding kinematics values.)""";
        } ctor;
        // Symbol: drake::geometry::KinematicsVector::clear
        struct /* clear */ {
          // Source: drake/geometry/kinematics_vector.h
          const char* doc =
R"""(Clears all values, resetting the size to zero.)""";
        } clear;
        // Symbol: drake::geometry::KinematicsVector::has_id
        struct /* has_id */ {
          // Source: drake/geometry/kinematics_vector.h
          const char* doc =
R"""(Reports true if the given id is a member of this data.)""";
        } has_id;
        // Symbol: drake::geometry::KinematicsVector::ids
        struct /* ids */ {
          // Source: drake/geometry/kinematics_vector.h
          const char* doc =
R"""(Provides a range object for all of the existing ids in the vector.
This is intended to be used as:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    for (Id id : this_vector.ids()) {
    ...
    // Obtain the KinematicsValue of an id by `this_vector.value(id)`
    ...
    }

.. raw:: html

    </details>)""";
        } ids;
        // Symbol: drake::geometry::KinematicsVector::set_value
        struct /* set_value */ {
          // Source: drake/geometry/kinematics_vector.h
          const char* doc =
R"""(Sets the kinematics ``value`` for the given ``id``.)""";
        } set_value;
        // Symbol: drake::geometry::KinematicsVector::size
        struct /* size */ {
          // Source: drake/geometry/kinematics_vector.h
          const char* doc = R"""(Returns number of ids().)""";
        } size;
        // Symbol: drake::geometry::KinematicsVector::value
        struct /* value */ {
          // Source: drake/geometry/kinematics_vector.h
          const char* doc =
R"""(Returns the value associated with the given ``id``.

Raises:
    RuntimeError if ``id`` is not in the specified set of ids.)""";
        } value;
      } KinematicsVector;
      // Symbol: drake::geometry::MakePhongIllustrationProperties
      struct /* MakePhongIllustrationProperties */ {
        // Source: drake/geometry/geometry_roles.h
        const char* doc =
R"""(Constructs an IllustrationProperties instance compatible with a simple
"phong" material using only the given ``diffuse`` color.)""";
      } MakePhongIllustrationProperties;
      // Symbol: drake::geometry::Mesh
      struct /* Mesh */ {
        // Source: drake/geometry/shape_specification.h
        const char* doc =
R"""(Definition of a general (possibly non-convex) mesh.

The mesh may be a triangular surface mesh or a tetrahedral volume
mesh, depending on how it used.

Meshes can be used with all three roles but, currently, the support
for the proximity role is limited. Where a general mesh is not
supported, the mesh is replaced by its convex hull. See the
documentation of QueryObject's proximity queries for more details. The
notable cases where the actual mesh topology is used includes:

- Computing signed distances from the Mesh to query points (when it
references a .obj file or a tetrahedral .vtk file).
- Specifying the Mesh as rigid hydroelastic (when it references a triangle
.obj file or a tetrahedral .vtk file).
- Specifying the Mesh as compliant hydroelastic (when it references a
tetrahedral .vtk file).
- Specifying the Mesh as deformable (when it references a tetrahedral .vtk
file).

This convex-hull substitution is a regrettable stop-gap solution until
we fully support general, non-convex meshes throughout proximity
queries.

For visual roles (illustration and perception), the specified mesh
file is used as directly as possible.

The mesh is defined in a canonical frame C, implicit in the file
parsed. Upon loading it in SceneGraph it can be scaled around the
origin of C by a given ``scale`` amount.

Note: a negative scale can be applied. This can be useful in mirroring
the geometry (e.g., using a right hand mesh for a left hand).
Mirroring the geometry will typically change the "winding" of the mesh
elements. By convention, Drake looks at the *ordering* of the vertices
that form mesh elements (triangles and tetrahedra) and derives the
notion of "inside" and "outside" relative to that element. In order to
preserve the input mesh's definition of "inside" and "outside", when
the mesh gets mirrored Drake may perturb the ordering of the vertex
indices. For example, a triangle originally referencing vertices ``[0
1 2]``, when mirrored may change to ``[2 1 0]``, so don't be surprised
if you introspect the details of the loaded mesh and you see such a
change. An analogous change can affect the vertex ordering of
tetrahedra in a volume mesh (i.e., a perturbation of the original
vertex index list ``[0 1 2 3]`` to ``[2 1 0 3]``).)""";
        // Symbol: drake::geometry::Mesh::GetConvexHull
        struct /* GetConvexHull */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc =
R"""(Reports the convex hull of the named mesh.

Note: the convex hull is computed on demand on the first invocation.
All subsequent invocations should have an O(1) cost.

Raises:
    if the referenced mesh data cannot be read or is degenerate
    (insufficient number of vertices, co-linear or coincident
    vertices, etc.) All of the vertices lying on a plane is *not*
    considered degenerate.)""";
        } GetConvexHull;
        // Symbol: drake::geometry::Mesh::Mesh
        struct /* ctor */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc_2args_filename_scale =
R"""(Constructs a mesh shape specification from the mesh file located at
the given file path. Optionally uniformly scaled by the given scale
factor.

The mesh file referenced can be an .obj, a volume mesh in a .vtk, or a
.gltf file. However, not all file formats are appropriate for all
roles. (E.g., a tetrahedral .vtk file should not be assigned a
perception role.)

Parameter ``filename``:
    The file name; if it is not absolute, it will be interpreted
    relative to the current working directory.

Parameter ``scale``:
    An optional scale to coordinates.

Raises:
    RuntimeError if |scale| < 1e-8. Note that a negative scale is
    considered valid. We want to preclude scales near zero but
    recognise that scale is a convenience tool for "tweaking" models.
    8 orders of magnitude should be plenty without considering
    revisiting the model itself.)""";
          // Source: drake/geometry/shape_specification.h
          const char* doc_2args_filename_scale3 =
R"""(Mesh-file variant that allows for specification of non-uniform scale.)""";
          // Source: drake/geometry/shape_specification.h
          const char* doc_2args_mesh_data_scale =
R"""(Constructs a mesh shape specification from the contents of a
Drake-supported mesh file type.

The mesh is defined by the contents of a supported_file_types "mesh
file format supported by Drake". Those contents are passed in as
``mesh_data``. The mesh data should include the main mesh file's
contents as well as any supporting file contents as needed. See
InMemoryMesh.

Parameter ``mesh_data``:
    The in-memory file contents that define the mesh data for this
    shape.

Parameter ``scale``:
    An optional scale to coordinates.)""";
          // Source: drake/geometry/shape_specification.h
          const char* doc_2args_mesh_data_scale3 =
R"""(Mesh-contents variant that allows for specification of non-uniform
scale.)""";
          // Source: drake/geometry/shape_specification.h
          const char* doc_2args_source_scale =
R"""(Constructs a mesh shape specification from the given ``source``.

Parameter ``source``:
    The source for the mesh data.

Parameter ``scale``:
    An optional scale to coordinates.)""";
          // Source: drake/geometry/shape_specification.h
          const char* doc_2args_source_scale3 =
R"""(Mesh-source variant that allows for specification of non-uniform
scale.)""";
        } ctor;
        // Symbol: drake::geometry::Mesh::extension
        struct /* extension */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc =
R"""(Returns the extension of the mesh type -- all lower case and including
the dot. If ``this`` is constructed from a file path, the extension is
extracted from the path. I.e., /foo/bar/mesh.obj and /foo/bar/mesh.OBJ
would both report the ".obj" extension. The "extension" portion of the
filename is defined as in std∷filesystem∷path∷extension().

If ``this`` is constructed using in-memory file contents, it is the
extension of the MemoryFile passed to the constructor.)""";
        } extension;
        // Symbol: drake::geometry::Mesh::scale
        struct /* scale */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc =
R"""(Returns a single scale representing the *uniform* scale factor.

Raises:
    if the scale is not uniform in all directions.)""";
        } scale;
        // Symbol: drake::geometry::Mesh::scale3
        struct /* scale3 */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc =
R"""(Returns general scale factors for this mesh.)""";
        } scale3;
        // Symbol: drake::geometry::Mesh::source
        struct /* source */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc =
R"""(Returns the source for this specification's mesh data.)""";
        } source;
      } Mesh;
      // Symbol: drake::geometry::MeshSource
      struct /* MeshSource */ {
        // Source: drake/geometry/mesh_source.h
        const char* doc =
R"""(Provides a general abstraction to the definition of a mesh. A mesh
definition can come from disk or memory. APIs that support both can
take as specification an instance of MeshSource to communicate that
ability.)""";
        // Symbol: drake::geometry::MeshSource::MeshSource
        struct /* ctor */ {
          // Source: drake/geometry/mesh_source.h
          const char* doc_1args_path =
R"""(Constructs from a file path. Note: the path will not be validated in
any way (existence, availability, naming an actual mesh file, etc.).
Validation occurs where the MeshSource's path is *used*.)""";
          // Source: drake/geometry/mesh_source.h
          const char* doc_1args_mesh = R"""(Constructs from an in-memory mesh.)""";
        } ctor;
        // Symbol: drake::geometry::MeshSource::description
        struct /* description */ {
          // Source: drake/geometry/mesh_source.h
          const char* doc =
R"""(Provides a source-agnostic description of the mesh. If is_path() is
true, it is the path. If is_in_memory() is true, it is the
filename_hint for the in-memory mesh file. If the in-memory mesh file
has an empty filename hint, the description will explicitly
communicate that; the empty string will *never* be returned.)""";
        } description;
        // Symbol: drake::geometry::MeshSource::extension
        struct /* extension */ {
          // Source: drake/geometry/mesh_source.h
          const char* doc =
R"""(Returns the extension of the mesh type -- all lower case and including
the dot. If is_path() is ``True``, the extension is extracted from the
path. I.e., /foo/bar/mesh.obj and /foo/bar/mesh.OBJ would both report
the ".obj" extension. The "extension" portion of the filename is
defined as in std∷filesystem∷path∷extension().

If is_in_memory() is ``True``, it is the extension reported by the
MemoryFile.)""";
        } extension;
        // Symbol: drake::geometry::MeshSource::in_memory
        struct /* in_memory */ {
          // Source: drake/geometry/mesh_source.h
          const char* doc =
R"""(Returns the source's in-memory mesh data.

Precondition:
    is_in_memory() returns ``True``.)""";
        } in_memory;
        // Symbol: drake::geometry::MeshSource::is_in_memory
        struct /* is_in_memory */ {
          // Source: drake/geometry/mesh_source.h
          const char* doc =
R"""(Reports ``True`` if this source contains an in-memory mesh definition.)""";
        } is_in_memory;
        // Symbol: drake::geometry::MeshSource::is_path
        struct /* is_path */ {
          // Source: drake/geometry/mesh_source.h
          const char* doc =
R"""(Reports ``True`` if this source is a filesystem path.)""";
        } is_path;
        // Symbol: drake::geometry::MeshSource::path
        struct /* path */ {
          // Source: drake/geometry/mesh_source.h
          const char* doc =
R"""(Returns the source's file path.

Precondition:
    is_path() returns ``True``.)""";
        } path;
      } MeshSource;
      // Symbol: drake::geometry::Meshcat
      struct /* Meshcat */ {
        // Source: drake/geometry/meshcat.h
        const char* doc =
R"""(Provides an interface to Meshcat
(https://github.com/meshcat-dev/meshcat).

Each instance of this class spawns a thread which runs an
http/websocket server. Users can navigate their browser to the hosted
URL to visualize the Meshcat scene. Note that, unlike many
visualizers, one cannot open the visualizer until this server is
running.

Warning:
    In the current implementation, Meshcat methods must be called from
    the same thread where the class instance was constructed. For
    example, running multiple simulations in parallel using the same
    Meshcat instance is not yet supported. We may generalize this in
    the future.

Meshcat paths and the scene tree
================================

https://github.com/meshcat-dev/meshcat#api provides a nice
introduction to the websocket API that we wrap with this class. One of
the core concepts is the "scene tree" -- a directory-like structure of
objects, transforms, and properties. The scene tree is viewable in the
browser by clicking on "Open Controls" in the top right corner.

Elements of the tree are referenced programmatically by a
"/"-delimited string indicating the object's **path** in the scene
tree. An object at path "/foo/bar" is a child of an object at path
"/foo", so setting the transform of (or deleting) "/foo" will also
affect its children.

The string path arguments to the methods in this class use the
following semantics: - A path that begins with "/" is treated as an
absolute path, and is used without modification. - A path that *does
not* begin with "/" is treated as a relative path to the default
working directory. - The "working directory" is fixed to be "/drake"
in the current implementation. So any relative path "foo" will be
treated as "/drake/foo". - Delete("/foo") will remove all objects,
transforms, and properties in "/foo" and its children from the scene
tree. - Delete() is equivalent to Delete("") or Delete("/drake/"). -
SetObject("/foo", ...) places the object at "/foo/<object>", where
"<object>" is a hard-coded suffix used for all objects. You can use
the Delete("/foo/<object>") to delete the object only (and not the
children of "/foo") and must use SetProperty("/foo/<object>", ...) to
change object-specific properties.

The root directory contains a number of elements that are set up
automatically in the browser. These include "/Background", "/Lights",
"/Grid", and "/Cameras". To find more details please see the @link
https://github.com/meshcat-dev/meshcat#api meshcat documentation
@endlink and the @link https://threejs.org/docs/index.html three.js
documentation @endlink. - You can modify these elements, create new
lights/cameras, and even delete these elements (one at a time). -
Delete("/") is not allowed. It will be silently ignored.

Recommended workflow
====================

For convenience, Meshcat fosters a work flow in which all user-created
objects created in Drake are contained in the "/drake" folder. Objects
added with a relative path are placed in the "/drake" folder for you.
The benefits are: - It's simple to distinguish between user objects
and "infrastructure" objects in the visualizer. - All user objects can
easily be cleared by a single, parameter-free call to Delete(). You
are welcome to use absolute paths to organize your data, but the
burden on tracking and cleaning them up lie on you.

Parameters for the hosted Meshcat page
======================================

Meshcat has an *experimental* AR/VR option (using WebXR). It can be
enabled through url parameters. For example, for a meshcat url
``http://localhost:7000``, the following will enable the VR mode:

http://localhost:7000?webxr=vr

To to use augmented reality (where the meshcat background is replaced
with your device's camera image), use:

http://localhost:7000?webxr=ar

If augmented reality is not available, it will fallback to VR mode.

Some notes on using the AR/VR modes:

- Before starting the WebXR session, position the interactive camera to be
approximately where you want the origin of the head set's origin to be.
- The meshcat scene controls are disabled while the WebXR session is active.
- WebXR sessions can only be run with *perspective* cameras.
- The controllers can be *visualized* but currently can't interact with the
Drake simulation physically. To visualize the controllers append the
additional url parameter ``controller=on`` as in
``http://localhost:7000?webxr=vr&controller=on``.

If you do not have AR/VR hardware, you can use an emulator in your
browser to experiment with the mode. Use an browser plugin like WebXR
API Emulator (i.e., for
[Chrome](https://chrome.google.com/webstore/detail/webxr-api-emulator/mjddjgeghkdijejnciaefnkjmkafnnje)
or
[Firefox](https://addons.mozilla.org/en-US/firefox/addon/webxr-api-emulator/)).

The AR/VR mode is not currently supported in offline mode (i.e., when
saving as StaticHtml()).

Network access
==============

See MeshcatParams for options to control the hostname and port to bind
to.

See allow_network "DRAKE_ALLOW_NETWORK" for an environment variable
option to deny Meshcat entirely.)""";
        // Symbol: drake::geometry::Meshcat::AddButton
        struct /* AddButton */ {
          // Source: drake/geometry/meshcat.h
          const char* doc =
R"""(Adds a button with the label ``name`` to the meshcat browser controls
GUI. If the optional ``keycode`` is set to a javascript string key
code (such as "KeyG" or "ArrowLeft", see
https://developer.mozilla.org/en-US/docs/Web/API/UI_Events/Keyboard_event_code_values),
then a keydown callback is registered in the GUI which will also
``click`` the button. If the button already existed, then resets its
click count to zero; and sets the keycode if no keycode was set
before.

Raises:
    RuntimeError if ``name`` has already been added as any other type
    of control (e.g., slider).

Raises:
    RuntimeError if a button of the same ``name`` has already been
    assigned a different keycode.)""";
        } AddButton;
        // Symbol: drake::geometry::Meshcat::AddSlider
        struct /* AddSlider */ {
          // Source: drake/geometry/meshcat.h
          const char* doc =
R"""(Adds a slider with the label ``name`` to the meshcat browser controls
GUI. The slider range is given by [`min`, ``max`]. `step`` is the
smallest increment by which the slider can change values (and
therefore send updates back to this Meshcat instance). ``value``
specifies the initial value; it will be truncated to the slider range
and rounded to the nearest increment. If the optional
``decrement_keycode`` or ``increment_keycode`` are set to a javascript
string key code (such as "KeyG" or "ArrowLeft", see
https://developer.mozilla.org/en-US/docs/Web/API/UI_Events/Keyboard_event_code_values),
then keydown callbacks will be registered in the GUI that will move
the slider by ``step`` (within the limits) when those buttons are
pressed.

Returns:
    the truncated and rounded value that was actually set.

Raises:
    RuntimeError if ``name`` has already been added as any type of
    control (e.g., either button or slider).)""";
        } AddSlider;
        // Symbol: drake::geometry::Meshcat::Delete
        struct /* Delete */ {
          // Source: drake/geometry/meshcat.h
          const char* doc =
R"""(Deletes the object at the given ``path`` as well as all of its
children. See meshcat_path for the detailed semantics of deletion.)""";
        } Delete;
        // Symbol: drake::geometry::Meshcat::DeleteAddedControls
        struct /* DeleteAddedControls */ {
          // Source: drake/geometry/meshcat.h
          const char* doc =
R"""(Removes all buttons and sliders from the GUI that have been registered
by this Meshcat instance. It does *not* clear the default GUI elements
set in the meshcat browser (e.g. for cameras and lights).)""";
        } DeleteAddedControls;
        // Symbol: drake::geometry::Meshcat::DeleteButton
        struct /* DeleteButton */ {
          // Source: drake/geometry/meshcat.h
          const char* doc =
R"""(Removes the button ``name`` from the GUI.

Returns:
    true iff the button was removed.

Raises:
    RuntimeError if ``strict`` is true and ``name`` is not a
    registered button.)""";
        } DeleteButton;
        // Symbol: drake::geometry::Meshcat::DeleteRecording
        struct /* DeleteRecording */ {
          // Source: drake/geometry/meshcat.h
          const char* doc =
R"""(Deletes the current animation holding the recorded frames. Animation
options (autoplay, repetitions, etc) will also be reset, and any
pointers obtained from get_mutable_recording() will be rendered
invalid. This does not* currently remove the animation from Meshcat.)""";
        } DeleteRecording;
        // Symbol: drake::geometry::Meshcat::DeleteSlider
        struct /* DeleteSlider */ {
          // Source: drake/geometry/meshcat.h
          const char* doc =
R"""(Removes the slider ``name`` from the GUI.

Returns:
    true iff the slider was removed.

Raises:
    RuntimeError if ``strict`` is true and ``name`` is not a
    registered slider.)""";
        } DeleteSlider;
        // Symbol: drake::geometry::Meshcat::Flush
        struct /* Flush */ {
          // Source: drake/geometry/meshcat.h
          const char* doc =
R"""(Blocks the calling thread until all buffered data in the websocket
thread has been sent to any connected clients. This can be especially
useful when sending many or large mesh files / texture maps, to avoid
large "backpressure" and/or simply to make sure that the simulation
does not get far ahead of the visualization.)""";
        } Flush;
        // Symbol: drake::geometry::Meshcat::Gamepad
        struct /* Gamepad */ {
          // Source: drake/geometry/meshcat.h
          const char* doc =
R"""(Status of a gamepad obtained from the Meshcat javascript client.)""";
          // Symbol: drake::geometry::Meshcat::Gamepad::Serialize
          struct /* Serialize */ {
            // Source: drake/geometry/meshcat.h
            const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
          } Serialize;
          // Symbol: drake::geometry::Meshcat::Gamepad::axes
          struct /* axes */ {
            // Source: drake/geometry/meshcat.h
            const char* doc =
R"""(An array of floating point values representing e.g. analog
thumbsticks. Each entry in the array is a floating point value in the
range -1.0 – 1.0, representing the axis position from the lowest value
(-1.0) to the highest value (1.0).

In the standard gamepad mapping, we have: - axes[0] Left stick x
(negative left/positive right) - axes[1] Left stick y (negative
up/positive down) - axes[2] Right stick x (negative left/positive
right) - axes[3] Right stick y (negative up/positive down)

Note that a stick that is left alone may not output all zeros.
https://beej.us/blog/data/javascript-gamepad/ gives some useful advice
for applying a deadzone to these values.)""";
          } axes;
          // Symbol: drake::geometry::Meshcat::Gamepad::button_values
          struct /* button_values */ {
            // Source: drake/geometry/meshcat.h
            const char* doc =
R"""(An array of floating point values representing analog buttons, such as
the triggers on many modern gamepads. The values are normalized to the
range [0.0, 1.0], with 0.0 representing a button that is not pressed,
and 1.0 representing a button that is fully pressed.

See https://w3c.github.io/gamepad/#dfn-standard-gamepad for the
standard mapping of gamepad buttons to this vector.)""";
          } button_values;
          // Symbol: drake::geometry::Meshcat::Gamepad::index
          struct /* index */ {
            // Source: drake/geometry/meshcat.h
            const char* doc =
R"""(An an integer that is auto-incremented to be unique for each device
currently connected to the system. If ``index.has_value() == false``,
then we have not yet received any gamepad status from the Meshcat
browser.)""";
          } index;
          auto Serialize__fields() const {
            return std::array{
              std::make_pair("axes", axes.doc),
              std::make_pair("button_values", button_values.doc),
              std::make_pair("index", index.doc),
            };
          }
        } Gamepad;
        // Symbol: drake::geometry::Meshcat::GetButtonClicks
        struct /* GetButtonClicks */ {
          // Source: drake/geometry/meshcat.h
          const char* doc =
R"""(Returns the number of times the button ``name`` has been clicked in
the GUI, from the time that it was added to ``this``. If multiple
browsers are open, then this number is the cumulative number of clicks
in all browsers.

Raises:
    RuntimeError if ``name`` is not a registered button.)""";
        } GetButtonClicks;
        // Symbol: drake::geometry::Meshcat::GetButtonNames
        struct /* GetButtonNames */ {
          // Source: drake/geometry/meshcat.h
          const char* doc = R"""(Returns the names of all buttons.)""";
        } GetButtonNames;
        // Symbol: drake::geometry::Meshcat::GetGamepad
        struct /* GetGamepad */ {
          // Source: drake/geometry/meshcat.h
          const char* doc =
R"""(Returns the status from the most recently updated gamepad data in the
Meshcat. See Gamepad for details on the returned values.

Note that in javascript, gamepads are not detected until users
"opt-in" by pressing a gamepad button or moving the thumbstick in the
Meshcat window. If no gamepad information is available in javascript,
then no messages are sent and the returned gamepad index will not have
a value.

Currently Meshcat only attempts to support one gamepad. If multiple
gamepads are detected in the same Meshcat window, then only the status
of the first connected gamepad in ``navigator.GetGamepads()`` is
returned. If multiple Meshcat windows are connected to this Meshcat
instance, and gamepads are being used in multiple windows, then the
returned status will be the most recently received status message.
Therefore using multiple gamepads simultaneously is not recommended.

This feature is provided primarily to support applications where Drake
is running on a remote machine (e.g. in the cloud), and the Meshcat
javascript in the browser is the only code running on the local
machine which has access to the gamepad.

For more details on javascript support for gamepads (or to test that
your gamepad is working), see
https://beej.us/blog/data/javascript-gamepad/.)""";
        } GetGamepad;
        // Symbol: drake::geometry::Meshcat::GetNumActiveConnections
        struct /* GetNumActiveConnections */ {
          // Source: drake/geometry/meshcat.h
          const char* doc =
R"""((Advanced) Returns the number of currently-open websocket connections.)""";
        } GetNumActiveConnections;
        // Symbol: drake::geometry::Meshcat::GetPackedObject
        struct /* GetPackedObject */ {
          // Source: drake/geometry/meshcat.h
          const char* doc = R"""()""";
        } GetPackedObject;
        // Symbol: drake::geometry::Meshcat::GetPackedProperty
        struct /* GetPackedProperty */ {
          // Source: drake/geometry/meshcat.h
          const char* doc = R"""()""";
        } GetPackedProperty;
        // Symbol: drake::geometry::Meshcat::GetPackedTransform
        struct /* GetPackedTransform */ {
          // Source: drake/geometry/meshcat.h
          const char* doc = R"""()""";
        } GetPackedTransform;
        // Symbol: drake::geometry::Meshcat::GetRealtimeRate
        struct /* GetRealtimeRate */ {
          // Source: drake/geometry/meshcat.h
          const char* doc =
R"""(Gets the realtime rate that was last broadcast by this instance
(typically, the value displayed in the meshcat visualizer stats
chart). See SetRealtimeRate().)""";
        } GetRealtimeRate;
        // Symbol: drake::geometry::Meshcat::GetSimulationTime
        struct /* GetSimulationTime */ {
          // Source: drake/geometry/meshcat.h
          const char* doc =
R"""(Gets the last time value passed to SetSimulationTime().)""";
        } GetSimulationTime;
        // Symbol: drake::geometry::Meshcat::GetSliderNames
        struct /* GetSliderNames */ {
          // Source: drake/geometry/meshcat.h
          const char* doc = R"""(Returns the names of all sliders.)""";
        } GetSliderNames;
        // Symbol: drake::geometry::Meshcat::GetSliderValue
        struct /* GetSliderValue */ {
          // Source: drake/geometry/meshcat.h
          const char* doc =
R"""(Gets the current ``value`` of the slider ``name``.

Raises:
    RuntimeError if ``name`` is not a registered slider.)""";
        } GetSliderValue;
        // Symbol: drake::geometry::Meshcat::GetTrackedCameraPose
        struct /* GetTrackedCameraPose */ {
          // Source: drake/geometry/meshcat.h
          const char* doc =
R"""(Returns the most recently received camera pose.

A meshcat browser session can be configured to transmit its camera
pose. It is enabled by appending a url parameter. For example, if the
url for the meshcat server is:

http://localhost:7000

A particular browser can be configured to transmit its camera pose
back to Drake by supplying the following url:

http://localhost:7000/?tracked_camera=on

It is possible to use that URL in multiple browsers simultaneously. A
particular view will only transmit its camera position when its camera
position actually *changes*. As such, the returned camera pose will
reflect the pose of the camera from that most-recently manipulated
browser.

std∷nullopt is returned if:

- No meshcat session has transmitted its camera pose.
- The meshcat session that last transmitted its pose is no longer
connected.
- The meshcat session transmitting has an orthographic camera.)""";
        } GetTrackedCameraPose;
        // Symbol: drake::geometry::Meshcat::HasPath
        struct /* HasPath */ {
          // Source: drake/geometry/meshcat.h
          const char* doc = R"""()""";
        } HasPath;
        // Symbol: drake::geometry::Meshcat::InjectMockTimer
        struct /* InjectMockTimer */ {
          // Source: drake/geometry/meshcat.h
          const char* doc = R"""()""";
        } InjectMockTimer;
        // Symbol: drake::geometry::Meshcat::InjectWebsocketMessage
        struct /* InjectWebsocketMessage */ {
          // Source: drake/geometry/meshcat.h
          const char* doc = R"""()""";
        } InjectWebsocketMessage;
        // Symbol: drake::geometry::Meshcat::InjectWebsocketThreadFault
        struct /* InjectWebsocketThreadFault */ {
          // Source: drake/geometry/meshcat.h
          const char* doc = R"""()""";
        } InjectWebsocketThreadFault;
        // Symbol: drake::geometry::Meshcat::Meshcat
        struct /* ctor */ {
          // Source: drake/geometry/meshcat.h
          const char* doc_1args_port =
R"""(Constructs the Meshcat instance on ``port``. If no port is specified,
it will listen on the first available port starting at 7000 (up to
7999). If port 0 is specified, it will listen on an arbitrary
"ephemeral" port.

Precondition:
    We require ``port`` == 0 || ``port`` >= 1024.

Raises:
    RuntimeError if no requested ``port`` is available.)""";
          // Source: drake/geometry/meshcat.h
          const char* doc_1args_params =
R"""(Constructs the Meshcat instance using the given ``params``.)""";
        } ctor;
        // Symbol: drake::geometry::Meshcat::OrthographicCamera
        struct /* OrthographicCamera */ {
          // Source: drake/geometry/meshcat.h
          const char* doc =
R"""(Properties for an orthographic camera in three.js:
https://threejs.org/docs/#api/en/cameras/OrthographicCamera)""";
          // Symbol: drake::geometry::Meshcat::OrthographicCamera::Serialize
          struct /* Serialize */ {
            // Source: drake/geometry/meshcat.h
            const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
          } Serialize;
          // Symbol: drake::geometry::Meshcat::OrthographicCamera::bottom
          struct /* bottom */ {
            // Source: drake/geometry/meshcat.h
            const char* doc = R"""(Camera frustum bottom plane.)""";
          } bottom;
          // Symbol: drake::geometry::Meshcat::OrthographicCamera::far
          struct /* far */ {
            // Source: drake/geometry/meshcat.h
            const char* doc = R"""(Camera frustum far plane.)""";
          } far;
          // Symbol: drake::geometry::Meshcat::OrthographicCamera::left
          struct /* left */ {
            // Source: drake/geometry/meshcat.h
            const char* doc = R"""(Camera frustum left plane.)""";
          } left;
          // Symbol: drake::geometry::Meshcat::OrthographicCamera::near
          struct /* near */ {
            // Source: drake/geometry/meshcat.h
            const char* doc = R"""(Camera frustum near plane.)""";
          } near;
          // Symbol: drake::geometry::Meshcat::OrthographicCamera::right
          struct /* right */ {
            // Source: drake/geometry/meshcat.h
            const char* doc = R"""(Camera frustum right plane.)""";
          } right;
          // Symbol: drake::geometry::Meshcat::OrthographicCamera::top
          struct /* top */ {
            // Source: drake/geometry/meshcat.h
            const char* doc = R"""(Camera frustum top plane.)""";
          } top;
          // Symbol: drake::geometry::Meshcat::OrthographicCamera::zoom
          struct /* zoom */ {
            // Source: drake/geometry/meshcat.h
            const char* doc = R"""(The zoom factor of the camera.)""";
          } zoom;
          auto Serialize__fields() const {
            return std::array{
              std::make_pair("bottom", bottom.doc),
              std::make_pair("far", far.doc),
              std::make_pair("left", left.doc),
              std::make_pair("near", near.doc),
              std::make_pair("right", right.doc),
              std::make_pair("top", top.doc),
              std::make_pair("zoom", zoom.doc),
            };
          }
        } OrthographicCamera;
        // Symbol: drake::geometry::Meshcat::PerspectiveCamera
        struct /* PerspectiveCamera */ {
          // Source: drake/geometry/meshcat.h
          const char* doc =
R"""(Properties for a perspective camera in three.js:
https://threejs.org/docs/#api/en/cameras/PerspectiveCamera)""";
          // Symbol: drake::geometry::Meshcat::PerspectiveCamera::Serialize
          struct /* Serialize */ {
            // Source: drake/geometry/meshcat.h
            const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
          } Serialize;
          // Symbol: drake::geometry::Meshcat::PerspectiveCamera::aspect
          struct /* aspect */ {
            // Source: drake/geometry/meshcat.h
            const char* doc = R"""(Camera frustum aspect ratio.)""";
          } aspect;
          // Symbol: drake::geometry::Meshcat::PerspectiveCamera::far
          struct /* far */ {
            // Source: drake/geometry/meshcat.h
            const char* doc = R"""(Camera frustum far plane.)""";
          } far;
          // Symbol: drake::geometry::Meshcat::PerspectiveCamera::fov
          struct /* fov */ {
            // Source: drake/geometry/meshcat.h
            const char* doc = R"""(Camera frustum vertical field of view.)""";
          } fov;
          // Symbol: drake::geometry::Meshcat::PerspectiveCamera::near
          struct /* near */ {
            // Source: drake/geometry/meshcat.h
            const char* doc = R"""(Camera frustum near plane.)""";
          } near;
          // Symbol: drake::geometry::Meshcat::PerspectiveCamera::zoom
          struct /* zoom */ {
            // Source: drake/geometry/meshcat.h
            const char* doc = R"""(The zoom factor of the camera.)""";
          } zoom;
          auto Serialize__fields() const {
            return std::array{
              std::make_pair("aspect", aspect.doc),
              std::make_pair("far", far.doc),
              std::make_pair("fov", fov.doc),
              std::make_pair("near", near.doc),
              std::make_pair("zoom", zoom.doc),
            };
          }
        } PerspectiveCamera;
        // Symbol: drake::geometry::Meshcat::PlotSurface
        struct /* PlotSurface */ {
          // Source: drake/geometry/meshcat.h
          const char* doc =
R"""(Sets the "object" at ``path`` to be a triangle surface mesh
representing a 3D surface, via an API that roughly follows
matplotlib's plot_surface() method.

Parameter ``X``:
    matrix of ``x`` coordinate values defining the vertices of the
    mesh.

Parameter ``Y``:
    matrix of ``y`` coordinate values.

Parameter ``Z``:
    matrix of ``z`` coordinate values.

Parameter ``rgba``:
    is the mesh face or wireframe color.

Parameter ``wireframe``:
    if "true", then only the triangle edges are visualized, not the
    faces.

Parameter ``wireframe_line_width``:
    is the width in pixels. Due to limitations in WebGL
    implementations, the line width may be 1 regardless of the set
    value.

Typically, X and Y are obtained via, e.g.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    constexpr int nx = 15, ny = 11;
    X = RowVector<double, nx>∷LinSpaced(0, 1).replicate<ny, 1>();
    Y = Vector<double, ny>∷LinSpaced(0, 1).replicate<1, nx>();

.. raw:: html

    </details>

in C++ or e.g.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    xs = np.linspace(0, 1, 15)
    ys = np.linspace(0, 1, 11)
    [X, Y] = np.meshgrid(xs, ys)

.. raw:: html

    </details>

in Python, and Z is the surface evaluated on each X, Y value.

Precondition:
    X, Y, and Z must be the same shape.)""";
        } PlotSurface;
        // Symbol: drake::geometry::Meshcat::PublishRecording
        struct /* PublishRecording */ {
          // Source: drake/geometry/meshcat.h
          const char* doc =
R"""(Sends the recording to Meshcat as an animation. The published
animation only includes transforms and properties; the objects that
they modify must be sent to the visualizer separately (e.g. by calling
Publish()).)""";
        } PublishRecording;
        // Symbol: drake::geometry::Meshcat::ResetRenderMode
        struct /* ResetRenderMode */ {
          // Source: drake/geometry/meshcat.h
          const char* doc =
R"""(Resets the default camera, camera target, background, grid lines, and
axes to their default settings.)""";
        } ResetRenderMode;
        // Symbol: drake::geometry::Meshcat::Set2dRenderMode
        struct /* Set2dRenderMode */ {
          // Source: drake/geometry/meshcat.h
          const char* doc =
R"""(Applies a number of settings to make Meshcat act as a 2D renderer. The
camera is set to an orthographic camera with ``X_WC`` specifying the
pose of the camera in world. The camera looks down the +Cy axis, with
+Cz corresponding to positive y in the 2D frame, and -Cx corresponding
to positive x in the 2D frame.

Additionally sets the background, grid lines, and axes "visible"
properties to false.)""";
        } Set2dRenderMode;
        // Symbol: drake::geometry::Meshcat::SetAnimation
        struct /* SetAnimation */ {
          // Source: drake/geometry/meshcat.h
          const char* doc =
R"""(Sets the MeshcatAnimation, which creates a slider interface element to
play/pause/rewind through a series of animation frames in the
visualizer.

See also StartRecording(), which records supported calls to ``this``
into a MeshcatAnimation, and PublishRecording(), which calls
SetAnimation() with the recording.)""";
        } SetAnimation;
        // Symbol: drake::geometry::Meshcat::SetCamera
        struct /* SetCamera */ {
          // Source: drake/geometry/meshcat.h
          const char* doc_perspective =
R"""(Sets the Meshcat object on ``path`` to a perspective camera. We
provide a default value of ``path`` corresponding to the default
camera object in Meshcat.)""";
          // Source: drake/geometry/meshcat.h
          const char* doc_orthographic =
R"""(Sets the Meshcat object on ``path`` to an orthographic camera. We
provide a default value of ``path`` corresponding to the default
camera object in Meshcat.)""";
        } SetCamera;
        // Symbol: drake::geometry::Meshcat::SetCameraPose
        struct /* SetCameraPose */ {
          // Source: drake/geometry/meshcat.h
          const char* doc =
R"""(A convenience function for positioning the camera and its view target
in the world frame. The camera is placed at ``camera_in_world`` and
looks toward ``target_in_world``. The camera is oriented around this
view direction so that the camera's up vector points in the positive
Wz direction as much as possible.

Unlike SetCameraTarget() this can be used to orient orthographic
cameras as well.

Note:
    This is Drake's z-up world frame and not the three.js world frame
    you'd have to use if you set the "position" on the camera
    directly.

Warning:
    The behavior is undefined if camera and target positions are
    coincident.

Parameter ``camera_in_world``:
    the position of the camera's origin C in Drake's z-up world frame
    (p_WC).

Parameter ``target_in_world``:
    the position of the target point T in Drake's z-up world frame
    (p_WT).)""";
        } SetCameraPose;
        // Symbol: drake::geometry::Meshcat::SetCameraTarget
        struct /* SetCameraTarget */ {
          // Source: drake/geometry/meshcat.h
          const char* doc =
R"""(Positions the camera's view target point T to the location in
``target_in_world`` (`p_WT`).

If the camera is orthographic (i.e., by calling Set2DRenderMode() or
SetCamera(OrthographicCamera)), this will have no effect.

Warning:
    Setting the target position to be coincident with the camera
    position will lead to undefined behavior.

Parameter ``target_in_world``:
    the position of the target point T in Drake's z-up world frame
    (p_WT).)""";
        } SetCameraTarget;
        // Symbol: drake::geometry::Meshcat::SetEnvironmentMap
        struct /* SetEnvironmentMap */ {
          // Source: drake/geometry/meshcat.h
          const char* doc =
R"""(Sets the *environment* texture. For objects with physically-based
rendering (PBR) material properties (e.g., metallic surfaces), this
defines the luminance environment, contributing to total illumination
and appearing in reflections.

The image should be of a format typically supported by web browsers:
e.g., jpg, png, etc. Furthermore, the image must be an `
equirectangular image
<https://en.wikipedia.org/wiki/Equirectangular_projection>`_ (as
opposed to a `cube-map
<https://en.wikipedia.org/wiki/Cube_mapping>`_).

If the path is empty, the environment map will be cleared.

Raises:
    if ``image_path`` is *not* empty and the file isn't accessible.

Precondition:
    If ``image_path`` names an accessible file, it is an appropriate
    image type.)""";
        } SetEnvironmentMap;
        // Symbol: drake::geometry::Meshcat::SetLine
        struct /* SetLine */ {
          // Source: drake/geometry/meshcat.h
          const char* doc =
R"""(Sets the "object" at ``path`` in the scene tree to a piecewise-linear
interpolation between the ``vertices``.

Parameter ``path``:
    a "/"-delimited string indicating the path in the scene tree. See
    meshcat_path "Meshcat paths" for the semantics.

Parameter ``vertices``:
    are the 3D points defining the lines.

Parameter ``line_width``:
    is the width in pixels. Due to limitations in WebGL
    implementations, the line width may be 1 regardless of the set
    value.

Parameter ``rgba``:
    is the line color.)""";
        } SetLine;
        // Symbol: drake::geometry::Meshcat::SetLineSegments
        struct /* SetLineSegments */ {
          // Source: drake/geometry/meshcat.h
          const char* doc =
R"""(Sets the "object" at ``path`` in the scene tree to a number of line
segments.

Parameter ``path``:
    a "/"-delimited string indicating the path in the scene tree. See
    meshcat_path "Meshcat paths" for the semantics.

Parameter ``start``:
    is a 3-by-N matrix of 3D points defining the start of each
    segment.

Parameter ``end``:
    is a 3-by-N matrix of 3D points defining the end of each segment.

Parameter ``line_width``:
    is the width in pixels. Due to limitations in WebGL
    implementations, the line width may be 1 regardless of the set
    value.

Parameter ``rgba``:
    is the line color.

Raises:
    RuntimeError if start.cols != end.cols().)""";
        } SetLineSegments;
        // Symbol: drake::geometry::Meshcat::SetObject
        struct /* SetObject */ {
          // Source: drake/geometry/meshcat.h
          const char* doc_shape =
R"""(Sets the 3D object at a given ``path`` in the scene tree. Note that
``path`="/foo" will always set an object in the tree at
"/foo/<object>". See meshcat_path. Any objects previously set at this
`path`` will be replaced.

Parameter ``path``:
    a "/"-delimited string indicating the path in the scene tree. See
    meshcat_path "Meshcat paths" for the semantics.

Parameter ``shape``:
    a Shape that specifies the geometry of the object.

Parameter ``rgba``:
    an Rgba that specifies the (solid) color of the object.

Note:
    If ``shape`` is a mesh, the file referred to can be either an .obj
    file or an *embedded* .gltf file (it has all geometry data and
    texture data contained within the single .gltf file).)""";
          // Source: drake/geometry/meshcat.h
          const char* doc_cloud =
R"""(Sets the "object" at a given ``path`` in the scene tree to be
``point_cloud``. Note that ``path`="/foo" will always set an object in
the tree at "/foo/<object>". See meshcat_path. Any objects previously
set at this `path`` will be replaced.

Parameter ``path``:
    a "/"-delimited string indicating the path in the scene tree. See
    meshcat_path "Meshcat paths" for the semantics.

Parameter ``point_cloud``:
    a perception∷PointCloud; if ``point_cloud.has_rgbs()`` is true,
    then meshcat will render the colored points.

Parameter ``point_size``:
    is the size of each rendered point.

Parameter ``rgba``:
    is the default color, which is only used if
    ``point_cloud.has_rgbs() == false``.)""";
          // Source: drake/geometry/meshcat.h
          const char* doc_triangle_surface_mesh =
R"""(Sets the "object" at ``path`` in the scene tree to a
TriangleSurfaceMesh.

Parameter ``path``:
    a "/"-delimited string indicating the path in the scene tree. See
    meshcat_path "Meshcat paths" for the semantics.

Parameter ``mesh``:
    is a TriangleSurfaceMesh object.

Parameter ``rgba``:
    is the mesh face or wireframe color.

Parameter ``wireframe``:
    if "true", then only the triangle edges are visualized, not the
    faces.

Parameter ``wireframe_line_width``:
    is the width in pixels. Due to limitations in WebGL
    implementations, the line width may be 1 regardless of the set
    value.)""";
        } SetObject;
        // Symbol: drake::geometry::Meshcat::SetProperty
        struct /* SetProperty */ {
          // Source: drake/geometry/meshcat.h
          const char* doc_bool =
R"""(Sets a single named property of the object at the given path. For
example,


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    meshcat.SetProperty("/Background", "visible", false);

.. raw:: html

    </details>

will turn off the background. See meshcat_path "Meshcat paths" for
more details about these properties and how to address them.

Parameter ``path``:
    a "/"-delimited string indicating the path in the scene tree. See
    meshcat_path for the semantics.

Parameter ``property``:
    the string name of the property to set

Parameter ``value``:
    the new value.

Parameter ``time_in_recording``:
    (optional). If recording (see StartRecording()), then in addition
    to publishing the property to any meshcat browsers immediately,
    this transform is saved to the current animation at
    ``time_in_recording``.)""";
          // Source: drake/geometry/meshcat.h
          const char* doc_double =
R"""(Sets a single named property of the object at the given path. For
example,


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    meshcat.SetProperty("/Lights/DirectionalLight/<object>", "intensity", 1.0);

.. raw:: html

    </details>

See meshcat_path "Meshcat paths" for more details about these
properties and how to address them.

Parameter ``path``:
    a "/"-delimited string indicating the path in the scene tree. See
    meshcat_path for the semantics.

Parameter ``property``:
    the string name of the property to set

Parameter ``value``:
    the new value.

Parameter ``time_in_recording``:
    (optional) the time at which this property should be applied, if
    Meshcat is current recording (see StartRecording()). If Meshcat is
    not currently recording, then this value is simply ignored.)""";
          // Source: drake/geometry/meshcat.h
          const char* doc_vector_double =
R"""(Sets a single named property of the object at the given path. For
example,


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    meshcat.SetProperty("/Background", "top_color", {1.0, 0.0, 0.0});

.. raw:: html

    </details>

See meshcat_path "Meshcat paths" for more details about these
properties and how to address them.

Parameter ``path``:
    a "/"-delimited string indicating the path in the scene tree. See
    meshcat_path for the semantics.

Parameter ``property``:
    the string name of the property to set

Parameter ``value``:
    the new value.

Parameter ``time_in_recording``:
    (optional) the time at which this property should be applied, if
    Meshcat is current recording (see StartRecording()). If Meshcat is
    not currently recording, then this value is simply ignored.)""";
        } SetProperty;
        // Symbol: drake::geometry::Meshcat::SetRealtimeRate
        struct /* SetRealtimeRate */ {
          // Source: drake/geometry/meshcat.h
          const char* doc =
R"""(Immediately broadcasts the given realtime rate to all connected
clients.

Parameter ``rate``:
    the realtime rate value to broadcast, will be converted to a
    percentage (multiplied by 100))""";
        } SetRealtimeRate;
        // Symbol: drake::geometry::Meshcat::SetSimulationTime
        struct /* SetSimulationTime */ {
          // Source: drake/geometry/meshcat.h
          const char* doc =
R"""(Updates Meshcat's knowledge of simulation time. Changes to simulation
time *may* trigger a realtime rate message to the meshcat visualizer
client based on the configured value in
MeshcatParams∷realtime_rate_period value.

Invoking this method *may* dispatch a message to clients. The
following rules apply to invocations and messages:

- The first invocation is necessary to *initialize* the calculation; it
defines the the starting point from which all calculations are performed
(for both wall clock time as well as simulation time). As no interval can
be measured from a single invocation, no rate can be computed. Therefore,
the first invocation will *never* broadcast the message.
- Wall clock time must advance at least MeshcatParams∷realtime_rate_period
seconds for a message to be sent.
- Meshcat promises to broadcast one message per elapsed period -- starting
from initialization -- regardless of the frequency at which
SetSimulationTime() actually gets invoked. If the elapsed time between
invocations exceeds MeshcatParams∷realtime_rate_period, multiple
messages will be broadcast; one for each complete period.
- This implies that each column of pixels in the realtime rate chart in
the client visualizer represents a fixed amount of wall clock time.

When the realtime rate is broadcast, that value will be reported by
GetRealtimeRate().

The realtime rate calculation can be "reset" by passing a simulation
time value that is *strictly less* than the value of the previous
invocation. This has the effect of re-initializing the calculation
with the passed time and the wall clock time of the invocation. So, if
the simulation's context gets reset to ``time = 0``, calls to this
method passing the context time will implicitly reset realtime rate
calculations accordingly. Resetting the calculator will reset the
value reported by GetRealtimeRate() to zero.

This function is safe to be called redundantly with the same
simulation time. Redundant calls are a no-op.

Parameter ``sim_time``:
    The *absolute* sim time being visualized.

See also:
    drake∷systems∷Simulator∷set_target_realtime_rate())""";
        } SetSimulationTime;
        // Symbol: drake::geometry::Meshcat::SetSliderValue
        struct /* SetSliderValue */ {
          // Source: drake/geometry/meshcat.h
          const char* doc =
R"""(Sets the current ``value`` of the slider ``name``. `value` will be
truncated to the slider range and rounded to the nearest increment
specified by the slider ``step``. This will update the slider element
in all connected meshcat browsers.

Returns:
    the truncated and rounded value that was actually set.

Raises:
    RuntimeError if ``name`` is not a registered slider.)""";
        } SetSliderValue;
        // Symbol: drake::geometry::Meshcat::SetTransform
        struct /* SetTransform */ {
          // Source: drake/geometry/meshcat.h
          const char* doc_RigidTransform =
R"""(Set the RigidTransform for a given path in the scene tree relative to
its parent path. An object's pose is the concatenation of all of the
transforms along its path, so setting the transform of "/foo" will
move the objects at "/foo/box1" and "/foo/robots/HAL9000".

Parameter ``path``:
    a "/"-delimited string indicating the path in the scene tree. See
    meshcat_path "Meshcat paths" for the semantics.

Parameter ``X_ParentPath``:
    the relative transform from the path to its immediate parent.

Parameter ``time_in_recording``:
    (optional). If recording (see StartRecording()), then in addition
    to publishing the transform to any meshcat browsers immediately,
    this transform is saved to the current animation at
    ``time_in_recording``.)""";
          // Source: drake/geometry/meshcat.h
          const char* doc_matrix =
R"""(Set the homogeneous transform for a given path in the scene tree
relative to its parent path. An object's pose is the concatenation of
all of the transforms along its path, so setting the transform of
"/foo" will move the objects at "/foo/box1" and "/foo/robots/HAL9000".

Parameter ``path``:
    a "/"-delimited string indicating the path in the scene tree. See
    meshcat_path "Meshcat paths" for the semantics.

Parameter ``matrix``:
    the relative transform from the path to its immediate parent.

Note: Prefer to use the overload which takes a RigidTransformd unless
you need the fully parameterized homogeneous transform (which
additionally allows scale and sheer).

Note: Meshcat does not properly support non-uniform scaling. See Drake
issue #18095.)""";
        } SetTransform;
        // Symbol: drake::geometry::Meshcat::SetTriangleColorMesh
        struct /* SetTriangleColorMesh */ {
          // Source: drake/geometry/meshcat.h
          const char* doc =
R"""(Sets the "object" at ``path`` in the scene tree to a triangular mesh
with per-vertex coloring.

Parameter ``path``:
    a "/"-delimited string indicating the path in the scene tree. See
    meshcat_path "Meshcat paths" for the semantics.

Parameter ``vertices``:
    is a 3-by-N matrix of 3D point defining the vertices of the mesh.

Parameter ``faces``:
    is a 3-by-M integer matrix with each entry denoting an index into
    vertices and each column denoting one face (aka SurfaceTriangle).

Parameter ``colors``:
    is a 3-by-N matrix of RGB color values, one color per vertex of
    the mesh. Color values are in the range [0, 1].

Parameter ``wireframe``:
    if "true", then only the triangle edges are visualized, not the
    faces.

Parameter ``wireframe_line_width``:
    is the width in pixels. Due to limitations in WebGL
    implementations, the line width may be 1 regardless of the set
    value.)""";
        } SetTriangleColorMesh;
        // Symbol: drake::geometry::Meshcat::SetTriangleMesh
        struct /* SetTriangleMesh */ {
          // Source: drake/geometry/meshcat.h
          const char* doc =
R"""(Sets the "object" at ``path`` in the scene tree to a triangular mesh.

Parameter ``path``:
    a "/"-delimited string indicating the path in the scene tree. See
    meshcat_path "Meshcat paths" for the semantics.

Parameter ``vertices``:
    is a 3-by-N matrix of 3D point defining the vertices of the mesh.

Parameter ``faces``:
    is a 3-by-M integer matrix with each entry denoting an index into
    vertices and each column denoting one face (aka SurfaceTriangle).

Parameter ``rgba``:
    is the mesh face or wireframe color.

Parameter ``wireframe``:
    if "true", then only the triangle edges are visualized, not the
    faces.

Parameter ``wireframe_line_width``:
    is the width in pixels. Due to limitations in WebGL
    implementations, the line width may be 1 regardless of the set
    value.)""";
        } SetTriangleMesh;
        // Symbol: drake::geometry::Meshcat::SideOfFaceToRender
        struct /* SideOfFaceToRender */ {
          // Source: drake/geometry/meshcat.h
          const char* doc = R"""()""";
          // Symbol: drake::geometry::Meshcat::SideOfFaceToRender::kBackSide
          struct /* kBackSide */ {
            // Source: drake/geometry/meshcat.h
            const char* doc = R"""()""";
          } kBackSide;
          // Symbol: drake::geometry::Meshcat::SideOfFaceToRender::kDoubleSide
          struct /* kDoubleSide */ {
            // Source: drake/geometry/meshcat.h
            const char* doc = R"""()""";
          } kDoubleSide;
          // Symbol: drake::geometry::Meshcat::SideOfFaceToRender::kFrontSide
          struct /* kFrontSide */ {
            // Source: drake/geometry/meshcat.h
            const char* doc = R"""()""";
          } kFrontSide;
        } SideOfFaceToRender;
        // Symbol: drake::geometry::Meshcat::StartRecording
        struct /* StartRecording */ {
          // Source: drake/geometry/meshcat.h
          const char* doc =
R"""(Sets a flag indicating that subsequent calls to SetTransform and
SetProperty should also be "recorded" into a MeshcatAnimation when
their optional time_in_recording argument is supplied. The data in
these events will be combined with any frames previously added to the
animation; if the same transform/property is set at the same time,
then it will overwrite the existing frame in the animation.

Parameter ``set_visualizations_while_recording``:
    if true, then each method will send the visualization immediately
    to Meshcat *and* record the visualization in the animation. Set to
    false to avoid updating the visualization during recording. One
    exception is calls to SetObject, which will always be sent to the
    visualizer immediately (because meshcat animations do not support
    SetObject).)""";
        } StartRecording;
        // Symbol: drake::geometry::Meshcat::StaticHtml
        struct /* StaticHtml */ {
          // Source: drake/geometry/meshcat.h
          const char* doc =
R"""(Returns an HTML string that can be saved to a file for a snapshot of
the visualizer and its contents. The HTML can be viewed in the browser
without any connection to a Meshcat "server" (e.g. ``this``). This is
a great way to save and share your 3D content.

Note that controls (e.g. sliders and buttons) are not included in the
HTML output, because their usefulness relies on a connection to the
server.

You can also use your browser to download this file, by typing
"/download" on the end of the URL (i.e., accessing ``web_url() +
"/download"``).)""";
        } StaticHtml;
        // Symbol: drake::geometry::Meshcat::StaticZip
        struct /* StaticZip */ {
          // Source: drake/geometry/meshcat.h
          const char* doc =
R"""(Like StaticHtml(), returns a standalone snapshot of the visualizer and
its contents; the return value is a ZIP file containing a thin
``meshcat.html`` page and the assets (meshes, textures, etc.) as
separate files.

When you are uploading the unzipped files to a website, this will
typically be a more efficient representation as compared to
StaticHtml(). However, it cannot be opened directly by a browser from
disk. A simple web server like ``python -m http.server`` is required.

You can also use your browser to download this file, by typing
"/download.zip" on the end of the URL (i.e., accessing ``web_url() +
"/download.zip"``).)""";
        } StaticZip;
        // Symbol: drake::geometry::Meshcat::StopRecording
        struct /* StopRecording */ {
          // Source: drake/geometry/meshcat.h
          const char* doc =
R"""(Sets a flag to pause/stop recording. When stopped, publish events will
not add frames to the animation.)""";
        } StopRecording;
        // Symbol: drake::geometry::Meshcat::get_mutable_recording
        struct /* get_mutable_recording */ {
          // Source: drake/geometry/meshcat.h
          const char* doc =
R"""(Returns a mutable reference to this Meshcat's MeshcatAnimation object.
This can be used to set animation properties (like autoplay, the loop
mode, number of repetitions, etc). The return value will only remain
valid for the lifetime of ``this`` or until DeleteRecording() is
called.)""";
        } get_mutable_recording;
        // Symbol: drake::geometry::Meshcat::get_recording
        struct /* get_recording */ {
          // Source: drake/geometry/meshcat.h
          const char* doc =
R"""(Returns a const reference to this Meshcat's MeshcatAnimation object.
This can be used to check animation properties (e.g., autoplay). The
return value will only remain valid for the lifetime of ``this`` or
until DeleteRecording() is called.)""";
        } get_recording;
        // Symbol: drake::geometry::Meshcat::port
        struct /* port */ {
          // Source: drake/geometry/meshcat.h
          const char* doc =
R"""(Returns the port on localhost listening for http connections.)""";
        } port;
        // Symbol: drake::geometry::Meshcat::web_url
        struct /* web_url */ {
          // Source: drake/geometry/meshcat.h
          const char* doc = R"""(Returns the hosted http URL.)""";
        } web_url;
        // Symbol: drake::geometry::Meshcat::ws_url
        struct /* ws_url */ {
          // Source: drake/geometry/meshcat.h
          const char* doc =
R"""((Advanced) Returns the ws:// URL for direct connection to the
websocket interface. Most users should connect via a browser opened to
web_url().)""";
        } ws_url;
      } Meshcat;
      // Symbol: drake::geometry::MeshcatAnimation
      struct /* MeshcatAnimation */ {
        // Source: drake/geometry/meshcat_animation.h
        const char* doc =
R"""(An interface for recording/playback animations in Meshcat. Use
Meshcat∷SetAnimation to publish a MeshcatAnimation to the visualizer.

Currently, an animation consists of (only) transforms and properties
that are set at a particular integer frame number. Although we do not
support calls to SetObject/Delete in an animation, you can consider
using ``SetProperty(frame, path, "visible", true/false)`` in your
animation to make the object appear or disappear at a particular
frame.)""";
        // Symbol: drake::geometry::MeshcatAnimation::LoopMode
        struct /* LoopMode */ {
          // Source: drake/geometry/meshcat_animation.h
          const char* doc = R"""()""";
          // Symbol: drake::geometry::MeshcatAnimation::LoopMode::kLoopOnce
          struct /* kLoopOnce */ {
            // Source: drake/geometry/meshcat_animation.h
            const char* doc = R"""(Plays the clip once.)""";
          } kLoopOnce;
          // Symbol: drake::geometry::MeshcatAnimation::LoopMode::kLoopPingPong
          struct /* kLoopPingPong */ {
            // Source: drake/geometry/meshcat_animation.h
            const char* doc =
R"""(Plays the clip with the chosen number of repetitions, alternately
playing forward and backward.)""";
          } kLoopPingPong;
          // Symbol: drake::geometry::MeshcatAnimation::LoopMode::kLoopRepeat
          struct /* kLoopRepeat */ {
            // Source: drake/geometry/meshcat_animation.h
            const char* doc =
R"""(Plays the clip with the chosen number of repetitions, each time
jumping from the end of the clip directly to its beginning.)""";
          } kLoopRepeat;
        } LoopMode;
        // Symbol: drake::geometry::MeshcatAnimation::MeshcatAnimation
        struct /* ctor */ {
          // Source: drake/geometry/meshcat_animation.h
          const char* doc =
R"""(Constructs the animation object.

Parameter ``frames_per_second``:
    a positive integer specifying the timing at which the frames are
    played back.)""";
        } ctor;
        // Symbol: drake::geometry::MeshcatAnimation::SetProperty
        struct /* SetProperty */ {
          // Source: drake/geometry/meshcat_animation.h
          const char* doc_bool =
R"""(Sets a single named property of the object at the given ``path`` at
the specified ``frame`` in the animation.

See also:
    Meshcat∷SetProperty.

Parameter ``frame``:
    a non-negative integer indicating the frame at which this
    transform is applied.

Parameter ``path``:
    a "/"-delimited string indicating the path in the scene tree. See
    meshcat_path for the semantics.

Parameter ``property``:
    the string name of the property to set

Parameter ``value``:
    the new value.

Raises:
    RuntimeError if this path/property has already been set with a
    different type.)""";
          // Source: drake/geometry/meshcat_animation.h
          const char* doc_double =
R"""(Sets a single named property of the object at the given ``path`` at
the specified ``frame`` in the animation.

See also:
    Meshcat∷SetProperty.

Parameter ``frame``:
    a non-negative integer indicating the frame at which this
    transform is applied.

Parameter ``path``:
    a "/"-delimited string indicating the path in the scene tree. See
    meshcat_path for the semantics.

Parameter ``property``:
    the string name of the property to set

Parameter ``value``:
    the new value.

Raises:
    RuntimeError if this path/property has already been set with a
    different type.)""";
          // Source: drake/geometry/meshcat_animation.h
          const char* doc_vector_double =
R"""(Sets a single named property of the object at the given ``path`` at
the specified ``frame`` in the animation.

See also:
    Meshcat∷SetProperty.

Parameter ``frame``:
    a non-negative integer indicating the frame at which this
    transform is applied.

Parameter ``path``:
    a "/"-delimited string indicating the path in the scene tree. See
    meshcat_path for the semantics.

Parameter ``property``:
    the string name of the property to set

Parameter ``value``:
    the new value.

Raises:
    RuntimeError if this path/property has already been set with a
    different type.)""";
        } SetProperty;
        // Symbol: drake::geometry::MeshcatAnimation::SetTransform
        struct /* SetTransform */ {
          // Source: drake/geometry/meshcat_animation.h
          const char* doc =
R"""(Set the RigidTransform at ``frame`` in the animation for a given
``path`` in the the scene tree.

See also:
    Meshcat∷SetTransform.

Parameter ``frame``:
    a non-negative integer indicating the frame at which this
    transform is applied.

Parameter ``path``:
    a "/"-delimited string indicating the path in the scene tree. See
    meshcat_path "Meshcat paths" for the semantics.

Parameter ``X_ParentPath``:
    the relative transform from the path to its immediate parent.

Raises:
    RuntimeError if the position or quaternion properties of this path
    have already been set to an incorrect type.)""";
        } SetTransform;
        // Symbol: drake::geometry::MeshcatAnimation::autoplay
        struct /* autoplay */ {
          // Source: drake/geometry/meshcat_animation.h
          const char* doc = R"""()""";
        } autoplay;
        // Symbol: drake::geometry::MeshcatAnimation::clamp_when_finished
        struct /* clamp_when_finished */ {
          // Source: drake/geometry/meshcat_animation.h
          const char* doc = R"""()""";
        } clamp_when_finished;
        // Symbol: drake::geometry::MeshcatAnimation::frame
        struct /* frame */ {
          // Source: drake/geometry/meshcat_animation.h
          const char* doc =
R"""(Uses the frame rate to convert from time to the frame number, using
std∷floor.

Precondition:
    ``time`` ≥ start_time().)""";
        } frame;
        // Symbol: drake::geometry::MeshcatAnimation::frames_per_second
        struct /* frames_per_second */ {
          // Source: drake/geometry/meshcat_animation.h
          const char* doc =
R"""(Returns the frame rate at which the animation will be played back.)""";
        } frames_per_second;
        // Symbol: drake::geometry::MeshcatAnimation::get_javascript_type
        struct /* get_javascript_type */ {
          // Source: drake/geometry/meshcat_animation.h
          const char* doc =
R"""(Returns the javascript type for a particular path/property, or the
empty string if nothing has been set. This method is intended
primarily for testing.)""";
        } get_javascript_type;
        // Symbol: drake::geometry::MeshcatAnimation::get_key_frame
        struct /* get_key_frame */ {
          // Source: drake/geometry/meshcat_animation.h
          const char* doc =
R"""(Returns the value information for a particular path/property at a
particular frame if a value of type T has been set, otherwise returns
std∷nullopt. This method is intended primarily for testing.

Template parameter ``T``:
    One of ``bool``, `double`, or ``vector<double>``)""";
        } get_key_frame;
        // Symbol: drake::geometry::MeshcatAnimation::loop_mode
        struct /* loop_mode */ {
          // Source: drake/geometry/meshcat_animation.h
          const char* doc = R"""()""";
        } loop_mode;
        // Symbol: drake::geometry::MeshcatAnimation::repetitions
        struct /* repetitions */ {
          // Source: drake/geometry/meshcat_animation.h
          const char* doc = R"""()""";
        } repetitions;
        // Symbol: drake::geometry::MeshcatAnimation::set_autoplay
        struct /* set_autoplay */ {
          // Source: drake/geometry/meshcat_animation.h
          const char* doc =
R"""(Set the behavior when the animation is first sent to the visualizer.
The animation will play immediately iff ``play`` is true. The default
is true.)""";
        } set_autoplay;
        // Symbol: drake::geometry::MeshcatAnimation::set_clamp_when_finished
        struct /* set_clamp_when_finished */ {
          // Source: drake/geometry/meshcat_animation.h
          const char* doc =
R"""(Sets the behavior at the end of the animation. If true, then the
animation will automatically be paused on its last frame. If false,
the scene will be reset to before the animation. The default is true.

Note: This setting has no impact if the action is interrupted (it has
only an effect if its last loop has really finished).)""";
        } set_clamp_when_finished;
        // Symbol: drake::geometry::MeshcatAnimation::set_loop_mode
        struct /* set_loop_mode */ {
          // Source: drake/geometry/meshcat_animation.h
          const char* doc =
R"""(Sets the loop behavior on play.

See also:
    LoopMode for details. The default is kLoopRepeat.)""";
        } set_loop_mode;
        // Symbol: drake::geometry::MeshcatAnimation::set_repetitions
        struct /* set_repetitions */ {
          // Source: drake/geometry/meshcat_animation.h
          const char* doc =
R"""(Sets the number of repetitions of the animation each time it is
played. This number has no effect when the loop mode is set to
kLoopOnce. ``repetitions`` must be a positive integer. The default
value is 1.)""";
        } set_repetitions;
        // Symbol: drake::geometry::MeshcatAnimation::set_start_time
        struct /* set_start_time */ {
          // Source: drake/geometry/meshcat_animation.h
          const char* doc =
R"""(Set the start time of the animation. This is only for convenience; it
is used in the frame() method to allow callers to look up the frame
number based on the current time, the start time, and the frame rate.
It is not passed to Meshcat. It does not change any frames that have
previously been set. The default is zero.)""";
        } set_start_time;
        // Symbol: drake::geometry::MeshcatAnimation::start_time
        struct /* start_time */ {
          // Source: drake/geometry/meshcat_animation.h
          const char* doc = R"""()""";
        } start_time;
      } MeshcatAnimation;
      // Symbol: drake::geometry::MeshcatCone
      struct /* MeshcatCone */ {
        // Source: drake/geometry/shape_specification.h
        const char* doc =
R"""(Definition of a cone. Its point is at the origin, its height extends
in the direction of the frame's +z axis. Or, more formally: a finite
section of a Lorentz cone (aka "second-order cone"), defined by

sqrt(x²/a² + y²/b²) ≤ z/height; z ∈ [0, height],

where ``a`` and ``b`` are the lengths of the principal semi-axes of
the horizontal section at ``z=height()``.

This shape is currently only supported by Meshcat. It will not appear
in any renderings, proximity queries, or other visualizers.)""";
        // Symbol: drake::geometry::MeshcatCone::MeshcatCone
        struct /* ctor */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc_3args =
R"""(Constructs the parameterized cone.

Raises:
    RuntimeError if any measure is not finite positive.)""";
          // Source: drake/geometry/shape_specification.h
          const char* doc_1args =
R"""(Constructs a cone with a vector of measures: height and principal
semi-axes.

Raises:
    RuntimeError if any measure is not finite positive.)""";
        } ctor;
        // Symbol: drake::geometry::MeshcatCone::a
        struct /* a */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc = R"""()""";
        } a;
        // Symbol: drake::geometry::MeshcatCone::b
        struct /* b */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc = R"""()""";
        } b;
        // Symbol: drake::geometry::MeshcatCone::height
        struct /* height */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc = R"""()""";
        } height;
      } MeshcatCone;
      // Symbol: drake::geometry::MeshcatParams
      struct /* MeshcatParams */ {
        // Source: drake/geometry/meshcat_params.h
        const char* doc =
R"""(The set of parameters for configuring Meshcat.)""";
        // Symbol: drake::geometry::MeshcatParams::PropertyTuple
        struct /* PropertyTuple */ {
          // Source: drake/geometry/meshcat_params.h
          const char* doc =
R"""(A helper struct for the ``initial_properties`` params. The members
follow the same semantics as calls to Meshcat∷SetProperty(path,
property, value).)""";
          // Symbol: drake::geometry::MeshcatParams::PropertyTuple::Serialize
          struct /* Serialize */ {
            // Source: drake/geometry/meshcat_params.h
            const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
          } Serialize;
          // Symbol: drake::geometry::MeshcatParams::PropertyTuple::path
          struct /* path */ {
            // Source: drake/geometry/meshcat_params.h
            const char* doc = R"""()""";
          } path;
          // Symbol: drake::geometry::MeshcatParams::PropertyTuple::property
          struct /* property */ {
            // Source: drake/geometry/meshcat_params.h
            const char* doc = R"""()""";
          } property;
          // Symbol: drake::geometry::MeshcatParams::PropertyTuple::value
          struct /* value */ {
            // Source: drake/geometry/meshcat_params.h
            const char* doc = R"""()""";
          } value;
          auto Serialize__fields() const {
            return std::array{
              std::make_pair("path", path.doc),
              std::make_pair("property", property.doc),
              std::make_pair("value", value.doc),
            };
          }
        } PropertyTuple;
        // Symbol: drake::geometry::MeshcatParams::Serialize
        struct /* Serialize */ {
          // Source: drake/geometry/meshcat_params.h
          const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
        } Serialize;
        // Symbol: drake::geometry::MeshcatParams::host
        struct /* host */ {
          // Source: drake/geometry/meshcat_params.h
          const char* doc =
R"""(Meshcat will listen only on the given hostname (e.g., "localhost"). If
"*" is specified, then it will listen on all interfaces. If empty, an
appropriate default value will be chosen (currently "*").)""";
        } host;
        // Symbol: drake::geometry::MeshcatParams::initial_properties
        struct /* initial_properties */ {
          // Source: drake/geometry/meshcat_params.h
          const char* doc =
R"""(Configures the initial conditions for Meshcat. These properties will
be applied immediately during construction. This can be used to change
defaults such as background, lighting, etc.

In other words, instead calling the Meshcat() constructor and then
immediately making a bunch of SetProperty(path, property, value) calls
to configure the newly-created object, instead you can append those
(path, property, value) to to this list, and the Meshcat() constructor
will take care of it.)""";
        } initial_properties;
        // Symbol: drake::geometry::MeshcatParams::port
        struct /* port */ {
          // Source: drake/geometry/meshcat_params.h
          const char* doc =
R"""(Meshcat will listen on the given http ``port``. If no port is
specified, then it will listen on the first available port starting at
7000 (up to 7999). If port 0 is specified, it will listen on an
arbitrary "ephemeral" port.

Precondition:
    We require ``port`` == 0 || ``port`` >= 1024.)""";
        } port;
        // Symbol: drake::geometry::MeshcatParams::realtime_rate_period
        struct /* realtime_rate_period */ {
          // Source: drake/geometry/meshcat_params.h
          const char* doc =
R"""(The minimum period of wall clock time (in seconds) between updates to
the broadcast realtime rate. If the period is too short, the reported
realtime rate can become visually noisy. Too long, and acute changes
in performance may be masked. It must be strictly *positive*.

Meshcat promises to broadcast messages to clients at this fixed
period. See Meshcat∷SetSimulationTime() for details.)""";
        } realtime_rate_period;
        // Symbol: drake::geometry::MeshcatParams::show_stats_plot
        struct /* show_stats_plot */ {
          // Source: drake/geometry/meshcat_params.h
          const char* doc =
R"""(Determines whether or not to display the stats plot widget in the
Meshcat user interface. This plot including realtime rate and WebGL
render statistics.)""";
        } show_stats_plot;
        // Symbol: drake::geometry::MeshcatParams::web_url_pattern
        struct /* web_url_pattern */ {
          // Source: drake/geometry/meshcat_params.h
          const char* doc =
R"""(The ``web_url_pattern`` may be used to change the web_url() (and
therefore the ws_url()) reported by Meshcat. This may be useful in
case Meshcat sits behind a firewall or proxy.

The pattern follows the `std∷format
<https://en.cppreference.com/w/cpp/utility/format>`_ specification
language, except that ``arg-id`` substitutions are performed using
named arguments instead of positional indices.

There are two arguments available to the pattern: - ``{port}`` will be
substituted with the Meshcat server's listen port number; - ``{host}``
will be substituted with this params structure's ``host`` field, or
else with "localhost" in case the ``host`` was one of the placeholders
for "all interfaces".)""";
        } web_url_pattern;
        auto Serialize__fields() const {
          return std::array{
            std::make_pair("host", host.doc),
            std::make_pair("initial_properties", initial_properties.doc),
            std::make_pair("port", port.doc),
            std::make_pair("realtime_rate_period", realtime_rate_period.doc),
            std::make_pair("show_stats_plot", show_stats_plot.doc),
            std::make_pair("web_url_pattern", web_url_pattern.doc),
          };
        }
      } MeshcatParams;
      // Symbol: drake::geometry::MeshcatPointCloudVisualizer
      struct /* MeshcatPointCloudVisualizer */ {
        // Source: drake/geometry/meshcat_point_cloud_visualizer.h
        const char* doc =
R"""(MeshcatPointCloudVisualizer is a systems∷LeafSystem that publishes a
perception∷PointCloud from its input port to Meshcat.

.. pydrake_system::

    name: MeshcatPointCloudVisualizer
    input_ports:
    - cloud
    - X_ParentCloud

The PointCloud on the ``cloud`` input port must have XYZ values. RGB
values are optional. The optional input port ``X_ParentCloud`` sets
the MeshcatTransform at the path representing the ``cloud``. If it is
not connected, then we set ``X_ParentCloud`` to the identity
transform.)""";
        // Symbol: drake::geometry::MeshcatPointCloudVisualizer::Delete
        struct /* Delete */ {
          // Source: drake/geometry/meshcat_point_cloud_visualizer.h
          const char* doc =
R"""(Calls Meshcat∷Delete(path), where ``path`` is the value passed in the
constructor.)""";
        } Delete;
        // Symbol: drake::geometry::MeshcatPointCloudVisualizer::MeshcatPointCloudVisualizer<T>
        struct /* ctor */ {
          // Source: drake/geometry/meshcat_point_cloud_visualizer.h
          const char* doc =
R"""(Creates an instance of MeshcatPointCloudVisualizer

Parameter ``meshcat``:
    is a shared Meshcat instance.

Parameter ``path``:
    is the Meshcat path (see meshcat_path)

Parameter ``publish_period``:
    is the duration (in simulation seconds) between updates sent to
    the visualizer. It must be non-negative.)""";
          // Source: drake/geometry/meshcat_point_cloud_visualizer.h
          const char* doc_copyconvert =
R"""(Scalar-converting copy constructor. See system_scalar_conversion. It
should only be used to convert *from* double *to* other scalar types.)""";
        } ctor;
        // Symbol: drake::geometry::MeshcatPointCloudVisualizer::cloud_input_port
        struct /* cloud_input_port */ {
          // Source: drake/geometry/meshcat_point_cloud_visualizer.h
          const char* doc =
R"""(Returns the RigidTransform-valued input port.)""";
        } cloud_input_port;
        // Symbol: drake::geometry::MeshcatPointCloudVisualizer::pose_input_port
        struct /* pose_input_port */ {
          // Source: drake/geometry/meshcat_point_cloud_visualizer.h
          const char* doc =
R"""(Returns the PointCloud-valued input port.)""";
        } pose_input_port;
        // Symbol: drake::geometry::MeshcatPointCloudVisualizer::set_default_rgba
        struct /* set_default_rgba */ {
          // Source: drake/geometry/meshcat_point_cloud_visualizer.h
          const char* doc =
R"""(Sets the default color, which is applied to all points only if
``has_rgbs() == false`` for the cloud on the input port.)""";
        } set_default_rgba;
        // Symbol: drake::geometry::MeshcatPointCloudVisualizer::set_point_size
        struct /* set_point_size */ {
          // Source: drake/geometry/meshcat_point_cloud_visualizer.h
          const char* doc =
R"""(Sets the size of each point in the cloud. The default is 0.001. The
units are undocumented in threejs
(https://threejs.org/docs/index.html?q=PointsMaterial#api/en/materials/PointsMaterial.size),
but we believe they are in meters.)""";
        } set_point_size;
      } MeshcatPointCloudVisualizer;
      // Symbol: drake::geometry::MeshcatPointCloudVisualizerd
      struct /* MeshcatPointCloudVisualizerd */ {
        // Source: drake/geometry/meshcat_point_cloud_visualizer.h
        const char* doc =
R"""(A convenient alias for the MeshcatPointCloudVisualizer class when
using the ``double`` scalar type.)""";
      } MeshcatPointCloudVisualizerd;
      // Symbol: drake::geometry::MeshcatVisualizer
      struct /* MeshcatVisualizer */ {
        // Source: drake/geometry/meshcat_visualizer.h
        const char* doc =
R"""(A system wrapper for Meshcat that publishes the current state of a
SceneGraph instance (whose QueryObject-valued output port is connected
to this system's input port). While this system will add geometry to
Meshcat, the Meshcat instance is also available for users to add their
own visualization alongside the MeshcatVisualizer visualizations. This
can be enormously valuable for impromptu visualizations.

.. pydrake_system::

    name: MeshcatVisualizer
    input_ports:
    - query_object

The system uses the versioning mechanism provided by SceneGraph to
detect changes to the geometry so that a change in SceneGraph's data
will propagate to Meshcat.

By default, MeshcatVisualizer visualizes geometries with the
illustration role (see geometry_roles for more details). It can be
configured to visualize geometries with other roles. Only one role can
be specified. See DrakeVisualizer which uses the same mechanisms for
more details.

Warning:
    MeshcatVisualizer does not support Context-per-thread parallelism.
    This is because of limitations in both Meshcat and
    MeshcatVisualizer. We may generalize this in the future if Meshcat
    limitations are removed.

Instances of MeshcatVisualizer created by scalar-conversion will
publish to the same Meshcat instance.)""";
        // Symbol: drake::geometry::MeshcatVisualizer::AddToBuilder
        struct /* AddToBuilder */ {
          // Source: drake/geometry/meshcat_visualizer.h
          const char* doc_4args_builder_scene_graph_meshcat_params =
R"""(Adds a MeshcatVisualizer and connects it to the given SceneGraph's
QueryObject-valued output port. See
MeshcatVisualizer∷MeshcatVisualizer(MeshcatVisualizer*,
MeshcatVisualizerParams) for details. The MeshcatVisualizer's name
(see systems∷SystemBase∷set_name) will be set to a sensible default
value, unless the default name was already in use by another system.)""";
          // Source: drake/geometry/meshcat_visualizer.h
          const char* doc_4args_builder_query_object_port_meshcat_params =
R"""(Adds a MeshcatVisualizer and connects it to the given
QueryObject-valued output port. See
MeshcatVisualizer∷MeshcatVisualizer(MeshcatVisualizer*,
MeshcatVisualizerParams) for details. The MeshcatVisualizer's name
(see systems∷SystemBase∷set_name) will be set to a sensible default
value, unless the default name was already in use by another system.)""";
        } AddToBuilder;
        // Symbol: drake::geometry::MeshcatVisualizer::Delete
        struct /* Delete */ {
          // Source: drake/geometry/meshcat_visualizer.h
          const char* doc =
R"""(Calls Meshcat∷Delete(std∷string path), with the path set to
MeshcatVisualizerParams∷prefix. Since this visualizer will only ever
add geometry under this prefix, this will remove all
geometry/transforms added by the visualizer, or by a previous instance
of this visualizer using the same prefix. Use
MeshcatVisualizer∷delete_on_initialization_event to determine whether
this should be called on initialization.)""";
        } Delete;
        // Symbol: drake::geometry::MeshcatVisualizer::DeleteRecording
        struct /* DeleteRecording */ {
          // Source: drake/geometry/meshcat_visualizer.h
          const char* doc =
R"""(Convenience function that calls Meshcat∷DeleteRecording on the
underlying Meshcat object; refer to Meshcat∷DeleteRecording for full
documentation.)""";
        } DeleteRecording;
        // Symbol: drake::geometry::MeshcatVisualizer::MeshcatVisualizer<T>
        struct /* ctor */ {
          // Source: drake/geometry/meshcat_visualizer.h
          const char* doc =
R"""(Creates an instance of MeshcatVisualizer.

Parameter ``meshcat``:
    A Meshcat instance. This class will assume shared ownership for
    the lifetime of the object.

Parameter ``params``:
    The set of parameters to control this system's behavior.

Raises:
    RuntimeError if ``params.publish_period <= 0``.

Raises:
    RuntimeError if ``params.role == Role∷kUnassigned``.)""";
          // Source: drake/geometry/meshcat_visualizer.h
          const char* doc_copyconvert =
R"""(Scalar-converting copy constructor. See system_scalar_conversion. It
should only be used to convert *from* double *to* other scalar types.)""";
        } ctor;
        // Symbol: drake::geometry::MeshcatVisualizer::PublishRecording
        struct /* PublishRecording */ {
          // Source: drake/geometry/meshcat_visualizer.h
          const char* doc =
R"""(Convenience function that calls Meshcat∷PublishRecording on the
underlying Meshcat object; refer to Meshcat∷PublishRecording for full
documentation.)""";
        } PublishRecording;
        // Symbol: drake::geometry::MeshcatVisualizer::StartRecording
        struct /* StartRecording */ {
          // Source: drake/geometry/meshcat_visualizer.h
          const char* doc =
R"""(Convenience function that calls Meshcat∷StartRecording on the
underlying Meshcat object, with ``frames_per_second = 1 /
publish_period``; refer to Meshcat∷StartRecording for full
documentation.)""";
        } StartRecording;
        // Symbol: drake::geometry::MeshcatVisualizer::StopRecording
        struct /* StopRecording */ {
          // Source: drake/geometry/meshcat_visualizer.h
          const char* doc =
R"""(Convenience function that calls Meshcat∷StopRecording on the
underlying Meshcat object; refer to Meshcat∷StopRecording for full
documentation.)""";
        } StopRecording;
        // Symbol: drake::geometry::MeshcatVisualizer::get_mutable_recording
        struct /* get_mutable_recording */ {
          // Source: drake/geometry/meshcat_visualizer.h
          const char* doc =
R"""(Convenience function that calls Meshcat∷get_mutable_recording on the
underlying Meshcat object; refer to Meshcat∷get_mutable_recording for
full documentation.)""";
        } get_mutable_recording;
        // Symbol: drake::geometry::MeshcatVisualizer::query_object_input_port
        struct /* query_object_input_port */ {
          // Source: drake/geometry/meshcat_visualizer.h
          const char* doc =
R"""(Returns the QueryObject-valued input port. It should be connected to
SceneGraph's QueryObject-valued output port. Failure to do so will
cause a runtime error when attempting to broadcast messages.)""";
        } query_object_input_port;
      } MeshcatVisualizer;
      // Symbol: drake::geometry::MeshcatVisualizerParams
      struct /* MeshcatVisualizerParams */ {
        // Source: drake/geometry/meshcat_visualizer_params.h
        const char* doc =
R"""(The set of parameters for configuring MeshcatVisualizer.)""";
        // Symbol: drake::geometry::MeshcatVisualizerParams::Serialize
        struct /* Serialize */ {
          // Source: drake/geometry/meshcat_visualizer_params.h
          const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
        } Serialize;
        // Symbol: drake::geometry::MeshcatVisualizerParams::default_color
        struct /* default_color */ {
          // Source: drake/geometry/meshcat_visualizer_params.h
          const char* doc =
R"""(The color to apply to any geometry that hasn't defined one.)""";
        } default_color;
        // Symbol: drake::geometry::MeshcatVisualizerParams::delete_on_initialization_event
        struct /* delete_on_initialization_event */ {
          // Source: drake/geometry/meshcat_visualizer_params.h
          const char* doc =
R"""(Determines whether to send a Meshcat∷Delete(prefix) message on an
initialization event to remove any visualizations e.g. from a previous
simulation. See declare_initialization_events "Declare initialization
events" for more information.)""";
        } delete_on_initialization_event;
        // Symbol: drake::geometry::MeshcatVisualizerParams::enable_alpha_slider
        struct /* enable_alpha_slider */ {
          // Source: drake/geometry/meshcat_visualizer_params.h
          const char* doc =
R"""(Determines whether to enable the alpha slider for geometry display.)""";
        } enable_alpha_slider;
        // Symbol: drake::geometry::MeshcatVisualizerParams::include_unspecified_accepting
        struct /* include_unspecified_accepting */ {
          // Source: drake/geometry/meshcat_visualizer_params.h
          const char* doc =
R"""((Advanced) For a given geometry, if the GeometryProperties for our
``role`` has the property ``(meshcat, accepting)`` then the visualizer
will show the geometry only if the property's value matches our
``prefix``. If that property is absent then the geometry will be shown
only if ``include_unspecified_accepting`` is true.)""";
        } include_unspecified_accepting;
        // Symbol: drake::geometry::MeshcatVisualizerParams::initial_alpha_slider_value
        struct /* initial_alpha_slider_value */ {
          // Source: drake/geometry/meshcat_visualizer_params.h
          const char* doc =
R"""(Initial alpha slider value. This value should lie in the range [0, 1].
Furthermore, the slider value is *quantized* which means the value
used here will be replaced with the nearest quantized value supported
by the slider implementation.)""";
        } initial_alpha_slider_value;
        // Symbol: drake::geometry::MeshcatVisualizerParams::prefix
        struct /* prefix */ {
          // Source: drake/geometry/meshcat_visualizer_params.h
          const char* doc =
R"""(A prefix to add to the path for all objects and transforms curated by
the MeshcatVisualizer. It can be an absolute path or relative path. If
relative, this ``prefix`` will be appended to the Meshcat ``prefix``
based on the standard path semantics in Meshcat. See meshcat_path
"Meshcat paths" for details.)""";
        } prefix;
        // Symbol: drake::geometry::MeshcatVisualizerParams::publish_period
        struct /* publish_period */ {
          // Source: drake/geometry/meshcat_visualizer_params.h
          const char* doc =
R"""(The duration (in simulation seconds) between attempts to update poses
in the visualizer. (To help avoid small simulation time steps, we use
a default period that has an exact representation in binary floating
point; see drake#15021 for details.))""";
        } publish_period;
        // Symbol: drake::geometry::MeshcatVisualizerParams::role
        struct /* role */ {
          // Source: drake/geometry/meshcat_visualizer_params.h
          const char* doc =
R"""(The role of the geometries to be sent to the visualizer.)""";
        } role;
        // Symbol: drake::geometry::MeshcatVisualizerParams::show_hydroelastic
        struct /* show_hydroelastic */ {
          // Source: drake/geometry/meshcat_visualizer_params.h
          const char* doc =
R"""(When using the hydroelastic contact model, collision geometries that
are *declared* as geometric primitives are frequently represented by
some discretely tessellated mesh when computing contact. It can be
quite helpful in assessing contact behavior to visualize these
discrete meshes (in place of the idealized primitives).

To visualize these representations it is necessary to request
visualization of geometries with the Role∷kProximity role (see the
role field). It is further necessary to explicitly request the
hydroelastic meshes where available (setting show_hydroelastic to
``True``).

Setting this ``show_hydroelastic`` to ``True`` will have no apparent
effect if none of the collision meshes have a hydroelastic mesh
associated with them.

This option is ignored by MeshcatVisualizer<T> when T is not
``double``, e.g. if T == AutoDiffXd.)""";
        } show_hydroelastic;
        // Symbol: drake::geometry::MeshcatVisualizerParams::visible_by_default
        struct /* visible_by_default */ {
          // Source: drake/geometry/meshcat_visualizer_params.h
          const char* doc =
R"""(Determines whether our meshcat path should be default to being
visible.)""";
        } visible_by_default;
        auto Serialize__fields() const {
          return std::array{
            std::make_pair("default_color", default_color.doc),
            std::make_pair("delete_on_initialization_event", delete_on_initialization_event.doc),
            std::make_pair("enable_alpha_slider", enable_alpha_slider.doc),
            std::make_pair("include_unspecified_accepting", include_unspecified_accepting.doc),
            std::make_pair("initial_alpha_slider_value", initial_alpha_slider_value.doc),
            std::make_pair("prefix", prefix.doc),
            std::make_pair("publish_period", publish_period.doc),
            std::make_pair("role", role.doc),
            std::make_pair("show_hydroelastic", show_hydroelastic.doc),
            std::make_pair("visible_by_default", visible_by_default.doc),
          };
        }
      } MeshcatVisualizerParams;
      // Symbol: drake::geometry::MeshcatVisualizerd
      struct /* MeshcatVisualizerd */ {
        // Source: drake/geometry/meshcat_visualizer.h
        const char* doc =
R"""(A convenient alias for the MeshcatVisualizer class when using the
``double`` scalar type.)""";
      } MeshcatVisualizerd;
      // Symbol: drake::geometry::PerceptionProperties
      struct /* PerceptionProperties */ {
        // Source: drake/geometry/geometry_roles.h
        const char* doc =
R"""(The set of properties for geometry used in a "perception" role.

Examples of functionality that depends on the perception role: -
render∷RenderEngineVtk)""";
        // Symbol: drake::geometry::PerceptionProperties::PerceptionProperties
        struct /* ctor */ {
          // Source: drake/geometry/geometry_roles.h
          const char* doc = R"""()""";
        } ctor;
      } PerceptionProperties;
      // Symbol: drake::geometry::ProximityProperties
      struct /* ProximityProperties */ {
        // Source: drake/geometry/geometry_roles.h
        const char* doc =
R"""(The set of properties for geometry used in a *proximity* role.

Examples of functionality that depends on the proximity role:)""";
        // Symbol: drake::geometry::ProximityProperties::ProximityProperties
        struct /* ctor */ {
          // Source: drake/geometry/geometry_roles.h
          const char* doc = R"""()""";
        } ctor;
      } ProximityProperties;
      // Symbol: drake::geometry::QueryObject
      struct /* QueryObject */ {
        // Source: drake/geometry/query_object.h
        const char* doc =
R"""(The QueryObject serves as a mechanism to perform geometry queries on
the world's geometry. The SceneGraph has an abstract-valued port that
contains a QueryObject (i.e., a QueryObject-valued output port).

To perform geometry queries on SceneGraph: - a LeafSystem must have a
QueryObject-valued input port and connect it to the corresponding
query output port on SceneGraph, - the querying LeafSystem can
evaluate the input port, retrieving a ``const QueryObject&`` in
return, and, finally, - invoke the appropriate method on the
QueryObject.

The const reference returned by the input port is considered "live" -
it is linked to the context, system, and cache (making full use of all
of those mechanisms). This const reference should *never* be
persisted; doing so can lead to erroneous query results. It is simpler
and more advisable to acquire it for evaluation in a limited scope
(e.g., CalcTimeDerivatives()) and then discard it. If a QueryObject is
needed for many separate functions in a LeafSystem, each should
re-evaluate the input port. The underlying caching mechanism should
make the cost of this negligible.

The QueryObject *can* be copied. The copied instance is no longer
"live"; it is now "baked". Essentially, it freezes the state of the
live scene graph in its current configuration and disconnects it from
the system and context. This means, even if the original context
changes values, the copied/baked instance will always reproduce the
same query results. This baking process is not cheap and should not be
done without consideration.

Queries and scalar type
-----------------------

A QueryObject *cannot* be converted to a different scalar type. A
QueryObject of scalar type T can only be acquired from the output port
of a SceneGraph of type T evaluated on a corresponding Context, also
of type T.

QueryObject's support for arbitrary scalar type is incomplete. Not all
queries support all scalar types to the same degree. Furthermore, the
queries are typically served by *families* of algorithms. The
evaluation of a query between a particular pair of geometries will
depend on the query, the pair of geometry types involved, and the
scalar type. From query to query, the treatment of a geometry (or
geometry pair) for a given scalar type can vary in many ways,
including but not limited to: ignoring the geometry, throwing an
exception, results with limited precision, or full, accurate support.
The queries below provide tables to help inform your expectations when
evaluating queries. The tables all use a common methodology and admit
a common interpretation.

For each (query, geometry-pair, scalar) combination, we create a set
of geometric configurations with known answers. We evaluate the
precision of the query result (if supported at all) over the entire
set and report the *worst* observed error. This is a purely empirical
approach and doesn't fully characterize the families of underlying
algorithms, and the reported error may be misleading in that we've
missed far worse latent error or that the error reported doesn't well
represent the average case.

The families of algorithms also differ in how their inherent errors
scale with the scale of the problem (e.g., size of geometries,
magnitude of distance/depth, etc.) Attempting to fully characterize
that aspect is both arduous and problematic, so, we've chosen a more
*representative* approach.

Because Drake is primarily intended for robot simulation, we've
created geometric configurations on the scale of common robot
manipulators (on the order of 20cm). We position them with a known
penetration depth (or separating distance) of 2 mm. The error reported
is the deviation from that expected result.

When interpreting the tables, keep the following in mind: - The table
illustrates trends in *broad* strokes, only. It does not represent an
exhaustive analysis. - If your problem involves larger geometries,
greater penetration depths, or larger separating distances, the error
will vary. Do not assume that observed error in this context is
necessarily relative -- there truly is that much variability in the
families of algorithms. - These values are an attempt to capture the
*worst* case outcome. The error shown is a single-significant-digit
approximation of that observed error. - These values may not actually
represent the true worst case; discovering the true worst case is
generally challenging. These represent our best effort to date. If you
find outcomes that are worse those reported here, please ` submit a
bug <https://github.com/RobotLocomotion/drake/issues/new>`_. - These
tables represent Drake's transient state. The eventual goal is to
report no more than 1e-14 error across all supportable geometry pairs
and scalars. At that point, the table will simply disappear.)""";
        // Symbol: drake::geometry::QueryObject::ComputeAabbInWorld
        struct /* ComputeAabbInWorld */ {
          // Source: drake/geometry/query_object.h
          const char* doc =
R"""(Reports the axis-aligned bounding box of the geometry indicated by
``geometry_id`` in the world frame. Returns std∷nullopt if the
geometry is not supported for this query. Currently, only deformable
geometries are supported.

Raises:
    RuntimeError if the ``geometry_id`` is not valid.)""";
        } ComputeAabbInWorld;
        // Symbol: drake::geometry::QueryObject::ComputeContactSurfaces
        struct /* ComputeContactSurfaces */ {
          // Source: drake/geometry/query_object.h
          const char* doc =
R"""(Reports pairwise intersections and characterizes each non-empty
intersection as a ContactSurface for hydroelastic contact model. The
computation is subject to collision filtering.

For two intersecting geometries g_A and g_B, it is guaranteed that
they will map to ``id_A`` and ``id_B`` in a fixed, repeatable manner,
where ``id_A`` and ``id_B`` are GeometryId's of geometries g_A and g_B
respectively.

In the current incarnation, this function represents an incomplete
implementation. That has several implications, as described below:

- This table shows which shapes can be declared for use in hydroelastic
contact, and what compliance can be assigned.

| Shape | Compliant | Rigid | | :-------: | :-------: | :---: | |
Sphere | yes | yes | | Cylinder | yes | yes | | Box | yes | yes | |
Capsule | yes | yes | | Ellipsoid | yes | yes | | HalfSpace | yes |
yes | | Mesh | yesᵃ | yesᵇ | | Convex | yesᶜ | yesᶜ |

- ᵃ The exact representation of a compliant mesh depends on the type of
mesh file it references:
- .obj: the convex hull of the mesh will be used (as if it were
declared to be a Convex shape).
- .vtk: the tetrahedral mesh will be used directly. This external
working
<a href="https://docs.google.com/document/d/1VZtVsxIjOLKvgQ8SNSrF6PtWuPW5z9PP7-dQuxfmqpc/edit?usp=sharing">
document</a> provides guidance how to generate a tetrahedral mesh
in a VTK file from a surface mesh in an OBJ file.
- ᵇ For rigid Mesh, please specify a surface mesh in an OBJ file in
Mesh(filename). A tetrahedral mesh in a VTK file can also be
specified.
- ᶜ The Convex shape can reference either an .obj or a .vtk tetrahedral
mesh. In both cases, its convex hull will be used to define the
hydroelastic representation.

- We do not support contact between two rigid geometries. One geometry
must* be compliant, and the other could be rigid or compliant. If
two rigid geometries collide, an exception will be thrown. More
particularly, if such a geometry pair *cannot be culled* an exception
will be thrown. No exception is thrown if the pair has been filtered.
- If you need all combinations of rigid-rigid contact, rigid-compliant
contact, and compliant-compliant contact, you might consider
ComputeContactSurfacesWithFallback().
- The hydroelastic modulus (N/m^2) of each compliant geometry is set in
ProximityProperties by AddCompliantHydroelasticProperties().
- The tessellation of the corresponding meshes is controlled by the
resolution hint (where appropriate), as defined by
AddCompliantHydroelasticProperties() and
AddRigidHydroelasticProperties().

**Scalar support**

This method provides support for both double and AutoDiffXd, but not
Expression. Like with the other proximity queries, derivatives can
only be introduced via geometry *poses*. We cannot differentiate
w.r.t. geometric properties (e.g., radius, length, etc.)

Parameter ``representation``:
    Controls the mesh representation of the contact surface. See
    contact_surface_discrete_representation "contact surface
    representation" for more details.

Returns:
    A vector populated with all detected intersections characterized
    as contact surfaces. The ordering of the results is guaranteed to
    be consistent -- for fixed geometry poses, the results will remain
    the same.)""";
        } ComputeContactSurfaces;
        // Symbol: drake::geometry::QueryObject::ComputeContactSurfacesWithFallback
        struct /* ComputeContactSurfacesWithFallback */ {
          // Source: drake/geometry/query_object.h
          const char* doc =
R"""(Reports pairwise intersections and characterizes each non-empty
intersection as a ContactSurface *where possible* and as a
PenetrationAsPointPair where not.

This method can be thought of as a combination of
ComputeContactSurfaces() and ComputePointPairPenetration(). For each
geometry pair, we attempt to compute a ContactSurface. If that fails,
rather than throwing, we attempt to characterize the contact as a
point pair. If that fails, we throw. See the documentation of those
constituent methods to understand the circumstances in which they
fail.

The ordering of the *added* results is guaranteed to be consistent --
for fixed geometry poses, the results will remain the same.

**Scalar support**

The scalar support is a combination of the scalar support offered by
ComputeContactSurfaces() and ComputePointPairPenetration(). This
method supports double and AutoDiffXd to the extent that those
constituent methods do, but does not support Expression.

Parameter ``representation``:
    Controls the mesh representation of the contact surface. See
    contact_surface_discrete_representation "contact surface
    representation" for more details.

Parameter ``surfaces``:
    The vector that contact surfaces will be added to. The vector will
    *not* be cleared.

Parameter ``point_pairs``:
    The vector that fall back point pair data will be added to. The
    vector will *not* be cleared.

Precondition:
    Neither ``surfaces`` nor ``point_pairs`` is nullptr.

Raises:
    RuntimeError for the reasons described in ComputeContactSurfaces()
    and ComputePointPairPenetration().

Note:
    The ``surfaces`` and ``point_pairs`` are output pointers in C++,
    but are return values in the Python bindings.)""";
        } ComputeContactSurfacesWithFallback;
        // Symbol: drake::geometry::QueryObject::ComputeDeformableContact
        struct /* ComputeDeformableContact */ {
          // Source: drake/geometry/query_object.h
          const char* doc =
R"""(Reports contact information among all deformable geometries. It
includes contacts between two deformable geometries or contacts
between a deformable geometry and a non-deformable geometry. This
function only supports double as the scalar type.

Parameter ``deformable_contact``:
    Contains all deformable contact data on output. Any data passed in
    is cleared before the computation.

Precondition:
    deformable_contact != nullptr.

Warning:
    This feature is considered to be **experimental** and may change
    or be removed at any time, without any deprecation notice ahead of
    time.)""";
        } ComputeDeformableContact;
        // Symbol: drake::geometry::QueryObject::ComputeObbInWorld
        struct /* ComputeObbInWorld */ {
          // Source: drake/geometry/query_object.h
          const char* doc =
R"""(Reports the oriented bounding box of the geometry indicated by
``geometry_id`` in the world frame. Returns std∷nullopt if the
geometry is an HalfSpace (and doesn't have a bounding box).

Note:
    If geometry_id refers to a deformable geometry, the OBB is
    computed using the deformed mesh in the world frame. See
    SceneGraphInspector∷GetObbInGeometryFrame() for computing the OBB
    of the reference mesh in its canonical frame.

Raises:
    RuntimeError if the ``geometry_id`` is not valid.)""";
        } ComputeObbInWorld;
        // Symbol: drake::geometry::QueryObject::ComputePointPairPenetration
        struct /* ComputePointPairPenetration */ {
          // Source: drake/geometry/query_object.h
          const char* doc =
R"""(Computes the penetrations across all pairs of geometries in the world
with the penetrations characterized by pairs of points (see
PenetrationAsPointPair), providing some measure of the penetration
"depth" of the two objects, but *not* the overlapping volume.

Only reports results for *penetrating* geometries; if two geometries
are separated, there will be no result for that pair. Geometries whose
surfaces are just touching (osculating) are not considered in
penetration. Surfaces whose penetration is within an epsilon of
osculation, are likewise not considered penetrating. Pairs of
*anchored* geometry are also not reported. This method is affected by
collision filtering.

For two penetrating geometries g_A and g_B, it is guaranteed that they
will map to ``id_A`` and ``id_B`` in a fixed, repeatable manner.

**Characterizing the returned values**

As discussed in the query_object_precision_methodology "class's
documentation", these tables document the support given by this query
for pairs of geometry types and scalar. See the description in the
link for details on how to interpret the tables' results. The query is
symmetric with respect to shape *ordering*, the pair (ShapeA, ShapeB)
will be the same as (ShapeB, ShapeA), so we only fill in half of each
table.

| | Box | Capsule | Convex | Cylinder | Ellipsoid | HalfSpace | Mesh |
Sphere | | --------: | :-----: | :------: | :-----: | :-------: |
:--------: | :--------: | :-----: | :-----: | | Box | 2e-15 | ░░░░░░ |
░░░░░ | ░░░░░░░ | ░░░░░░ | ░░░░░░ | ░░░░░ | ░░░░░ | | Capsule | 3e-5ᶜ
| 2e-5ᶜ | ░░░░░ | ░░░░░░░ | ░░░░░░ | ░░░░░░ | ░░░░░ | ░░░░░ | | Convex
| 2e-15ᶜ | 3e-5ᶜ | 2e-15ᶜ | ░░░░░░░ | ░░░░░░ | ░░░░░░ | ░░░░░ | ░░░░░
| | Cylinder | 1e-3ᶜ | 4e-5ᶜ | 1e-3ᶜ | 2e-3ᶜ | ░░░░░░ | ░░░░░░ | ░░░░░
| ░░░░░ | | Ellipsoid | 4e-4ᶜ | 2e-4ᶜ | 4e-4ᶜ | 2e-3ᶜ | 5e-4ᶜ | ░░░░░░
| ░░░░░ | ░░░░░ | | HalfSpace | 6e-15 | 4e-15 | 3e-15ᶜ | 4e-15 | 3e-15
| throwsᵃ | ░░░░░ | ░░░░░ | | Mesh | ᵇ | ᵇ | ᵇ | ᵇ | ᵇ | ᵇ | ᵇ | ░░░░░
| | Sphere | 3e-15 | 5e-15 | 3e-5ᶜ | 5e-15 | 2e-4ᶜ | 3e-15 | ᵇ | 5e-15
| ***Table 1***: Worst observed error (in m) for 2mm penetration
between geometries approximately 20cm in size for ``T`` = ``double``.

| | Box | Capsule | Convex | Cylinder | Ellipsoid | HalfSpace | Mesh |
Sphere | | --------: | :-----: | :------: | :-----: | :-------: |
:--------: | :--------: | :-----: | :-----: | | Box | throwsᵈ | ░░░░░░
| ░░░░░ | ░░░░░░░ | ░░░░░░ | ░░░░░░ | ░░░░░ | ░░░░░ | | Capsule |
throwsᵈ | throwsᵈ | ░░░░░ | ░░░░░░░ | ░░░░░░ | ░░░░░░ | ░░░░░ | ░░░░░
| | Convex | throwsᵈ | throwsᵈ | throwsᵈ | ░░░░░░░ | ░░░░░░ | ░░░░░░ |
░░░░░ | ░░░░░ | | Cylinder | throwsᵈ | throwsᵈ | throwsᵈ | throwsᵈ |
░░░░░░ | ░░░░░░ | ░░░░░ | ░░░░░ | | Ellipsoid | throwsᵈ | throwsᵈ |
throwsᵈ | throwsᵈ | throwsᵈ | ░░░░░░ | ░░░░░ | ░░░░░ | | HalfSpace |
throwsᵈ | throwsᵈ | throwsᵈ | throwsᵈ | throwsᵈ | throwsᵃ | ░░░░░ |
░░░░░ | | Mesh | ᵇ | ᵇ | ᵇ | ᵇ | ᵇ | ᵇ | ᵇ | ░░░░░ | | Sphere | 2e-15
| 3e-15 | throwsᵈ | 2e-15 | throwsᵈ | 2e-15 | ᵇ | 5e-15 | ***Table
2***: Worst observed error (in m) for 2mm penetration between
geometries approximately 20cm in size for ``T`` = drake∷AutoDiffXd
"AutoDiffXd".

| | Box | Capsule | Convex | Cylinder | Ellipsoid | HalfSpace | Mesh |
Sphere | | --------: | :-----: | :------: | :-----: | :-------: |
:--------: | :--------: | :-----: | :-----: | | Box | throwsᵉ | ░░░░░░
| ░░░░░ | ░░░░░░░ | ░░░░░░ | ░░░░░░ | ░░░░░ | ░░░░░ | | Capsule |
throwsᵉ | throwsᵉ | ░░░░░ | ░░░░░░░ | ░░░░░░ | ░░░░░░ | ░░░░░ | ░░░░░
| | Convex | throwsᵉ | throwsᵉ | throwsᵉ | ░░░░░░░ | ░░░░░░ | ░░░░░░ |
░░░░░ | ░░░░░ | | Cylinder | throwsᵉ | throwsᵉ | throwsᵉ | throwsᵉ |
░░░░░░ | ░░░░░░ | ░░░░░ | ░░░░░ | | Ellipsoid | throwsᵉ | throwsᵉ |
throwsᵉ | throwsᵉ | throwsᵉ | ░░░░░░ | ░░░░░ | ░░░░░ | | HalfSpace |
throwsᵉ | throwsᵉ | throwsᵉ | throwsᵉ | throwsᵉ | throwsᵃ | ░░░░░ |
░░░░░ | | Mesh | ᵇ | ᵇ | ᵇ | ᵇ | ᵇ | ᵇ | ᵇ | ░░░░░ | | Sphere |
throwsᵉ | throwsᵉ | throwsᵉ | throwsᵉ | throwsᵉ | throwsᵉ | ᵇ |
throwsᵉ | ***Table 3***: Support for ``T`` =
drake∷symbolic∷Expression.

- ᵃ Penetration depth between two HalfSpace instances has no meaning; either
they don't intersect, or they have infinite penetration.
- ᵇ Meshes are represented by the *convex* hull of the mesh, therefore the
results for Mesh are assumed to be the same as for Convex.
- ᶜ These results are computed using an iterative algorithm. For particular
configurations, the solution may be correct to machine precision. The
values reported here are confirmed, observed worst case answers.
- ᵈ These results are simply not supported for
``T`` = drake∷AutoDiffXd "AutoDiffXd" at this time.
- ᵉ These results are simply not supported for
``T`` = drake∷symbolic∷Expression at this time.

Returns:
    A vector populated with all detected penetrations characterized as
    point pairs. The ordering of the results is guaranteed to be
    consistent -- for fixed geometry poses, the results will remain
    the same.

Warning:
    For Mesh shapes, their convex hulls are used in this query. It is
    not* computationally efficient or particularly accurate.

Raises:
    RuntimeError if a Shape-Shape pair is in collision and indicated
    as ``throws`` in the support table above.)""";
        } ComputePointPairPenetration;
        // Symbol: drake::geometry::QueryObject::ComputeSignedDistanceGeometryToPoint
        struct /* ComputeSignedDistanceGeometryToPoint */ {
          // Source: drake/geometry/query_object.h
          const char* doc =
R"""(A variant of ComputeSignedDistanceToPoint(). Instead of finding
distances to all geometries, provides the distance to only the
geometries indicated by the given set of ``geometries``.

Parameter ``p_WQ``:
    Position of a query point Q in world frame W.

Parameter ``geometries``:
    The set of geometries to query against. The distances between the
    surface of each geometry and the point Q will be returned.

Returns:
    the distance measurements. The ordering of the results is
    guaranteed to be consistent -- for a fixed set of geometries, the
    results will remain the same.

Raises:
    RuntimeError if any GeometryId in ``geometries`` is invalid.

Raises:
    RuntimeError if the combination of an indicated geometry's Shape
    type and the given scalar Type T are unsupported in
    ComputeSignedDistanceToPoint()'s support table.

Raises:
    RuntimeError if any indicated geometry is deformable.)""";
        } ComputeSignedDistanceGeometryToPoint;
        // Symbol: drake::geometry::QueryObject::ComputeSignedDistancePairClosestPoints
        struct /* ComputeSignedDistancePairClosestPoints */ {
          // Source: drake/geometry/query_object.h
          const char* doc =
R"""(A variant of ComputeSignedDistancePairwiseClosestPoints() which
computes the signed distance (and witnesses) between a specific pair
of geometries indicated by id. This function has the same restrictions
on scalar report as ComputeSignedDistancePairwiseClosestPoints().

Note:
    This query is unique among the distance queries in that it doesn't
    respect collision filters. As long as the geometry ids refer to
    geometries with proximity roles, signed distance can be computed
    (subject to supported scalar tables above).

**Characterizing the returned values**

This method merely exercises the same mechanisms as
ComputeSignedDistancePairwiseClosestPoints() for evaluating signed
distance. Refer to query_object_compute_pairwise_distance_table "the
table for ComputeSignedDistancePairwiseClosestPoints()" for details.

Raises:
    RuntimeError if either geometry id is invalid (e.g., doesn't refer
    to an existing geometry, lacking proximity role, etc.), the pair
    is unsupported as indicated by the scalar support table.

Warning:
    For Mesh shapes, their convex hulls are used in this query. It is
    *not* computationally efficient or particularly accurate.)""";
        } ComputeSignedDistancePairClosestPoints;
        // Symbol: drake::geometry::QueryObject::ComputeSignedDistancePairwiseClosestPoints
        struct /* ComputeSignedDistancePairwiseClosestPoints */ {
          // Source: drake/geometry/query_object.h
          const char* doc =
R"""(Computes the signed distance together with the nearest points across
all pairs of geometries in the world. Reports both the separating
geometries and penetrating geometries.

This query provides φ(A, B), the signed distance between two objects A
and B.

If the objects do not overlap (i.e., A ⋂ B = ∅), φ > 0 and represents
the minimal distance between the two objects. More formally: φ =
min(|Aₚ - Bₚ|) ∀ Aₚ ∈ A and Bₚ ∈ B.

Note:
    The pair (Aₚ, Bₚ) is a "witness" of the distance. The pair need
    not be unique (think of two parallel planes).

If the objects touch or overlap (i.e., A ⋂ B ≠ ∅), φ ≤ 0 and can be
interpreted as the negative penetration depth. It is the smallest
length of the vector v, such that by shifting one object along that
vector relative to the other, the two objects will no longer be
overlapping. More formally, φ(A, B) = -min |v|. s.t (Tᵥ · A) ⋂ B = ∅
where Tᵥ is a rigid transformation that displaces A by the vector v,
namely Tᵥ · A = {u + v | ∀ u ∈ A}. By implication, there exist points
Aₚ and Bₚ on the surfaces of objects A and B, respectively, such that
Aₚ + v = Bₚ, Aₚ ∈ A ∩ B, Bₚ ∈ A ∩ B. These points are the witnesses to
the penetration.

This method is affected by collision filtering; geometry pairs that
have been filtered will not produce signed distance query results.

For a geometry pair (A, B), the returned results will always be
reported in a fixed order (e.g., always (A, B) and never (B, A)). The
*basis* for the ordering is arbitrary (and therefore undocumented),
but guaranteed to be fixed and repeatable.

Notice that this is an O(N²) operation, where N is the number of
geometries remaining in the world after applying collision filter. We
report the distance between dynamic objects, and between dynamic and
anchored objects. We DO NOT report the distance between two anchored
objects.

**Using maximum distance**

While the algorithm generally has O(N²) complexity in time and space,
that can be reduced by the judicious use of the ``max_distance``
parameter. If ``φ(A, B) > max_distance``, the pair (A, B) will not be
included in the results (making it O(M²) in space where M < N).
Furthermore, the broadphase culling algorithm can exploit
``max_distance`` to *cheaply* eliminate pairs of geometry that are
"obviously" too far (likewise reducing the time complexity).

Passing ``max_distance = 0`` is conceptually related to calling
HasCollisions(). If contact is sparse (very few actually contacting
geometry pairs), the two invocations will be quite similar in cost.
However, the distinction between the two is that *this* method would
have to include *all* pairs that satisfy ``φ(A, B) <= 0``, whereas
HasCollisions() stops at the first. So, the more actually colliding
geometry pairs there are, the bigger the difference in cost between
the two approaches.

**Characterizing the returned values**

As discussed in the query_object_precision_methodology "class's
documentation", this table documents the support given by this query
for pairs of geometry types and scalar. See the description in the
link for details on how to interpret the table results. The query is
symmetric with respect to shape *ordering*, the pair (ShapeA, ShapeB)
will be the same as (ShapeB, ShapeA), so we only fill in half the
table.

| | Box | Capsule | Convex | Cylinder | Ellipsoid | HalfSpace | Mesh |
Sphere | | --------: | :-----: | :------: | :-----: | :-------: |
:--------: | :--------: | :-----: | :-----: | | Box | 4e-15 | ░░░░░░ |
░░░░░ | ░░░░░░░ | ░░░░░░ | ░░░░░░ | ░░░░░ | ░░░░░ | | Capsule | 3e-6 |
2e-5 | ░░░░░ | ░░░░░░░ | ░░░░░░ | ░░░░░░ | ░░░░░ | ░░░░░ | | Convex |
3e-15 | 2e-5 | 3e-15 | ░░░░░░░ | ░░░░░░ | ░░░░░░ | ░░░░░ | ░░░░░ | |
Cylinder | 6e-6 | 1e-5 | 6e-6 | 2e-5 | ░░░░░░ | ░░░░░░ | ░░░░░ | ░░░░░
| | Ellipsoid | 9e-6 | 5e-6 | 9e-6 | 5e-5 | 2e-5 | ░░░░░░ | ░░░░░ |
░░░░░ | | HalfSpace | throwsᵃ | throwsᵃ | throwsᵃ | throwsᵃ | throwsᵃ
| throwsᵃ | ░░░░░ | ░░░░░ | | Mesh | ᶜ | ᶜ | ᶜ | ᶜ | ᶜ | throwsᵃ | ᶜ |
░░░░░ | | Sphere | 3e-15 | 6e-15 | 3e-6 | 5e-15 | 4e-5 | 3e-15 | ᶜ |
6e-15 | ***Table 4***: Worst observed error (in m) for 2mm
penetration/separation between geometries approximately 20cm in size
for ``T`` = ``double``.

| | Box | Capsule | Convex | Cylinder | Ellipsoid | HalfSpace | Mesh |
Sphere | | --------: | :-----: | :------: | :-----: | :-------: |
:--------: | :--------: | :-----: | :-----: | | Box | throwsᵇ | ░░░░░░
| ░░░░░ | ░░░░░░░ | ░░░░░░ | ░░░░░░ | ░░░░░ | ░░░░░ | | Capsule |
throwsᵇ | throwsᵇ | ░░░░░ | ░░░░░░░ | ░░░░░░ | ░░░░░░ | ░░░░░ | ░░░░░
| | Convex | throwsᵇ | throwsᵇ | throwsᵇ | ░░░░░░░ | ░░░░░░ | ░░░░░░ |
░░░░░ | ░░░░░ | | Cylinder | throwsᵇ | throwsᵇ | throwsᵇ | throwsᵇ |
░░░░░░ | ░░░░░░ | ░░░░░ | ░░░░░ | | Ellipsoid | throwsᵇ | throwsᵇ |
throwsᵇ | throwsᵇ | throwsᵇ | ░░░░░░ | ░░░░░ | ░░░░░ | | HalfSpace |
throwsᵃ | throwsᵃ | throwsᵃ | throwsᵃ | throwsᵃ | throwsᵃ | ░░░░░ |
░░░░░ | | Mesh | ᶜ | ᶜ | ᶜ | ᶜ | ᶜ | throwsᵃ | ᶜ | ░░░░░ | | Sphere |
2e-15 | throwsᵇ | throwsᵇ | throwsᵇ | throwsᵇ | 2e-15 | ᶜ | 5e-15 |
***Table 5***: Worst observed error (in m) for 2mm
penetration/separation between geometries approximately 20cm in size
for ``T`` = drake∷AutoDiffXd "AutoDiffXd".

| | Box | Capsule | Convex | Cylinder | Ellipsoid | HalfSpace | Mesh |
Sphere | | --------: | :-----: | :------: | :-----: | :-------: |
:--------: | :--------: | :-----: | :-----: | | Box | throwsᵈ | ░░░░░░
| ░░░░░ | ░░░░░░░ | ░░░░░░ | ░░░░░░ | ░░░░░ | ░░░░░ | | Capsule |
throwsᵈ | throwsᵈ | ░░░░░ | ░░░░░░░ | ░░░░░░ | ░░░░░░ | ░░░░░ | ░░░░░
| | Convex | throwsᵈ | throwsᵈ | throwsᵈ | ░░░░░░░ | ░░░░░░ | ░░░░░░ |
░░░░░ | ░░░░░ | | Cylinder | throwsᵈ | throwsᵈ | throwsᵈ | throwsᵈ |
░░░░░░ | ░░░░░░ | ░░░░░ | ░░░░░ | | Ellipsoid | throwsᵈ | throwsᵈ |
throwsᵈ | throwsᵈ | throwsᵈ | ░░░░░░ | ░░░░░ | ░░░░░ | | HalfSpace |
throwsᵃ | throwsᵃ | throwsᵃ | throwsᵃ | throwsᵃ | throwsᵃ | ░░░░░ |
░░░░░ | | Mesh | ᶜ | ᶜ | ᶜ | ᶜ | ᶜ | throwsᵃ | ᶜ | ░░░░░ | | Sphere |
throwsᵈ | throwsᵈ | throwsᵈ | throwsᵈ | throwsᵈ | throwsᵈ | ᶜ |
throwsᵈ | ***Table 6***: Support for ``T`` =
drake∷symbolic∷Expression.

- ᵃ We don't currently support queries between HalfSpace and any other shape
(except for Sphere).
- ᵇ These results are simply not supported for
``T`` = drake∷AutoDiffXd "AutoDiffXd" at this time.
- ᶜ Meshes are represented by the *convex* hull of the mesh, therefore the
results for Mesh are the same as for Convex.
- ᵈ These results are simply not supported for
``T`` = drake∷symbolic∷Expression at this time.

**Characterizing the returned gradients**

In most cases, the returned gradient vectors are the normalized
displacement vectors between two witness points. However, when two
geometries touch at zero distance, their witness points have a zero
displacement vector that we cannot normalize.

When two geometries touch at zero distance, we have special
implementation to choose reasonable gradients for some cases shown as
"Ok" in the table below. Otherwise, they are "NaN" in the table. In
general, we try to choose the gradient, when two geometries touch, in
the most consistent way, but the problem sometimes doesn't have a
unique solution. For example, any direction is qualified to be the
gradient for two concentric spheres. Or two boxes touching at their
vertices can pick a gradient from a continuous family of directions.

| | Box | Capsule | Convex | Cylinder | Ellipsoid | HalfSpace | Mesh |
Sphere | | --------: | :--: | :------: | :-----: | :-------: |
:--------: | :--------: | :-----: | :-----: | | Box | Ok | ░░░░░░ |
░░░░░ | ░░░░░░░ | ░░░░░░ | ░░░░░░ | ░░░░░ | ░░░░░ | | Capsule | NaN |
NaN | ░░░░░ | ░░░░░░░ | ░░░░░░ | ░░░░░░ | ░░░░░ | ░░░░░ | | Convex |
NaN | NaN | NaN | ░░░░░░░ | ░░░░░░ | ░░░░░░ | ░░░░░ | ░░░░░ | |
Cylinder | NaN | NaN | NaN | NaN | ░░░░░░ | ░░░░░░ | ░░░░░ | ░░░░░ | |
Ellipsoid | NaN | NaN | NaN | NaN | NaN | ░░░░░░ | ░░░░░ | ░░░░░ | |
HalfSpace | NaN | NaN | NaN | NaN | NaN | NaN | ░░░░░ | ░░░░░ | | Mesh
| NaN | NaN | NaN | NaN | NaN | NaN | NaN | ░░░░░ | | Sphere | Ok | Ok
| Okᵃ | Ok | Okᵃ | Ok | Okᵃ | Ok | ***Table 7***: Support for
signed-distance gradients when two geometries touch at zero distance.

- ᵃ Return the gradient as a Vector3d of NaN if the sphere has zero radius.

Parameter ``max_distance``:
    The maximum distance at which distance data is reported.

Returns:
    The signed distance (and supporting data) for all unfiltered
    geometry pairs whose distance is less than or equal to
    ``max_distance``. The ordering of the results is guaranteed to be
    consistent -- for fixed geometry poses, the results will remain
    the same.

Raises:
    RuntimeError as indicated in the table above.

Warning:
    For Mesh shapes, their convex hulls are used in this query. It is
    not* computationally efficient or particularly accurate.)""";
        } ComputeSignedDistancePairwiseClosestPoints;
        // Symbol: drake::geometry::QueryObject::ComputeSignedDistanceToPoint
        struct /* ComputeSignedDistanceToPoint */ {
          // Source: drake/geometry/query_object.h
          const char* doc =
R"""(Computes the signed distances and gradients to a query point from each
geometry in the scene.

This query provides φᵢ(p), φᵢ:ℝ³→ℝ, the signed distance to the
position p of a query point from geometry Gᵢ in the scene. It returns
an array of the signed distances from all geometries.

Optionally you can specify a threshold distance that will filter out
any object beyond the threshold. By default, we report distances from
the query point to every object.

This query also provides the gradient vector ∇φᵢ(p) of the signed
distance function from geometry Gᵢ. Note that, in general, if p is
outside Gᵢ, then ∇φᵢ(p) equals the unit vector in the direction from
the nearest point Nᵢ on Gᵢ's surface to p. If p is inside Gᵢ, then
∇φᵢ(p) is in the direction from p to Nᵢ. This observation is written
formally as:

∇φᵢ(p) = (p - Nᵢ)/|p - Nᵢ| if p is outside Gᵢ

∇φᵢ(p) = -(p - Nᵢ)/|p - Nᵢ| if p is inside Gᵢ

Note that ∇φᵢ(p) is also defined on Gᵢ's surface, but we cannot use
the above formula.

**Characterizing the returned values**

This table is a variant of that described in this
query_object_precision_methodology "class's documentation". The query
evaluates signed distance between *one* shape and a point (in contrast
to other queries which involve two shapes). Therefore, we don't need a
matrix of shape pairs, but a list of shapes. Otherwise, the
methodology is the same as described, with the point being represented
as a zero-radius sphere.

| Scalar | Box | Capsule | Convex | Cylinder | Ellipsoid | HalfSpace |
Mesh | Sphere | | :--------: | :-----: | :------: | :-----: |
:-------: | :--------: | :--------: | :-----: | :-----: | | double |
2e-15 | 4e-15 | 6e-15 | 3e-15 | 3e-5ᵇ | 5e-15 | 6e-15ᶜ | 4e-15 | |
AutoDiffXd | 1e-15 | 7e-15 | ᵃ | ᵃ | ᵃ | 5e-15 | ᵃ | 3e-15 | |
Expression | ᵃ | ᵃ | ᵃ | ᵃ | ᵃ | ᵃ | ᵃ | ᵃ | ***Table 8***: Worst
observed error (in m) for 2mm penetration/separation between geometry
approximately 20cm in size and a point.

- ᵃ Unsupported geometry/scalar combinations are simply ignored; no results
are reported for that geometry.
- ᵇ This uses an *iterative* algorithm which introduces a relatively large
and variable error. For example, as the eccentricity of the ellipsoid
increases, this error may get worse. It also depends on the location of
the projection of the query point on the ellipsoid; the closer that point
is to the high curvature area, the bigger the effect. It is not
immediately clear how much worse the answer will get.
- ᶜ Only supports OBJ and tetrahedral VTK meshes. Unsupported meshes are
simply ignored; no results are reported for that geometry. For OBJ meshes
the surface mesh must satisfy specific requirements (see below). Unlike
the other Shapes, witness points and gradients can be discontinuous on a
mesh's exterior if it is non-convex.

Precondition:
    The Mesh of a triangular surface mesh must be a closed manifold
    without duplicate vertices or self-intersection, and every
    triangle's face winding gives an outward-pointing face normal.
    Drake does not currently validate the input mesh with respect to
    these properties. Instead, it does a good-faith computation
    assuming the properties, possibly returning incorrect results.
    Non-compliant meshes will introduce regions in which the query
    point will report the wrong sign (and, therefore, the wrong
    gradient) due to a misclassification of being inside or outside.
    This leads to discontinuities in the distance field across the
    boundaries of these regions; the distance sign will flip while the
    magnitude of the distance value is arbitrarily far away from zero.
    For open meshes, the same principle holds. The open mesh, which
    has no true concept of "inside", will nevertheless report some
    query points as being inside.

Precondition:
    The Mesh of a tetrahedral volume mesh has positive-volume
    tetrahedra, no duplicate vertices, no self-intersection, and any
    two tetrahedra intersect in a common triangular face, edge, or
    vertex or not at all. A "tetrahedron soup" is, in general,
    non-compliant to this condition. Violating meshes will introduce
    areas inside the volumes that are incorrectly treated as boundary
    surfaces. The query points near such problematic areas will report
    the wrong nearest points, distances, and gradients.

Note:
    For a sphere G, the signed distance function φᵢ(p) has an
    undefined gradient vector at the center of the sphere--every point
    on the sphere's surface has the same distance to the center. In
    this case, we will assign ∇φᵢ(p) the unit vector Gx (x-directional
    vector of G's frame) expressed in World frame.

Note:
    For a box, at a point p on an edge or a corner of the box, the
    signed distance function φᵢ(p) has an undefined gradient vector.
    In this case, we will assign a unit vector in the direction of the
    average of the outward face unit normals of the incident faces of
    the edge or the corner. A point p is considered being on a face,
    or an edge, or a corner of the box if it lies within a certain
    tolerance from them.

Note:
    For a box B, if a point p is inside the box, and it is equidistant
    to multiple nearest faces, the signed distance function φᵢ(p) at p
    will have an undefined gradient vector. There is a nearest point
    candidate associated with each nearest face. In this case, we
    arbitrarily pick the point Nᵢ associated with one of the nearest
    faces. Please note that, due to the possible round off error
    arising from applying a pose X_WG to B, there is no guarantee
    which of the nearest faces will be used.

Note:
    The signed distance function is a continuous function with respect
    to the position of the query point, but its gradient vector field
    may not be continuous. Specifically at a position equidistant to
    multiple nearest points, its gradient vector field is not
    continuous.

Note:
    For a convex object, outside the object at positive distance from
    the boundary, the signed distance function is smooth (having
    continuous first-order partial derivatives).

Parameter ``p_WQ``:
    Position of a query point Q in world frame W.

Parameter ``threshold``:
    We ignore any object beyond this distance. By default, it is
    infinity, so we report distances from the query point to every
    object.

Returns ``signed_distances``:
    A vector populated with per-object signed distance values (and
    supporting data) for every supported geometry as shown in the
    table. See SignedDistanceToPoint. The ordering of the results is
    guaranteed to be consistent -- for fixed geometry poses, the
    results will remain the same.

Raises:
    RuntimeError if there are meshes with extremely sharp features
    where the calculation of feature normals become unstable.)""";
        } ComputeSignedDistanceToPoint;
        // Symbol: drake::geometry::QueryObject::FindCollisionCandidates
        struct /* FindCollisionCandidates */ {
          // Source: drake/geometry/query_object.h
          const char* doc =
R"""(Applies a conservative culling mechanism to create a subset of all
possible geometry pairs based on non-zero intersections. A geometry
pair that is *absent* from the results is either a) culled by
collision filters or b) *known* to be separated. The caller is
responsible for confirming that the remaining, unculled geometry pairs
are *actually* in collision.

Returns:
    A vector populated with collision pair candidates (the order will
    remain constant for a fixed population but can change as geometry
    ids are added/removed).)""";
        } FindCollisionCandidates;
        // Symbol: drake::geometry::QueryObject::GetConfigurationsInWorld
        struct /* GetConfigurationsInWorld */ {
          // Source: drake/geometry/query_object.h
          const char* doc =
R"""(Reports the configuration of the deformable geometry indicated by
``deformable_geometry_id`` relative to the world frame.

See also:
    GetPoseInWorld().

Raises:
    RuntimeError if the geometry ``deformable_geometry_id`` is not
    valid or is not a deformable geometry.)""";
        } GetConfigurationsInWorld;
        // Symbol: drake::geometry::QueryObject::GetDrivenMeshConfigurationsInWorld
        struct /* GetDrivenMeshConfigurationsInWorld */ {
          // Source: drake/geometry/query_object.h
          const char* doc =
R"""(Reports the configurations of the driven meshes associated with the
given role for the deformable geometry indicated by
``deformable_geometry_id`` relative to the world frame if the
deformable geometry has that role.

Raises:
    RuntimeError if the geometry associated with
    ``deformable_geometry_id`` is not a registered deformable geometry
    with the given role.

Warning:
    This feature is considered to be **experimental** and may change
    or be removed at any time, without any deprecation notice ahead of
    time.)""";
        } GetDrivenMeshConfigurationsInWorld;
        // Symbol: drake::geometry::QueryObject::GetPoseInParent
        struct /* GetPoseInParent */ {
          // Source: drake/geometry/query_object.h
          const char* doc =
R"""(Reports the position of the frame indicated by ``frame_id`` relative
to its parent frame. If the frame was registered with the world frame
as its parent frame, this value will be identical to that returned by
GetPoseInWorld().

Raises:
    RuntimeError if the frame ``frame_id`` is not valid.)""";
        } GetPoseInParent;
        // Symbol: drake::geometry::QueryObject::GetPoseInWorld
        struct /* GetPoseInWorld */ {
          // Source: drake/geometry/query_object.h
          const char* doc_1args_frame_id =
R"""(Reports the position of the frame indicated by ``frame_id`` relative
to the world frame.

Raises:
    RuntimeError if the frame ``frame_id`` is not valid.)""";
          // Source: drake/geometry/query_object.h
          const char* doc_1args_geometry_id =
R"""(Reports the position of the frame of the rigid geometry indicated by
``geometry_id`` relative to the world frame (X_WG).

Note:
    This query is meaningless for deformable geometries. Their current
    state cannot be represented by a single rigid transformation.
    Instead, one should use GetConfigurationsInWorld() to get the
    current vertex positions of the deformable geometry in the world
    frame. On the other hand, it *is* meaningful to query the *fixed*
    pose of the *reference* geometry in its parent frame (see
    SceneGraphInspector∷GetPoseInFrame()).

Raises:
    RuntimeError if the geometry ``geometry_id`` is not valid or if it
    is deformable.)""";
        } GetPoseInWorld;
        // Symbol: drake::geometry::QueryObject::GetRenderEngineByName
        struct /* GetRenderEngineByName */ {
          // Source: drake/geometry/query_object.h
          const char* doc =
R"""(Returns the named render engine, if it exists. The RenderEngine is
guaranteed to be up to date w.r.t. the poses and data in the context.)""";
        } GetRenderEngineByName;
        // Symbol: drake::geometry::QueryObject::HasCollisions
        struct /* HasCollisions */ {
          // Source: drake/geometry/query_object.h
          const char* doc =
R"""(Reports true if there are *any* collisions between unfiltered pairs in
the world.

Warning:
    For Mesh shapes, their convex hulls are used in this query. It is
    not* computationally efficient or particularly accurate.)""";
        } HasCollisions;
        // Symbol: drake::geometry::QueryObject::QueryObject<T>
        struct /* ctor */ {
          // Source: drake/geometry/query_object.h
          const char* doc =
R"""(Constructs a default QueryObject (all pointers are null).)""";
        } ctor;
        // Symbol: drake::geometry::QueryObject::RenderColorImage
        struct /* RenderColorImage */ {
          // Source: drake/geometry/query_object.h
          const char* doc =
R"""(Renders an RGB image for the given ``camera`` posed with respect to
the indicated parent frame P.

Parameter ``camera``:
    The camera to render from.

Parameter ``parent_frame``:
    The id for the camera's parent frame.

Parameter ``X_PC``:
    The pose of the camera body in the parent frame.

Parameter ``color_image_out``:
    The rendered color image.)""";
        } RenderColorImage;
        // Symbol: drake::geometry::QueryObject::RenderDepthImage
        struct /* RenderDepthImage */ {
          // Source: drake/geometry/query_object.h
          const char* doc =
R"""(Renders a depth image for the given ``camera`` posed with respect to
the indicated parent frame P.

In contrast to the other rendering methods, rendering depth images
doesn't provide the option to display the window; generally, basic
depth images are not readily communicative to humans.

Parameter ``camera``:
    The camera to render from.

Parameter ``parent_frame``:
    The id for the camera's parent frame.

Parameter ``X_PC``:
    The pose of the camera body in the parent frame.

Parameter ``depth_image_out``:
    The rendered depth image.)""";
        } RenderDepthImage;
        // Symbol: drake::geometry::QueryObject::RenderLabelImage
        struct /* RenderLabelImage */ {
          // Source: drake/geometry/query_object.h
          const char* doc =
R"""(Renders a label image for the given ``camera`` posed with respect to
the indicated parent frame P.

Parameter ``camera``:
    The camera to render from.

Parameter ``parent_frame``:
    The id for the camera's parent frame.

Parameter ``X_PC``:
    The pose of the camera body in the parent frame.

Parameter ``label_image_out``:
    The rendered label image.)""";
        } RenderLabelImage;
        // Symbol: drake::geometry::QueryObject::inspector
        struct /* inspector */ {
          // Source: drake/geometry/query_object.h
          const char* doc =
R"""(Provides an inspector for the topological structure of the underlying
scene graph data (see SceneGraphInspector for details).)""";
        } inspector;
      } QueryObject;
      // Symbol: drake::geometry::ReadGltfToMemory
      struct /* ReadGltfToMemory */ {
        // Source: drake/geometry/read_gltf_to_memory.h
        const char* doc =
R"""(Given a file path to a .gltf file, loads the .gltf file contents into
memory. All named .bin and image files are loaded into its supporting
files as path-valued FileSource instances (i.e., absolute paths for
the external files are included).

Note: the path-valued supporting files are not validated with respect
to their accessibility or even their existence.)""";
      } ReadGltfToMemory;
      // Symbol: drake::geometry::Rgba
      struct /* Rgba */ {
        // Source: drake/geometry/rgba.h
        const char* doc =
R"""(Defines RGBA (red, green, blue, alpha) values on the range [0, 1].)""";
        // Symbol: drake::geometry::Rgba::AlmostEqual
        struct /* AlmostEqual */ {
          // Source: drake/geometry/rgba.h
          const char* doc =
R"""(Reports if two Rgba values are equal within a given absolute
``tolerance``. They are "equal" so long as the difference in no single
channel is larger than the specified ``tolerance``.)""";
        } AlmostEqual;
        // Symbol: drake::geometry::Rgba::Rgba
        struct /* ctor */ {
          // Source: drake/geometry/rgba.h
          const char* doc_0args =
R"""(Default constructor produces fully opaque white.)""";
          // Source: drake/geometry/rgba.h
          const char* doc_4args =
R"""(Constructs with given (r, g, b, a) values.

Precondition:
    All values are within the range of [0, 1].)""";
        } ctor;
        // Symbol: drake::geometry::Rgba::Serialize
        struct /* Serialize */ {
          // Source: drake/geometry/rgba.h
          const char* doc =
R"""(Passes this object to an Archive.

In YAML, an Rgba is represented by a mapping of the symbol ``rgba`` to
an array-like list of three or four numbers. E.g.,

rgba: [0.5, 0.5, 1.0]

or

rgba: [0.5, 0.5, 1.0, 0.5]

such that the first three values are red, green, and blue,
respectively. If no fourth value is provided, alpha is defined a 1.0.

When another struct has an Rgba-valued member (e.g.,
systems∷sensors∷CameraConfig∷background), remember to include the full
mapping. For example, imagine the struct:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    {cpp}
    struct Foo {
    Rgba color1;
    Rgba color2;
    };

.. raw:: html

    </details>

The correct yaml representation of this would be:


.. code-block:: yaml

    color1:
    rgba: [0.5, 0.5, 1.0]
    color2:
    rgba: [1.0, 0.0, 0.0, 0.5]

The *values* of ``color1`` and ``color2`` are the mapping from
``rgba`` to the desired color tuples.

Refer to yaml_serialization "YAML Serialization" for background.)""";
        } Serialize;
        // Symbol: drake::geometry::Rgba::a
        struct /* a */ {
          // Source: drake/geometry/rgba.h
          const char* doc = R"""(Alpha.)""";
        } a;
        // Symbol: drake::geometry::Rgba::b
        struct /* b */ {
          // Source: drake/geometry/rgba.h
          const char* doc = R"""(Blue.)""";
        } b;
        // Symbol: drake::geometry::Rgba::g
        struct /* g */ {
          // Source: drake/geometry/rgba.h
          const char* doc = R"""(Green.)""";
        } g;
        // Symbol: drake::geometry::Rgba::operator!=
        struct /* operator_ne */ {
          // Source: drake/geometry/rgba.h
          const char* doc = R"""()""";
        } operator_ne;
        // Symbol: drake::geometry::Rgba::operator*
        struct /* operator_mul */ {
          // Source: drake/geometry/rgba.h
          const char* doc =
R"""(Computes the element-wise product of two rgba colors. This type of
calculation is frequently used to modulate one color with another
(e.g., for lighting or texturing).)""";
        } operator_mul;
        // Symbol: drake::geometry::Rgba::r
        struct /* r */ {
          // Source: drake/geometry/rgba.h
          const char* doc = R"""(Red.)""";
        } r;
        // Symbol: drake::geometry::Rgba::rgba
        struct /* rgba */ {
          // Source: drake/geometry/rgba.h
          const char* doc = R"""(Returns all four elements in order.)""";
        } rgba;
        // Symbol: drake::geometry::Rgba::scale_rgb
        struct /* scale_rgb */ {
          // Source: drake/geometry/rgba.h
          const char* doc =
R"""(Computes a new Rgba color by multiplying the color channels (rgb) by
the given scalar ``scalar``. All resultant channel values saturate at
one. The result has ``this`` Rgba's alpha values.

Precondition:
    scale >= 0.)""";
        } scale_rgb;
        // Symbol: drake::geometry::Rgba::set
        struct /* set */ {
          // Source: drake/geometry/rgba.h
          const char* doc_4args =
R"""(Sets (r, g, b, a) values.

Raises:
    RuntimeError if any values are outside of the range [0, 1].)""";
          // Source: drake/geometry/rgba.h
          const char* doc_1args =
R"""(Sets an (r, g, b, a) from a vector.

Raises:
    RuntimeError if the vector is not size 3 or 4.

Raises:
    RuntimeError if any values are outside of the range [0, 1].)""";
        } set;
        // Symbol: drake::geometry::Rgba::to_string
        struct /* to_string */ {
          // Source: drake/geometry/rgba.h
          const char* doc =
R"""(Converts the Rgba value to a string representation.)""";
        } to_string;
        // Symbol: drake::geometry::Rgba::update
        struct /* update */ {
          // Source: drake/geometry/rgba.h
          const char* doc =
R"""(Updates individual (r, g, b, a) values; any values not provided will
remain unchanged.

Raises:
    RuntimeError if any values are outside of the range [0, 1].)""";
        } update;
      } Rgba;
      // Symbol: drake::geometry::Role
      struct /* Role */ {
        // Source: drake/geometry/geometry_roles.h
        const char* doc =
R"""(General enumeration for indicating geometry role.)""";
        // Symbol: drake::geometry::Role::kIllustration
        struct /* kIllustration */ {
          // Source: drake/geometry/geometry_roles.h
          const char* doc = R"""()""";
        } kIllustration;
        // Symbol: drake::geometry::Role::kPerception
        struct /* kPerception */ {
          // Source: drake/geometry/geometry_roles.h
          const char* doc = R"""()""";
        } kPerception;
        // Symbol: drake::geometry::Role::kProximity
        struct /* kProximity */ {
          // Source: drake/geometry/geometry_roles.h
          const char* doc = R"""()""";
        } kProximity;
        // Symbol: drake::geometry::Role::kUnassigned
        struct /* kUnassigned */ {
          // Source: drake/geometry/geometry_roles.h
          const char* doc = R"""()""";
        } kUnassigned;
      } Role;
      // Symbol: drake::geometry::RoleAssign
      struct /* RoleAssign */ {
        // Source: drake/geometry/geometry_roles.h
        const char* doc =
R"""(The operations that can be performed on the given properties when
assigning roles to geometry.)""";
        // Symbol: drake::geometry::RoleAssign::kNew
        struct /* kNew */ {
          // Source: drake/geometry/geometry_roles.h
          const char* doc =
R"""(Assign the properties to a geometry that doesn't already have the
role.)""";
        } kNew;
        // Symbol: drake::geometry::RoleAssign::kReplace
        struct /* kReplace */ {
          // Source: drake/geometry/geometry_roles.h
          const char* doc =
R"""(Replace the existing role properties completely.)""";
        } kReplace;
      } RoleAssign;
      // Symbol: drake::geometry::SceneGraph
      struct /* SceneGraph */ {
        // Source: drake/geometry/scene_graph.h
        const char* doc =
R"""(SceneGraph serves as the nexus for all geometry (and geometry-based
operations) in a Diagram. Through SceneGraph, other systems that
introduce geometry can *register* that geometry as part of a common
global domain, including it in geometric queries (e.g., cars
controlled by one LeafSystem can be observed by a different sensor
system). SceneGraph provides the interface for registering the
geometry, updating its position based on the current context, and
performing geometric queries.

.. pydrake_system::

    name: SceneGraph
    input_ports:
    - <em style="color:gray">(source name)</em>_pose
    - <em style="color:gray">(source name)</em>_configuration
    output_ports:
    - query

For each registered "geometry source", there is an input port whose
name begins with <em style="color:gray">(source name)</em>.

Only registered "geometry sources" can introduce geometry into
SceneGraph. Geometry sources will typically be other leaf systems,
but, in the case of *anchored* (i.e., stationary) geometry, it could
also be some other block of code (e.g., adding a common ground plane
with which all systems' geometries interact). For dynamic geometry
(geometry whose pose depends on a Context), the geometry source must
also provide pose/configuration values for all of the geometries the
source owns, via a port connection on SceneGraph. For N geometry
sources, the SceneGraph instance will have N pose/configuration input
ports.

The basic workflow for interacting with SceneGraph is:

- Register as a geometry source, acquiring a unique SourceId.
- Register geometry (anchored and dynamic) with the system.
- Connect source's geometry output ports to the corresponding SceneGraph
input ports.
- Implement appropriate ``Calc*`` methods on the geometry output ports to
update geometry pose/configuration values.

Inputs
======

For each registered geometry source, there is one input port for each
order of kinematics values (i.e., pose and configuration). If a source
registers a frame or a deformable geometry, it must connect to the
corresponding ports. Failure to connect to the ports (or to provide
valid kinematics values) will lead to runtime exceptions.

**pose port**: An abstract-valued port providing an instance of
FramePoseVector. For each registered frame, this "pose vector" maps
the registered FrameId to a pose value. All registered frames must be
accounted for and only frames registered by a source can be included
in its output port. See the details in KinematicsVector for details on
how to provide values for this port.

**configuration port**: An abstract-valued port providing an instance
of GeometryConfigurationVector. For each registered deformable
geometry, this "configuration vector" maps the registered GeometryId
to its world space configuration (i.e. the vertex positions of its
mesh representation in the world frame). All registered deformable
geometries must be accounted for and only geometries registered by a
source can be included in its output port.

Outputs
=======

SceneGraph has one output port:

**query port**: An abstract-valued port containing an instance of
QueryObject. To perform geometric queries, downstream LeafSystem
instances acquire the QueryObject from SceneGraph's output port and
invoke the appropriate methods on it. Use get_query_output_port() to
acquire the output port for the query handle.

Working with SceneGraph
=======================

LeafSystem instances can relate to SceneGraph in one of two ways: as a
*consumer* that performs queries, or as a *producer* that introduces
geometry into the shared world and defines its context-dependent
kinematics values. It is reasonable for systems to perform either role
singly, or both.

**Consumer**

Consumers perform geometric queries upon the world geometry.
SceneGraph *serves* those queries. As indicated above, in order for a
LeafSystem to act as a consumer, it must: 1. define a
QueryObject-valued input port and connect it to SceneGraph's
corresponding output port, and 2. have a reference to the connected
SceneGraph instance.

With those two requirements satisfied, a LeafSystem can perform
geometry queries by: 1. evaluating the QueryObject input port,
retrieving a ``const QueryObject&`` in return, and 2. invoking the
appropriate method on the QueryObject.

**Producer**

All producers introduce geometry into the shared geometric world. This
is called *registering* geometry. Depending on what exactly has been
registered, a producer may also have to *update kinematics*. Producers
themselves must be registered with SceneGraph as producers (a.k.a.
*geometry sources*). They do this by acquiring a SourceId (via
SceneGraph∷RegisterSource()). The SourceId serves as a unique handle
through which the producer's identity is validated.

*Registering Geometry*

SceneGraph cannot know what geometry *should* be part of the shared
world. Other systems are responsible for introducing geometry into the
world. This process (defining geometry and informing SceneGraph) is
called *registering* the geometry. Geometry can be registered as
*anchored* or *dynamic*, and is always registered to (associated with)
a SourceId.

Dynamic geometry can move; more specifically, its kinematics (e.g.,
pose) depends on a system's Context. Particularly, a non-deformable
dynamic geometry is *fixed* to a *frame* whose kinematics values
depend on a context. As the frame moves, the geometries fixed to it
move with it. On the other hand, a deformable dynamic geometry has a
mesh representation whose vertices' positions can change and are
expressed in the frame it is registered in. Therefore, to register
dynamic geometry a frame must be registered first. These registered
frames serve as the basis for repositioning geometry in the shared
world. The geometry source is responsible for providing up-to-date
kinematics values for those registered frames upon request (via an
appropriate output port on the source LeafSystem connecting to the
appropriate input port on SceneGraph). The geometry source that
registers deformable geometry is also responsible to provide the
positions of the mesh vertices of the deformable geometry in the
registered-in frame. The work flow is as follows: 1. A LeafSystem
registers itself as a geometry source, acquiring a SourceId
(RegisterSource()). 2. The source registers a frame
(GeometrySource∷RegisterFrame()). - A frame always has a "parent"
frame. It can implicitly be the world frame, *or* another frame
registered by the source. 3. Register one or more non-deformable
geometries to a frame (RegisterGeometry()), and/or one or more
deformable geometries to a frame (RegisterDeformableGeometry()). - A
non-deformable geometry's pose is relative to the frame to which the
geometry is fixed. For deformable geometries, the positions of their
mesh vertices are expressed in the registered-in frame. - Rigid
geometries can also be posed relative to another registered geometry.
It will be affixed to *that* geometry's frame.

Anchored geometry is *independent* of the context (i.e., it doesn't
move). Anchored geometries are always affixed to the immobile world
frame. As such, registering a frame is *not* required for registering
anchored geometry (see GeometrySource∷RegisterAnchoredGeometry()).
However, the source still "owns" the anchored geometry.

*Updating Kinematics*

Registering *dynamic* non-deformable geometry implies a contract
between the geometry source and SceneGraph. The geometry source must
do the following: - It must provide, populate, and connect two output
ports: the "id" port and the "pose" port. - The id port must contain
*all* the frame ids returned as a result of frame registration. - The
pose port must contain one pose per registered frame; the pose value
is expressed relative to the registered frame's *parent* frame. As
mentioned above, the iᵗʰ pose value should describe the frame
indicated by the iᵗʰ id in the id output port.

Similarly, if it registers deformable geometries, the geometry source
must provide, populate, and connect the "configuration" port. The
configuration port must contain a vector of vertex positions per
registered deformable geometry.

Failure to meet these requirements will lead to a run-time error.

Model versus Context
====================

Many (and eventually all) methods that configure the population of
SceneGraph have two variants that differ by whether they accept a
mutable Context or not. When no Context is provided, *this* SceneGraph
instance's underlying model is modified. When the SceneGraph instance
allocates a context, its model is copied into that context.

The second variant causes SceneGraph to modify the data stored in the
provided Context to be modified *instead of the internal model*.

The two interfaces *can* be used interchangeably. However,
modifications to ``this`` SceneGraph's underlying model will *not*
affect previously allocated Context instances. A new Context should be
allocated after modifying the model.

Note:
    In this initial version, the only methods with the
    Context-modifying variant are those methods that *do not* change
    the semantics of the input or output ports. Modifications that
    make such changes must be coordinated across systems.

Detecting changes
=================

The geometry data associated with SceneGraph is coarsely versioned.
Consumers of the geometry can query for the version of the data and
recognize if the data has been modified since last examined.

The versioning is associated with geometry roles: proximity,
illustration, and perception; each role has its own, independent
version. Any operation that affects geometry with one of those roles
will modify the corresponding version. For example:

In C++:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    // Does *not* modify any version; no roles have been assigned.
    const GeometryId geometry_id = scene_graph.RegisterGeometry(
    source_id, frame_id, make_unique<GeometryInstance>(...));
    // Modifies the proximity version.
    scene_graph.AssignRole(source_id, geometry_id, ProximityProperties());
    // Modifies the illustration version.
    scene_graph.AssignRole(source_id, geometry_id, IllustrationProperties());
    // Modifies the perception version if there exists a renderer that accepts the
    // geometry.
    scene_graph.AssignRole(source_id, geometry_id, PerceptionProperties());
    // Modifies the illustration version.
    scene_graph.RemoveRole(source_id, geometry_id, Role∷kIllustration);
    // Modifies proximity version and perception version if the geometry is
    // registered with any renderer.
    scene_graph.RemoveGeometry(source_id, geometry_id);

.. raw:: html

    </details>

In Python:


.. code-block:: python

    # Does *not* modify any version; no roles have been assigned.
    geometry_id = scene_graph.RegisterGeometry(
    source_id, frame_id, GeometryInstance(...))
    # Modifies the proximity version.
    scene_graph.AssignRole(source_id, geometry_id, ProximityProperties())
    # Modifies the illustration version.
    scene_graph.AssignRole(source_id, geometry_id, IllustrationProperties())
    # Modifies the perception version if there exists a renderer that accepts the
    # geometry.
    scene_graph.AssignRole(source_id, geometry_id, PerceptionProperties())
    # Modifies the illustration version.
    scene_graph.RemoveRole(source_id, geometry_id, Role.kIllustration)
    # Modifies proximity version and perception version if the geometry is
    # registered with any renderer.
    scene_graph.RemoveGeometry(source_id, geometry_id)

Each copy of geometry data maintains its own set of versions.
SceneGraph's model has its own version, and that version is the same
as the version in the Context provided by
SceneGraph∷CreateDefaultContext(). Modifications to the geometry data
contained in a Context modifies *that* data's version, but the
original model data's version is unmodified, reflecting the unchanged
model data.

The geometry data's version is accessed via a SceneGraphInspector
instance. model_inspector() will give access to SceneGraph's model
version. And QueryObject∷inspector() will give access to the geometry
data stored in a Context.

Current versions can be compared against previously examined versions.
If the versions match, then the geometry data is guaranteed to be the
same. If they don't match, that indicates that the two sets of data
underwent different revision processes. That, however, doesn't
necessarily imply that the two sets of data are distinct. In other
words, the versioning will report a difference unless it can guarantee
equivalence.

It is possible that two different contexts have different versions and
a downstream system can be evaluated with each context alternatingly.
If the system behavior depends on the geometry version, this will
cause it to thrash whatever components depends on geometry version.
The system should *clearly* document this fact.)""";
        // Symbol: drake::geometry::SceneGraph::AddRenderer
        struct /* AddRenderer */ {
          // Source: drake/geometry/scene_graph.h
          const char* doc_2args =
R"""(Adds a new render engine to this SceneGraph. The render engine's name
should be referenced in the render∷ColorRenderCamera
"ColorRenderCamera" or render∷DepthRenderCamera "DepthRenderCamera"
provided in the render queries (see QueryObject∷RenderColorImage() as
an example).

There is no restriction on when a renderer is added relative to
geometry registration and role assignment. Given a representative
sequence of registration and perception role assignment, the addition
of the renderer can be introduced anywhere in the sequence and the end
result would be the same.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    GeometryId id1 = scene_graph.RegisterGeometry(source_id, ...);
    scene_graph.AssignRole(source_id, id1, PerceptionProperties());
    GeometryId id2 = scene_graph.RegisterGeometry(source_id, ...);
    scene_graph.AssignRole(source_id, id2, PerceptionProperties());
    GeometryId id3 = scene_graph.RegisterGeometry(source_id, ...);
    scene_graph.AssignRole(source_id, id3, PerceptionProperties());

.. raw:: html

    </details>

Modifies the perception version if ``renderer`` accepts any previously
existing geometries (see scene_graph_versioning).

Parameter ``name``:
    The unique name of the renderer.

Parameter ``renderer``:
    The ``renderer`` to add. (It will be copied / cloned, which means
    the lifetime of the argument need not extend past this call.)

Raises:
    RuntimeError if the name is not unique.)""";
          // Source: drake/geometry/scene_graph.h
          const char* doc_3args =
R"""(systems∷Context-modifying variant of AddRenderer(). Rather than
modifying SceneGraph's model, it modifies the copy of the model stored
in the provided context.)""";
        } AddRenderer;
        // Symbol: drake::geometry::SceneGraph::AssignRole
        struct /* AssignRole */ {
          // Source: drake/geometry/scene_graph.h
          const char* doc_proximity_direct =
R"""(Assigns the proximity role to the geometry indicated by
``geometry_id``. Modifies the proximity version (see
scene_graph_versioning).)""";
          // Source: drake/geometry/scene_graph.h
          const char* doc_proximity_context =
R"""(systems∷Context-modifying variant of
AssignRole(SourceId,GeometryId,ProximityProperties) "AssignRole()" for
proximity properties. Rather than modifying SceneGraph's model, it
modifies the copy of the model stored in the provided context.)""";
          // Source: drake/geometry/scene_graph.h
          const char* doc_perception_direct =
R"""(Assigns the perception role to the geometry indicated by
``geometry_id``.

By default, a geometry with a perception role will be reified by all
render∷RenderEngine instances. This behavior can be changed. Renderers
can be explicitly whitelisted via the ('renderer', 'accepting')
perception property. Its type is std∷set<std∷string> and it contains
the names of all the renderers that *may* reify it. If no property is
defined (or an empty set is given), then the default behavior of all
renderers attempting to reify it will be restored. Modifies the
perception version if the geometry is added to any renderer (see
scene_graph_versioning).)""";
          // Source: drake/geometry/scene_graph.h
          const char* doc_perception_context =
R"""(systems∷Context-modifying variant of
AssignRole(SourceId,GeometryId,PerceptionProperties) "AssignRole()"
for perception properties. Rather than modifying SceneGraph's model,
it modifies the copy of the model stored in the provided context.)""";
          // Source: drake/geometry/scene_graph.h
          const char* doc_illustration_direct =
R"""(Assigns the illustration role to the geometry indicated by
``geometry_id``. Modifies the illustration version (see
scene_graph_versioning).

Warning:
    When changing illustration properties (``assign =
    RoleAssign∷kReplace``), there is no guarantee that these changes
    will affect the visualization. The visualizer needs to be able to
    "initialize" itself after changes to properties that will affect
    how a geometry appears. If changing a geometry's illustration
    properties doesn't seem to be affecting the visualization,
    retrigger its initialization action.)""";
          // Source: drake/geometry/scene_graph.h
          const char* doc_illustration_context =
R"""(systems∷Context-modifying variant of
AssignRole(SourceId,GeometryId,IllustrationProperties) "AssignRole()"
for illustration properties. Rather than modifying SceneGraph's model,
it modifies the copy of the model stored in the provided context.

Warning:
    When changing illustration properties (``assign =
    RoleAssign∷kReplace``), there is no guarantee that these changes
    will affect the visualization. The visualizer needs to be able to
    "initialize" itself after changes to properties that will affect
    how a geometry appears. If changing a geometry's illustration
    properties doesn't seem to be affecting the visualization,
    retrigger its initialization action.)""";
        } AssignRole;
        // Symbol: drake::geometry::SceneGraph::ChangeShape
        struct /* ChangeShape */ {
          // Source: drake/geometry/scene_graph.h
          const char* doc_model =
R"""(Changes the ``shape`` of the geometry indicated by the given
``geometry_id``.

The geometry is otherwise unchanged -- same geometry_id, same assigned
roles, same pose with respect to the parent (unless a new value for
``X_FG`` is given).

This method modifies the underlying model and requires a new Context
to be allocated. Potentially modifies proximity, perception, and
illustration versions based on the roles assigned to the geometry (see
scene_graph_versioning).

Parameter ``source_id``:
    The id for the source modifying the geometry.

Parameter ``geometry_id``:
    The id for the geometry whose shape is being modified.

Parameter ``shape``:
    The new shape to use.

Parameter ``X_FG``:
    The (optional) new pose of the geometry in its frame. If omitted,
    the old pose is used.

Raises:
    RuntimeError if a) the ``source_id`` does *not* map to a
    registered source, b) the ``geometry_id`` does not map to a valid
    geometry, c) the ``geometry_id`` maps to a geometry that does not
    belong to the indicated source, or d) the geometry is deformable.)""";
          // Source: drake/geometry/scene_graph.h
          const char* doc_context =
R"""(systems∷Context-modifying variant of ChangeShape(). Rather than
modifying SceneGraph's model, it modifies the copy of the model stored
in the provided context.)""";
        } ChangeShape;
        // Symbol: drake::geometry::SceneGraph::GetRendererParameterYaml
        struct /* GetRendererParameterYaml */ {
          // Source: drake/geometry/scene_graph.h
          const char* doc_1args =
R"""(Creates a Yaml-formatted string representing the named engine's
parameters. The YAML will be prefixed with the paramater type's name,
e.g:

RenderEngineVtkParams: default_diffuse: [1, 1, 1] ...

If no registered engine has the given ``name``, the returned string is
empty.)""";
          // Source: drake/geometry/scene_graph.h
          const char* doc_2args =
R"""(systems∷Context-query variant of GetRendererParameterYaml(). Rather
than querying SceneGraph's model, it queries the copy of the model
stored in the provided context.)""";
        } GetRendererParameterYaml;
        // Symbol: drake::geometry::SceneGraph::GetRendererTypeName
        struct /* GetRendererTypeName */ {
          // Source: drake/geometry/scene_graph.h
          const char* doc_1args =
R"""(Reports the type name for the RenderEngine registered with the given
``name``.

Returns:
    the name of the RenderEngine's most derived type (as produced by
    NiceTypeName∷Get()). An empty string if there is no RenderEngine
    registered with the given ``name``.)""";
          // Source: drake/geometry/scene_graph.h
          const char* doc_2args =
R"""(systems∷Context-query variant of GetRendererTypeName(). Rather than
querying SceneGraph's model, it queries the copy of the model stored
in the provided context.)""";
        } GetRendererTypeName;
        // Symbol: drake::geometry::SceneGraph::HasRenderer
        struct /* HasRenderer */ {
          // Source: drake/geometry/scene_graph.h
          const char* doc_1args =
R"""(Reports true if this SceneGraph has a renderer registered with the
given name.)""";
          // Source: drake/geometry/scene_graph.h
          const char* doc_2args =
R"""(systems∷Context-query variant of HasRenderer(). Rather than querying
SceneGraph's model, it queries the copy of the model stored in the
provided context.)""";
        } HasRenderer;
        // Symbol: drake::geometry::SceneGraph::RegisterAnchoredGeometry
        struct /* RegisterAnchoredGeometry */ {
          // Source: drake/geometry/scene_graph.h
          const char* doc =
R"""(Registers a new *anchored* geometry G for this source. This hangs
geometry G from the world frame (W). Its pose is defined in that frame
(i.e., ``X_WG``). Returns the corresponding unique geometry id.

Roles will be assigned to the registered geometry if the corresponding
GeometryInstance ``geometry`` has had properties assigned.

This method modifies the underlying model and requires a new Context
to be allocated. Potentially modifies proximity, perception, and
illustration versions based on the roles assigned to the geometry (see
scene_graph_versioning).

Parameter ``source_id``:
    The id for the source registering the frame.

Parameter ``geometry``:
    The anchored geometry G to add to the world.

Returns:
    A unique identifier for the added geometry.

Raises:
    RuntimeError if a) the ``source_id`` does *not* map to a
    registered source or b) the geometry's name doesn't satisfy the
    requirements outlined in GeometryInstance.)""";
        } RegisterAnchoredGeometry;
        // Symbol: drake::geometry::SceneGraph::RegisterDeformableGeometry
        struct /* RegisterDeformableGeometry */ {
          // Source: drake/geometry/scene_graph.h
          const char* doc_4args =
R"""(Registers a new deformable geometry G for this source. This registers
geometry G on a frame F (indicated by ``frame_id``). The registered
geometry has a meshed representation. The positions of the vertices of
this mesh representation are defined in the frame F (i.e., ``q_FG``).
Returns the corresponding unique geometry id.

Roles will be assigned to the registered geometry if the corresponding
GeometryInstance ``geometry`` has had properties assigned.

This method modifies the underlying model and requires a new Context
to be allocated. Potentially modifies proximity, perception, and
illustration versions based on the roles assigned to the geometry (see
scene_graph_versioning).

Warning:
    This feature is considered to be **experimental** and may change
    or be removed at any time, without any deprecation notice ahead of
    time.

Parameter ``source_id``:
    The id for the source registering the geometry.

Parameter ``frame_id``:
    The id for the frame F to put the geometry in.

Parameter ``geometry``:
    The geometry G to to be represented in frame F.

Parameter ``resolution_hint``:
    The parameter that guides the level of mesh refinement of the
    deformable geometry. It has length units (in meters) and roughly
    corresponds to a typical edge length in the resulting mesh for a
    primitive shape.

Returns:
    A unique identifier for the added geometry.

Precondition:
    resolution_hint > 0.

Raises:
    RuntimeError if a) the ``source_id`` does *not* map to a
    registered source, b) frame_id != world_frame_id(), c) the
    ``geometry`` is equal to ``nullptr``, d) the geometry's name
    doesn't satisfy the requirements outlined in GeometryInstance.)""";
          // Source: drake/geometry/scene_graph.h
          const char* doc_5args =
R"""(systems∷Context-modifying variant of RegisterDeformableGeometry().
Rather than modifying SceneGraph's model, it modifies the copy of the
model stored in the provided context.

Warning:
    This feature is considered to be **experimental** and may change
    or be removed at any time, without any deprecation notice ahead of
    time.)""";
        } RegisterDeformableGeometry;
        // Symbol: drake::geometry::SceneGraph::RegisterFrame
        struct /* RegisterFrame */ {
          // Source: drake/geometry/scene_graph.h
          const char* doc_2args =
R"""(Registers a new frame F for this source. This hangs frame F on the
world frame (W). Its pose is defined relative to the world frame (i.e,
``X_WF``). Returns the corresponding unique frame id.

This method modifies the underlying model and requires a new Context
to be allocated.

Parameter ``source_id``:
    The id for the source registering the frame.

Parameter ``frame``:
    The frame to register.

Returns:
    A unique identifier for the added frame.

Raises:
    RuntimeError if a) the ``source_id`` does *not* map to a
    registered source, b) ``frame`` has an id that has already been
    registered, or c) there is already a frame with the same name
    registered for the source.)""";
          // Source: drake/geometry/scene_graph.h
          const char* doc_3args =
R"""(Registers a new frame F for this source. This hangs frame F on another
previously registered frame P (indicated by ``parent_id``). The pose
of the new frame is defined relative to the parent frame (i.e.,
``X_PF``). Returns the corresponding unique frame id.

This method modifies the underlying model and requires a new Context
to be allocated.

Parameter ``source_id``:
    The id for the source registering the frame.

Parameter ``parent_id``:
    The id of the parent frame P.

Parameter ``frame``:
    The frame to register.

Returns:
    A unique identifier for the added frame.

Raises:
    RuntimeError if a) the ``source_id`` does *not* map to a
    registered source, b) the ``parent_id`` does *not* map to a known
    frame or does not belong to the source, c) ``frame`` has an id
    that has already been registered, or d) there is already a frame
    with the same name registered for the source.)""";
        } RegisterFrame;
        // Symbol: drake::geometry::SceneGraph::RegisterGeometry
        struct /* RegisterGeometry */ {
          // Source: drake/geometry/scene_graph.h
          const char* doc_3args =
R"""(Registers a new rigid geometry G for this source. This hangs geometry
G on a previously registered frame F (indicated by ``frame_id``). The
pose of the geometry is defined in a fixed pose relative to F (i.e.,
``X_FG``). Returns the corresponding unique geometry id.

Roles will be assigned to the registered geometry if the corresponding
GeometryInstance ``geometry`` has had properties assigned.

This method modifies the underlying model and requires a new Context
to be allocated. Potentially modifies proximity, perception, and
illustration versions based on the roles assigned to the geometry (see
scene_graph_versioning).

Parameter ``source_id``:
    The id for the source registering the geometry.

Parameter ``frame_id``:
    The id for the frame F to hang the geometry on.

Parameter ``geometry``:
    The geometry G to affix to frame F.

Returns:
    A unique identifier for the added geometry.

Raises:
    RuntimeError if a) the ``source_id`` does *not* map to a
    registered source, b) the ``frame_id`` doesn't belong to the
    source, c) the ``geometry`` is equal to ``nullptr``, or d) the
    geometry's name doesn't satisfy the requirements outlined in
    GeometryInstance.)""";
          // Source: drake/geometry/scene_graph.h
          const char* doc_4args =
R"""(systems∷Context-modifying variant of RegisterGeometry(). Rather than
modifying SceneGraph's model, it modifies the copy of the model stored
in the provided context.)""";
        } RegisterGeometry;
        // Symbol: drake::geometry::SceneGraph::RegisterSource
        struct /* RegisterSource */ {
          // Source: drake/geometry/scene_graph.h
          const char* doc =
R"""(Registers a new, named source to the geometry system. The caller must
save the returned SourceId; it is the token by which all other
operations on the geometry world are conducted.

This source id can be used to register arbitrary *anchored* geometry.
But if dynamic non-deformable geometry is registered (via
RegisterGeometry/RegisterFrame), then the Context-dependent pose
values must be provided on an input port. See get_source_pose_port().

Similarly, if deformable geometry (always dynamic) is registered (via
RegisterDeformableGeometry), then the Context-dependent configuration
values must be provided on an input port. See
get_source_configuration_port().

This method modifies the underlying model and requires a new Context
to be allocated.

Parameter ``name``:
    The optional name of the source. If none is provided (or the empty
    string) a default name will be defined by SceneGraph's logic.

Raises:
    RuntimeError if the name is not unique.)""";
        } RegisterSource;
        // Symbol: drake::geometry::SceneGraph::RegisteredRendererNames
        struct /* RegisteredRendererNames */ {
          // Source: drake/geometry/scene_graph.h
          const char* doc_0args =
R"""(Reports the names of all registered renderers.)""";
          // Source: drake/geometry/scene_graph.h
          const char* doc_1args =
R"""(systems∷Context-query variant of RegisteredRendererNames(). Rather
than querying SceneGraph's model, it queries the copy of the model
stored in the provided context.)""";
        } RegisteredRendererNames;
        // Symbol: drake::geometry::SceneGraph::RemoveGeometry
        struct /* RemoveGeometry */ {
          // Source: drake/geometry/scene_graph.h
          const char* doc_2args =
R"""(Removes the given geometry G (indicated by ``geometry_id``) from the
given source's registered geometries. All registered geometries
hanging from this geometry will also be removed.

This method modifies the underlying model and requires a new Context
to be allocated. Potentially modifies proximity, perception, and
illustration versions based on the roles assigned to the geometry (see
scene_graph_versioning).

Parameter ``source_id``:
    The identifier for the owner geometry source.

Parameter ``geometry_id``:
    The identifier of the geometry to remove (can be dynamic or
    anchored).

Raises:
    RuntimeError if a) the ``source_id`` does *not* map to a
    registered source, b) the ``geometry_id`` does not map to a valid
    geometry, or c) the ``geometry_id`` maps to a geometry that does
    not belong to the indicated source.)""";
          // Source: drake/geometry/scene_graph.h
          const char* doc_3args =
R"""(systems∷Context-modifying variant of RemoveGeometry(). Rather than
modifying SceneGraph's model, it modifies the copy of the model stored
in the provided context.)""";
        } RemoveGeometry;
        // Symbol: drake::geometry::SceneGraph::RemoveRenderer
        struct /* RemoveRenderer */ {
          // Source: drake/geometry/scene_graph.h
          const char* doc_1args =
R"""(Removes an existing renderer from this SceneGraph

Parameter ``name``:
    The unique name of the renderer to be removed.

Raises:
    RuntimeError if this SceneGraph doesn't have a renderer with the
    specified name.)""";
          // Source: drake/geometry/scene_graph.h
          const char* doc_2args =
R"""(systems∷Context-modifying variant of RemoveRenderer(). Rather than
modifying SceneGraph's model, it modifies the copy of the model stored
in the provided context.)""";
        } RemoveRenderer;
        // Symbol: drake::geometry::SceneGraph::RemoveRole
        struct /* RemoveRole */ {
          // Source: drake/geometry/scene_graph.h
          const char* doc_frame_direct =
R"""(Removes the indicated ``role`` from any geometry directly registered
to the frame indicated by ``frame_id`` (if the geometry has the role).
Potentially modifies the proximity, perception, or illustration
version based on the role being removed from the geometry (see
scene_graph_versioning).

Returns:
    The number of geometries affected by the removed role.

Raises:
    RuntimeError if a) ``source_id`` does not map to a registered
    source, b) ``frame_id`` does not map to a registered frame, c)
    ``frame_id`` does not belong to ``source_id`` (unless ``frame_id``
    is the world frame id), or d) the context has already been
    allocated.)""";
          // Source: drake/geometry/scene_graph.h
          const char* doc_frame_context =
R"""(systems∷Context-modifying variant of RemoveRole(SourceId,FrameId,Role)
"RemoveRole()" for frames. Rather than modifying SceneGraph's model,
it modifies the copy of the model stored in the provided context.)""";
          // Source: drake/geometry/scene_graph.h
          const char* doc_geometry_direct =
R"""(Removes the indicated ``role`` from the geometry indicated by
``geometry_id``. Potentially modifies the proximity, perception, or
illustration version based on the role being removed from the geometry
(see scene_graph_versioning).

Returns:
    One if the geometry had the role removed and zero if the geometry
    did not have the role assigned in the first place.

Raises:
    RuntimeError if a) ``source_id`` does not map to a registered
    source, b) ``geometry_id`` does not map to a registered geometry,
    c) ``geometry_id`` does not belong to ``source_id``, or d) the
    context has already been allocated.)""";
          // Source: drake/geometry/scene_graph.h
          const char* doc_geometry_context =
R"""(systems∷Context-modifying variant of
RemoveRole(SourceId,GeometryId,Role) "RemoveRole()" for individual
geometries. Rather than modifying SceneGraph's model, it modifies the
copy of the model stored in the provided context.)""";
        } RemoveRole;
        // Symbol: drake::geometry::SceneGraph::RenameFrame
        struct /* RenameFrame */ {
          // Source: drake/geometry/scene_graph.h
          const char* doc =
R"""(Renames the frame to ``name``.

This method modifies the underlying model and requires a new Context
to be allocated. It does not modify the model versions (see
scene_graph_versioning).

Parameter ``frame_id``:
    The id of the frame to rename.

Parameter ``name``:
    The new name.

Raises:
    RuntimeError if a) the ``frame_id`` does not map to a valid frame,
    or b) there is already a frame named ``name`` from the same
    source.)""";
        } RenameFrame;
        // Symbol: drake::geometry::SceneGraph::RenameGeometry
        struct /* RenameGeometry */ {
          // Source: drake/geometry/scene_graph.h
          const char* doc =
R"""(Renames the geometry to ``name``.

This method modifies the underlying model and requires a new Context
to be allocated. It does not modify the model versions (see
scene_graph_versioning).

Parameter ``geometry_id``:
    The id of the geometry to rename.

Parameter ``name``:
    The new name.

Raises:
    RuntimeError if a) the ``geometry_id`` does not map to a valid
    geometry, or b) ``name`` is not unique within any assigned role of
    the geometry in its associated frame.)""";
        } RenameGeometry;
        // Symbol: drake::geometry::SceneGraph::RendererCount
        struct /* RendererCount */ {
          // Source: drake/geometry/scene_graph.h
          const char* doc_0args =
R"""(Reports the number of renderers registered to this SceneGraph.)""";
          // Source: drake/geometry/scene_graph.h
          const char* doc_1args =
R"""(systems∷Context-query variant of RendererCount(). Rather than querying
SceneGraph's model, it queries the copy of the model stored in the
provided context.)""";
        } RendererCount;
        // Symbol: drake::geometry::SceneGraph::SceneGraph<T>
        struct /* ctor */ {
          // Source: drake/geometry/scene_graph.h
          const char* doc_0args =
R"""(Constructs a default (empty) scene graph.)""";
          // Source: drake/geometry/scene_graph.h
          const char* doc_1args =
R"""(Constructs an empty scene graph with the provided configuration.)""";
          // Source: drake/geometry/scene_graph.h
          const char* doc_copyconvert =
R"""(Constructor used for scalar conversions.)""";
        } ctor;
        // Symbol: drake::geometry::SceneGraph::SourceIsRegistered
        struct /* SourceIsRegistered */ {
          // Source: drake/geometry/scene_graph.h
          const char* doc =
R"""(Reports if the given source id is registered.

Parameter ``id``:
    The id of the source to query.)""";
        } SourceIsRegistered;
        // Symbol: drake::geometry::SceneGraph::collision_filter_manager
        struct /* collision_filter_manager */ {
          // Source: drake/geometry/scene_graph.h
          const char* doc_0args =
R"""(Returns the collision filter manager for this SceneGraph instance's
model*.)""";
          // Source: drake/geometry/scene_graph.h
          const char* doc_1args =
R"""(Returns the collision filter manager for data stored in ``context``.
The context must remain alive for at least as long as the returned
manager.)""";
        } collision_filter_manager;
        // Symbol: drake::geometry::SceneGraph::get_config
        struct /* get_config */ {
          // Source: drake/geometry/scene_graph.h
          const char* doc_0args =
R"""(Returns:
    the current configuration.)""";
          // Source: drake/geometry/scene_graph.h
          const char* doc_1args =
R"""(Returns:
    the scene graph configuration from the given context. Note: there
    is no matching per-Context set_config() function. The context's
    scene graph configuration is copied from the main scene graph
    configuration at context construction time. Thereafter, the
    context's scene graph configuration is not mutable.)""";
        } get_config;
        // Symbol: drake::geometry::SceneGraph::get_query_output_port
        struct /* get_query_output_port */ {
          // Source: drake/geometry/scene_graph.h
          const char* doc =
R"""(Returns the output port which produces the QueryObject for performing
geometric queries.)""";
        } get_query_output_port;
        // Symbol: drake::geometry::SceneGraph::get_source_configuration_port
        struct /* get_source_configuration_port */ {
          // Source: drake/geometry/scene_graph.h
          const char* doc =
R"""(Given a valid source ``id``, returns a *configuration* input port
associated with that ``id``. This port is used to communicate
configuration data for registered deformable geometries.

Raises:
    RuntimeError if the source_id is *not* recognized.)""";
        } get_source_configuration_port;
        // Symbol: drake::geometry::SceneGraph::get_source_pose_port
        struct /* get_source_pose_port */ {
          // Source: drake/geometry/scene_graph.h
          const char* doc =
R"""(Given a valid source ``id``, returns a *pose* input port associated
with that ``id``. This port is used to communicate *pose* data for
registered frames.

Raises:
    RuntimeError if the source_id is *not* recognized.)""";
        } get_source_pose_port;
        // Symbol: drake::geometry::SceneGraph::model_inspector
        struct /* model_inspector */ {
          // Source: drake/geometry/scene_graph.h
          const char* doc =
R"""(Returns an inspector on the system's *model* scene graph data.)""";
        } model_inspector;
        // Symbol: drake::geometry::SceneGraph::set_config
        struct /* set_config */ {
          // Source: drake/geometry/scene_graph.h
          const char* doc = R"""(Sets the configuration.)""";
        } set_config;
        // Symbol: drake::geometry::SceneGraph::world_frame_id
        struct /* world_frame_id */ {
          // Source: drake/geometry/scene_graph.h
          const char* doc =
R"""(Reports the identifier for the world frame.)""";
        } world_frame_id;
      } SceneGraph;
      // Symbol: drake::geometry::SceneGraphConfig
      struct /* SceneGraphConfig */ {
        // Source: drake/geometry/scene_graph_config.h
        const char* doc =
R"""(The set of configurable properties on a SceneGraph.)""";
        // Symbol: drake::geometry::SceneGraphConfig::Serialize
        struct /* Serialize */ {
          // Source: drake/geometry/scene_graph_config.h
          const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
        } Serialize;
        // Symbol: drake::geometry::SceneGraphConfig::ValidateOrThrow
        struct /* ValidateOrThrow */ {
          // Source: drake/geometry/scene_graph_config.h
          const char* doc = R"""(Throws if the values are inconsistent.)""";
        } ValidateOrThrow;
        // Symbol: drake::geometry::SceneGraphConfig::default_proximity_properties
        struct /* default_proximity_properties */ {
          // Source: drake/geometry/scene_graph_config.h
          const char* doc =
R"""(Provides SceneGraph-wide contact material values to use when none have
been otherwise specified.)""";
        } default_proximity_properties;
        auto Serialize__fields() const {
          return std::array{
            std::make_pair("default_proximity_properties", default_proximity_properties.doc),
          };
        }
      } SceneGraphConfig;
      // Symbol: drake::geometry::SceneGraphInspector
      struct /* SceneGraphInspector */ {
        // Source: drake/geometry/scene_graph_inspector.h
        const char* doc =
R"""(The SceneGraphInspector serves as a mechanism to query the topological
structure of a SceneGraph instance. The topological structure consists
of all of the SceneGraph data that does *not* depend on input pose
data. Including, but not limited to:

- names of frames and geometries
- hierarchies (parents of geometries, parents of frames, etc.)
- geometry parameters (e.g., contact, rendering, visualization)
- fixed poses of geometries relative to frames

In contrast, the following pieces of data *do* depend on input pose
data and *cannot* be performed with the SceneGraphInspector (see the
QueryObject instead):

- world pose of frames or geometry
- collision queries
- proximity queries

A SceneGraphInspector cannot be instantiated explicitly. Nor can it be
copied or moved. A *reference* to a SceneGraphInspector instance can
be acquired from

- a SceneGraph instance (to inspect the state of the system's *model*), or
- a QueryObject instance (to inspect the state of the scene graph data stored
in the context).

The reference should not be persisted (and, as previously indicated,
cannot be copied). SceneGraphInspector instances are cheap; they can
be created, queried, and thrown out. If there is any doubt about the
valid lifespan of a SceneGraphInspector, throw out the old instance
and request a new instance.

Template parameter ``T``:
    The scalar of the associated SceneGraph instance. The template
    parameter is provided for the sake of compatibility, although no
    queries (or their results) depend on the scalar.)""";
        // Symbol: drake::geometry::SceneGraphInspector::BelongsToSource
        struct /* BelongsToSource */ {
          // Source: drake/geometry/scene_graph_inspector.h
          const char* doc_2args_frame_id_source_id =
R"""(Reports if the frame with given ``frame_id`` was registered to the
source with the given ``source_id``.

Parameter ``frame_id``:
    The query frame id.

Parameter ``source_id``:
    The query source id.

Returns:
    True if ``frame_id`` was registered on ``source_id``.

Raises:
    RuntimeError If ``frame_id`` does not map to a registered frame or
    ``source_id`` does not map to a registered source.)""";
          // Source: drake/geometry/scene_graph_inspector.h
          const char* doc_2args_geometry_id_source_id =
R"""(Reports if the given geometry id was registered to the source with the
given source id.

Parameter ``geometry_id``:
    The query geometry id.

Parameter ``source_id``:
    The query source id.

Returns:
    True if ``geometry_id`` was registered on ``source_id``.

Raises:
    RuntimeError If ``geometry_id`` does not map to a registered
    geometry or ``source_id`` does not map to a registered source.)""";
        } BelongsToSource;
        // Symbol: drake::geometry::SceneGraphInspector::CloneGeometryInstance
        struct /* CloneGeometryInstance */ {
          // Source: drake/geometry/scene_graph_inspector.h
          const char* doc =
R"""(Obtains a new GeometryInstance that copies the geometry indicated by
the given ``geometry_id``.

Returns:
    A new GeometryInstance that is ready to be added as a new
    geometry. All roles/properties will be copied, the shape will be
    cloned based off of the original, but the returned id() will
    completely unique.

Raises:
    RuntimeError if the ``geometry_id`` does not refer to a valid
    geometry.)""";
        } CloneGeometryInstance;
        // Symbol: drake::geometry::SceneGraphInspector::CollisionFiltered
        struct /* CollisionFiltered */ {
          // Source: drake/geometry/scene_graph_inspector.h
          const char* doc =
R"""(Reports true if the two geometries with given ids ``geometry_id1`` and
``geometry_id2``, define a collision pair that has been filtered out.

Raises:
    RuntimeError if either id does not map to a registered geometry or
    if any of the geometries do not have a proximity role.)""";
        } CollisionFiltered;
        // Symbol: drake::geometry::SceneGraphInspector::FramesForSource
        struct /* FramesForSource */ {
          // Source: drake/geometry/scene_graph_inspector.h
          const char* doc =
R"""(Reports the ids of all of the frames registered to the source with the
given source ``source_id``.

Raises:
    RuntimeError if ``source_id`` does not map to a registered source.)""";
        } FramesForSource;
        // Symbol: drake::geometry::SceneGraphInspector::GetAllDeformableGeometryIds
        struct /* GetAllDeformableGeometryIds */ {
          // Source: drake/geometry/scene_graph_inspector.h
          const char* doc =
R"""(Returns all geometry ids that correspond to deformable geometries. The
order is guaranteed to be stable and consistent.)""";
        } GetAllDeformableGeometryIds;
        // Symbol: drake::geometry::SceneGraphInspector::GetAllFrameIds
        struct /* GetAllFrameIds */ {
          // Source: drake/geometry/scene_graph_inspector.h
          const char* doc =
R"""(Returns all of the frame ids in the scene graph. The order is
guaranteed to be stable and consistent. The ids include the world
frame's id.)""";
        } GetAllFrameIds;
        // Symbol: drake::geometry::SceneGraphInspector::GetAllGeometryIds
        struct /* GetAllGeometryIds */ {
          // Source: drake/geometry/scene_graph_inspector.h
          const char* doc =
R"""(Returns the set of all ids for registered geometries. The order is
guaranteed to be stable and consistent.

Parameter ``role``:
    The requested role; if omitted, all geometries are returned.)""";
        } GetAllGeometryIds;
        // Symbol: drake::geometry::SceneGraphInspector::GetAllSourceIds
        struct /* GetAllSourceIds */ {
          // Source: drake/geometry/scene_graph_inspector.h
          const char* doc =
R"""(Returns all of the source ids in the scene graph. The order is
guaranteed to be stable and consistent. The first element is the
SceneGraph-internal source.)""";
        } GetAllSourceIds;
        // Symbol: drake::geometry::SceneGraphInspector::GetCollisionCandidates
        struct /* GetCollisionCandidates */ {
          // Source: drake/geometry/scene_graph_inspector.h
          const char* doc =
R"""(Returns all pairs of geometries that are candidates for collision (in
no particular order). See CollisionFilterDeclaration and
CollisionFilterManager∷Apply() for information on why a particular
pair may *not* be a candidate.

For candidate pair (A, B), the candidate is always guaranteed to be
reported in a fixed order (i.e., always (A, B) and *never* (B, A)).
This is the same ordering as would be returned by, e.g.,
QueryObject∷ComputePointPairPenetration().)""";
        } GetCollisionCandidates;
        // Symbol: drake::geometry::SceneGraphInspector::GetConvexHull
        struct /* GetConvexHull */ {
          // Source: drake/geometry/scene_graph_inspector.h
          const char* doc =
R"""(Returns the convex hull (a polytope) associated with the given
``geometry_id``, if it exists. Basic primitive shapes don't have
convex hulls (including, arbitrarily, Box). But Mesh and Convex shapes
do. Alternatively, if you already have a Mesh or Convex you can call
Mesh∷GetConvexHull() or Convex∷GetConvexHull(), respectively.)""";
        } GetConvexHull;
        // Symbol: drake::geometry::SceneGraphInspector::GetDrivenRenderMeshes
        struct /* GetDrivenRenderMeshes */ {
          // Source: drake/geometry/scene_graph_inspector.h
          const char* doc =
R"""(Returns the render mesh representation of the driven meshes associated
with the given ``role`` of the geometry with the given
``geometry_id``.

Parameter ``geometry_id``:
    The identifier for the queried geometry.

Parameter ``role``:
    The role whose driven mesh representations are acquired.

Raises:
    RuntimeError if ``geometry_id`` does not map to a registered
    deformable geometry with the given ``role`` or if ``role`` is
    undefined.

Warning:
    This feature is considered to be **experimental** and may change
    or be removed at any time, without any deprecation notice ahead of
    time.)""";
        } GetDrivenRenderMeshes;
        // Symbol: drake::geometry::SceneGraphInspector::GetFrameGroup
        struct /* GetFrameGroup */ {
          // Source: drake/geometry/scene_graph_inspector.h
          const char* doc =
R"""(Reports the frame group for the frame with the given ``frame_id``.

Raises:
    RuntimeError if ``frame_id`` does not map to a registered frame.
    This value is equivalent to the old "model instance id".)""";
        } GetFrameGroup;
        // Symbol: drake::geometry::SceneGraphInspector::GetFrameId
        struct /* GetFrameId */ {
          // Source: drake/geometry/scene_graph_inspector.h
          const char* doc =
R"""(Reports the id of the frame to which the given geometry with the given
``geometry_id`` is registered.

Raises:
    RuntimeError if ``geometry_id`` does not map to a registered
    geometry.)""";
        } GetFrameId;
        // Symbol: drake::geometry::SceneGraphInspector::GetGeometries
        struct /* GetGeometries */ {
          // Source: drake/geometry/scene_graph_inspector.h
          const char* doc =
R"""(Returns geometry ids that have been registered directly to the frame
indicated by ``frame_id``. If a ``role`` is provided, only geometries
with that role assigned will be returned, otherwise all geometries
will be returned. The order of the ids is guaranteed to be stable and
consistent.

Parameter ``frame_id``:
    The id of the frame in question.

Parameter ``role``:
    The requested role; if omitted, all geometries registered to the
    frame are returned.

Returns:
    The requested unique geometry ids in a consistent order.

Raises:
    RuntimeError if ``id`` does not map to a registered frame.)""";
        } GetGeometries;
        // Symbol: drake::geometry::SceneGraphInspector::GetGeometryIdByName
        struct /* GetGeometryIdByName */ {
          // Source: drake/geometry/scene_graph_inspector.h
          const char* doc =
R"""(Reports the id of the geometry with the given ``name`` and ``role``,
attached to the frame with the given frame ``frame_id``.

Parameter ``frame_id``:
    The frame_id of the frame whose geometry is being queried.

Parameter ``role``:
    The assigned role of the desired geometry.

Parameter ``name``:
    The name of the geometry to query for. The name will be
    canonicalized prior to lookup (see canonicalized_geometry_names
    "GeometryInstance" for details).

Returns:
    The id of the queried geometry.

Raises:
    RuntimeError if no such geometry exists, multiple geometries have
    that name, or if the ``frame_id`` does not map to a registered
    frame.)""";
        } GetGeometryIdByName;
        // Symbol: drake::geometry::SceneGraphInspector::GetGeometryIds
        struct /* GetGeometryIds */ {
          // Source: drake/geometry/scene_graph_inspector.h
          const char* doc =
R"""(Returns the geometry ids that are *implied* by the given GeometrySet
and ``role``. Remember that a GeometrySet can reference a FrameId in
place of the ids of the individual geometries affixed to it. If a
``role`` is provided, only geometries with that role assigned will be
returned, otherwise all geometries will be returned.

Note:
    Specifying ``role`` *can* have the effect of filtering geometries
    *from* the given ``geometry_set`` -- if a GeometryId is an
    explicit member of the geometry set but does not have the
    requested role, it will not be contained in the output.

Parameter ``geometry_set``:
    The encoding of the set of geometries.

Parameter ``role``:
    The requested role; if omitted, all geometries registered to the
    frame are returned.

Returns:
    The requested unique geometry ids.

Warning:
    For C++ users: this returns an *unordered* set, which means
    iteration order will be non-deterministic.)""";
        } GetGeometryIds;
        // Symbol: drake::geometry::SceneGraphInspector::GetIllustrationProperties
        struct /* GetIllustrationProperties */ {
          // Source: drake/geometry/scene_graph_inspector.h
          const char* doc =
R"""(Returns a pointer to the const illustration properties of the geometry
with the given ``geometry_id``.

Parameter ``geometry_id``:
    The identifier for the queried geometry.

Returns:
    A pointer to the properties (or ``nullptr`` if there are no such
    properties).

Raises:
    RuntimeError if ``geometry_id`` does not map to a registered
    geometry.)""";
        } GetIllustrationProperties;
        // Symbol: drake::geometry::SceneGraphInspector::GetName
        struct /* GetName */ {
          // Source: drake/geometry/scene_graph_inspector.h
          const char* doc_1args_source_id =
R"""(Reports the name for the source with the given ``source_id``.

Raises:
    RuntimeError if ``source_id`` does not map to a registered source.)""";
          // Source: drake/geometry/scene_graph_inspector.h
          const char* doc_1args_frame_id =
R"""(Reports the name of the frame with the given ``frame_id``.

Raises:
    RuntimeError if ``frame_id`` does not map to a registered frame.)""";
          // Source: drake/geometry/scene_graph_inspector.h
          const char* doc_1args_geometry_id =
R"""(Reports the stored, canonical name of the geometry with the given
``geometry_id`` (see canonicalized_geometry_names "GeometryInstance"
for details).

Raises:
    RuntimeError if ``geometry_id`` does not map to a registered
    geometry.)""";
        } GetName;
        // Symbol: drake::geometry::SceneGraphInspector::GetObbInGeometryFrame
        struct /* GetObbInGeometryFrame */ {
          // Source: drake/geometry/scene_graph_inspector.h
          const char* doc =
R"""(Returns the oriented bounding box (OBB) associated with the given
``geometry_id`` in the geometry's frame, G. The OBB is defined in the
geometry's canonical frame such that it tightly bounds the geometry.
For primitive shapes, the OBB is computed analytically. For mesh-based
shapes (Mesh and Convex), the OBB is computed using the mesh vertices.

Parameter ``geometry_id``:
    The identifier for the queried geometry.

Note:
    If geometry_id refers to a deformable geometry, the OBB is
    computed using the reference mesh. See
    QueryObject∷ComputeObbInWorld() for computing the OBB of the
    deformed mesh in the world frame.

Returns:
    The oriented bounding box (or ``std∷nullopt`` if the geometry is
    an HalfSpace and doesn't have a bounding box).

Raises:
    RuntimeError if ``geometry_id`` does not map to a registered
    geometry.)""";
        } GetObbInGeometryFrame;
        // Symbol: drake::geometry::SceneGraphInspector::GetOwningSourceName
        struct /* GetOwningSourceName */ {
          // Source: drake/geometry/scene_graph_inspector.h
          const char* doc_1args_frame_id =
R"""(Reports the *name* of the geometry source that registered the frame
with the given ``frame_id``.

Raises:
    RuntimeError If ``frame_id`` does not map to a registered frame.)""";
          // Source: drake/geometry/scene_graph_inspector.h
          const char* doc_1args_geometry_id =
R"""(Reports the *name* of the geometry source that registered the geometry
with the given ``geometry_id``.

Raises:
    RuntimeError If ``geometry_id`` does not map to a registered
    geometry.)""";
        } GetOwningSourceName;
        // Symbol: drake::geometry::SceneGraphInspector::GetParentFrame
        struct /* GetParentFrame */ {
          // Source: drake/geometry/scene_graph_inspector.h
          const char* doc =
R"""(Reports the FrameId of the parent of ``frame_id``.

Raises:
    RuntimeError if ``frame_id`` does not map to a registered frame.)""";
        } GetParentFrame;
        // Symbol: drake::geometry::SceneGraphInspector::GetPerceptionProperties
        struct /* GetPerceptionProperties */ {
          // Source: drake/geometry/scene_graph_inspector.h
          const char* doc =
R"""(Returns a pointer to the const perception properties of the geometry
with the given ``geometry_id``.

Parameter ``geometry_id``:
    The identifier for the queried geometry.

Returns:
    A pointer to the properties (or ``nullptr`` if there are no such
    properties).

Raises:
    RuntimeError if ``geometry_id`` does not map to a registered
    geometry.)""";
        } GetPerceptionProperties;
        // Symbol: drake::geometry::SceneGraphInspector::GetPoseInFrame
        struct /* GetPoseInFrame */ {
          // Source: drake/geometry/scene_graph_inspector.h
          const char* doc =
R"""(Reports the pose of the geometry G with the given ``geometry_id`` in
its registered frame F.

Note:
    For deformable geometries, this returns the pose of the reference
    mesh.

Raises:
    RuntimeError if ``geometry_id`` does not map to a registered
    geometry.)""";
        } GetPoseInFrame;
        // Symbol: drake::geometry::SceneGraphInspector::GetProperties
        struct /* GetProperties */ {
          // Source: drake/geometry/scene_graph_inspector.h
          const char* doc =
R"""(Return a pointer to the const properties indicated by ``role`` of the
geometry with the given ``geometry_id``.

Parameter ``geometry_id``:
    The identifier for the queried geometry.

Parameter ``role``:
    The role whose properties are acquired.

Returns:
    A pointer to the properties (or ``nullptr`` if there are no such
    properties).

Raises:
    RuntimeError if ``geometry_id`` does not map to a registered
    geometry.)""";
        } GetProperties;
        // Symbol: drake::geometry::SceneGraphInspector::GetProximityProperties
        struct /* GetProximityProperties */ {
          // Source: drake/geometry/scene_graph_inspector.h
          const char* doc =
R"""(Returns a pointer to the const proximity properties of the geometry
with the given ``geometry_id``.

Parameter ``geometry_id``:
    The identifier for the queried geometry.

Returns:
    A pointer to the properties (or ``nullptr`` if there are no such
    properties).

Raises:
    RuntimeError if ``geometry_id`` does not map to a registered
    geometry.)""";
        } GetProximityProperties;
        // Symbol: drake::geometry::SceneGraphInspector::GetReferenceMesh
        struct /* GetReferenceMesh */ {
          // Source: drake/geometry/scene_graph_inspector.h
          const char* doc =
R"""(Returns the reference mesh of the geometry with the given
``geometry_id``, measured and expressed in the geometry's frame, G.

Parameter ``geometry_id``:
    The identifier for the queried geometry.

Returns:
    A pointer to the reference mesh of the geometry if the geometry is
    deformable, or ``nullptr`` if the geometry is rigid.

Note:
    GetPoseInFrame() provides the transform necessary to measure and
    express the reference mesh's vertex positions in the geometry's
    registered frame F.

Raises:
    RuntimeError if ``geometry_id`` does not map to a registered
    geometry.)""";
        } GetReferenceMesh;
        // Symbol: drake::geometry::SceneGraphInspector::GetShape
        struct /* GetShape */ {
          // Source: drake/geometry/scene_graph_inspector.h
          const char* doc =
R"""(Returns the shape specified for the geometry with the given
``geometry_id``. In order to extract the details of the shape, it
should be passed through an implementation of a ShapeReifier.)""";
        } GetShape;
        // Symbol: drake::geometry::SceneGraphInspector::IsDeformableGeometry
        struct /* IsDeformableGeometry */ {
          // Source: drake/geometry/scene_graph_inspector.h
          const char* doc =
R"""(Returns true if the geometry with the given ``geometry_id`` is
deformable.

Parameter ``geometry_id``:
    The identifier for the queried geometry.

Raises:
    RuntimeError if ``geometry_id`` does not map to a registered
    geometry.)""";
        } IsDeformableGeometry;
        // Symbol: drake::geometry::SceneGraphInspector::NumAnchoredGeometries
        struct /* NumAnchoredGeometries */ {
          // Source: drake/geometry/scene_graph_inspector.h
          const char* doc =
R"""(Reports the total number of *anchored* non-deformable geometries. This
should provide the same answer as calling NumGeometriesForFrame() with
the world frame id.)""";
        } NumAnchoredGeometries;
        // Symbol: drake::geometry::SceneGraphInspector::NumDeformableGeometriesWithRole
        struct /* NumDeformableGeometriesWithRole */ {
          // Source: drake/geometry/scene_graph_inspector.h
          const char* doc =
R"""(Reports the *total* number of *deformable* geometries in the scene
graph with the indicated role.)""";
        } NumDeformableGeometriesWithRole;
        // Symbol: drake::geometry::SceneGraphInspector::NumDynamicGeometries
        struct /* NumDynamicGeometries */ {
          // Source: drake/geometry/scene_graph_inspector.h
          const char* doc =
R"""(Reports the total number of *dynamic* geometries in the scene graph.
This include all deformable geometries.)""";
        } NumDynamicGeometries;
        // Symbol: drake::geometry::SceneGraphInspector::NumFramesForSource
        struct /* NumFramesForSource */ {
          // Source: drake/geometry/scene_graph_inspector.h
          const char* doc =
R"""(Reports the number of frames registered to the source with the given
``source_id``.

Raises:
    RuntimeError if ``source_id`` does not map to a registered source.)""";
        } NumFramesForSource;
        // Symbol: drake::geometry::SceneGraphInspector::NumGeometriesForFrame
        struct /* NumGeometriesForFrame */ {
          // Source: drake/geometry/scene_graph_inspector.h
          const char* doc =
R"""(Reports the number of geometries affixed to the frame with the given
``frame_id``. This count does *not* include geometries attached to
frames that are descendants of this frame.

Raises:
    RuntimeError if ``frame_id`` does not map to a registered frame.)""";
        } NumGeometriesForFrame;
        // Symbol: drake::geometry::SceneGraphInspector::NumGeometriesForFrameWithRole
        struct /* NumGeometriesForFrameWithRole */ {
          // Source: drake/geometry/scene_graph_inspector.h
          const char* doc =
R"""(Reports the total number of geometries with the given ``role``
directly registered to the frame with the given ``frame_id``. This
count does *not* include geometries attached to frames that are
descendants of this frame.

Raises:
    RuntimeError if ``frame_id`` does not map to a registered frame.)""";
        } NumGeometriesForFrameWithRole;
        // Symbol: drake::geometry::SceneGraphInspector::NumGeometriesWithRole
        struct /* NumGeometriesWithRole */ {
          // Source: drake/geometry/scene_graph_inspector.h
          const char* doc =
R"""(Reports the *total* number of geometries in the scene graph with the
indicated role.)""";
        } NumGeometriesWithRole;
        // Symbol: drake::geometry::SceneGraphInspector::SceneGraphInspector<T>
        struct /* ctor */ {
          // Source: drake/geometry/scene_graph_inspector.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::geometry::SceneGraphInspector::SourceIsRegistered
        struct /* SourceIsRegistered */ {
          // Source: drake/geometry/scene_graph_inspector.h
          const char* doc =
R"""(Reports ``True`` if the given ``source_id`` maps to a registered
source.)""";
        } SourceIsRegistered;
        // Symbol: drake::geometry::SceneGraphInspector::geometry_version
        struct /* geometry_version */ {
          // Source: drake/geometry/scene_graph_inspector.h
          const char* doc =
R"""(Returns the geometry version that can be used to detect changes to the
geometry data associated with geometry roles. The reference returned
should not be persisted. If it needs to be persisted, it should be
copied.)""";
        } geometry_version;
        // Symbol: drake::geometry::SceneGraphInspector::maybe_get_hydroelastic_mesh
        struct /* maybe_get_hydroelastic_mesh */ {
          // Source: drake/geometry/scene_graph_inspector.h
          const char* doc =
R"""(Returns the *mesh* used to represent this geometry in hydroelastic
contact calculations, if it exists. Most primitives (sphere, cylinder,
etc.) are actually represented by discrete approximations (i.e., the
mesh). If there is no mesh, the returned variant will hold neither the
TriangleSurfaceMesh<double> nor the VolumeMesh<double> alternatives.
If either alternative is present, the pointer is guaranteed to be
non-null.

Just because hydroelastic properties have been assigned to a geometry
does not* mean there is necessarily a mesh associated with it. Some
shape types (e.g., half space) have non-mesh representations.

The result can be tested as follows:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    auto maybe_mesh = inspector.maybe_get_hydroelastic_mesh(id);
    
    // These two methods are equivalent, although testing index is more
    // brittle.
    const bool no_mesh1 = maybe_mesh.index() == 0;
    const bool no_mesh2 = std∷holds_alternative<std∷monostate>(maybe_mesh);

.. raw:: html

    </details>

Parameter ``geometry_id``:
    The id of the geometry to query.

Returns:
    The associated mesh, if it exists.)""";
        } maybe_get_hydroelastic_mesh;
        // Symbol: drake::geometry::SceneGraphInspector::num_frames
        struct /* num_frames */ {
          // Source: drake/geometry/scene_graph_inspector.h
          const char* doc =
R"""(Reports the *total* number of frames registered in the scene graph
(including the world frame).)""";
        } num_frames;
        // Symbol: drake::geometry::SceneGraphInspector::num_geometries
        struct /* num_geometries */ {
          // Source: drake/geometry/scene_graph_inspector.h
          const char* doc =
R"""(Reports the *total* number of geometries in the scene graph.)""";
        } num_geometries;
        // Symbol: drake::geometry::SceneGraphInspector::num_sources
        struct /* num_sources */ {
          // Source: drake/geometry/scene_graph_inspector.h
          const char* doc =
R"""(Reports the number of registered sources -- whether they have
registered frames/geometries or not. This will always be at least 1;
the SceneGraph itself counts as a source.)""";
        } num_sources;
        // Symbol: drake::geometry::SceneGraphInspector::world_frame_id
        struct /* world_frame_id */ {
          // Source: drake/geometry/scene_graph_inspector.h
          const char* doc = R"""(Reports the id for the world frame.)""";
        } world_frame_id;
      } SceneGraphInspector;
      // Symbol: drake::geometry::Shape
      struct /* Shape */ {
        // Source: drake/geometry/shape_specification.h
        const char* doc =
R"""(The abstract base class for all shape specifications. Concrete
subclasses exist for specific shapes (e.g., Box, Mesh, etc.).

The Shape class has two key properties:

- it is cloneable, and
- it can be "reified" (see ShapeReifier).

Note that the Shape class hierarchy is closed to third-party
extensions. All Shape classes must be defined within Drake directly
(and in this h/cc file pair in particular).)""";
        // Symbol: drake::geometry::Shape::Clone
        struct /* Clone */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc = R"""(Creates a unique copy of this shape.)""";
        } Clone;
        // Symbol: drake::geometry::Shape::DoClone
        struct /* DoClone */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc = R"""((Internal use only) NVI for Clone().)""";
        } DoClone;
        // Symbol: drake::geometry::Shape::DoReify
        struct /* DoReify */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc = R"""((Internal use only) NVI for Reify().)""";
        } DoReify;
        // Symbol: drake::geometry::Shape::Reify
        struct /* Reify */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc =
R"""(Causes this description to be reified in the given ``reifier``. Each
concrete subclass must invoke the single, matching method on the
reifier. Provides optional user-data (cast as a void*) for the reifier
to consume.)""";
        } Reify;
        // Symbol: drake::geometry::Shape::Shape
        struct /* ctor */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc =
R"""((Internal use only) Constructor for use by derived classes. All
subclasses of Shape must be marked ``final``.)""";
          // Source: drake/geometry/shape_specification.h
          const char* doc_copy =
R"""((Internal use only) For derived classes.)""";
        } ctor;
        // Symbol: drake::geometry::Shape::VariantShapeConstPtr
        struct /* VariantShapeConstPtr */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc =
R"""((Internal use only) All concrete subclasses, as const pointers.)""";
        } VariantShapeConstPtr;
        // Symbol: drake::geometry::Shape::Visit
        struct /* Visit */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc =
R"""(Calls the given ``visitor`` function with ``*this`` as the sole
argument, but with ``*this`` downcast to be the shape's concrete
subclass. For example, if this shape is a Box then calls
``visitor(static_cast<const Box&>(*this))``.

Template parameter ``ReturnType``:
    The return type to coerce return values into. When not ``void``,
    anything returned by the visitor must be implicitly convertible to
    this type. When ``void``, the return type will be whatever the
    Vistor's call operator returns by default.

To see examples of how this is used, you can check the Drake source
code, e.g., check the implementation of CalcVolume() for one example.)""";
        } Visit;
        // Symbol: drake::geometry::Shape::do_to_string
        struct /* do_to_string */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc =
R"""((Internal use only) NVI for to_string().)""";
        } do_to_string;
        // Symbol: drake::geometry::Shape::do_type_name
        struct /* do_type_name */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc =
R"""((Internal use only) NVI for type_name().)""";
        } do_type_name;
        // Symbol: drake::geometry::Shape::get_variant_this
        struct /* get_variant_this */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc =
R"""((Internal use only) NVI-like helper function for Visit().)""";
        } get_variant_this;
        // Symbol: drake::geometry::Shape::to_string
        struct /* to_string */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc =
R"""(Returns a string representation of this shape.)""";
        } to_string;
        // Symbol: drake::geometry::Shape::type_name
        struct /* type_name */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc =
R"""(Returns the (unqualified) type name of this Shape, e.g., "Box".)""";
        } type_name;
      } Shape;
      // Symbol: drake::geometry::ShapeReifier
      struct /* ShapeReifier */ {
        // Source: drake/geometry/shape_specification.h
        const char* doc =
R"""(The interface for converting shape descriptions to real shapes. Any
entity that consumes shape descriptions *must* implement this
interface.

This class explicitly enumerates all concrete shapes in its methods.
The addition of a new concrete shape class requires the addition of a
new corresponding method. There should *never* be an ImplementGeometry
method that accepts the Shape base class as an argument; it should
*only* operate on concrete derived classes.

The expected workflow is for a class that needs to turn shape
specifications into concrete geometry instances to implement the
ShapeReifier interface *and* invoke the Shape∷Reify() method. For
example, a simple reifier that requires no user data would look like:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    class SimpleReifier : public ShapeReifier {
    void ProcessShape(const Shape& shape) {
    // Requires no user data.
    shape.Reify(this);
    }
    ...
    void ImplementGeometry(const Sphere& sphere, void*) override {
    // Do work to create a sphere.
    }
    };

.. raw:: html

    </details>

Or a complex reifier that requires user data would look like:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    class ComplexReifier : public ShapeReifier {
    void ProcessShape(const Shape& shape) {
    ImportantData data{...};
    shape.Reify(this, &data);
    }
    ...
    void ImplementGeometry(const Sphere& sphere, void* data) override {
    DRAKE_ASSERT(data != nullptr);
    ImportantData& data = *static_cast<ImportantData*>(data);
    // Do work to create a sphere using the provided user data.
    }
    };

.. raw:: html

    </details>

Implementing a particular shape may require more data than is strictly
encapsulated in the Shape. The Implement* interface supports passing
user data through a type-erased ``void*``. Because a single class
invoked Shape∷Reify() it is in a position to provide exactly the data
the shape implementations require.)""";
        // Symbol: drake::geometry::ShapeReifier::DefaultImplementGeometry
        struct /* DefaultImplementGeometry */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc =
R"""(The default implementation of ImplementGeometry(): it throws an
exception using ThrowUnsupportedGeometry(). The purpose of this
function is to facilitate reifiers that would call the same API on all
shapes (e.g., call an API with a Shape& parameter). This reduces the
implementation boilerplate.)""";
        } DefaultImplementGeometry;
        // Symbol: drake::geometry::ShapeReifier::ImplementGeometry
        struct /* ImplementGeometry */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc = R"""()""";
        } ImplementGeometry;
        // Symbol: drake::geometry::ShapeReifier::ShapeReifier
        struct /* ctor */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::geometry::ShapeReifier::ThrowUnsupportedGeometry
        struct /* ThrowUnsupportedGeometry */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc =
R"""(Derived ShapeReifiers can replace the default message for unsupported
geometries by overriding this method. The name of the unsupported
shape type is given as the single parameter.)""";
        } ThrowUnsupportedGeometry;
      } ShapeReifier;
      // Symbol: drake::geometry::SourceId
      struct /* SourceId */ {
        // Source: drake/geometry/geometry_ids.h
        const char* doc =
R"""(Type used to identify geometry sources in SceneGraph.)""";
      } SourceId;
      // Symbol: drake::geometry::Sphere
      struct /* Sphere */ {
        // Source: drake/geometry/shape_specification.h
        const char* doc =
R"""(Definition of sphere. It is centered in its canonical frame with the
given radius.)""";
        // Symbol: drake::geometry::Sphere::Sphere
        struct /* ctor */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc =
R"""(Constructs a sphere with the given ``radius``.

Raises:
    RuntimeError if ``radius`` is not finite *non-negative*. Note that
    a zero radius is considered valid.)""";
        } ctor;
        // Symbol: drake::geometry::Sphere::radius
        struct /* radius */ {
          // Source: drake/geometry/shape_specification.h
          const char* doc = R"""()""";
        } radius;
      } Sphere;
      // Symbol: drake::geometry::to_string
      struct /* to_string */ {
        // Source: drake/geometry/geometry_roles.h
        const char* doc = R"""()""";
      } to_string;
    } geometry;
    // Symbol: drake::systems
    struct /* systems */ {
      // Symbol: drake::systems::scalar_conversion
      struct /* scalar_conversion */ {
        // Symbol: drake::systems::scalar_conversion::Traits
        struct /* Traits */ {
          // Source: drake/geometry/drake_visualizer.h
          const char* doc = R"""()""";
        } Traits;
      } scalar_conversion;
    } systems;
  } drake;
} pydrake_doc_geometry;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
