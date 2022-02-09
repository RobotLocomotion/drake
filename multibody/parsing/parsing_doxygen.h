/** @file
 Doxygen-only documentation for @ref multibody_parsing. */

/**
@defgroup multibody_parsing Parsing Models for Multibody Dynamics
@ingroup multibody

Drake's drake::multibody::Parser accepts model files written in either SDFormat
or URDF. In both formats, however, there are Drake-specific extensions and
Drake-specific limitations.

<!-- TODO(rpoyner-tri): document mujoco format support -->

The result of the parse is an in-memory model realized within
drake::multibody::MultibodyPlant and (optionally)
drake::geometry::SceneGraph. Note that parses that do not use a `SceneGraph`
will effectively ignore geometric model elements, especially `//visual` and
`//collision` elements.

In the reference sections below, the relevant usage paths for various tags
are indicated using [XPath](https://www.w3.org/TR/xpath-31/) notation.

@section multibody_parsing_sdf SDFormat Support
Drake supports SDFormat files following the specification at
http://sdformat.org/spec. As of this writing, the supported spec version is
1.9, but check also the Drake release notes for current status.

For Drake extensions to SDFormat files, see
@ref multibody_parsing_drake_extensions.

Drake's parser does not implement all of the features of SDFormat. In the
future, we intend to update this documentation to itemize what isn't supported.

@section multibody_parsing_urdf URDF Support
Drake supports URDF files as described here: http://wiki.ros.org/urdf/XML.

For Drake extensions to URDF format files, see
@ref multibody_parsing_drake_extensions.

Drake's parser does not implement all of the features of URDF. In the future,
we intend to update this documentation to itemize what isn't supported.

@section multibody_parsing_drake_extensions Drake Extensions

Drake extends both SDFormat and URDF in similar ways, to allow access to
Drake-specific features. This section provides a compact reference to all
Drake extensions and their use in both formats.

Note that in both formats, it is proper to declare the `drake:` namespace
prefix. For SDFormat, declare it like this:

    <sdf xmlns:drake="http://drake.mit.edu" version="1.9">

For URDF, declare the namespace prefix like this:

    <robot xmlns:drake="http://drake.mit.edu" name="test_robot">

Here is the full list of custom elements:
- @ref tag_drake_acceleration
- @ref tag_drake_accepting_renderer
- @ref tag_drake_bushing_force_damping
- @ref tag_drake_bushing_force_stiffness
- @ref tag_drake_bushing_frameA
- @ref tag_drake_bushing_frameC
- @ref tag_drake_bushing_torque_damping
- @ref tag_drake_bushing_torque_stiffness
- @ref tag_drake_capsule
- @ref tag_drake_child
- @ref tag_drake_collision_filter_group
- @ref tag_drake_compliant_hydroelastic
- @ref tag_drake_damping
- @ref tag_drake_declare_convex
- @ref tag_drake_diffuse_map
- @ref tag_drake_ellipsoid
- @ref tag_drake_gear_ratio
- @ref tag_drake_hunt_crossley_dissipation
- @ref tag_drake_hydroelastic_modulus
- @ref tag_drake_ignored_collision_filter_group
- @ref tag_drake_joint
- @ref tag_drake_linear_bushing_rpy
- @ref tag_drake_member
- @ref tag_drake_mesh_resolution_hint
- @ref tag_drake_mu_dynamic
- @ref tag_drake_mu_static
- @ref tag_drake_parent
- @ref tag_drake_point_contact_stiffness
- @ref tag_drake_proximity_properties
- @ref tag_drake_rigid_hydroelastic
- @ref tag_drake_rotor_inertia

@subsection tag_drake_acceleration drake:acceleration

- SDFormat path: `//model/joint/axis/limit/drake:acceleration`
- URDF path: `/robot/joint/limit/@drake:acceleration`
- Syntax: One floating point value, non-negative.

@subsubsection tag_drake_acceleration_semantics Semantics

If the joint type is one of (continuous, prismatic, revolute), specifies
acceleration limits of (-VALUE, +VALUE). Units are determined by the type of
joint, either radians/sec^2 for continuous and revolute joints, or m/s^2 for
prismatic joints.

@subsection tag_drake_accepting_renderer drake:accepting_renderer

- SDFormat path: `//model/link/visual/drake:accepting_renderer`
- URDF path: `/robot/link/visual/drake:accepting_renderer/@name`
- Syntax: String.

@subsubsection tag_drake_accepting_renderer_semantics Semantics

The tag serves as a list of renderers for which this visual is targeted.

  - The _value_ of the tag is the name of the renderer.
  - If the _value_ is empty, that is a parsing error.
  - If no instance of `<drake:accepting_renderer>` every renderer will be given
    the chance to reify this visual geometry.
  - Multiple instances of this tag are allowed. Each instance adds a renderer to
    the list of targeted renderers.

This feature is one way to provide multiple visual representations of a body.

@subsection tag_drake_bushing_force_damping drake:bushing_force_damping

- SDFormat path: `//model/drake:linear_bushing_rpy/drake:bushing_force_damping`
- URDF path: `/robot/drake:linear_bushing_rpy/drake:bushing_force_damping/@value`
- Syntax: Three floating point values.

@subsubsection tag_drake_bushing_force_damping_semantics Semantics

The three floating point values (units of N⋅s/m) are formed into a vector and
passed to the constructor of drake::multibody::LinearBushingRollPitchYaw as the
`force_damping_constants` parameter.

@see @ref tag_drake_linear_bushing_rpy,
drake::multibody::LinearBushingRollPitchYaw,
@ref Basic_bushing_force_stiffness_and_damping "How to choose force stiffness and damping constants"

@subsection tag_drake_bushing_force_stiffness drake:bushing_force_stiffness

- SDFormat path: `//model/drake:linear_bushing_rpy/drake:bushing_force_stiffness`
- URDF path: `/robot/drake:linear_bushing_rpy/drake:bushing_force_stiffness/@value`
- Syntax: Three floating point values.

@subsubsection tag_drake_bushing_force_stiffness_semantics Semantics

The three floating point values (units of N/m) are formed into a vector and
passed to the constructor of drake::multibody::LinearBushingRollPitchYaw as the
`force_stiffness_constants` parameter.

@see @ref tag_drake_linear_bushing_rpy,
drake::multibody::LinearBushingRollPitchYaw,
@ref Basic_bushing_force_stiffness_and_damping "How to choose force stiffness and damping constants"

@subsection tag_drake_bushing_frameA drake:bushing_frameA

- SDFormat path: `//model/drake:linear_bushing_rpy/drake:bushing_frameA`
- URDF path: `/robot/drake:linear_bushing_rpy/drake:bushing_frameA/@name`
- Syntax: String.

@subsubsection tag_drake_bushing_frameA_semantics Semantics

The string names a frame (expected to already be defined by this model) that
will be passed to the constructor of
drake::multibody::LinearBushingRollPitchYaw as the `frameA` parameter.

@see @ref tag_drake_linear_bushing_rpy, drake::multibody::LinearBushingRollPitchYaw

@subsection tag_drake_bushing_frameC drake:bushing_frameC

- SDFormat path: `//model/drake:linear_bushing_rpy/drake:bushing_frameC`
- URDF path: `/robot/drake:linear_bushing_rpy/drake:bushing_frameC/@name`
- Syntax: String.

@subsubsection tag_drake_bushing_frameC_semantics Semantics

The string names a frame (expected to already be defined by this model) that
will be passed to the constructor of
drake::multibody::LinearBushingRollPitchYaw as the `frameC` parameter.

@see @ref tag_drake_linear_bushing_rpy, drake::multibody::LinearBushingRollPitchYaw

@subsection tag_drake_bushing_torque_damping drake:bushing_torque_damping

- SDFormat path: `//model/drake:linear_bushing_rpy/drake:bushing_torque_damping`
- URDF path: `/robot/drake:linear_bushing_rpy/drake:bushing_torque_damping/@value`
- Syntax: Three floating point values.

@subsubsection tag_drake_bushing_torque_damping_semantics Semantics

The three floating point values (units of N⋅m⋅s/rad) are formed into a vector
and passed to the constructor of drake::multibody::LinearBushingRollPitchYaw as
the `torque_damping_constants` parameter.

@see @ref tag_drake_linear_bushing_rpy,
drake::multibody::LinearBushingRollPitchYaw,
@ref Basic_bushing_torque_stiffness_and_damping "How to choose torque stiffness and damping constants"

@subsection tag_drake_bushing_torque_stiffness drake:bushing_torque_stiffness

- SDFormat path: `//model/drake:linear_bushing_rpy/drake:bushing_torque_stiffness`
- URDF path: `/robot/drake:linear_bushing_rpy/drake:bushing_torque_stiffness/@value`
- Syntax: Three floating point values.

@subsubsection tag_drake_bushing_torque_stiffness_semantics Semantics

The three floating point values (units of N⋅m/rad) are formed into a vector and
passed to the constructor of drake::multibody::LinearBushingRollPitchYaw as the
`torque_stiffness_constants` parameter.

@see @ref tag_drake_linear_bushing_rpy,
drake::multibody::LinearBushingRollPitchYaw,
@ref Basic_bushing_torque_stiffness_and_damping "How to choose torque stiffness and damping constants"

@subsection tag_drake_capsule drake:capsule

- SDFormat path: `//geometry[parent::visual|parent::collision]/drake:capsule`
- URDF path: `//geometry[parent::visual|parent::collision]/drake:capsule`
- Syntax: Two nested elements `radius` and `length`, each of which contain a
          non-negative floating point value.

@subsubsection tag_drake_capsule_semantics Semantics

Constructs a drake::geometry::Capsule of the given radius (meters) and length
(meters), and adds it to either the visual or collision geometry of the model.

SDFormat note: `drake:capsule` is proposed for deprecation; see
[issue #14387](https://github.com/RobotLocomotion/drake/issues/14837).

URDF note: Drake supports both `drake:capsule`, and a non-standard
(non-prefixed) `capsule` tag with the same syntax and semantics.

@subsection tag_drake_child drake:child

- SDFormat path: `//model/drake:joint/drake:child`
- URDF path: N/A
- Syntax: String.

@subsubsection tag_drake_child_semantics Semantics

The string names a frame (defined elsewhere in the model) that is associated
with the child link of the joint being defined.

@see @ref tag_drake_joint

@subsection tag_drake_collision_filter_group drake:collision_filter_group

- SDFormat path: `//model/drake:collision_filter_group`
- URDF path: `/robot/drake:collision_filter_group`
- Syntax: Attributes `name` (string) and `ignore` (boolean); nested elements
          `drake:member` and `drake:ignored_collision_filter_group`.

@subsubsection tag_drake_collision_filter_group_semantics Semantics

This element names a group of bodies to participate in collision filtering
rules. If the `ignore` attribute is present and true-valued, the entire element
is ignored. The nested elements must included one or more `drake:member`
elements, and zero or more `drake:ignored_collision_filter_group` elements.

Collision filtering rules are only constructed with **pairs** of groups, where
both sides of the pair may name the same group. The
`drake:ignored_collision_filter_group` element establishes a pairing between
the group it names and the group within which it is nested. A pair containing
different collision groups excludes collisions between members of those groups
(see drake::geometry::CollisionFilterDeclaration::ExcludeBetween()). A pair
naming the same group twice excludes collisions with the group (see
drake::geometry::CollisionFilterDeclaration::ExcludeWithin()).

@see @ref tag_drake_member, @ref tag_drake_ignored_collision_filter_group

@subsection tag_drake_compliant_hydroelastic drake:compliant_hydroelastic

- SDFormat path: `//model/link/collision/drake:proximity_properties/drake:compliant_hydroelastic`
- URDF path: `/robot/link/collision/drake:proximity_properties/drake:compliant_hydroelastic`
- Syntax: Empty element.

@subsubsection tag_drake_compliant_hydroelastic_semantics Semantics

If present, this element sets the compliance type of the element being defined
to be compliant, as opposed to rigid, in hydroelastic contact models.

@see @ref tag_drake_proximity_properties, @ref creating_hydro_reps

@subsection tag_drake_damping drake:damping

- SDFormat path: `//model/drake:joint/drake:damping`
- URDF path: N/A
- Syntax: Three non-negative floating point values.

@subsubsection tag_drake_damping_semantics Semantics

If the parent joint type is planar, construct a vector from the values (units
of (N⋅s/m, N⋅s/m, N⋅m⋅s) respectively) and pass it to the `damping` parameter
of the constructor of drake::multibody::PlanarJoint. The first two values are
for translation; the third is for rotation. See that class for discussion of
units and detailed semantics.

URDF Note: The comparable feature in URDF is the standard
`/robot/link/joint/dynamics/@damping` attribute.

@subsection tag_drake_declare_convex drake:declare_convex

- SDFormat path: `//geometry[parent::visual|parent::collision]/mesh/drake:declare_convex`
- URDF path: `//geometry[parent::visual|parent::collision]/mesh/drake:declare_convex`
- Syntax: Empty element.

@subsubsection tag_drake_declare_convex_semantics Semantics

If present, this element causes Drake to treat the parent mesh geometry as
convex, without any further validation. The resulting geometry in memory will
be drake::geometry::Convex, rather than drake::geometry::Mesh.

@see drake::geometry::Convex, drake::geometry::Mesh

@subsection tag_drake_diffuse_map drake:diffuse_map

- SDFormat path: `//model/link/visual/material/drake:diffuse_map`
- URDF path: N/A
- Syntax: URI.

@subsubsection tag_drake_diffuse_map_semantics Semantics

If present, this element indicates (by filename or `package:` URI) a PNG file
to use as a diffuse texture map. The resolved path name is stored in a
PerceptionProperties object under `(phong, diffuse_map)`.

@see drake::geometry::PerceptionProperties, drake::multibody::PackageMap,
@ref render_engine_vtk_properties "Geometry perception properties"

@subsection tag_drake_ellipsoid drake:ellipsoid

- SDFormat path: `//geometry[parent::visual|parent::collision]/drake:ellipsoid`
- URDF path: `//geometry[parent::visual|parent::collision]/drake:ellipsoid`
- Syntax: Three nested elements (SDFormat) or attributes (URDF) `a`, `b`, `c`,
          each of which contain a positive floating point value.

@subsubsection tag_drake_ellipsoid_semantics Semantics

Constructs a drake::geometry::Ellipsoid with the given half-axis length
parameters `a`, `b`, and `c` (all in units of meters), and adds it to either
the visual or collision geometry of the model.

@subsection tag_drake_gear_ratio drake:gear_ratio

- SDFormat path: `//model/link/joint/drake:gear_ratio`
- URDF path: `/robot/link/joint/actuator/drake:gear_ratio@value`
- Syntax: Non-negative floating point value.

@subsubsection tag_drake_gear_ratio_semantics Semantics

Applies the indicated gear ratio value to the appropriate JointActuator
object. This value is only used for reflected inertia calculations, and not as
a torque multiplier. The value is dimensionless for revolute joints, and has
units of 1/m for prismatic joints.

@see drake::multibody::JointActuator, @ref tag_drake_rotor_inertia,
@ref reflected_inertia "Reflected Inertia"

@subsection tag_drake_hunt_crossley_dissipation drake:hunt_crossley_dissipation

- SDFormat path: `//model/link/collision/drake:proximity_properies/drake:hunt_crossley_dissipation`
- URDF path: `/robot/link/collision/drake:proximity_properties/drake:hunt_crossley_dissipation/@value`
- Syntax: Non-negative floating point value.

@subsubsection tag_drake_hunt_crossley_dissipation_semantics Semantics

If present, this element provides a value (units of inverse of velocity,
i.e. s/m) for the Hunt-Crossley dissipation model. It is stored in a
ProximityProperties object under `(material, hunt_crossley_dissipation)`.

@see drake::geometry::ProximityProperties,
@ref mbp_hydroelastic_materials_properties "Hydroelastic contact",
@ref mbp_dissipation_model "Modeling Dissipation"

@subsection tag_drake_hydroelastic_modulus drake:hydroelastic_modulus

- SDFormat path: `//model/link/collision/drake:proximity_properties/drake:hydroelastic_modulus`
- URDF path: `/robot/link/collision/drake:proximity_properties/drake:hydroelastic_modulus/@value`
- Syntax: Positive floating point value.

@subsubsection tag_drake_hydroelastic_modulus_semantics Semantics

If present, this element provides a value (units of pressure, i.e. Pa (N/m²))
for the hydroelastic modulus. It is stored in a ProximityProperties object
under `(hydroelastic, hydroelastic_modulus)`.

@see drake::geometry::ProximityProperties,
@ref mbp_hydroelastic_materials_properties "Hydroelastic contact",
@ref hug_properties

@subsection tag_drake_ignored_collision_filter_group drake:ignored_collision_filter_group

- SDFormat path: `//model/drake:collision_filter_group/drake:ignored_collision_filter_group`
- URDF path: `/robot/drake:collision_filter_group/drake:ignored_collision_filter_group/@name`
- Syntax: String.

@subsubsection tag_drake_ignored_collision_filter_group_semantics Semantics

The string names a collision filter group that will be paired with the parent
group when constructing filtering rules. It may name the parent group.

@see @ref tag_drake_collision_filter_group

@subsection tag_drake_joint drake:joint

- SDFormat path: `//model/drake:joint`
- URDF path: `/robot/drake:joint`
- Syntax: Attributes `name` (string) and `type` (choice: see below), and nested
          elements (see below).

@subsubsection tag_drake_joint_semantics Semantics

In both formats, the drake:joint element is used as a substitute for standard
joint elements, to allow support for non-standard joint types. The overall
semantics are the same as for a standard joint.

In SDFormat, the only supported `type` value is `planar`. The element must
contain nested `drake:parent`, `drake:child`, and `drake:damping` elements.

In URDF, supported `type` values are one of `ball`, `planar`, or
`universal`. The nested elements are the same as those defined by the standard
joint element.

@see @ref tag_drake_parent, @ref tag_drake_child, @ref tag_drake_damping

@subsection tag_drake_linear_bushing_rpy drake:linear_bushing_rpy

- SDFormat path: `//model/drake:linear_bushing_rpy`
- URDF path: `/robot/drake:linear_bushing_rpy`

- Syntax: Nested elements `drake:bushing_frameA`, `drake:bushing_frameC`,
  `drake:bushing_torque_stiffness`, `drake:bushing_torque_damping`,
  `drake:bushing_force_stiffness`, and `drake:bushing_force_damping`.

@subsubsection tag_drake_linear_bushing_rpy_semantics Semantics

This element adds a drake::multibody::LinearBushingRollPitchYaw to the model.

@see drake::multibody::LinearBushingRollPitchYaw,
@ref tag_drake_bushing_force_damping, @ref tag_drake_bushing_force_stiffness,
@ref tag_drake_bushing_frameA, @ref tag_drake_bushing_frameC,
@ref tag_drake_bushing_torque_damping, @ref tag_drake_bushing_torque_stiffness

@subsection tag_drake_member drake:member

- SDFormat path: `//model/drake:collission_filter_group/drake:member`
- URDF path: `/robot/drake:collission_filter_group/drake:member/@link`
- Syntax: String.

@subsubsection tag_drake_member_semantics Semantics

This element names a link (defined elsewhere in the model) to be a member of
the parent collision filter group.

@see @ref tag_drake_collision_filter_group

@subsection tag_drake_mesh_resolution_hint drake:mesh_resolution_hint

- SDFormat path: `//model/link/collision/drake:proximity_properties/drake:mesh_resolution_hint`
- URDF path: `/robot/link/collision/drake:proximity_properties/drake:mesh_resolution_hint/@value`
- Syntax: A positive floating point value.

@subsubsection tag_drake_mesh_resolution_hint_semantics Semantics

The provided value (in units of meters) helps select the resolution of meshes
generated from geometric primitives (sphere, cylinder, capsule, etc.). The
exact semantics depend on the geometry being generated. Within some practical
limits, smaller values will select shorter edge lengths and a finer mesh, larger
values will select longer edge lengths and a coarser mesh.

@see @ref tag_drake_proximity_properties, @ref hug_properties

@subsection tag_drake_mu_dynamic drake:mu_dynamic

- SDFormat path: `//model/link/collision/drake:proximity_properties/drake:mu_dynamic`
- URDF path: `/robot/link/collision/drake:proximity_properties/drake:mu_dynamic/@value`
- Syntax: A non-negative floating-point value.

@subsubsection tag_drake_mu_dynamic_semantics Semantics

The provided (dimensionless) value sets the dynamic friction parameter for
CoulombFriction. Refer to @ref stribeck_approximation for details on the
friction model.

@see @ref tag_drake_proximity_properties, drake::multibody::CoulombFriction,
@ref stribeck_approximation

@subsection tag_drake_mu_static drake:mu_static

- SDFormat path: `//model/link/collision/drake:proximity_properties/drake:mu_static`
- URDF path: `/robot/link/collision/drake:proximity_properties/drake:mu_static/@value`
- Syntax: A non-negative floating-point value.

@subsubsection tag_drake_mu_static_semantics Semantics

The provided (dimensionless) value sets the static friction parameter for
CoulombFriction. Refer to @ref stribeck_approximation for details on the
friction model.

@warning This value is ignored when modeling the multibody system with discrete
dynamics, refer to MultibodyPlant's constructor documentation for details, in
particular the parameter `time_step`.

@see @ref tag_drake_proximity_properties, drake::multibody::CoulombFriction,
@ref stribeck_approximation

@subsection tag_drake_parent drake:parent

- SDFormat path: `//model/drake:joint/drake:parent`
- URDF path: N/A
- Syntax: String.

@subsubsection tag_drake_parent_semantics Semantics

The string names a frame (defined elsewhere in the model) that is associated
with the parent link of the joint being defined.

@see @ref tag_drake_joint

@subsection tag_drake_point_contact_stiffness drake:point_contact_stiffness

- SDFormat path: `//model/link/collision/drake:proximity_properties/drake:point_contact_stiffness`
- URDF path: `/robot/link/collision/drake:proximity_properties/drake:point_contact_stiffness/@value`
- Syntax: Positive floating point value.

@subsubsection tag_drake_point_contact_stiffness_semantics Semantics

If present, this element provides a stiffness value (units of N/m) for point
contact calculations for this specific geometry. It is stored in a
ProximityProperties object under `(material, point_contact_stiffness)`.

@see @ref accessing_contact_properties, @ref mbp_penalty_method,
drake::geometry::ProximityProperties

@subsection tag_drake_proximity_properties drake:proximity_properties

- SDFormat path: `//model/link/collision/drake:proximity_properties`
- URDF path: `/robot/link/collision/drake:proximity_properties`
- Syntax: Nested elements; see below.

@subsubsection tag_drake_proximity_properties_semantics Semantics

If present, this element defines proximity properties for the parent link. The
following nested elements may be present:
- `drake:compliant_hydroelastic`
- `drake:hunt_crossley_dissipation`
- `drake:hydroelastic_modulus`
- `drake:mesh_resolution_hint`
- `drake:mu_dynamic`
- `drake:mu_static`
- `drake:point_contact_stiffness`
- `drake:rigid_hydroelastic`

@see @ref tag_drake_compliant_hydroelastic,
@ref tag_drake_hunt_crossley_dissipation,
@ref tag_drake_hydroelastic_modulus,
@ref tag_drake_mesh_resolution_hint,
@ref tag_drake_mu_dynamic,
@ref tag_drake_mu_static,
@ref tag_drake_point_contact_stiffness,
@ref tag_drake_rigid_hydroelastic,
drake::geometry::ProximityProperties

@subsection tag_drake_rigid_hydroelastic drake:rigid_hydroelastic

- SDFormat path: `//model/link/collision/drake:proximity_properties/drake:rigid_hydroelastic`
- URDF path: `/robot/link/collision/drake:proximity_properties/drake:rigid_hydroelastic`
- Syntax: Empty element.

@subsubsection tag_drake_rigid_hydroelastic_semantics Semantics

If present, this element sets the compliance type of the element being defined
to be rigid, as opposed to compliant, in hydroelastic contact models.

@see @ref tag_drake_proximity_properties, @ref creating_hydro_reps

@subsection tag_drake_rotor_inertia drake:rotor_inertia

- SDFormat path: `//model/link/joint/drake:rotor_inertia`
- URDF path: `/robot/link/joint/actuator/drake:rotor_inertia@value`
- Syntax: Non-negative floating point value.

@subsubsection tag_drake_rotor_inertia_semantics Semantics

Applies the indicated rotor inertia value to the appropriate JointActuator
object. Units are kg⋅m² for revolute joints, and kg for prismatic joints.

@see drake::multibody::JointActuator, @ref tag_drake_gear_ratio,
@ref reflected_inertia "Reflected Inertia"

*/
