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

@anchor multibody_parsing_sdf
<h2>SDFormat Support</h2>
Drake supports SDFormat files following the specification at
http://sdformat.org/spec. As of this writing, the supported spec version is
1.9, but check also the Drake release notes for current status.

For Drake extensions to SDFormat files, see @ref multibody_parsing_drake_extensions.

<h3>SDFormat Standard Tags Not Supported by Drake</h3>
TODO(rpoyner-tri): reference section covering sdf unsupported

@anchor multibody_parsing_urdf
<h2>URDF Support</h2>
Drake supports URDF files as described here: http://wiki.ros.org/urdf/XML.
TODO(rpoyner-tri): is the URDF spec versioned?

For Drake extensions to URDF format files, see
@ref multibody_parsing_drake_extensions.

<h3>URDF Standard Tags Not Supported by Drake</h3>
TODO(rpoyner-tri): reference section covering urdf unsupported

@anchor urdf_community_extensions
<h3>Commonly Used Non-standard URDF Tags Supported by Drake</h3>

<h4>capsule</h4>

- Path: `//geometry[parent::visual|parent::collision]/capsule`
- Syntax: Two attributes `radius` and `length`, each of which contain a
          non-negative floating point value.
- Semantics: see @ref tag_drake_capsule

@anchor multibody_parsing_drake_extensions
<h2>Drake Extensions</h2>

Drake extends both SDFormat and URDF in similar ways, to allow access to
Drakes-specific features. This section provides a compact reference to all Drake
extensions and their use in both formats.

Note that in both formats, it is proper to declare the `drake:` namespace
prefix. For SDFormat, declare it like this:

    <sdf xmlns:drake="http://drake.mit.edu" version="1.7">

For URDF, declare the namespace prefix like this:

    <robot xmlns:drake="http://drake.mit.edu" name="test_robot">

@anchor tag_drake_acceleration
<h3>drake:acceleration</h3>

- SDFormat path: `//model/joint/axis/limit/drake:acceleration`
- URDF path: `/robot/joint/limit/@drake:acceleration`
- Syntax: One floating point value, non-negative.

<h4>Semantics</h4>

If the joint type is one of (prismatic, revolute), apply acceleration limits at
(-VALUE, +VALUE). Units are determined by the type of joint, typically either
m/s or radians/s.

@anchor tag_drake_accepting_renderer
<h3>drake:accepting_renderer</h3>

- SDFormat path: `//model/link/visual/drake:accepting_renderer`
- URDF path: `/robot/link/visual/drake:accepting_renderer/@name`
- Syntax: String.

<h4>Semantics</h4>

The tag serves as a list of renderers for which this visual is targeted.

  - The _value_ of the tag is the name of the renderer.
  - If the _value_ is empty, that is a parsing error.
  - If no instance of `<drake:accepting_renderer>` every renderer will be given
    the chance to reify this visual geometry.
  - Multiple instances of this tag are allowed. Each instance adds a renderer to
    the list of targeted renderers.

This feature is one way to provide multiple visual representations of a body.

@anchor tag_drake_bushing_force_damping
<h3>drake:bushing_force_damping</h3>

- SDFormat path: `//model/drake:linear_bushing_rpy/drake:bushing_force_damping`
- URDF path: `/robot/drake:linear_bushing_rpy/drake:bushing_force_damping/@value`
- Syntax: Three floating point values.

<h4>Semantics</h4>

The three floating point values are formed into a vector and passed to the
constructor of drake::multibody::LinearBushingRollPitchYaw as the
`force_damping_constants` parameter.

@see @ref tag_drake_linear_bushing_rpy, drake::multibody::LinearBushingRollPitchYaw, @ref Basic_bushing_force_stiffness_and_damping

@anchor tag_drake_bushing_force_stiffness
<h3>drake:bushing_force_stiffness</h3>

- SDFormat path: `//model/drake:linear_bushing_rpy/drake:bushing_force_stiffness`
- URDF path: `/robot/drake:linear_bushing_rpy/drake:bushing_force_stiffness/@value`
- Syntax: Three floating point values.

<h4>Semantics</h4>

The three floating point values are formed into a vector and passed to the
constructor of drake::multibody::LinearBushingRollPitchYaw as the
`force_stiffness_constants` parameter.

@see @ref tag_drake_linear_bushing_rpy, drake::multibody::LinearBushingRollPitchYaw, @ref Basic_bushing_force_stiffness_and_damping

@anchor tag_drake_bushing_frameA
<h3>drake:bushing_frameA</h3>

- SDFormat path: `//model/drake:linear_bushing_rpy/drake:bushing_frameA`
- URDF path: `/robot/drake:linear_bushing_rpy/drake:bushing_frameA/@name`
- Syntax: String.

<h4>Semantics</h4>

The string names a frame (defined elsewhere in the model) that will be used to
construct a drake::multibody::Frame, which will then pass to the constructor of
drake::multibody::LinearBushingRollPitchYaw as the `frameA` parameter.

@see @ref tag_drake_linear_bushing_rpy, drake::multibody::LinearBushingRollPitchYaw

@anchor tag_drake_bushing_frameC
<h3>drake:bushing_frameC</h3>

- SDFormat path: `//model/drake:linear_bushing_rpy/drake:bushing_frameC`
- URDF path: `/robot/drake:linear_bushing_rpy/drake:bushing_frameC/@name`
- Syntax: String.

<h4>Semantics</h4>

The string names a frame (defined elsewhere in the model) that will be used to
construct a drake::multibody::Frame, which will then pass to the constructor of
drake::multibody::LinearBushingRollPitchYaw as the `frameC` parameter.

@see @ref tag_drake_linear_bushing_rpy, drake::multibody::LinearBushingRollPitchYaw

@anchor tag_drake_bushing_torque_damping
<h3>drake:bushing_torque_damping</h3>

- SDFormat path: `//model/drake:linear_bushing_rpy/drake:bushing_torque_damping`
- URDF path: `/robot/drake:linear_bushing_rpy/drake:bushing_torque_damping/@value`
- Syntax: Three floating point values.

<h4>Semantics</h4>

The three floating point values are formed into a vector and passed to the
constructor of drake::multibody::LinearBushingRollPitchYaw as the
`torque_damping_constants` parameter.

@see @ref tag_drake_linear_bushing_rpy, drake::multibody::LinearBushingRollPitchYaw, @ref Basic_bushing_torque_stiffness_and_damping

@anchor tag_drake_bushing_torque_stiffness
<h3>drake:bushing_torque_stiffness</h3>

- SDFormat path: `//model/drake:linear_bushing_rpy/drake:bushing_torque_stiffness`
- URDF path: `/robot/drake:linear_bushing_rpy/drake:bushing_torque_stiffness/@value`
- Syntax: Three floating point values.

<h4>Semantics</h4>

The three floating point values are formed into a vector and passed to the
constructor of drake::multibody::LinearBushingRollPitchYaw as the
`torque_stiffness_constants` parameter.

@see @ref tag_drake_linear_bushing_rpy, drake::multibody::LinearBushingRollPitchYaw, @ref Basic_bushing_torque_stiffness_and_damping

@anchor tag_drake_capsule
<h3>drake:capsule</h3>

- SDFormat path: `//geometry[parent::visual|parent::collision]/drake:capsule`
- URDF path: N/A
- Syntax: Two nested elements `radius` and `length`, each of which contain a
          non-negative floating point value.

<h4>Semantics</h4>

Constructs a drake::geometry::Capsule of the given radius and length, and adds
it to either the visual or collision geometry of the model.

SDFormat note: `drake:capsule` is proposed for deprecation; see
[issue #14387](https://github.com/RobotLocomotion/drake/issues/14837).

URDF note: drake suports a non-standard `capsule` tag with the same
semantics as `drake:capsule`. See @ref urdf_community_extensions.

@anchor tag_drake_child
<h3>drake:child</h3>

- SDFormat path: `//model/drake:joint/drake:child`
- URDF path: N/A
- Syntax: String.

<h4>Semantics</h4>

The string names a frame (defined elsewhere in the model) that is associated
the child link of the joint being defined.

@see @ref tag_drake_joint

@anchor tag_drake_collision_filter_group
<h3>drake:collision_filter_group</h3>

- SDFormat path: `//model/drake:collision_filter_group`
- URDF path: `/robot/drake:collision_filter_group`
- Syntax: Attributes `name` (string) and `ignore` (boolean); nested elements
          `drake:member` and `drake:ignored_collision_filter_group`.

<h4>Semantics</h4>

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

@anchor tag_drake_compliant_hydroelastic
<h3>drake:compliant_hydroelastic</h3>

- SDFormat path: `//model/link/collision/drake:proximity_properties/drake:compliant_hydroelastic`
- URDF path: `/robot/link/collision/drake:proximity_properties/drake:compliant_hydroelastic`
- Syntax: Empty element.

<h4>Semantics</h4>

If present, this element sets the compliance type of the element being defined
to be compliant, as opposed to rigid.

@see @ref tag_drake_proximity_properties, @ref MODULE_NOT_WRITTEN_YET

@anchor tag_drake_damping
<h3>drake:damping</h3>

- SDFormat path: `//model/drake:joint/drake:damping`
- URDF path: N/A
- Syntax: Three non-negative floating point values.

<h4>Semantics</h4>

If the parent joint type is planar, construct a vector from the values and pass
it to the `damping` parameter of the constructor of
drake::multibody::PlanarJoint. See that class for dicussion of units and
detailed semantics.

@anchor tag_drake_declare_convex
<h3>drake:declare_convex</h3>

- SDFormat path: `//geometry[parent::visual|parent::collision]/mesh/drake:declare_convex`
- URDF path: `//geometry[parent::visual|parent::collision]/mesh/drake:declare_convex`
- Syntax: Empty element.

<h4>Semantics</h4>

If present, this element causes Drake to treat the parent mesh geometry as
convex, without any further validation.

@anchor tag_drake_diffuse_map
<h3>drake:diffuse_map</h3>

- SDFormat path: `//model`
- URDF path: N/A
- Syntax: URI.

<h4>Semantics</h4>

@anchor tag_drake_elastic_modulus
<h3>drake:elastic_modulus</h3>

- SDFormat path: `//model`
- URDF path: `/robot`
- Syntax: String.

<h4>Semantics</h4>

@anchor tag_drake_ellipsoid
<h3>drake:ellipsoid</h3>

- SDFormat path: `//model`
- URDF path: `/robot`
- Syntax: String.

<h4>Semantics</h4>

@anchor tag_drake_gear_ratio
<h3>drake:gear_ratio</h3>

- SDFormat path: `//model`
- URDF path: `/robot`
- Syntax: String.

<h4>Semantics</h4>

@anchor tag_drake_half_space
<h3>drake:half_space</h3>

- SDFormat path: `//model`
- URDF path: `/robot`
- Syntax: String.

<h4>Semantics</h4>

@anchor tag_drake_hunt_crossley_dissipation
<h3>drake:hunt_crossley_dissipation</h3>

- SDFormat path: `//model`
- URDF path: `/robot`
- Syntax: String.

<h4>Semantics</h4>

@anchor tag_drake_hydroelastic_modulus
<h3>drake:hydroelastic_modulus</h3>

- SDFormat path: `//model`
- URDF path: `/robot`
- Syntax: String.

<h4>Semantics</h4>

@anchor tag_drake_ignored_collision_filter_group
<h3>drake:ignored_collision_filter_group</h3>

- SDFormat path: `//model`
- URDF path: `/robot`
- Syntax: String.

<h4>Semantics</h4>

@anchor tag_drake_joint
<h3>drake:joint</h3>

- SDFormat path: `//model`
- URDF path: `/robot`
- Syntax: String.

<h4>Semantics</h4>

@anchor tag_drake_linear_bushing_rpy
<h3>drake:linear_bushing_rpy</h3>

- SDFormat path: `//model`
- URDF path: `/robot`
- Syntax: String.

<h4>Semantics</h4>

@anchor tag_drake_member
<h3>drake:member</h3>

- SDFormat path: `//model`
- URDF path: `/robot`
- Syntax: String.

<h4>Semantics</h4>

@anchor tag_drake_mesh_resolution_hint
<h3>drake:mesh_resolution_hint</h3>

- SDFormat path: `//model`
- URDF path: `/robot`
- Syntax: String.

<h4>Semantics</h4>

@anchor tag_drake_mu_dynamic
<h3>drake:mu_dynamic</h3>

- SDFormat path: `//model`
- URDF path: `/robot`
- Syntax: String.

<h4>Semantics</h4>

@anchor tag_drake_mu_static
<h3>drake:mu_static</h3>

- SDFormat path: `//model`
- URDF path: `/robot`
- Syntax: String.

<h4>Semantics</h4>

@anchor tag_drake_parent
<h3>drake:parent</h3>

- SDFormat path: `//model`
- URDF path: `/robot`
- Syntax: String.

<h4>Semantics</h4>

@anchor tag_drake_point_contact_stiffness
<h3>drake:point_contact_stiffness</h3>

- SDFormat path: `//model`
- URDF path: `/robot`
- Syntax: String.

<h4>Semantics</h4>

@anchor tag_drake_proximity_properties
<h3>drake:proximity_properties</h3>

- SDFormat path: `//model`
- URDF path: `/robot`
- Syntax: String.

<h4>Semantics</h4>

@anchor tag_drake_rigid_hydroelastic
<h3>drake:rigid_hydroelastic</h3>

- SDFormat path: `//model`
- URDF path: `/robot`
- Syntax: String.

<h4>Semantics</h4>

@anchor tag_drake_rotor_inertia
<h3>drake:rotor_inertia</h3>

- SDFormat path: `//model`
- URDF path: `/robot`
- Syntax: String.

<h4>Semantics</h4>

@anchor tag_drake_soft_hydroelastic
<h3>drake:soft_hydroelastic</h3>

- SDFormat path: `//model`
- URDF path: `/robot`
- Syntax: String.

<h4>Semantics</h4>


*/
