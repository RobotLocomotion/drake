/** @file
 Doxygen-only documentation for @ref multibody_parsing. */

/**
@addtogroup multibody_parsing

Drake's drake::multibody::Parser accepts model files written in a variety of
input formats. Drake's parsing of URDF, SDFormat, and MJCF (Mujoco XML) has
Drake-specific extensions and limitations.

The result of the parse is an in-memory model realized within
drake::multibody::MultibodyPlant and (optionally)
drake::geometry::SceneGraph. Note that parses that do not use a `SceneGraph`
will effectively ignore geometric model elements, especially `//visual` and
`//collision` elements.

In the reference sections below, when discussing XML formats, the relevant
usage paths for various tags are indicated using
[XPath](https://www.w3.org/TR/xpath-31/) notation.

@section multibody_parsing_dmd Drake Model Directives Support

Drake Model Directives is a Drake-native model description format, primarily
intended for combining models written in other formats into complex scenes. It
is YAML based, and follows a limited data schema.  See
`multibody/parsing/README_model_directives.md` for more detail.

@section multibody_parsing_mjcf MJCF (Mujoco XML) Support

There is limited, undocumented support for parsing MJCF (Mujoco XML) files. The
files are recognized by an .xml file extension. The scope of features that are
actually supported still need to be documented.

<!-- TODO(rpoyner-tri): document mujoco format support -->

@section multibody_parsing_sdf SDFormat Support
Drake supports SDFormat files following the specification at
http://sdformat.org/spec. As of this writing, the supported spec version is
1.9, but check also the Drake release notes for current status.

For Drake extensions to SDFormat files, see
@ref multibody_parsing_drake_extensions.

Drake's parser does not implement all of the features of SDFormat. In the
future, we intend to update this documentation to itemize what isn't supported.

@subsection model_composition SDFormat model composition

Drake's SDFormat parsing supports composing multiple models into a single
model, via lexical nesting and file inclusion. The file inclusion feature
supports both SDFormat files and URDF files. Note that included URDF files pass
through the Drake URDF parser, with all of its extensions and limitations.

For full details, see the
<a href='http://sdformat.org/tutorials?tut=composition_proposal#model-composition-proposed-behavior'>SDFormat documentation of model composition.</a>

An important feature of SDFormat model composition is the ability to
cross-reference features of other models. Cross-references are denoted by using
scoped names.

@subsubsection scoped_names Scoped names

Scoped names allow referring to an element of a model nested within the current
model. They take the form of some number of nested model names, plus the
element name, all joined by the delimiter `::`. Names not using the `::`
delimiter are not considered scoped names; they are used to define or refer to
elements of the current model. For example, suppose that model A contains model
B, which in turn contains model C. Here are some valid scoped names:

- within A:
  - `some_frame_in_A`
  - `B::some_frame_in_B`
  - `B::C::some_frame_in_C`
- within B:
  - `some_frame_in_B`
  - `C::some_frame_in_C`
- within C:
  - `some_frame_in_C`

Note that (deliberately) there is no way to refer to elements outward or upward
from the current model; all names are relative to the current model and can
only refer to the current or a nested model. In particular, names must not
start with `::`; there is no way to denote any "outer" or "outermost" scope.

For a detailed design discussion with examples, see the
<a href='http://sdformat.org/tutorials?tut=composition_proposal#1-3-name-scoping-and-cross-referencing'>SDFormat documentation of name scoping.</a>

@section multibody_parsing_urdf URDF Support
Drake supports URDF files as described here: http://wiki.ros.org/urdf/XML.

For Drake extensions to URDF format files, see
@ref multibody_parsing_drake_extensions.

@subsection multibody_parsing_urdf_unsupported URDF not supported by Drake

Drake's parser does not implement all of the features of URDF. Here is a list
of known URDF features that Drake does not use. For each, the parser applies
one of several treatments:

- Issue a warning that the tag is unused.
- Ignore silently, as documented below.
- Apply special treatment, as documented below.

@subsubsection urdf_ignored_warning Tags that provoke a warning

- `/robot/@version`
- `/robot/joint/calibration`
- `/robot/joint/safety_controller`
- `/robot/link/@type`
- `/robot/link/collision/verbose`
- `/robot/transmission/@name`
- `/robot/transmission/flexJoint`
- `/robot/transmission/gap_joint`
- `/robot/transmission/leftActuator`
- `/robot/transmission/passive_joint`
- `/robot/transmission/rightActuator`
- `/robot/transmission/rollJoint`
- `/robot/transmission/use_simulated_gripper_joint`

@subsubsection urdf_ignored_silent Tags ignored silently

- `/robot/gazebo`
- `/robot/transmission/actuator/hardwareInterface`
- `/robot/transmission/joint/hardwareInterface`

@subsubsection urdf_ignored_special Tags given special treatment.

- `/robot/transmission/actuator/mechanicalReduction`
- `/robot/transmission/mechanicalReduction`

Both versions of `mechanicalReduction` will be silently ignored if the supplied
value is 1; otherwise the they will provoke a warning that the value is being
ignored.

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
- @ref tag_drake_ball_constraint
- @ref tag_drake_ball_constraint_body_A
- @ref tag_drake_ball_constraint_body_B
- @ref tag_drake_ball_constraint_p_AP
- @ref tag_drake_ball_constraint_p_BQ
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
- @ref tag_drake_controller_gains
- @ref tag_drake_damping
- @ref tag_drake_declare_convex
- @ref tag_drake_diffuse_map
- @ref tag_drake_ellipsoid
- @ref tag_drake_gear_ratio
- @ref tag_drake_hunt_crossley_dissipation
- @ref tag_drake_hydroelastic_margin
- @ref tag_drake_hydroelastic_modulus
- @ref tag_drake_ignored_collision_filter_group
- @ref tag_drake_illustration_properties
- @ref tag_drake_joint
- @ref tag_drake_linear_bushing_rpy
- @ref tag_drake_member
- @ref tag_drake_member_group
- @ref tag_drake_mesh_resolution_hint
- @ref tag_drake_mimic
- @ref tag_drake_mu_dynamic
- @ref tag_drake_mu_static
- @ref tag_drake_parent
- @ref tag_drake_perception_properties
- @ref tag_drake_point_contact_stiffness
- @ref tag_drake_proximity_properties
- @ref tag_drake_relaxation_time
- @ref tag_drake_rigid_hydroelastic
- @ref tag_drake_rotor_inertia
- @ref tag_drake_screw_thread_pitch
- @ref tag_drake_visual

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

Specifying this tag for a visual whose perception role has been disabled will
emit a warning.

@see @ref drake_tag_perception_properties

@subsection tag_drake_ball_constraint drake:ball_constraint

- SDFormat path: `//model/drake:ball_constraint`
- URDF path: `/robot/drake:ball_constraint`
- Syntax: Nested elements @ref tag_drake_ball_constraint_body_A, @ref
tag_drake_ball_constraint_body_B, @ref tag_drake_ball_constraint_p_AP, and @ref
tag_drake_ball_constraint_p_BQ

@subsection tag_drake_ball_constraint_semantics Semantics

The element adds a ball constraint to the model via
drake::multibody::MultibodyPlant::AddBallConstraint().

@subsection tag_drake_ball_constraint_body_A drake:ball_constraint_body_A

- SDFormat path: `//model/drake:ball_constraint/drake:ball_constraint_body_A`
- URDF path: `/robot/drake:ball_constraint/drake:ball_constraint_body_A/@name`
- Syntax: String.

@subsection tag_drake_ball_constraint_body_A_semantics Semantics

The string names a body (expected to already be defined by this model) that
will be passed to drake::multibody::MultibodyPlant::AddBallConstraint()
as the `body_A` parameter.

@see @ref tag_drake_ball_constraint,
drake::multibody::MultibodyPlant::AddBallConstraint()

@subsection tag_drake_ball_constraint_body_B drake:ball_constraint_body_B

- SDFormat path: `//model/drake:ball_constraint/drake:ball_constraint_body_B`
- URDF path: `/robot/drake:ball_constraint/drake:ball_constraint_body_B/@name`
- Syntax: String.

@subsection tag_drake_ball_constraint_body_B_semantics Semantics

The string names a body (expected to already be defined by this model) that
will be passed to drake::multibody::MultibodyPlant::AddBallConstraint()
as the `body_B` parameter.

@see @ref tag_drake_ball_constraint,
drake::multibody::MultibodyPlant::AddBallConstraint()

@subsection tag_drake_ball_constraint_p_AP drake:ball_constraint_p_AP

- SDFormat path: `//model/drake:ball_constraint/drake:ball_constraint_p_AP`
- URDF path: `/robot/drake:ball_constraint/drake:ball_constraint_p_AP/@value`
- Syntax: Three floating point values.

@subsection tag_drake_ball_constraint_p_AP_semantics Semantics

The three floating point values (units of meters) are formed into a
vector and passed into drake::multibody::MultibodyPlant::AddBallConstraint() as
the `p_AP` parameter.

@see @ref tag_drake_ball_constraint,
drake::multibody::MultibodyPlant::AddBallConstraint()

@subsection tag_drake_ball_constraint_p_BQ drake:ball_constraint_p_BQ

- SDFormat path: `//model/drake:ball_constraint/drake:ball_constraint_p_BQ`
- URDF path: `/robot/drake:ball_constraint/drake:ball_constraint_p_BQ/@value`
- Syntax: Three floating point values.

@subsection tag_drake_ball_constraint_p_BQ_semantics Semantics

The three floating point values (units of meters) are formed into a
vector and passed into drake::multibody::MultibodyPlant::AddBallConstraint() as
the `p_BQ` parameter.

@see @ref tag_drake_ball_constraint,
drake::multibody::MultibodyPlant::AddBallConstraint()

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
          `drake:member`, `drake:member_group`, and
          `drake:ignored_collision_filter_group`.

@subsubsection tag_drake_collision_filter_group_semantics Semantics

This element names a group of bodies to participate in collision filtering
rules. If the `ignore` attribute is present and true-valued, the entire element
is skipped during parsing. The nested elements must included one or more
`drake:member` or `drake:member_group` elements, and zero or more
`drake:ignored_collision_filter_group` elements.

This element defines a new group name that is only available during parsing. It
must not be a scoped name.

Collision filtering rules are only constructed with **pairs** of groups, where
both sides of the pair may name the same group. The
`drake:ignored_collision_filter_group` element establishes a pairing between
the group it names and the group within which it is nested. A pair containing
different collision groups excludes collisions between members of those groups
(see drake::geometry::CollisionFilterDeclaration::ExcludeBetween()). A pair
naming the same group twice excludes collisions within the group (see
drake::geometry::CollisionFilterDeclaration::ExcludeWithin()).

@see @ref tag_drake_member, @ref tag_drake_member_group, @ref tag_drake_ignored_collision_filter_group, @ref scoped_names

@subsection tag_drake_compliant_hydroelastic drake:compliant_hydroelastic

- SDFormat path: `//model/link/collision/drake:proximity_properties/drake:compliant_hydroelastic`
- URDF path: `/robot/link/collision/drake:proximity_properties/drake:compliant_hydroelastic`
- Syntax: Empty element.

@subsubsection tag_drake_compliant_hydroelastic_semantics Semantics

If present, this element sets the compliance type of the element being defined
to be compliant, as opposed to rigid, in hydroelastic contact models.

@see @ref tag_drake_proximity_properties, @ref creating_hydro_reps

@subsection tag_drake_controller_gains drake:controller_gains

- SDFormat path: `//model/joint/drake:controller_gains`
- URDF path: `/robot/joint/actuator/drake:controller_gains`
- Syntax: Two attributes `p` (proportional gain) containing a positive floating
          point value and `d` (derivative gain) containing a non-negative
          floating point value.

@subsection tag_drake_controller_gains_semantics Semantics

If present, this element provides proportional and derivative gains for a low
level PD controller for the drake::multibody::JointActuator associated with the
drake::multibody::Joint the element is defined under. It is stored in a
drake::multibody::PdControllerGains object in the
drake::multibody::JointActuator class. Both attributes `p` and `d` are required.

@see @ref mbp_actuation, @ref pd_controlled_joint_actuator

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
`/robot/joint/dynamics/@damping` attribute.

@subsection tag_drake_declare_convex drake:declare_convex

- SDFormat path: `//geometry[parent::visual|parent::collision]/mesh/drake:declare_convex`
- URDF path: `//geometry[parent::visual|parent::collision]/mesh/drake:declare_convex`
- Syntax: Empty element.

@subsubsection tag_drake_declare_convex_semantics Semantics

If present, this element causes Drake to use the convex hull of the named mesh
geometry. The resulting geometry in memory will be drake::geometry::Convex,
rather than drake::geometry::Mesh.

@see drake::geometry::Convex, drake::geometry::Mesh

@subsection tag_drake_diffuse_map drake:diffuse_map

- SDFormat path: `//model/link/visual/material/drake:diffuse_map`
- URDF path: N/A (see a note below).
- Syntax: URI.

@subsubsection tag_drake_diffuse_map_semantics Semantics

If present, this element indicates (by filename or `package:` URI) a PNG file
to use as a diffuse texture map. The resolved path name is stored in a
PerceptionProperties object under `(phong, diffuse_map)`. URDF provides a
built-in tag, i.e., `//robot/material/texture` or
`//robot/link/visual/material/texture`, to specify a diffuse texture map for a
link.

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

- SDFormat path: `//model/joint/drake:gear_ratio`
- URDF path: `/robot/joint/actuator/drake:gear_ratio@value`
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
@ref hydro_model_parameters "Hydroelastic contact parameters",
@ref mbp_dissipation_model "Modeling Dissipation"

@subsection tag_drake_hydroelastic_margin drake:hydroelastic_margin

- SDFormat path: `//model/link/collision/drake:proximity_properties/drake:hydroelastic_margin`
- URDF path: `/robot/link/collision/drake:proximity_properties/drake:hydroelastic_margin/@value`
- Syntax: Non-negative floating point value.

@subsubsection tag_drake_hydroelastic_margin_semantics Semantics

If present, this element provides a value (units of length, i.e. m)
for the hydroelastic margin. It is stored in a ProximityProperties object
under `(hydroelastic, margin)`.

@see drake::geometry::ProximityProperties,
@ref hydro_model_parameters "Hydroelastic contact parameters",
@ref hug_properties, @ref hydro_margin

@subsection tag_drake_hydroelastic_modulus drake:hydroelastic_modulus

- SDFormat path: `//model/link/collision/drake:proximity_properties/drake:hydroelastic_modulus`
- URDF path: `/robot/link/collision/drake:proximity_properties/drake:hydroelastic_modulus/@value`
- Syntax: Positive floating point value.

@subsubsection tag_drake_hydroelastic_modulus_semantics Semantics

If present, this element provides a value (units of pressure, i.e. Pa (N/m²))
for the hydroelastic modulus. It is stored in a ProximityProperties object
under `(hydroelastic, hydroelastic_modulus)`.

@see drake::geometry::ProximityProperties,
@ref hydro_model_parameters "Hydroelastic contact parameters",
@ref hug_properties

@subsection tag_drake_ignored_collision_filter_group drake:ignored_collision_filter_group

- SDFormat path: `//model/drake:collision_filter_group/drake:ignored_collision_filter_group`
- URDF path: `/robot/drake:collision_filter_group/drake:ignored_collision_filter_group/@name`
- Syntax: String.

@subsubsection tag_drake_ignored_collision_filter_group_semantics Semantics

The string names a collision filter group that will be paired with the parent
group when constructing filtering rules. It may name the parent group.

In SDFormat files only, the name may refer to a group within a nested model
(either URDF or SDFormat) by using a scoped name.

@see @ref tag_drake_collision_filter_group, @ref scoped_names

@subsection tag_drake_illustration_properties drake:illustration_properties

- SDFormat path: `//model/link/visual/drake:illustration_properties`
- URDF path: n/a
- Syntax: Single attribute: `enabled` (bool).

@subsubsection tag_drake_illustration_properties_semantics Semantics

`<visual>` geometries are assigned illustration roles by default. Their
appearance in visualizers reflect the materials associated with the `<visual>`
tag (or, as for meshes, the materials associated with the mesh). If no materials
are defined, then they pick up whatever default materials the visualizers
define. A `<visual>` tag can _opt out_ of the illustration role by setting the
`enabled` attribute to `false`. Setting it to `true` is equivalent to omitting
the tag completely.

Note: if a `<visual>` tag has disabled both illustration properties _and_
perception properties, a warning will be emitted. Essentially, the model
defines a geometry that would be consumed by typical loaders, but it has been
completely disabled for Drake. The definition for Drake should _refine_ the more
generic model, but it shouldn't consist of an arbitrarily different set of
visuals.

@see @ref tag_drake_perception_properties
@see @ref tag_drake_visual

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

In URDF, supported `type` values are one of `ball`, `planar`, `screw` or
`universal`. The nested elements are the same as those defined by the standard
joint element with the exception of the `screw` joint type, which requires
a nested `drake:screw_thread_pitch` element.

@see @ref tag_drake_parent, @ref tag_drake_child, @ref tag_drake_damping,
@ref tag_drake_screw_thread_pitch

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

- SDFormat path: `//model/drake:collision_filter_group/drake:member`
- URDF path: `/robot/drake:collision_filter_group/drake:member/@link`
- Syntax: String.

@subsubsection tag_drake_member_semantics Semantics

This element names a link (defined elsewhere in the model) to be a member of
the parent collision filter group.

In SDFormat files only, the name may refer to a link within a nested model
(either URDF or SDFormat) by using a scoped name.

@see @ref tag_drake_collision_filter_group, @ref scoped_names

@subsection tag_drake_member_group drake:member_group

- SDFormat path: `//model/drake:collision_filter_group/drake:member_group`
- URDF path: `/robot/drake:collision_filter_group/drake:member_group/@name`
- Syntax: String.

@subsubsection tag_drake_member_group_semantics Semantics

This element names a collision filter group (defined elsewhere in the model), all
of whose members become members of the parent collision filter group.

In SDFormat files only, the name may refer to a link within a nested model
(either URDF or SDFormat) by using a scoped name.

@see @ref tag_drake_collision_filter_group, @ref scoped_names

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

@subsection tag_drake_mimic drake:mimic

- SDFormat path: `//model/joint/drake:mimic`
- URDF path: unsupported
- Syntax: Attributes `joint` (string), `multiplier` (double) and `offset` (double)

@subsubsection tag_drake_mimic_semantics Semantics

This tag has equivalent semantics to those of the native URDF <mimic> tag. If
`q0` is the position of the `<joint>` and `q1` the position of the joint
specified by the `joint` attribute, the two joints are constrained to enforce
the relation: `q0 = multiplier * q1 + offset`. The units of `multiplier` and
`offset` depend on the type of joints specified. This tag only supports single
degree of freedom joints that exist in the same model instance.

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

@warning Both `mu_dynamic` and `mu_static` are used by MultibodyPlant when the plant
`time_step=0`, but only `mu_dynamic` is used when `time_step>0`. Refer to
MultibodyPlant's constructor documentation for details.

@see @ref tag_drake_proximity_properties, drake::multibody::CoulombFriction,
@ref stribeck_approximation

@subsection tag_drake_parent drake:parent

- SDFormat path: `//model/drake:joint/drake:parent`
- URDF path: N/A
- Syntax: String.

@subsection tag_drake_perception_properties drake:perception_properties

- SDFormat path: `//model/link/visual/drake:perception_properties`
- URDF path: n/a
- Syntax: Single attribute: `enabled` (bool).

@subsubsection tag_drake_perception_properties_semantics Semantics

`<visual>` geometries are assigned perception roles by default. Their appearance
in renderers reflect the materials associated with the `<visual>` tag (or, as
for meshes, the materials associated with the mesh). If no materials are
defined, then they pick up whatever default materials the renderers define. A
`<visual>` tag can _opt out_ of the perception role by setting the `enabled`
attribute to `false`. Setting it to `true` is equivalent to omitting the tag
completely.

Note: if a `<visual>` tag has disabled both perception properties _and_
illustration properties, a warning will be emitted. Essentially, the model
defines a geometry that would be consumed by typical loaders, but it has been
completely disabled for Drake. The definition for Drake should _refine_ the more
generic model, but it shouldn't consist of an arbitrarily different set of
visuals.

@see @ref tag_drake_illustration_properties
@see @ref tag_drake_visual

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

@see @ref accessing_contact_properties "Accessing point contact parameters",
@ref point_forces_modeling "Point Contact Forces", and
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

@subsection tag_drake_relaxation_time drake:relaxation_time

- SDFormat path: `//model/link/collision/drake:proximity_properies/drake:relaxation_time`
- URDF path: `/robot/link/collision/drake:proximity_properties/drake:relaxation_time/@value`
- Syntax: Non-negative floating point value.

@subsubsection tag_drake_relaxation_time_semantics Semantics

If present, this element provides a value (units of time, i.e. seconds) for a
linear Kelvin-Voigt model of dissipation. It is stored in a ProximityProperties
object under `(material, relaxation_time)`.

@see drake::geometry::ProximityProperties,
@ref mbp_dissipation_model "Modeling Dissipation"

@subsection tag_drake_rigid_hydroelastic drake:rigid_hydroelastic

- SDFormat path: `//model/link/collision/drake:proximity_properties/drake:rigid_hydroelastic`
- URDF path: `/robot/link/collision/drake:proximity_properties/drake:rigid_hydroelastic`
- Syntax: Empty element.

@subsubsection tag_drake_rigid_hydroelastic_semantics Semantics

If present, this element sets the compliance type of the element being defined
to be rigid, as opposed to compliant, in hydroelastic contact models.

@see @ref tag_drake_proximity_properties, @ref creating_hydro_reps

@subsection tag_drake_rotor_inertia drake:rotor_inertia

- SDFormat path: `//model/joint/drake:rotor_inertia`
- URDF path: `/robot/joint/actuator/drake:rotor_inertia@value`
- Syntax: Non-negative floating point value.

@subsubsection tag_drake_rotor_inertia_semantics Semantics

Applies the indicated rotor inertia value to the appropriate JointActuator
object. Units are kg⋅m² for revolute joints, and kg for prismatic joints.

@see drake::multibody::JointActuator, @ref tag_drake_gear_ratio,
@ref reflected_inertia "Reflected Inertia"

@subsection tag_drake_screw_thread_pitch drake:screw_thread_pitch

- SDFormat path: `//model/joint/screw_thread_pitch` <br/>
  Note this is **not** the custom attribute.
- URDF path: `/robot/joint/actuator/drake:screw_thread_pitch@value`
- Syntax: Non-zero floating point value.

@subsubsection tag_drake_screw_thread_pitch_semantics Semantics

Applies the indicated thread pitch value to the appropriate ScrewJoint object.
This kinematic parameter specifies the axial distance traveled for each
revolution of the joint. Units are m/revolution, with a positive value
corresponding to a right-handed thread.

@see drake::multibody::ScrewJoint

@subsection tag_drake_visual drake:visual

- SDFormat path: `//model/link`
- URDF path: n/a
- Syntax: Identical to SDFormat's `<visual>` tag, but with the `drake` namespace
  prepended. All tags that can be found under the `<visual>` tag (e.g.,
  `<pose>`, `<geometry>`, `<sphere>`, etc.) can be included under the
  `<drake:visual>` tag, provided they have the `drake` namespace affixed (e.g.,
  `<drake:pose>`, `<drake:geometry>`, `<drake:sphere>`, etc.).

@subsubsection tag_drake_visual_semantics Semantics

The `<drake:visual>` tag is provided for the purpose of defining visual elements
that only Drake will consume. This could be used to do Drake-specific
augmentation of a model. But the main intended use case is for associating
*different* geometries with a link based on its role. Use one geometry for the
illustration role and another for the perception role. To use different
geometries per role:

  1. Choose the illustration and perception geometries.
  2. Decide which geometry you want to serve as the default geometry in other
     SDFormat loaders.
  3. Use the SDFormat `<visual>` tag for that default geometry (including all
     typical descendant tags).
  4. In the `<visual>` tag, include the `<drake:??_properties>` tag of the
     role this visual should *not* serve. Set its `enabled` attribute to
     `false`. Remember, by default the visual would get both roles, so you have
     to opt out of the role you don't want the geometry to have.
  5. As a sibling to that main `<visual>`, add a `<drake:visual>` tag and define
     its subtree as you normally would (making sure to preface everything with
     the `drake:` namespace).
  6. Under the `<drake:visual>` tag, include the _other_ `<drake:??_properties>`
     tag and set its `enabled` tag to `false`.

This will allow Drake to use different visual representations for each visual
role but still keep a more general SDFormat definition for other loaders.

Note: disabling both visual roles on a `<drake:visual>` element will _not_
emit a warning as it would for doing the same to a `<visual>` tag.

@see @ref tag_drake_perception_properties
@see @ref tag_drake_illustration_properties

*/
