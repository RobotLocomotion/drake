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

// #include "drake/multibody/meshcat/contact_visualizer.h"
// #include "drake/multibody/meshcat/contact_visualizer_params.h"
// #include "drake/multibody/meshcat/hydroelastic_contact_visualizer.h"
// #include "drake/multibody/meshcat/joint_sliders.h"
// #include "drake/multibody/meshcat/meshcat_mouse_spring.h"
// #include "drake/multibody/meshcat/point_contact_visualizer.h"

// Symbol: pydrake_doc_multibody_meshcat
constexpr struct /* pydrake_doc_multibody_meshcat */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::multibody
    struct /* multibody */ {
      // Symbol: drake::multibody::meshcat
      struct /* meshcat */ {
        // Symbol: drake::multibody::meshcat::ContactVisualizer
        struct /* ContactVisualizer */ {
          // Source: drake/multibody/meshcat/contact_visualizer.h
          const char* doc =
R"""(ContactVisualizer is a system that publishes a ContactResults to
geometry∷Meshcat; For point contact results, it draws double-sided
arrows at the location of the contact force with length scaled by the
magnitude of the contact force. For hydroelastic contact, it draws
single-sided arrows at the centroid of the contact patch, one for
force and one for the moment of the contact results. The length of
these vectors are scaled by the magnitude of the contact force/moment.
The direction of the arrow is essentially arbitrary (based on the
GeometryIds) but is stable during a simulation. The most common use of
this system is to connect its input port to the contact results output
port of a MultibodyPlant.

.. pydrake_system::

    name: ContactVisualizer
    input_ports:
    - contact_results
    - query_object (optional)

Warning:
    In the current implementation, ContactVisualizer methods must be
    called from the same thread where the class instance was
    constructed. For example, running multiple simulations in parallel
    using the same ContactVisualizer instance is not yet supported. We
    may generalize this in the future.)""";
          // Symbol: drake::multibody::meshcat::ContactVisualizer::AddToBuilder
          struct /* AddToBuilder */ {
            // Source: drake/multibody/meshcat/contact_visualizer.h
            const char* doc_4args_builder_plant_meshcat_params =
R"""(Adds a ContactVisualizer and connects it to the given MultibodyPlant's
multibody∷ContactResults-valued output port and
geometry∷QueryObject-valued output port. The ContactVisualizer's name
(see systems∷SystemBase∷set_name) will be set to a sensible default
value, unless the default name was already in use by another system.)""";
            // Source: drake/multibody/meshcat/contact_visualizer.h
            const char* doc_5args_builder_contact_results_port_query_object_port_meshcat_params =
R"""(Adds a ContactVisualizer and connects it to the given
multibody∷ContactResults-valued output port and the given
geometry∷QueryObject-valued output port. The ContactVisualizer's name
(see systems∷SystemBase∷set_name) will be set to a sensible default
value, unless the default name was already in use by another system.)""";
            // Source: drake/multibody/meshcat/contact_visualizer.h
            const char* doc_4args_builder_contact_results_port_meshcat_params =
R"""(Adds a ContactVisualizer and connects it to the given
multibody∷ContactResults-valued output port. The ContactVisualizer's
name (see systems∷SystemBase∷set_name) will be set to a sensible
default value, unless the default name was already in use by another
system.

Warning:
    This overload is dispreferred because it cannot show any geometry
    names in the visualizer.)""";
          } AddToBuilder;
          // Symbol: drake::multibody::meshcat::ContactVisualizer::ContactVisualizer<T>
          struct /* ctor */ {
            // Source: drake/multibody/meshcat/contact_visualizer.h
            const char* doc =
R"""(Creates an instance of ContactVisualizer)""";
            // Source: drake/multibody/meshcat/contact_visualizer.h
            const char* doc_copyconvert =
R"""(Scalar-converting copy constructor. See system_scalar_conversion. It
should only be used to convert *from* double *to* other scalar types.)""";
          } ctor;
          // Symbol: drake::multibody::meshcat::ContactVisualizer::Delete
          struct /* Delete */ {
            // Source: drake/multibody/meshcat/contact_visualizer.h
            const char* doc =
R"""(Calls geometry∷Meshcat∷Delete(std∷string_view path), with the path set
to ``prefix``. Since this visualizer will only ever add geometry under
this prefix, this will remove all geometry/transforms added by the
visualizer, or by a previous instance of this visualizer using the
same prefix. Use the ``delete_on_initialization_event`` in the
parameters to determine whether this should be called on
initialization.)""";
          } Delete;
          // Symbol: drake::multibody::meshcat::ContactVisualizer::contact_results_input_port
          struct /* contact_results_input_port */ {
            // Source: drake/multibody/meshcat/contact_visualizer.h
            const char* doc =
R"""(Returns the multibody∷ContactResults-valued input port. It should be
connected to MultibodyPlant's multibody∷ContactResults-valued output
port. Failure to do so will cause a runtime error when attempting to
broadcast messages.)""";
          } contact_results_input_port;
          // Symbol: drake::multibody::meshcat::ContactVisualizer::query_object_input_port
          struct /* query_object_input_port */ {
            // Source: drake/multibody/meshcat/contact_visualizer.h
            const char* doc =
R"""(Returns the geometry∷QueryObject-valued input port. It should be
(optionally) connected to SceneGraph's get_query_output_port().
Failure to do so will prevent the display from showing names for the
geometry.)""";
          } query_object_input_port;
        } ContactVisualizer;
        // Symbol: drake::multibody::meshcat::ContactVisualizerParams
        struct /* ContactVisualizerParams */ {
          // Source: drake/multibody/meshcat/contact_visualizer_params.h
          const char* doc =
R"""(The set of parameters for configuring ContactVisualizer.)""";
          // Symbol: drake::multibody::meshcat::ContactVisualizerParams::Serialize
          struct /* Serialize */ {
            // Source: drake/multibody/meshcat/contact_visualizer_params.h
            const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
          } Serialize;
          // Symbol: drake::multibody::meshcat::ContactVisualizerParams::color
          struct /* color */ {
            // Source: drake/multibody/meshcat/contact_visualizer_params.h
            const char* doc =
R"""(The color used to draw the point contact force arrows.)""";
          } color;
          // Symbol: drake::multibody::meshcat::ContactVisualizerParams::delete_on_initialization_event
          struct /* delete_on_initialization_event */ {
            // Source: drake/multibody/meshcat/contact_visualizer_params.h
            const char* doc =
R"""(Determines whether to send a Meshcat∷Delete(prefix) message on an
initialization event to remove any visualizations e.g. from a previous
simulation. See declare_initialization_events "Declare initialization
events" for more information.)""";
          } delete_on_initialization_event;
          // Symbol: drake::multibody::meshcat::ContactVisualizerParams::force_threshold
          struct /* force_threshold */ {
            // Source: drake/multibody/meshcat/contact_visualizer_params.h
            const char* doc =
R"""(The threshold (in N) below which forces will no longer be drawn. The
ContactVisualizer constructor enforces that this must be strictly
positive; zero forces do not have a meaningful direction and cannot be
visualized.)""";
          } force_threshold;
          // Symbol: drake::multibody::meshcat::ContactVisualizerParams::hydro_force_color
          struct /* hydro_force_color */ {
            // Source: drake/multibody/meshcat/contact_visualizer_params.h
            const char* doc =
R"""(The color used to draw the hydroelastic contact force arrows.)""";
          } hydro_force_color;
          // Symbol: drake::multibody::meshcat::ContactVisualizerParams::hydro_moment_color
          struct /* hydro_moment_color */ {
            // Source: drake/multibody/meshcat/contact_visualizer_params.h
            const char* doc =
R"""(The color used to draw the hydroelastic contact moment arrows.)""";
          } hydro_moment_color;
          // Symbol: drake::multibody::meshcat::ContactVisualizerParams::moment_threshold
          struct /* moment_threshold */ {
            // Source: drake/multibody/meshcat/contact_visualizer_params.h
            const char* doc =
R"""(The threshold (in N⋅m) below which moments will no longer be drawn.
The ContactVisualizer constructor enforces that this must be strictly
positive; zero moments do not have a meaningful direction and cannot
be visualized.)""";
          } moment_threshold;
          // Symbol: drake::multibody::meshcat::ContactVisualizerParams::newton_meters_per_meter
          struct /* newton_meters_per_meter */ {
            // Source: drake/multibody/meshcat/contact_visualizer_params.h
            const char* doc =
R"""(Sets the length scale of the moment vectors.)""";
          } newton_meters_per_meter;
          // Symbol: drake::multibody::meshcat::ContactVisualizerParams::newtons_per_meter
          struct /* newtons_per_meter */ {
            // Source: drake/multibody/meshcat/contact_visualizer_params.h
            const char* doc =
R"""(Sets the length scale of the force vectors.)""";
          } newtons_per_meter;
          // Symbol: drake::multibody::meshcat::ContactVisualizerParams::prefix
          struct /* prefix */ {
            // Source: drake/multibody/meshcat/contact_visualizer_params.h
            const char* doc =
R"""(A prefix to add to the path for all objects and transforms curated by
the ContactVisualizer. It can be an absolute path or relative path. If
relative, this ``prefix`` will be appended to the geometry∷Meshcat
``prefix`` based on the standard path semantics in Meshcat. See
meshcat_path "Meshcat paths" for details.)""";
          } prefix;
          // Symbol: drake::multibody::meshcat::ContactVisualizerParams::publish_period
          struct /* publish_period */ {
            // Source: drake/multibody/meshcat/contact_visualizer_params.h
            const char* doc =
R"""(The duration (in simulation seconds) between attempts to update poses
in the visualizer. (To help avoid small simulation time steps, we use
a default period that has an exact representation in binary floating
point; see drake#15021 for details.))""";
          } publish_period;
          // Symbol: drake::multibody::meshcat::ContactVisualizerParams::radius
          struct /* radius */ {
            // Source: drake/multibody/meshcat/contact_visualizer_params.h
            const char* doc =
R"""(The radius of cylinder geometry used in the force/moment vector.)""";
          } radius;
          auto Serialize__fields() const {
            return std::array{
              std::make_pair("color", color.doc),
              std::make_pair("delete_on_initialization_event", delete_on_initialization_event.doc),
              std::make_pair("force_threshold", force_threshold.doc),
              std::make_pair("hydro_force_color", hydro_force_color.doc),
              std::make_pair("hydro_moment_color", hydro_moment_color.doc),
              std::make_pair("moment_threshold", moment_threshold.doc),
              std::make_pair("newton_meters_per_meter", newton_meters_per_meter.doc),
              std::make_pair("newtons_per_meter", newtons_per_meter.doc),
              std::make_pair("prefix", prefix.doc),
              std::make_pair("publish_period", publish_period.doc),
              std::make_pair("radius", radius.doc),
            };
          }
        } ContactVisualizerParams;
        // Symbol: drake::multibody::meshcat::ContactVisualizerd
        struct /* ContactVisualizerd */ {
          // Source: drake/multibody/meshcat/contact_visualizer.h
          const char* doc =
R"""(A convenient alias for the ContactVisualizer class when using the
``double`` scalar type.)""";
        } ContactVisualizerd;
        // Symbol: drake::multibody::meshcat::JointSliders
        struct /* JointSliders */ {
          // Source: drake/multibody/meshcat/joint_sliders.h
          const char* doc =
R"""(JointSliders adds slider bars to the Meshcat control panel for the
joints of a MultibodyPlant. These might be useful for interactive or
teleoperation demos. The sliders' current values are available on the
``positions`` output port of this system.

.. pydrake_system::

    name: JointSliders
    output_ports:
    - positions

The output port is of size ``plant.num_positions()``, and the order of
its elements matches ``plant.GetPositions()``.

Beware that the output port of this system always provides the
sliders' current values, even if evaluated by multiple different
downstream input ports during a single computation. If you need to
have a synchronized view of the slider data, place a
systems∷ZeroOrderHold system between the sliders and downstream
calculations.)""";
          // Symbol: drake::multibody::meshcat::JointSliders::Delete
          struct /* Delete */ {
            // Source: drake/multibody/meshcat/joint_sliders.h
            const char* doc =
R"""(Removes our sliders from the associated meshcat instance.

From this point on, our output port produces the initial_value from
the constructor, not any slider values.

Warning:
    It is not safe to call this when a CalcOutput might be happening
    on this instance concurrently on another thread.)""";
          } Delete;
          // Symbol: drake::multibody::meshcat::JointSliders::JointSliders<T>
          struct /* ctor */ {
            // Source: drake/multibody/meshcat/joint_sliders.h
            const char* doc =
R"""(Creates a meshcat slider for each joint in the given plant.

Parameter ``meshcat``:
    The Meshcat instance where the sliders will be added.

Parameter ``plant``:
    The MultibodyPlant to create sliders for. The plant pointer is
    aliased for the lifetime of this JointSliders object.

Parameter ``initial_value``:
    (Optional) If provided, the sliders' initial values will be as
    given; otherwise, the plant's default values will be used.

Parameter ``lower_limit``:
    (Optional) The lower limit of the slider will be the maximum value
    of this number and any limit specified in the Joint. May be a
    single value or else a vector of length plant.num_positions(). If
    no value is provided, a sensible default will be used.

Parameter ``upper_limit``:
    (Optional) The upper limit of the slider will be the minimum value
    of this number and any limit specified in the Joint. May be a
    single value or else a vector of length plant.num_positions(). If
    no value is provided, a sensible default will be used.

Parameter ``step``:
    (Optional) The step argument of the slider, which is the smallest
    increment by which the slider can change values (and therefore
    update our output port's value). May be a single value or else a
    vector of length plant.num_positions(). If no value is provided, a
    sensible default will be used.

Parameter ``decrement_keycodes``:
    (Optional) A vector of length plant.num_positions() with keycodes
    to assign to decrement the value of each individual joint slider.
    See Meshcat∷AddSlider for more details.

Parameter ``increment_keycodes``:
    (Optional) A vector of length plant.num_positions() with keycodes
    to assign to increment the value of each individual joint slider.
    See Meshcat∷AddSlider for more details.)""";
          } ctor;
          // Symbol: drake::multibody::meshcat::JointSliders::Run
          struct /* Run */ {
            // Source: drake/multibody/meshcat/joint_sliders.h
            const char* doc =
R"""(Publishes the given Diagram (typically, to cause it to be visualized)
whenever our sliders' values change. Blocks until the user clicks a
"Stop" button in the MeshCat control panel, or if the timeout limit is
reached.

Parameter ``timeout``:
    (Optional) In the absence of a button click, the duration (in
    seconds) to wait before returning. If the button is clicked, this
    function will return promptly, without waiting for the timeout.
    When no timeout is given, this function will block indefinitely.

Parameter ``stop_button_keycode``:
    a keycode that will be assigned to the "Stop" button. Setting this
    to the empty string means no keycode. See Meshcat∷AddButton for
    details. $*Default:* "Escape".

Returns:
    the output of plant.GetPositions() given the most recently
    published value of the plant Context.

Precondition:
    ``diagram`` must be a top-level (i.e., "root") diagram.

Precondition:
    ``diagram`` must contain the ``plant`` that was passed into this
    JointSliders system's constructor.

Precondition:
    ``diagram`` must contain this JointSliders system, however the
    output of these sliders need not be connected (even indirectly) to
    any ``plant`` input port. The positions of the ``plant`` will be
    updated directly using a call to ``plant.SetPositions(...)`` when
    the slider values change.)""";
          } Run;
          // Symbol: drake::multibody::meshcat::JointSliders::SetPositions
          struct /* SetPositions */ {
            // Source: drake/multibody/meshcat/joint_sliders.h
            const char* doc =
R"""(Sets our meshcat sliders to the values in ``q``.

Additionally, the "initial state" vector of positions tracked by this
instance will be updated to the values in ``q``. This "initial state"
vector update will persist even if sliders are removed (e.g., via
Delete).

Parameter ``q``:
    A vector whose length is equal to the associated
    MultibodyPlant∷num_positions().)""";
          } SetPositions;
        } JointSliders;
        // Symbol: drake::multibody::meshcat::MeshcatMouseSpring
        struct /* MeshcatMouseSpring */ {
          // Source: drake/multibody/meshcat/meshcat_mouse_spring.h
          const char* doc =
R"""(MeshcatMouseSpring lets a user drag the bodies of a MultibodyPlant
with the mouse in a Meshcat browser: holding Ctrl and dragging a body
with the left mouse button applies a virtual spring force that pulls
the grabbed point toward the cursor.

This system reads the drag state from Meshcat (see
geometry∷Meshcat∷GetObjectDrag()) and outputs a corresponding
geometry∷ExternallyAppliedSpatialForce on the dragged body. Connecting
that output to MultibodyPlant∷get_applied_spatial_force_input_port()
applies the force; AddToBuilder() performs that connection along with
the input connections.

.. pydrake_system::

    name: MeshcatMouseSpring
    input_ports:
    - body_poses
    - body_spatial_velocities
    output_ports:
    - spatial_forces

The ``body_poses`` and ``body_spatial_velocities`` inputs come from
the same-named MultibodyPlant output ports.

With ``m`` the dragged body's mass, the applied force (in the world
frame) is ``m * stiffness * (target - anchor) - m * sqrt(stiffness) *
v_anchor``, where ``anchor`` is the grabbed point on the body,
``target`` is the cursor position, and ``v_anchor`` is the world
velocity of the grabbed point. Scaling by ``m`` makes the
translational response frequency ``sqrt(stiffness)`` and damping ratio
independent of the body's mass.

When no drag is in progress the output is empty. Any body with
geometry published to Meshcat by a geometry∷MeshcatVisualizer can be
dragged; the world body cannot.

This system is ``double``-only, because Meshcat reports drag state as
plain doubles and mouse interaction is not meaningful for other scalar
types.)""";
          // Symbol: drake::multibody::meshcat::MeshcatMouseSpring::AddToBuilder
          struct /* AddToBuilder */ {
            // Source: drake/multibody/meshcat/meshcat_mouse_spring.h
            const char* doc =
R"""(Adds a MeshcatMouseSpring to ``builder`` and connects it to `plant`'s
body-pose and body-spatial-velocity output ports and its
applied-spatial-force input port. Returns a reference to the
newly-added system.

Precondition:
    plant is part of builder and is finalized.

Precondition:
    `plant`'s applied-spatial-force input port is not already
    connected.)""";
          } AddToBuilder;
          // Symbol: drake::multibody::meshcat::MeshcatMouseSpring::MeshcatMouseSpring
          struct /* ctor */ {
            // Source: drake/multibody/meshcat/meshcat_mouse_spring.h
            const char* doc =
R"""(Constructs a MeshcatMouseSpring for the given ``plant``.

Parameter ``meshcat``:
    The Meshcat instance the user will interact with. The pointer is
    aliased and must outlive this system.

Parameter ``plant``:
    The MultibodyPlant whose bodies can be dragged. The pointer is
    aliased and must outlive this system; the plant must already be
    finalized.

Parameter ``stiffness``:
    The mass-normalized spring stiffness, in 1/s²; see the class
    overview for the force it produces.

Precondition:
    plant->is_finalized() is true.

Precondition:
    stiffness >= 0.)""";
          } ctor;
          // Symbol: drake::multibody::meshcat::MeshcatMouseSpring::get_body_poses_input_port
          struct /* get_body_poses_input_port */ {
            // Source: drake/multibody/meshcat/meshcat_mouse_spring.h
            const char* doc =
R"""(Returns the input port for the bodies' poses (a
``std∷vector<math∷RigidTransform<double>>``).)""";
          } get_body_poses_input_port;
          // Symbol: drake::multibody::meshcat::MeshcatMouseSpring::get_body_spatial_velocities_input_port
          struct /* get_body_spatial_velocities_input_port */ {
            // Source: drake/multibody/meshcat/meshcat_mouse_spring.h
            const char* doc =
R"""(Returns the input port for the bodies' spatial velocities (a
``std∷vector<SpatialVelocity<double>>``).)""";
          } get_body_spatial_velocities_input_port;
          // Symbol: drake::multibody::meshcat::MeshcatMouseSpring::get_spatial_forces_output_port
          struct /* get_spatial_forces_output_port */ {
            // Source: drake/multibody/meshcat/meshcat_mouse_spring.h
            const char* doc =
R"""(Returns the output port for the applied spatial forces (a
``std∷vector<ExternallyAppliedSpatialForce<double>>``).)""";
          } get_spatial_forces_output_port;
        } MeshcatMouseSpring;
      } meshcat;
    } multibody;
    // Symbol: drake::systems
    struct /* systems */ {
      // Symbol: drake::systems::scalar_conversion
      struct /* scalar_conversion */ {
        // Symbol: drake::systems::scalar_conversion::Traits
        struct /* Traits */ {
          // Source: drake/multibody/meshcat/contact_visualizer.h
          const char* doc = R"""()""";
        } Traits;
      } scalar_conversion;
    } systems;
  } drake;
} pydrake_doc_multibody_meshcat;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
