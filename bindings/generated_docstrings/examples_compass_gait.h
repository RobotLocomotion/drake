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

// #include "drake/examples/compass_gait/compass_gait.h"
// #include "drake/examples/compass_gait/compass_gait_continuous_state.h"
// #include "drake/examples/compass_gait/compass_gait_geometry.h"
// #include "drake/examples/compass_gait/compass_gait_params.h"

// Symbol: pydrake_doc_examples_compass_gait
constexpr struct /* pydrake_doc_examples_compass_gait */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::examples
    struct /* examples */ {
      // Symbol: drake::examples::compass_gait
      struct /* compass_gait */ {
        // Symbol: drake::examples::compass_gait::CompassGait
        struct /* CompassGait */ {
          // Source: drake/examples/compass_gait/compass_gait.h
          const char* doc =
R"""(Dynamical representation of the idealized hybrid dynamics of a
"compass gait", as described in
http://underactuated.mit.edu/underactuated.html?chapter=simple_legs .
This implementation has two additional state variables that are not
required in the mathematical model:

- a discrete state for the position of the stance toe along the ramp
- a Boolean indicator for "left support" (true when the stance leg is
  the left leg).

These are helpful for outputting the floating-base model coordinate,
e.g. for visualization.

Note:
    This model only supports walking downhill on the ramp, because
    that restriction enables a clean / numerically robust
    implementation of the foot collision witness function that avoids
    fall detection on the "foot scuffing" collision.

.. pydrake_system::

    name: CompassGait
    input_ports:
    - hip_torque (optional)
    output_ports:
    - minimal_state
    - floating_base_state

Continuous States: stance, swing, stancedot, swingdot.

Discrete State: stance toe position.

Abstract State: left support indicator.

Note: If the hip_torque input port is not connected, then the torque
is taken to be zero.)""";
          // Symbol: drake::examples::compass_gait::CompassGait::CompassGait<T>
          struct /* ctor */ {
            // Source: drake/examples/compass_gait/compass_gait.h
            const char* doc = R"""(Constructs the plant.)""";
            // Source: drake/examples/compass_gait/compass_gait.h
            const char* doc_copyconvert =
R"""(Scalar-converting copy constructor. See system_scalar_conversion.)""";
          } ctor;
          // Symbol: drake::examples::compass_gait::CompassGait::DynamicsBiasTerm
          struct /* DynamicsBiasTerm */ {
            // Source: drake/examples/compass_gait/compass_gait.h
            const char* doc =
R"""(Manipulator equation of CompassGait: M(q)v̇ + bias(q,v) = 0.

- M is the 2x2 mass matrix. - bias is a 2x1 vector that includes the
Coriolis term and gravity term, i.e. bias = C(q,v)*v - τ_g(q).)""";
          } DynamicsBiasTerm;
          // Symbol: drake::examples::compass_gait::CompassGait::MassMatrix
          struct /* MassMatrix */ {
            // Source: drake/examples/compass_gait/compass_gait.h
            const char* doc = R"""()""";
          } MassMatrix;
          // Symbol: drake::examples::compass_gait::CompassGait::get_continuous_state
          struct /* get_continuous_state */ {
            // Source: drake/examples/compass_gait/compass_gait.h
            const char* doc = R"""(Returns the CompassGaitContinuousState.)""";
          } get_continuous_state;
          // Symbol: drake::examples::compass_gait::CompassGait::get_floating_base_state_output_port
          struct /* get_floating_base_state_output_port */ {
            // Source: drake/examples/compass_gait/compass_gait.h
            const char* doc =
R"""(Returns reference to the output port that provides the state in the
floating-base coordinates (described via left leg xyz & rpy + hip
angle + derivatives).)""";
          } get_floating_base_state_output_port;
          // Symbol: drake::examples::compass_gait::CompassGait::get_minimal_state_output_port
          struct /* get_minimal_state_output_port */ {
            // Source: drake/examples/compass_gait/compass_gait.h
            const char* doc =
R"""(Returns reference to the output port that publishes only
[theta_stance, theta_swing, thetatdot_stance, thetadot_swing].)""";
          } get_minimal_state_output_port;
          // Symbol: drake::examples::compass_gait::CompassGait::get_mutable_continuous_state
          struct /* get_mutable_continuous_state */ {
            // Source: drake/examples/compass_gait/compass_gait.h
            const char* doc =
R"""(Returns the mutable CompassGaitContinuousState.)""";
          } get_mutable_continuous_state;
          // Symbol: drake::examples::compass_gait::CompassGait::get_parameters
          struct /* get_parameters */ {
            // Source: drake/examples/compass_gait/compass_gait.h
            const char* doc = R"""(Access the CompassGaitParams.)""";
          } get_parameters;
          // Symbol: drake::examples::compass_gait::CompassGait::get_toe_position
          struct /* get_toe_position */ {
            // Source: drake/examples/compass_gait/compass_gait.h
            const char* doc = R"""()""";
          } get_toe_position;
          // Symbol: drake::examples::compass_gait::CompassGait::left_leg_is_stance
          struct /* left_leg_is_stance */ {
            // Source: drake/examples/compass_gait/compass_gait.h
            const char* doc = R"""()""";
          } left_leg_is_stance;
          // Symbol: drake::examples::compass_gait::CompassGait::set_left_leg_is_stance
          struct /* set_left_leg_is_stance */ {
            // Source: drake/examples/compass_gait/compass_gait.h
            const char* doc = R"""()""";
          } set_left_leg_is_stance;
          // Symbol: drake::examples::compass_gait::CompassGait::set_toe_position
          struct /* set_toe_position */ {
            // Source: drake/examples/compass_gait/compass_gait.h
            const char* doc = R"""()""";
          } set_toe_position;
        } CompassGait;
        // Symbol: drake::examples::compass_gait::CompassGaitContinuousState
        struct /* CompassGaitContinuousState */ {
          // Source: drake/examples/compass_gait/compass_gait_continuous_state.h
          const char* doc =
R"""(Specializes BasicVector with specific getters and setters.)""";
          // Symbol: drake::examples::compass_gait::CompassGaitContinuousState::CompassGaitContinuousState<T>
          struct /* ctor */ {
            // Source: drake/examples/compass_gait/compass_gait_continuous_state.h
            const char* doc =
R"""(Default constructor. Sets all rows to their default value:

* ``stance`` defaults to 0.0 radians.

* ``swing`` defaults to 0.0 radians.

* ``stancedot`` defaults to 0.0 rad/sec.

* ``swingdot`` defaults to 0.0 rad/sec.)""";
          } ctor;
          // Symbol: drake::examples::compass_gait::CompassGaitContinuousState::DoClone
          struct /* DoClone */ {
            // Source: drake/examples/compass_gait/compass_gait_continuous_state.h
            const char* doc = R"""()""";
          } DoClone;
          // Symbol: drake::examples::compass_gait::CompassGaitContinuousState::GetCoordinateNames
          struct /* GetCoordinateNames */ {
            // Source: drake/examples/compass_gait/compass_gait_continuous_state.h
            const char* doc =
R"""(See CompassGaitContinuousStateIndices∷GetCoordinateNames().)""";
          } GetCoordinateNames;
          // Symbol: drake::examples::compass_gait::CompassGaitContinuousState::IsValid
          struct /* IsValid */ {
            // Source: drake/examples/compass_gait/compass_gait_continuous_state.h
            const char* doc =
R"""(Returns whether the current values of this vector are well-formed.)""";
          } IsValid;
          // Symbol: drake::examples::compass_gait::CompassGaitContinuousState::K
          struct /* K */ {
            // Source: drake/examples/compass_gait/compass_gait_continuous_state.h
            const char* doc =
R"""(An abbreviation for our row index constants.)""";
          } K;
          // Symbol: drake::examples::compass_gait::CompassGaitContinuousState::Serialize
          struct /* Serialize */ {
            // Source: drake/examples/compass_gait/compass_gait_continuous_state.h
            const char* doc =
R"""(Visit each field of this named vector, passing them (in order) to the
given Archive. The archive can read and/or write to the vector values.
One common use of Serialize is the //common/yaml tools.)""";
          } Serialize;
          // Symbol: drake::examples::compass_gait::CompassGaitContinuousState::SetToNamedVariables
          struct /* SetToNamedVariables */ {
            // Source: drake/examples/compass_gait/compass_gait_continuous_state.h
            const char* doc =
R"""(Create a symbolic∷Variable for each element with the known variable
name. This is only available for T == symbolic∷Expression.)""";
          } SetToNamedVariables;
          // Symbol: drake::examples::compass_gait::CompassGaitContinuousState::set_stance
          struct /* set_stance */ {
            // Source: drake/examples/compass_gait/compass_gait_continuous_state.h
            const char* doc = R"""(Setter that matches stance().)""";
          } set_stance;
          // Symbol: drake::examples::compass_gait::CompassGaitContinuousState::set_stancedot
          struct /* set_stancedot */ {
            // Source: drake/examples/compass_gait/compass_gait_continuous_state.h
            const char* doc = R"""(Setter that matches stancedot().)""";
          } set_stancedot;
          // Symbol: drake::examples::compass_gait::CompassGaitContinuousState::set_swing
          struct /* set_swing */ {
            // Source: drake/examples/compass_gait/compass_gait_continuous_state.h
            const char* doc = R"""(Setter that matches swing().)""";
          } set_swing;
          // Symbol: drake::examples::compass_gait::CompassGaitContinuousState::set_swingdot
          struct /* set_swingdot */ {
            // Source: drake/examples/compass_gait/compass_gait_continuous_state.h
            const char* doc = R"""(Setter that matches swingdot().)""";
          } set_swingdot;
          // Symbol: drake::examples::compass_gait::CompassGaitContinuousState::stance
          struct /* stance */ {
            // Source: drake/examples/compass_gait/compass_gait_continuous_state.h
            const char* doc =
R"""(The orientation of the stance leg, measured clockwise from the
vertical axis.

Note:
    ``stance`` is expressed in units of radians.)""";
          } stance;
          // Symbol: drake::examples::compass_gait::CompassGaitContinuousState::stancedot
          struct /* stancedot */ {
            // Source: drake/examples/compass_gait/compass_gait_continuous_state.h
            const char* doc =
R"""(The angular velocity of the stance leg.

Note:
    ``stancedot`` is expressed in units of rad/sec.)""";
          } stancedot;
          // Symbol: drake::examples::compass_gait::CompassGaitContinuousState::swing
          struct /* swing */ {
            // Source: drake/examples/compass_gait/compass_gait_continuous_state.h
            const char* doc =
R"""(The orientation of the swing leg, measured clockwise from the vertical
axis.

Note:
    ``swing`` is expressed in units of radians.)""";
          } swing;
          // Symbol: drake::examples::compass_gait::CompassGaitContinuousState::swingdot
          struct /* swingdot */ {
            // Source: drake/examples/compass_gait/compass_gait_continuous_state.h
            const char* doc =
R"""(The angular velocity of the swing leg.

Note:
    ``swingdot`` is expressed in units of rad/sec.)""";
          } swingdot;
          // Symbol: drake::examples::compass_gait::CompassGaitContinuousState::with_stance
          struct /* with_stance */ {
            // Source: drake/examples/compass_gait/compass_gait_continuous_state.h
            const char* doc =
R"""(Fluent setter that matches stance(). Returns a copy of ``this`` with
stance set to a new value.)""";
          } with_stance;
          // Symbol: drake::examples::compass_gait::CompassGaitContinuousState::with_stancedot
          struct /* with_stancedot */ {
            // Source: drake/examples/compass_gait/compass_gait_continuous_state.h
            const char* doc =
R"""(Fluent setter that matches stancedot(). Returns a copy of ``this``
with stancedot set to a new value.)""";
          } with_stancedot;
          // Symbol: drake::examples::compass_gait::CompassGaitContinuousState::with_swing
          struct /* with_swing */ {
            // Source: drake/examples/compass_gait/compass_gait_continuous_state.h
            const char* doc =
R"""(Fluent setter that matches swing(). Returns a copy of ``this`` with
swing set to a new value.)""";
          } with_swing;
          // Symbol: drake::examples::compass_gait::CompassGaitContinuousState::with_swingdot
          struct /* with_swingdot */ {
            // Source: drake/examples/compass_gait/compass_gait_continuous_state.h
            const char* doc =
R"""(Fluent setter that matches swingdot(). Returns a copy of ``this`` with
swingdot set to a new value.)""";
          } with_swingdot;
        } CompassGaitContinuousState;
        // Symbol: drake::examples::compass_gait::CompassGaitContinuousStateIndices
        struct /* CompassGaitContinuousStateIndices */ {
          // Source: drake/examples/compass_gait/compass_gait_continuous_state.h
          const char* doc =
R"""(Describes the row indices of a CompassGaitContinuousState.)""";
          // Symbol: drake::examples::compass_gait::CompassGaitContinuousStateIndices::GetCoordinateNames
          struct /* GetCoordinateNames */ {
            // Source: drake/examples/compass_gait/compass_gait_continuous_state.h
            const char* doc =
R"""(Returns a vector containing the names of each coordinate within this
class. The indices within the returned vector matches that of this
class. In other words,
``CompassGaitContinuousStateIndices∷GetCoordinateNames()[i]`` is the
name for ``BasicVector∷GetAtIndex(i)``.)""";
          } GetCoordinateNames;
        } CompassGaitContinuousStateIndices;
        // Symbol: drake::examples::compass_gait::CompassGaitGeometry
        struct /* CompassGaitGeometry */ {
          // Source: drake/examples/compass_gait/compass_gait_geometry.h
          const char* doc =
R"""(Expresses a CompassGait's geometry to a SceneGraph.

.. pydrake_system::

    name: CompassGaitGeometry
    input_ports:
    - floating_base_state
    output_ports:
    - geometry_pose

This class has no public constructor; instead use the AddToBuilder()
static method to create and add it to a DiagramBuilder directly.)""";
          // Symbol: drake::examples::compass_gait::CompassGaitGeometry::AddToBuilder
          struct /* AddToBuilder */ {
            // Source: drake/examples/compass_gait/compass_gait_geometry.h
            const char* doc_4args =
R"""(Creates, adds, and connects a CompassGaitGeometry system into the
given ``builder``. Both the ``floating_base_state_port.get_system()``
and ``scene_graph`` systems must have been added to the given
``builder`` already. The ``compass_gait_params`` sets the parameters
of the geometry registered with ``scene_graph``; the visualization
changes based on the leg length and the ration of leg mass to hip mass
(the leg mass sphere is scaled assuming a constant density).

The ``scene_graph`` pointer is not retained by the CompassGaitGeometry
system. The return value pointer is an alias of the new
CompassGaitGeometry system that is owned by the ``builder``.)""";
            // Source: drake/examples/compass_gait/compass_gait_geometry.h
            const char* doc_3args =
R"""(Creates, adds, and connects a CompassGaitGeometry system into the
given ``builder``. Both the ``floating_base_state_port.get_system()``
and ``scene_graph`` systems must have been added to the given
``builder`` already. CompassGaitParams are set to their default
values.

The ``scene_graph`` pointer is not retained by the CompassGaitGeometry
system. The return value pointer is an alias of the new
CompassGaitGeometry system that is owned by the ``builder``.)""";
          } AddToBuilder;
          // Symbol: drake::examples::compass_gait::CompassGaitGeometry::CompassGaitGeometry
          struct /* ctor */ {
            // Source: drake/examples/compass_gait/compass_gait_geometry.h
            const char* doc = R"""()""";
          } ctor;
        } CompassGaitGeometry;
        // Symbol: drake::examples::compass_gait::CompassGaitParams
        struct /* CompassGaitParams */ {
          // Source: drake/examples/compass_gait/compass_gait_params.h
          const char* doc =
R"""(Specializes BasicVector with specific getters and setters.)""";
          // Symbol: drake::examples::compass_gait::CompassGaitParams::CompassGaitParams<T>
          struct /* ctor */ {
            // Source: drake/examples/compass_gait/compass_gait_params.h
            const char* doc =
R"""(Default constructor. Sets all rows to their default value:

* ``mass_hip`` defaults to 10.0 kg.

* ``mass_leg`` defaults to 5.0 kg.

* ``length_leg`` defaults to 1.0 m.

* ``center_of_mass_leg`` defaults to 0.5 m.

* ``gravity`` defaults to 9.81 m/s^2.

* ``slope`` defaults to 0.0525 radians.)""";
          } ctor;
          // Symbol: drake::examples::compass_gait::CompassGaitParams::DoClone
          struct /* DoClone */ {
            // Source: drake/examples/compass_gait/compass_gait_params.h
            const char* doc = R"""()""";
          } DoClone;
          // Symbol: drake::examples::compass_gait::CompassGaitParams::GetCoordinateNames
          struct /* GetCoordinateNames */ {
            // Source: drake/examples/compass_gait/compass_gait_params.h
            const char* doc =
R"""(See CompassGaitParamsIndices∷GetCoordinateNames().)""";
          } GetCoordinateNames;
          // Symbol: drake::examples::compass_gait::CompassGaitParams::GetElementBounds
          struct /* GetElementBounds */ {
            // Source: drake/examples/compass_gait/compass_gait_params.h
            const char* doc = R"""()""";
          } GetElementBounds;
          // Symbol: drake::examples::compass_gait::CompassGaitParams::IsValid
          struct /* IsValid */ {
            // Source: drake/examples/compass_gait/compass_gait_params.h
            const char* doc =
R"""(Returns whether the current values of this vector are well-formed.)""";
          } IsValid;
          // Symbol: drake::examples::compass_gait::CompassGaitParams::K
          struct /* K */ {
            // Source: drake/examples/compass_gait/compass_gait_params.h
            const char* doc =
R"""(An abbreviation for our row index constants.)""";
          } K;
          // Symbol: drake::examples::compass_gait::CompassGaitParams::Serialize
          struct /* Serialize */ {
            // Source: drake/examples/compass_gait/compass_gait_params.h
            const char* doc =
R"""(Visit each field of this named vector, passing them (in order) to the
given Archive. The archive can read and/or write to the vector values.
One common use of Serialize is the //common/yaml tools.)""";
          } Serialize;
          // Symbol: drake::examples::compass_gait::CompassGaitParams::SetToNamedVariables
          struct /* SetToNamedVariables */ {
            // Source: drake/examples/compass_gait/compass_gait_params.h
            const char* doc =
R"""(Create a symbolic∷Variable for each element with the known variable
name. This is only available for T == symbolic∷Expression.)""";
          } SetToNamedVariables;
          // Symbol: drake::examples::compass_gait::CompassGaitParams::center_of_mass_leg
          struct /* center_of_mass_leg */ {
            // Source: drake/examples/compass_gait/compass_gait_params.h
            const char* doc =
R"""(Distance from the hip to the center of mass of each leg.

Note:
    ``center_of_mass_leg`` is expressed in units of m.

Note:
    ``center_of_mass_leg`` has a limited domain of [0.0, +Inf].)""";
          } center_of_mass_leg;
          // Symbol: drake::examples::compass_gait::CompassGaitParams::gravity
          struct /* gravity */ {
            // Source: drake/examples/compass_gait/compass_gait_params.h
            const char* doc =
R"""(An approximate value for gravitational acceleration.

Note:
    ``gravity`` is expressed in units of m/s^2.

Note:
    ``gravity`` has a limited domain of [0.0, +Inf].)""";
          } gravity;
          // Symbol: drake::examples::compass_gait::CompassGaitParams::length_leg
          struct /* length_leg */ {
            // Source: drake/examples/compass_gait/compass_gait_params.h
            const char* doc =
R"""(The length of each leg.

Note:
    ``length_leg`` is expressed in units of m.

Note:
    ``length_leg`` has a limited domain of [0.0, +Inf].)""";
          } length_leg;
          // Symbol: drake::examples::compass_gait::CompassGaitParams::mass_hip
          struct /* mass_hip */ {
            // Source: drake/examples/compass_gait/compass_gait_params.h
            const char* doc =
R"""(Point mass at the hip.

Note:
    ``mass_hip`` is expressed in units of kg.

Note:
    ``mass_hip`` has a limited domain of [0.0, +Inf].)""";
          } mass_hip;
          // Symbol: drake::examples::compass_gait::CompassGaitParams::mass_leg
          struct /* mass_leg */ {
            // Source: drake/examples/compass_gait/compass_gait_params.h
            const char* doc =
R"""(Mass of each leg (modeled as a point mass at the center of mass).

Note:
    ``mass_leg`` is expressed in units of kg.

Note:
    ``mass_leg`` has a limited domain of [0.0, +Inf].)""";
          } mass_leg;
          // Symbol: drake::examples::compass_gait::CompassGaitParams::set_center_of_mass_leg
          struct /* set_center_of_mass_leg */ {
            // Source: drake/examples/compass_gait/compass_gait_params.h
            const char* doc =
R"""(Setter that matches center_of_mass_leg().)""";
          } set_center_of_mass_leg;
          // Symbol: drake::examples::compass_gait::CompassGaitParams::set_gravity
          struct /* set_gravity */ {
            // Source: drake/examples/compass_gait/compass_gait_params.h
            const char* doc = R"""(Setter that matches gravity().)""";
          } set_gravity;
          // Symbol: drake::examples::compass_gait::CompassGaitParams::set_length_leg
          struct /* set_length_leg */ {
            // Source: drake/examples/compass_gait/compass_gait_params.h
            const char* doc = R"""(Setter that matches length_leg().)""";
          } set_length_leg;
          // Symbol: drake::examples::compass_gait::CompassGaitParams::set_mass_hip
          struct /* set_mass_hip */ {
            // Source: drake/examples/compass_gait/compass_gait_params.h
            const char* doc = R"""(Setter that matches mass_hip().)""";
          } set_mass_hip;
          // Symbol: drake::examples::compass_gait::CompassGaitParams::set_mass_leg
          struct /* set_mass_leg */ {
            // Source: drake/examples/compass_gait/compass_gait_params.h
            const char* doc = R"""(Setter that matches mass_leg().)""";
          } set_mass_leg;
          // Symbol: drake::examples::compass_gait::CompassGaitParams::set_slope
          struct /* set_slope */ {
            // Source: drake/examples/compass_gait/compass_gait_params.h
            const char* doc = R"""(Setter that matches slope().)""";
          } set_slope;
          // Symbol: drake::examples::compass_gait::CompassGaitParams::slope
          struct /* slope */ {
            // Source: drake/examples/compass_gait/compass_gait_params.h
            const char* doc =
R"""(The angle of the ramp on which the compass gait is walking. Must have
0 <= slope < PI/2 so that forward == downhill (an assumption used in
the foot collision witness function).

Note:
    ``slope`` is expressed in units of radians.

Note:
    ``slope`` has a limited domain of [0.0, 1.5707].)""";
          } slope;
          // Symbol: drake::examples::compass_gait::CompassGaitParams::with_center_of_mass_leg
          struct /* with_center_of_mass_leg */ {
            // Source: drake/examples/compass_gait/compass_gait_params.h
            const char* doc =
R"""(Fluent setter that matches center_of_mass_leg(). Returns a copy of
``this`` with center_of_mass_leg set to a new value.)""";
          } with_center_of_mass_leg;
          // Symbol: drake::examples::compass_gait::CompassGaitParams::with_gravity
          struct /* with_gravity */ {
            // Source: drake/examples/compass_gait/compass_gait_params.h
            const char* doc =
R"""(Fluent setter that matches gravity(). Returns a copy of ``this`` with
gravity set to a new value.)""";
          } with_gravity;
          // Symbol: drake::examples::compass_gait::CompassGaitParams::with_length_leg
          struct /* with_length_leg */ {
            // Source: drake/examples/compass_gait/compass_gait_params.h
            const char* doc =
R"""(Fluent setter that matches length_leg(). Returns a copy of ``this``
with length_leg set to a new value.)""";
          } with_length_leg;
          // Symbol: drake::examples::compass_gait::CompassGaitParams::with_mass_hip
          struct /* with_mass_hip */ {
            // Source: drake/examples/compass_gait/compass_gait_params.h
            const char* doc =
R"""(Fluent setter that matches mass_hip(). Returns a copy of ``this`` with
mass_hip set to a new value.)""";
          } with_mass_hip;
          // Symbol: drake::examples::compass_gait::CompassGaitParams::with_mass_leg
          struct /* with_mass_leg */ {
            // Source: drake/examples/compass_gait/compass_gait_params.h
            const char* doc =
R"""(Fluent setter that matches mass_leg(). Returns a copy of ``this`` with
mass_leg set to a new value.)""";
          } with_mass_leg;
          // Symbol: drake::examples::compass_gait::CompassGaitParams::with_slope
          struct /* with_slope */ {
            // Source: drake/examples/compass_gait/compass_gait_params.h
            const char* doc =
R"""(Fluent setter that matches slope(). Returns a copy of ``this`` with
slope set to a new value.)""";
          } with_slope;
        } CompassGaitParams;
        // Symbol: drake::examples::compass_gait::CompassGaitParamsIndices
        struct /* CompassGaitParamsIndices */ {
          // Source: drake/examples/compass_gait/compass_gait_params.h
          const char* doc =
R"""(Describes the row indices of a CompassGaitParams.)""";
          // Symbol: drake::examples::compass_gait::CompassGaitParamsIndices::GetCoordinateNames
          struct /* GetCoordinateNames */ {
            // Source: drake/examples/compass_gait/compass_gait_params.h
            const char* doc =
R"""(Returns a vector containing the names of each coordinate within this
class. The indices within the returned vector matches that of this
class. In other words,
``CompassGaitParamsIndices∷GetCoordinateNames()[i]`` is the name for
``BasicVector∷GetAtIndex(i)``.)""";
          } GetCoordinateNames;
        } CompassGaitParamsIndices;
      } compass_gait;
    } examples;
  } drake;
} pydrake_doc_examples_compass_gait;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
