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

// #include "drake/examples/rimless_wheel/rimless_wheel.h"
// #include "drake/examples/rimless_wheel/rimless_wheel_continuous_state.h"
// #include "drake/examples/rimless_wheel/rimless_wheel_geometry.h"
// #include "drake/examples/rimless_wheel/rimless_wheel_params.h"

// Symbol: pydrake_doc_examples_rimless_wheel
constexpr struct /* pydrake_doc_examples_rimless_wheel */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::examples
    struct /* examples */ {
      // Symbol: drake::examples::rimless_wheel
      struct /* rimless_wheel */ {
        // Symbol: drake::examples::rimless_wheel::RimlessWheel
        struct /* RimlessWheel */ {
          // Source: drake/examples/rimless_wheel/rimless_wheel.h
          const char* doc =
R"""(Dynamical representation of the idealized hybrid dynamics of a
"rimless wheel", as described in
http://underactuated.mit.edu/underactuated.html?chapter=simple_legs In
addition, this model has two additional (discrete) state variables
that are not required in the mathematical model:

- the position of the stance toe along the ramp (helpful for outputting
  a floating-base model coordinate, e.g. for visualization),
- a boolean indicator for "double support" (to avoid the numerical
  challenges of simulation around the Zeno phenomenon at the standing
  fixed point).

.. pydrake_system::

    name: RimlessWheel
    output_ports:
    - minimal_state
    - floating_base_state

Continuous States: theta, thetadot (emitted at ``minimal_state``
port). Discrete States: stance toe position (emitted at
``floating_base_state`` port), double support indicator. Parameters:
mass, length, number of spokes, etc, are all set as Context parameters
using RimlessWheelParams.)""";
          // Symbol: drake::examples::rimless_wheel::RimlessWheel::CalcTotalEnergy
          struct /* CalcTotalEnergy */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel.h
            const char* doc =
R"""(Calculates the kinetic + potential energy.)""";
          } CalcTotalEnergy;
          // Symbol: drake::examples::rimless_wheel::RimlessWheel::RimlessWheel<T>
          struct /* ctor */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel.h
            const char* doc = R"""(Constructs the plant.)""";
            // Source: drake/examples/rimless_wheel/rimless_wheel.h
            const char* doc_copyconvert =
R"""(Scalar-converting copy constructor. See system_scalar_conversion.)""";
          } ctor;
          // Symbol: drake::examples::rimless_wheel::RimlessWheel::calc_alpha
          struct /* calc_alpha */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel.h
            const char* doc =
R"""(Alpha is half the interleg angle, and is used frequently.)""";
          } calc_alpha;
          // Symbol: drake::examples::rimless_wheel::RimlessWheel::get_continuous_state
          struct /* get_continuous_state */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel.h
            const char* doc = R"""(Access the RimlessWheelContinuousState.)""";
          } get_continuous_state;
          // Symbol: drake::examples::rimless_wheel::RimlessWheel::get_double_support
          struct /* get_double_support */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel.h
            const char* doc = R"""()""";
          } get_double_support;
          // Symbol: drake::examples::rimless_wheel::RimlessWheel::get_floating_base_state_output_port
          struct /* get_floating_base_state_output_port */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel.h
            const char* doc =
R"""(Returns reference to the output port that provides a 12 dimensional
state (FloatingBaseType∷kRollPitchYaw positions then velocities). This
is useful, e.g., for visualization. θ of the rimless wheel is the
pitch of the floating base (rotation around global y), and downhill
moves toward positive x. As always, we use vehicle coordinates (x-y on
the ground, z is up).)""";
          } get_floating_base_state_output_port;
          // Symbol: drake::examples::rimless_wheel::RimlessWheel::get_minimal_state_output_port
          struct /* get_minimal_state_output_port */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel.h
            const char* doc =
R"""(Return reference to the output port that publishes only [theta,
thetatdot].)""";
          } get_minimal_state_output_port;
          // Symbol: drake::examples::rimless_wheel::RimlessWheel::get_mutable_continuous_state
          struct /* get_mutable_continuous_state */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel.h
            const char* doc =
R"""(Access the mutable RimlessWheelContinuousState.)""";
          } get_mutable_continuous_state;
          // Symbol: drake::examples::rimless_wheel::RimlessWheel::get_mutable_double_support
          struct /* get_mutable_double_support */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel.h
            const char* doc = R"""()""";
          } get_mutable_double_support;
          // Symbol: drake::examples::rimless_wheel::RimlessWheel::get_mutable_toe_position
          struct /* get_mutable_toe_position */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel.h
            const char* doc = R"""()""";
          } get_mutable_toe_position;
          // Symbol: drake::examples::rimless_wheel::RimlessWheel::get_parameters
          struct /* get_parameters */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel.h
            const char* doc = R"""(Access the RimlessWheelParams.)""";
          } get_parameters;
          // Symbol: drake::examples::rimless_wheel::RimlessWheel::get_toe_position
          struct /* get_toe_position */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel.h
            const char* doc = R"""()""";
          } get_toe_position;
        } RimlessWheel;
        // Symbol: drake::examples::rimless_wheel::RimlessWheelContinuousState
        struct /* RimlessWheelContinuousState */ {
          // Source: drake/examples/rimless_wheel/rimless_wheel_continuous_state.h
          const char* doc =
R"""(Specializes BasicVector with specific getters and setters.)""";
          // Symbol: drake::examples::rimless_wheel::RimlessWheelContinuousState::DoClone
          struct /* DoClone */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel_continuous_state.h
            const char* doc = R"""()""";
          } DoClone;
          // Symbol: drake::examples::rimless_wheel::RimlessWheelContinuousState::GetCoordinateNames
          struct /* GetCoordinateNames */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel_continuous_state.h
            const char* doc =
R"""(See RimlessWheelContinuousStateIndices∷GetCoordinateNames().)""";
          } GetCoordinateNames;
          // Symbol: drake::examples::rimless_wheel::RimlessWheelContinuousState::IsValid
          struct /* IsValid */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel_continuous_state.h
            const char* doc =
R"""(Returns whether the current values of this vector are well-formed.)""";
          } IsValid;
          // Symbol: drake::examples::rimless_wheel::RimlessWheelContinuousState::K
          struct /* K */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel_continuous_state.h
            const char* doc =
R"""(An abbreviation for our row index constants.)""";
          } K;
          // Symbol: drake::examples::rimless_wheel::RimlessWheelContinuousState::RimlessWheelContinuousState<T>
          struct /* ctor */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel_continuous_state.h
            const char* doc =
R"""(Default constructor. Sets all rows to their default value:

* ``theta`` defaults to 0.0 radians.

* ``thetadot`` defaults to 0.0 rad/sec.)""";
          } ctor;
          // Symbol: drake::examples::rimless_wheel::RimlessWheelContinuousState::Serialize
          struct /* Serialize */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel_continuous_state.h
            const char* doc =
R"""(Visit each field of this named vector, passing them (in order) to the
given Archive. The archive can read and/or write to the vector values.
One common use of Serialize is the //common/yaml tools.)""";
          } Serialize;
          // Symbol: drake::examples::rimless_wheel::RimlessWheelContinuousState::SetToNamedVariables
          struct /* SetToNamedVariables */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel_continuous_state.h
            const char* doc =
R"""(Create a symbolic∷Variable for each element with the known variable
name. This is only available for T == symbolic∷Expression.)""";
          } SetToNamedVariables;
          // Symbol: drake::examples::rimless_wheel::RimlessWheelContinuousState::set_theta
          struct /* set_theta */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel_continuous_state.h
            const char* doc = R"""(Setter that matches theta().)""";
          } set_theta;
          // Symbol: drake::examples::rimless_wheel::RimlessWheelContinuousState::set_thetadot
          struct /* set_thetadot */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel_continuous_state.h
            const char* doc = R"""(Setter that matches thetadot().)""";
          } set_thetadot;
          // Symbol: drake::examples::rimless_wheel::RimlessWheelContinuousState::theta
          struct /* theta */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel_continuous_state.h
            const char* doc =
R"""(The orientation of the stance leg, measured clockwise from the
vertical axis.

Note:
    ``theta`` is expressed in units of radians.)""";
          } theta;
          // Symbol: drake::examples::rimless_wheel::RimlessWheelContinuousState::thetadot
          struct /* thetadot */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel_continuous_state.h
            const char* doc =
R"""(The angular velocity of the stance leg.

Note:
    ``thetadot`` is expressed in units of rad/sec.)""";
          } thetadot;
          // Symbol: drake::examples::rimless_wheel::RimlessWheelContinuousState::with_theta
          struct /* with_theta */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel_continuous_state.h
            const char* doc =
R"""(Fluent setter that matches theta(). Returns a copy of ``this`` with
theta set to a new value.)""";
          } with_theta;
          // Symbol: drake::examples::rimless_wheel::RimlessWheelContinuousState::with_thetadot
          struct /* with_thetadot */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel_continuous_state.h
            const char* doc =
R"""(Fluent setter that matches thetadot(). Returns a copy of ``this`` with
thetadot set to a new value.)""";
          } with_thetadot;
        } RimlessWheelContinuousState;
        // Symbol: drake::examples::rimless_wheel::RimlessWheelContinuousStateIndices
        struct /* RimlessWheelContinuousStateIndices */ {
          // Source: drake/examples/rimless_wheel/rimless_wheel_continuous_state.h
          const char* doc =
R"""(Describes the row indices of a RimlessWheelContinuousState.)""";
          // Symbol: drake::examples::rimless_wheel::RimlessWheelContinuousStateIndices::GetCoordinateNames
          struct /* GetCoordinateNames */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel_continuous_state.h
            const char* doc =
R"""(Returns a vector containing the names of each coordinate within this
class. The indices within the returned vector matches that of this
class. In other words,
``RimlessWheelContinuousStateIndices∷GetCoordinateNames()[i]`` is the
name for ``BasicVector∷GetAtIndex(i)``.)""";
          } GetCoordinateNames;
        } RimlessWheelContinuousStateIndices;
        // Symbol: drake::examples::rimless_wheel::RimlessWheelGeometry
        struct /* RimlessWheelGeometry */ {
          // Source: drake/examples/rimless_wheel/rimless_wheel_geometry.h
          const char* doc =
R"""(Expresses a RimlessWheel's geometry to a SceneGraph.

.. pydrake_system::

    name: RimlessWheelGeometry
    input_ports:
    - floating_base_state
    output_ports:
    - geometry_pose

This class has no public constructor; instead use the AddToBuilder()
static method to create and add it to a DiagramBuilder directly.)""";
          // Symbol: drake::examples::rimless_wheel::RimlessWheelGeometry::AddToBuilder
          struct /* AddToBuilder */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel_geometry.h
            const char* doc_4args =
R"""(Creates, adds, and connects a RimlessWheelGeometry system into the
given ``builder``. Both the ``floating_base_state_port.get_system()``
and ``scene_graph`` systems must have been added to the given
``builder`` already. The ``rimless_wheel_params`` sets the parameters
of the geometry registered with ``scene_graph``.

The ``scene_graph`` pointer is not retained by the
RimlessWheelGeometry system. The return value pointer is an alias of
the new RimlessWheelGeometry system that is owned by the ``builder``.)""";
            // Source: drake/examples/rimless_wheel/rimless_wheel_geometry.h
            const char* doc_3args =
R"""(Creates, adds, and connects a RimlessWheelGeometry system into the
given ``builder``. Both the ``floating_base_state_port.get_system()``
and ``scene_graph`` systems must have been added to the given
``builder`` already. RimlessWheelParams are set to their default
values.

The ``scene_graph`` pointer is not retained by the
RimlessWheelGeometry system. The return value pointer is an alias of
the new RimlessWheelGeometry system that is owned by the ``builder``.)""";
          } AddToBuilder;
          // Symbol: drake::examples::rimless_wheel::RimlessWheelGeometry::RimlessWheelGeometry
          struct /* ctor */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel_geometry.h
            const char* doc = R"""()""";
          } ctor;
        } RimlessWheelGeometry;
        // Symbol: drake::examples::rimless_wheel::RimlessWheelParams
        struct /* RimlessWheelParams */ {
          // Source: drake/examples/rimless_wheel/rimless_wheel_params.h
          const char* doc =
R"""(Specializes BasicVector with specific getters and setters.)""";
          // Symbol: drake::examples::rimless_wheel::RimlessWheelParams::DoClone
          struct /* DoClone */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel_params.h
            const char* doc = R"""()""";
          } DoClone;
          // Symbol: drake::examples::rimless_wheel::RimlessWheelParams::GetCoordinateNames
          struct /* GetCoordinateNames */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel_params.h
            const char* doc =
R"""(See RimlessWheelParamsIndices∷GetCoordinateNames().)""";
          } GetCoordinateNames;
          // Symbol: drake::examples::rimless_wheel::RimlessWheelParams::GetElementBounds
          struct /* GetElementBounds */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel_params.h
            const char* doc = R"""()""";
          } GetElementBounds;
          // Symbol: drake::examples::rimless_wheel::RimlessWheelParams::IsValid
          struct /* IsValid */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel_params.h
            const char* doc =
R"""(Returns whether the current values of this vector are well-formed.)""";
          } IsValid;
          // Symbol: drake::examples::rimless_wheel::RimlessWheelParams::K
          struct /* K */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel_params.h
            const char* doc =
R"""(An abbreviation for our row index constants.)""";
          } K;
          // Symbol: drake::examples::rimless_wheel::RimlessWheelParams::RimlessWheelParams<T>
          struct /* ctor */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel_params.h
            const char* doc =
R"""(Default constructor. Sets all rows to their default value:

* ``mass`` defaults to 1.0 kg.

* ``length`` defaults to 1.0 m.

* ``gravity`` defaults to 9.81 m/s^2.

* ``number_of_spokes`` defaults to 8 integer.

* ``slope`` defaults to 0.08 radians.)""";
          } ctor;
          // Symbol: drake::examples::rimless_wheel::RimlessWheelParams::Serialize
          struct /* Serialize */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel_params.h
            const char* doc =
R"""(Visit each field of this named vector, passing them (in order) to the
given Archive. The archive can read and/or write to the vector values.
One common use of Serialize is the //common/yaml tools.)""";
          } Serialize;
          // Symbol: drake::examples::rimless_wheel::RimlessWheelParams::SetToNamedVariables
          struct /* SetToNamedVariables */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel_params.h
            const char* doc =
R"""(Create a symbolic∷Variable for each element with the known variable
name. This is only available for T == symbolic∷Expression.)""";
          } SetToNamedVariables;
          // Symbol: drake::examples::rimless_wheel::RimlessWheelParams::gravity
          struct /* gravity */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel_params.h
            const char* doc =
R"""(An approximate value for gravitational acceleration.

Note:
    ``gravity`` is expressed in units of m/s^2.

Note:
    ``gravity`` has a limited domain of [0.0, +Inf].)""";
          } gravity;
          // Symbol: drake::examples::rimless_wheel::RimlessWheelParams::length
          struct /* length */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel_params.h
            const char* doc =
R"""(The length of each spoke.

Note:
    ``length`` is expressed in units of m.

Note:
    ``length`` has a limited domain of [0.0, +Inf].)""";
          } length;
          // Symbol: drake::examples::rimless_wheel::RimlessWheelParams::mass
          struct /* mass */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel_params.h
            const char* doc =
R"""(The rimless wheel has a single point mass at the hub.

Note:
    ``mass`` is expressed in units of kg.

Note:
    ``mass`` has a limited domain of [0.0, +Inf].)""";
          } mass;
          // Symbol: drake::examples::rimless_wheel::RimlessWheelParams::number_of_spokes
          struct /* number_of_spokes */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel_params.h
            const char* doc =
R"""(Total number of spokes on the wheel

Note:
    ``number_of_spokes`` is expressed in units of integer.

Note:
    ``number_of_spokes`` has a limited domain of [4, +Inf].)""";
          } number_of_spokes;
          // Symbol: drake::examples::rimless_wheel::RimlessWheelParams::set_gravity
          struct /* set_gravity */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel_params.h
            const char* doc = R"""(Setter that matches gravity().)""";
          } set_gravity;
          // Symbol: drake::examples::rimless_wheel::RimlessWheelParams::set_length
          struct /* set_length */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel_params.h
            const char* doc = R"""(Setter that matches length().)""";
          } set_length;
          // Symbol: drake::examples::rimless_wheel::RimlessWheelParams::set_mass
          struct /* set_mass */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel_params.h
            const char* doc = R"""(Setter that matches mass().)""";
          } set_mass;
          // Symbol: drake::examples::rimless_wheel::RimlessWheelParams::set_number_of_spokes
          struct /* set_number_of_spokes */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel_params.h
            const char* doc = R"""(Setter that matches number_of_spokes().)""";
          } set_number_of_spokes;
          // Symbol: drake::examples::rimless_wheel::RimlessWheelParams::set_slope
          struct /* set_slope */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel_params.h
            const char* doc = R"""(Setter that matches slope().)""";
          } set_slope;
          // Symbol: drake::examples::rimless_wheel::RimlessWheelParams::slope
          struct /* slope */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel_params.h
            const char* doc =
R"""(The angle of the ramp on which the rimless wheel is walking.

Note:
    ``slope`` is expressed in units of radians.)""";
          } slope;
          // Symbol: drake::examples::rimless_wheel::RimlessWheelParams::with_gravity
          struct /* with_gravity */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel_params.h
            const char* doc =
R"""(Fluent setter that matches gravity(). Returns a copy of ``this`` with
gravity set to a new value.)""";
          } with_gravity;
          // Symbol: drake::examples::rimless_wheel::RimlessWheelParams::with_length
          struct /* with_length */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel_params.h
            const char* doc =
R"""(Fluent setter that matches length(). Returns a copy of ``this`` with
length set to a new value.)""";
          } with_length;
          // Symbol: drake::examples::rimless_wheel::RimlessWheelParams::with_mass
          struct /* with_mass */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel_params.h
            const char* doc =
R"""(Fluent setter that matches mass(). Returns a copy of ``this`` with
mass set to a new value.)""";
          } with_mass;
          // Symbol: drake::examples::rimless_wheel::RimlessWheelParams::with_number_of_spokes
          struct /* with_number_of_spokes */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel_params.h
            const char* doc =
R"""(Fluent setter that matches number_of_spokes(). Returns a copy of
``this`` with number_of_spokes set to a new value.)""";
          } with_number_of_spokes;
          // Symbol: drake::examples::rimless_wheel::RimlessWheelParams::with_slope
          struct /* with_slope */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel_params.h
            const char* doc =
R"""(Fluent setter that matches slope(). Returns a copy of ``this`` with
slope set to a new value.)""";
          } with_slope;
        } RimlessWheelParams;
        // Symbol: drake::examples::rimless_wheel::RimlessWheelParamsIndices
        struct /* RimlessWheelParamsIndices */ {
          // Source: drake/examples/rimless_wheel/rimless_wheel_params.h
          const char* doc =
R"""(Describes the row indices of a RimlessWheelParams.)""";
          // Symbol: drake::examples::rimless_wheel::RimlessWheelParamsIndices::GetCoordinateNames
          struct /* GetCoordinateNames */ {
            // Source: drake/examples/rimless_wheel/rimless_wheel_params.h
            const char* doc =
R"""(Returns a vector containing the names of each coordinate within this
class. The indices within the returned vector matches that of this
class. In other words,
``RimlessWheelParamsIndices∷GetCoordinateNames()[i]`` is the name for
``BasicVector∷GetAtIndex(i)``.)""";
          } GetCoordinateNames;
        } RimlessWheelParamsIndices;
      } rimless_wheel;
    } examples;
    // Symbol: drake::systems
    struct /* systems */ {
      // Symbol: drake::systems::scalar_conversion
      struct /* scalar_conversion */ {
        // Symbol: drake::systems::scalar_conversion::Traits
        struct /* Traits */ {
          // Source: drake/examples/rimless_wheel/rimless_wheel.h
          const char* doc = R"""()""";
        } Traits;
      } scalar_conversion;
    } systems;
  } drake;
} pydrake_doc_examples_rimless_wheel;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
