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

// #include "drake/examples/pendulum/pendulum_geometry.h"
// #include "drake/examples/pendulum/pendulum_input.h"
// #include "drake/examples/pendulum/pendulum_params.h"
// #include "drake/examples/pendulum/pendulum_plant.h"
// #include "drake/examples/pendulum/pendulum_state.h"

// Symbol: pydrake_doc_examples_pendulum
constexpr struct /* pydrake_doc_examples_pendulum */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::examples
    struct /* examples */ {
      // Symbol: drake::examples::pendulum
      struct /* pendulum */ {
        // Symbol: drake::examples::pendulum::PendulumGeometry
        struct /* PendulumGeometry */ {
          // Source: drake/examples/pendulum/pendulum_geometry.h
          const char* doc =
R"""(Expresses a PendulumPlants's geometry to a SceneGraph.

.. pydrake_system::

    name: PendulumGeometry
    input_ports:
    - state
    output_ports:
    - geometry_pose

This class has no public constructor; instead use the AddToBuilder()
static method to create and add it to a DiagramBuilder directly.)""";
          // Symbol: drake::examples::pendulum::PendulumGeometry::AddToBuilder
          struct /* AddToBuilder */ {
            // Source: drake/examples/pendulum/pendulum_geometry.h
            const char* doc =
R"""(Creates, adds, and connects a PendulumGeometry system into the given
``builder``. Both the ``pendulum_state.get_system()`` and
``scene_graph`` systems must have been added to the given ``builder``
already.

The ``scene_graph`` pointer is not retained by the PendulumGeometry
system. The return value pointer is an alias of the new
PendulumGeometry system that is owned by the ``builder``.)""";
          } AddToBuilder;
          // Symbol: drake::examples::pendulum::PendulumGeometry::PendulumGeometry
          struct /* ctor */ {
            // Source: drake/examples/pendulum/pendulum_geometry.h
            const char* doc = R"""()""";
          } ctor;
        } PendulumGeometry;
        // Symbol: drake::examples::pendulum::PendulumInput
        struct /* PendulumInput */ {
          // Source: drake/examples/pendulum/pendulum_input.h
          const char* doc =
R"""(Specializes BasicVector with specific getters and setters.)""";
          // Symbol: drake::examples::pendulum::PendulumInput::DoClone
          struct /* DoClone */ {
            // Source: drake/examples/pendulum/pendulum_input.h
            const char* doc = R"""()""";
          } DoClone;
          // Symbol: drake::examples::pendulum::PendulumInput::GetCoordinateNames
          struct /* GetCoordinateNames */ {
            // Source: drake/examples/pendulum/pendulum_input.h
            const char* doc =
R"""(See PendulumInputIndices∷GetCoordinateNames().)""";
          } GetCoordinateNames;
          // Symbol: drake::examples::pendulum::PendulumInput::IsValid
          struct /* IsValid */ {
            // Source: drake/examples/pendulum/pendulum_input.h
            const char* doc =
R"""(Returns whether the current values of this vector are well-formed.)""";
          } IsValid;
          // Symbol: drake::examples::pendulum::PendulumInput::K
          struct /* K */ {
            // Source: drake/examples/pendulum/pendulum_input.h
            const char* doc =
R"""(An abbreviation for our row index constants.)""";
          } K;
          // Symbol: drake::examples::pendulum::PendulumInput::PendulumInput<T>
          struct /* ctor */ {
            // Source: drake/examples/pendulum/pendulum_input.h
            const char* doc =
R"""(Default constructor. Sets all rows to their default value:

* ``tau`` defaults to 0.0 Newton-meters.)""";
          } ctor;
          // Symbol: drake::examples::pendulum::PendulumInput::Serialize
          struct /* Serialize */ {
            // Source: drake/examples/pendulum/pendulum_input.h
            const char* doc =
R"""(Visit each field of this named vector, passing them (in order) to the
given Archive. The archive can read and/or write to the vector values.
One common use of Serialize is the //common/yaml tools.)""";
          } Serialize;
          // Symbol: drake::examples::pendulum::PendulumInput::SetToNamedVariables
          struct /* SetToNamedVariables */ {
            // Source: drake/examples/pendulum/pendulum_input.h
            const char* doc =
R"""(Create a symbolic∷Variable for each element with the known variable
name. This is only available for T == symbolic∷Expression.)""";
          } SetToNamedVariables;
          // Symbol: drake::examples::pendulum::PendulumInput::set_tau
          struct /* set_tau */ {
            // Source: drake/examples/pendulum/pendulum_input.h
            const char* doc = R"""(Setter that matches tau().)""";
          } set_tau;
          // Symbol: drake::examples::pendulum::PendulumInput::tau
          struct /* tau */ {
            // Source: drake/examples/pendulum/pendulum_input.h
            const char* doc =
R"""(Torque at the joint.

Note:
    ``tau`` is expressed in units of Newton-meters.)""";
          } tau;
          // Symbol: drake::examples::pendulum::PendulumInput::with_tau
          struct /* with_tau */ {
            // Source: drake/examples/pendulum/pendulum_input.h
            const char* doc =
R"""(Fluent setter that matches tau(). Returns a copy of ``this`` with tau
set to a new value.)""";
          } with_tau;
        } PendulumInput;
        // Symbol: drake::examples::pendulum::PendulumInputIndices
        struct /* PendulumInputIndices */ {
          // Source: drake/examples/pendulum/pendulum_input.h
          const char* doc =
R"""(Describes the row indices of a PendulumInput.)""";
          // Symbol: drake::examples::pendulum::PendulumInputIndices::GetCoordinateNames
          struct /* GetCoordinateNames */ {
            // Source: drake/examples/pendulum/pendulum_input.h
            const char* doc =
R"""(Returns a vector containing the names of each coordinate within this
class. The indices within the returned vector matches that of this
class. In other words,
``PendulumInputIndices∷GetCoordinateNames()[i]`` is the name for
``BasicVector∷GetAtIndex(i)``.)""";
          } GetCoordinateNames;
        } PendulumInputIndices;
        // Symbol: drake::examples::pendulum::PendulumParams
        struct /* PendulumParams */ {
          // Source: drake/examples/pendulum/pendulum_params.h
          const char* doc =
R"""(Specializes BasicVector with specific getters and setters.)""";
          // Symbol: drake::examples::pendulum::PendulumParams::DoClone
          struct /* DoClone */ {
            // Source: drake/examples/pendulum/pendulum_params.h
            const char* doc = R"""()""";
          } DoClone;
          // Symbol: drake::examples::pendulum::PendulumParams::GetCoordinateNames
          struct /* GetCoordinateNames */ {
            // Source: drake/examples/pendulum/pendulum_params.h
            const char* doc =
R"""(See PendulumParamsIndices∷GetCoordinateNames().)""";
          } GetCoordinateNames;
          // Symbol: drake::examples::pendulum::PendulumParams::GetElementBounds
          struct /* GetElementBounds */ {
            // Source: drake/examples/pendulum/pendulum_params.h
            const char* doc = R"""()""";
          } GetElementBounds;
          // Symbol: drake::examples::pendulum::PendulumParams::IsValid
          struct /* IsValid */ {
            // Source: drake/examples/pendulum/pendulum_params.h
            const char* doc =
R"""(Returns whether the current values of this vector are well-formed.)""";
          } IsValid;
          // Symbol: drake::examples::pendulum::PendulumParams::K
          struct /* K */ {
            // Source: drake/examples/pendulum/pendulum_params.h
            const char* doc =
R"""(An abbreviation for our row index constants.)""";
          } K;
          // Symbol: drake::examples::pendulum::PendulumParams::PendulumParams<T>
          struct /* ctor */ {
            // Source: drake/examples/pendulum/pendulum_params.h
            const char* doc =
R"""(Default constructor. Sets all rows to their default value:

* ``mass`` defaults to 1.0 kg.

* ``length`` defaults to 0.5 m.

* ``damping`` defaults to 0.1 kg m^2/s.

* ``gravity`` defaults to 9.81 m/s^2.)""";
          } ctor;
          // Symbol: drake::examples::pendulum::PendulumParams::Serialize
          struct /* Serialize */ {
            // Source: drake/examples/pendulum/pendulum_params.h
            const char* doc =
R"""(Visit each field of this named vector, passing them (in order) to the
given Archive. The archive can read and/or write to the vector values.
One common use of Serialize is the //common/yaml tools.)""";
          } Serialize;
          // Symbol: drake::examples::pendulum::PendulumParams::SetToNamedVariables
          struct /* SetToNamedVariables */ {
            // Source: drake/examples/pendulum/pendulum_params.h
            const char* doc =
R"""(Create a symbolic∷Variable for each element with the known variable
name. This is only available for T == symbolic∷Expression.)""";
          } SetToNamedVariables;
          // Symbol: drake::examples::pendulum::PendulumParams::damping
          struct /* damping */ {
            // Source: drake/examples/pendulum/pendulum_params.h
            const char* doc =
R"""(The damping friction coefficient relating angular velocity to torque.

Note:
    ``damping`` is expressed in units of kg m^2/s.

Note:
    ``damping`` has a limited domain of [0.0, +Inf].)""";
          } damping;
          // Symbol: drake::examples::pendulum::PendulumParams::gravity
          struct /* gravity */ {
            // Source: drake/examples/pendulum/pendulum_params.h
            const char* doc =
R"""(An approximate value for gravitational acceleration.

Note:
    ``gravity`` is expressed in units of m/s^2.

Note:
    ``gravity`` has a limited domain of [0.0, +Inf].)""";
          } gravity;
          // Symbol: drake::examples::pendulum::PendulumParams::length
          struct /* length */ {
            // Source: drake/examples/pendulum/pendulum_params.h
            const char* doc =
R"""(The length of the pendulum arm.

Note:
    ``length`` is expressed in units of m.

Note:
    ``length`` has a limited domain of [0.0, +Inf].)""";
          } length;
          // Symbol: drake::examples::pendulum::PendulumParams::mass
          struct /* mass */ {
            // Source: drake/examples/pendulum/pendulum_params.h
            const char* doc =
R"""(The simple pendulum has a single point mass at the end of the arm.

Note:
    ``mass`` is expressed in units of kg.

Note:
    ``mass`` has a limited domain of [0.0, +Inf].)""";
          } mass;
          // Symbol: drake::examples::pendulum::PendulumParams::set_damping
          struct /* set_damping */ {
            // Source: drake/examples/pendulum/pendulum_params.h
            const char* doc = R"""(Setter that matches damping().)""";
          } set_damping;
          // Symbol: drake::examples::pendulum::PendulumParams::set_gravity
          struct /* set_gravity */ {
            // Source: drake/examples/pendulum/pendulum_params.h
            const char* doc = R"""(Setter that matches gravity().)""";
          } set_gravity;
          // Symbol: drake::examples::pendulum::PendulumParams::set_length
          struct /* set_length */ {
            // Source: drake/examples/pendulum/pendulum_params.h
            const char* doc = R"""(Setter that matches length().)""";
          } set_length;
          // Symbol: drake::examples::pendulum::PendulumParams::set_mass
          struct /* set_mass */ {
            // Source: drake/examples/pendulum/pendulum_params.h
            const char* doc = R"""(Setter that matches mass().)""";
          } set_mass;
          // Symbol: drake::examples::pendulum::PendulumParams::with_damping
          struct /* with_damping */ {
            // Source: drake/examples/pendulum/pendulum_params.h
            const char* doc =
R"""(Fluent setter that matches damping(). Returns a copy of ``this`` with
damping set to a new value.)""";
          } with_damping;
          // Symbol: drake::examples::pendulum::PendulumParams::with_gravity
          struct /* with_gravity */ {
            // Source: drake/examples/pendulum/pendulum_params.h
            const char* doc =
R"""(Fluent setter that matches gravity(). Returns a copy of ``this`` with
gravity set to a new value.)""";
          } with_gravity;
          // Symbol: drake::examples::pendulum::PendulumParams::with_length
          struct /* with_length */ {
            // Source: drake/examples/pendulum/pendulum_params.h
            const char* doc =
R"""(Fluent setter that matches length(). Returns a copy of ``this`` with
length set to a new value.)""";
          } with_length;
          // Symbol: drake::examples::pendulum::PendulumParams::with_mass
          struct /* with_mass */ {
            // Source: drake/examples/pendulum/pendulum_params.h
            const char* doc =
R"""(Fluent setter that matches mass(). Returns a copy of ``this`` with
mass set to a new value.)""";
          } with_mass;
        } PendulumParams;
        // Symbol: drake::examples::pendulum::PendulumParamsIndices
        struct /* PendulumParamsIndices */ {
          // Source: drake/examples/pendulum/pendulum_params.h
          const char* doc =
R"""(Describes the row indices of a PendulumParams.)""";
          // Symbol: drake::examples::pendulum::PendulumParamsIndices::GetCoordinateNames
          struct /* GetCoordinateNames */ {
            // Source: drake/examples/pendulum/pendulum_params.h
            const char* doc =
R"""(Returns a vector containing the names of each coordinate within this
class. The indices within the returned vector matches that of this
class. In other words,
``PendulumParamsIndices∷GetCoordinateNames()[i]`` is the name for
``BasicVector∷GetAtIndex(i)``.)""";
          } GetCoordinateNames;
        } PendulumParamsIndices;
        // Symbol: drake::examples::pendulum::PendulumPlant
        struct /* PendulumPlant */ {
          // Source: drake/examples/pendulum/pendulum_plant.h
          const char* doc =
R"""(A model of a simple pendulum

.. math:: ml^2 \ddot\theta + b\dot\theta + mgl\sin\theta = \tau

.. pydrake_system::

name: PendulumPlant input_ports: - tau (optional) output_ports: -
state

Note: If the tau input port is not connected, then the torque is taken
to be zero.)""";
          // Symbol: drake::examples::pendulum::PendulumPlant::CalcTotalEnergy
          struct /* CalcTotalEnergy */ {
            // Source: drake/examples/pendulum/pendulum_plant.h
            const char* doc =
R"""(Calculates the kinetic + potential energy.)""";
          } CalcTotalEnergy;
          // Symbol: drake::examples::pendulum::PendulumPlant::PendulumPlant<T>
          struct /* ctor */ {
            // Source: drake/examples/pendulum/pendulum_plant.h
            const char* doc = R"""(Constructs a default plant.)""";
            // Source: drake/examples/pendulum/pendulum_plant.h
            const char* doc_copyconvert =
R"""(Scalar-converting copy constructor. See system_scalar_conversion.)""";
          } ctor;
          // Symbol: drake::examples::pendulum::PendulumPlant::get_mutable_parameters
          struct /* get_mutable_parameters */ {
            // Source: drake/examples/pendulum/pendulum_plant.h
            const char* doc = R"""()""";
          } get_mutable_parameters;
          // Symbol: drake::examples::pendulum::PendulumPlant::get_mutable_state
          struct /* get_mutable_state */ {
            // Source: drake/examples/pendulum/pendulum_plant.h
            const char* doc = R"""()""";
          } get_mutable_state;
          // Symbol: drake::examples::pendulum::PendulumPlant::get_parameters
          struct /* get_parameters */ {
            // Source: drake/examples/pendulum/pendulum_plant.h
            const char* doc = R"""()""";
          } get_parameters;
          // Symbol: drake::examples::pendulum::PendulumPlant::get_state
          struct /* get_state */ {
            // Source: drake/examples/pendulum/pendulum_plant.h
            const char* doc = R"""()""";
          } get_state;
          // Symbol: drake::examples::pendulum::PendulumPlant::get_state_output_port
          struct /* get_state_output_port */ {
            // Source: drake/examples/pendulum/pendulum_plant.h
            const char* doc = R"""(Returns the port to output state.)""";
          } get_state_output_port;
          // Symbol: drake::examples::pendulum::PendulumPlant::get_tau
          struct /* get_tau */ {
            // Source: drake/examples/pendulum/pendulum_plant.h
            const char* doc =
R"""(Evaluates the input port and returns the scalar value of the commanded
torque. If the input port is not connected, then the torque is taken
to be zero.)""";
          } get_tau;
        } PendulumPlant;
        // Symbol: drake::examples::pendulum::PendulumState
        struct /* PendulumState */ {
          // Source: drake/examples/pendulum/pendulum_state.h
          const char* doc =
R"""(Specializes BasicVector with specific getters and setters.)""";
          // Symbol: drake::examples::pendulum::PendulumState::DoClone
          struct /* DoClone */ {
            // Source: drake/examples/pendulum/pendulum_state.h
            const char* doc = R"""()""";
          } DoClone;
          // Symbol: drake::examples::pendulum::PendulumState::GetCoordinateNames
          struct /* GetCoordinateNames */ {
            // Source: drake/examples/pendulum/pendulum_state.h
            const char* doc =
R"""(See PendulumStateIndices∷GetCoordinateNames().)""";
          } GetCoordinateNames;
          // Symbol: drake::examples::pendulum::PendulumState::IsValid
          struct /* IsValid */ {
            // Source: drake/examples/pendulum/pendulum_state.h
            const char* doc =
R"""(Returns whether the current values of this vector are well-formed.)""";
          } IsValid;
          // Symbol: drake::examples::pendulum::PendulumState::K
          struct /* K */ {
            // Source: drake/examples/pendulum/pendulum_state.h
            const char* doc =
R"""(An abbreviation for our row index constants.)""";
          } K;
          // Symbol: drake::examples::pendulum::PendulumState::PendulumState<T>
          struct /* ctor */ {
            // Source: drake/examples/pendulum/pendulum_state.h
            const char* doc =
R"""(Default constructor. Sets all rows to their default value:

* ``theta`` defaults to 0.0 radians.

* ``thetadot`` defaults to 0.0 radians/sec.)""";
          } ctor;
          // Symbol: drake::examples::pendulum::PendulumState::Serialize
          struct /* Serialize */ {
            // Source: drake/examples/pendulum/pendulum_state.h
            const char* doc =
R"""(Visit each field of this named vector, passing them (in order) to the
given Archive. The archive can read and/or write to the vector values.
One common use of Serialize is the //common/yaml tools.)""";
          } Serialize;
          // Symbol: drake::examples::pendulum::PendulumState::SetToNamedVariables
          struct /* SetToNamedVariables */ {
            // Source: drake/examples/pendulum/pendulum_state.h
            const char* doc =
R"""(Create a symbolic∷Variable for each element with the known variable
name. This is only available for T == symbolic∷Expression.)""";
          } SetToNamedVariables;
          // Symbol: drake::examples::pendulum::PendulumState::set_theta
          struct /* set_theta */ {
            // Source: drake/examples/pendulum/pendulum_state.h
            const char* doc = R"""(Setter that matches theta().)""";
          } set_theta;
          // Symbol: drake::examples::pendulum::PendulumState::set_thetadot
          struct /* set_thetadot */ {
            // Source: drake/examples/pendulum/pendulum_state.h
            const char* doc = R"""(Setter that matches thetadot().)""";
          } set_thetadot;
          // Symbol: drake::examples::pendulum::PendulumState::theta
          struct /* theta */ {
            // Source: drake/examples/pendulum/pendulum_state.h
            const char* doc =
R"""(The angle of the pendulum.

Note:
    ``theta`` is expressed in units of radians.)""";
          } theta;
          // Symbol: drake::examples::pendulum::PendulumState::thetadot
          struct /* thetadot */ {
            // Source: drake/examples/pendulum/pendulum_state.h
            const char* doc =
R"""(The angular velocity of the pendulum.

Note:
    ``thetadot`` is expressed in units of radians/sec.)""";
          } thetadot;
          // Symbol: drake::examples::pendulum::PendulumState::with_theta
          struct /* with_theta */ {
            // Source: drake/examples/pendulum/pendulum_state.h
            const char* doc =
R"""(Fluent setter that matches theta(). Returns a copy of ``this`` with
theta set to a new value.)""";
          } with_theta;
          // Symbol: drake::examples::pendulum::PendulumState::with_thetadot
          struct /* with_thetadot */ {
            // Source: drake/examples/pendulum/pendulum_state.h
            const char* doc =
R"""(Fluent setter that matches thetadot(). Returns a copy of ``this`` with
thetadot set to a new value.)""";
          } with_thetadot;
        } PendulumState;
        // Symbol: drake::examples::pendulum::PendulumStateIndices
        struct /* PendulumStateIndices */ {
          // Source: drake/examples/pendulum/pendulum_state.h
          const char* doc =
R"""(Describes the row indices of a PendulumState.)""";
          // Symbol: drake::examples::pendulum::PendulumStateIndices::GetCoordinateNames
          struct /* GetCoordinateNames */ {
            // Source: drake/examples/pendulum/pendulum_state.h
            const char* doc =
R"""(Returns a vector containing the names of each coordinate within this
class. The indices within the returned vector matches that of this
class. In other words,
``PendulumStateIndices∷GetCoordinateNames()[i]`` is the name for
``BasicVector∷GetAtIndex(i)``.)""";
          } GetCoordinateNames;
        } PendulumStateIndices;
      } pendulum;
    } examples;
  } drake;
} pydrake_doc_examples_pendulum;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
