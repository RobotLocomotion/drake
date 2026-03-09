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

// #include "drake/examples/acrobot/acrobot_geometry.h"
// #include "drake/examples/acrobot/acrobot_input.h"
// #include "drake/examples/acrobot/acrobot_params.h"
// #include "drake/examples/acrobot/acrobot_plant.h"
// #include "drake/examples/acrobot/acrobot_state.h"
// #include "drake/examples/acrobot/spong_controller.h"
// #include "drake/examples/acrobot/spong_controller_params.h"

// Symbol: pydrake_doc_examples_acrobot
constexpr struct /* pydrake_doc_examples_acrobot */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::examples
    struct /* examples */ {
      // Symbol: drake::examples::acrobot
      struct /* acrobot */ {
        // Symbol: drake::examples::acrobot::AcrobotGeometry
        struct /* AcrobotGeometry */ {
          // Source: drake/examples/acrobot/acrobot_geometry.h
          const char* doc =
R"""(Expresses an AcrobotPlant's geometry to a SceneGraph.

.. pydrake_system::

    name: AcrobotGeometry
    input_ports:
    - state
    output_ports:
    - geometry_pose

This class has no public constructor; instead use the AddToBuilder()
static method to create and add it to a DiagramBuilder directly.)""";
          // Symbol: drake::examples::acrobot::AcrobotGeometry::AcrobotGeometry
          struct /* ctor */ {
            // Source: drake/examples/acrobot/acrobot_geometry.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::examples::acrobot::AcrobotGeometry::AddToBuilder
          struct /* AddToBuilder */ {
            // Source: drake/examples/acrobot/acrobot_geometry.h
            const char* doc_4args =
R"""(Creates, adds, and connects an AcrobotGeometry system into the given
``builder``. Both the ``acrobot_state.get_system()`` and
``scene_graph`` systems must have been added to the given ``builder``
already.

Parameter ``acrobot_params``:
    sets the parameters of the geometry registered with
    ``scene_graph``.

The ``scene_graph`` pointer is not retained by the AcrobotGeometry
system. The return value pointer is an alias of the new
AcrobotGeometry system that is owned by the ``builder``.)""";
            // Source: drake/examples/acrobot/acrobot_geometry.h
            const char* doc_3args =
R"""(Creates, adds, and connects an AcrobotGeometry system into the given
``builder``. Both the ``acrobot_state.get_system()`` and
``scene_graph`` systems must have been added to the given ``builder``
already.

Acrobot parameters are set to their default values.

The ``scene_graph`` pointer is not retained by the AcrobotGeometry
system. The return value pointer is an alias of the new
AcrobotGeometry system that is owned by the ``builder``.)""";
          } AddToBuilder;
        } AcrobotGeometry;
        // Symbol: drake::examples::acrobot::AcrobotInput
        struct /* AcrobotInput */ {
          // Source: drake/examples/acrobot/acrobot_input.h
          const char* doc =
R"""(Specializes BasicVector with specific getters and setters.)""";
          // Symbol: drake::examples::acrobot::AcrobotInput::AcrobotInput<T>
          struct /* ctor */ {
            // Source: drake/examples/acrobot/acrobot_input.h
            const char* doc =
R"""(Default constructor. Sets all rows to their default value:

* ``tau`` defaults to 0.0 Nm.)""";
          } ctor;
          // Symbol: drake::examples::acrobot::AcrobotInput::DoClone
          struct /* DoClone */ {
            // Source: drake/examples/acrobot/acrobot_input.h
            const char* doc = R"""()""";
          } DoClone;
          // Symbol: drake::examples::acrobot::AcrobotInput::GetCoordinateNames
          struct /* GetCoordinateNames */ {
            // Source: drake/examples/acrobot/acrobot_input.h
            const char* doc =
R"""(See AcrobotInputIndices∷GetCoordinateNames().)""";
          } GetCoordinateNames;
          // Symbol: drake::examples::acrobot::AcrobotInput::IsValid
          struct /* IsValid */ {
            // Source: drake/examples/acrobot/acrobot_input.h
            const char* doc =
R"""(Returns whether the current values of this vector are well-formed.)""";
          } IsValid;
          // Symbol: drake::examples::acrobot::AcrobotInput::K
          struct /* K */ {
            // Source: drake/examples/acrobot/acrobot_input.h
            const char* doc =
R"""(An abbreviation for our row index constants.)""";
          } K;
          // Symbol: drake::examples::acrobot::AcrobotInput::Serialize
          struct /* Serialize */ {
            // Source: drake/examples/acrobot/acrobot_input.h
            const char* doc =
R"""(Visit each field of this named vector, passing them (in order) to the
given Archive. The archive can read and/or write to the vector values.
One common use of Serialize is the //common/yaml tools.)""";
          } Serialize;
          // Symbol: drake::examples::acrobot::AcrobotInput::SetToNamedVariables
          struct /* SetToNamedVariables */ {
            // Source: drake/examples/acrobot/acrobot_input.h
            const char* doc =
R"""(Create a symbolic∷Variable for each element with the known variable
name. This is only available for T == symbolic∷Expression.)""";
          } SetToNamedVariables;
          // Symbol: drake::examples::acrobot::AcrobotInput::set_tau
          struct /* set_tau */ {
            // Source: drake/examples/acrobot/acrobot_input.h
            const char* doc = R"""(Setter that matches tau().)""";
          } set_tau;
          // Symbol: drake::examples::acrobot::AcrobotInput::tau
          struct /* tau */ {
            // Source: drake/examples/acrobot/acrobot_input.h
            const char* doc =
R"""(Torque at the elbow

Note:
    ``tau`` is expressed in units of Nm.)""";
          } tau;
          // Symbol: drake::examples::acrobot::AcrobotInput::with_tau
          struct /* with_tau */ {
            // Source: drake/examples/acrobot/acrobot_input.h
            const char* doc =
R"""(Fluent setter that matches tau(). Returns a copy of ``this`` with tau
set to a new value.)""";
          } with_tau;
        } AcrobotInput;
        // Symbol: drake::examples::acrobot::AcrobotInputIndices
        struct /* AcrobotInputIndices */ {
          // Source: drake/examples/acrobot/acrobot_input.h
          const char* doc =
R"""(Describes the row indices of a AcrobotInput.)""";
          // Symbol: drake::examples::acrobot::AcrobotInputIndices::GetCoordinateNames
          struct /* GetCoordinateNames */ {
            // Source: drake/examples/acrobot/acrobot_input.h
            const char* doc =
R"""(Returns a vector containing the names of each coordinate within this
class. The indices within the returned vector matches that of this
class. In other words, ``AcrobotInputIndices∷GetCoordinateNames()[i]``
is the name for ``BasicVector∷GetAtIndex(i)``.)""";
          } GetCoordinateNames;
        } AcrobotInputIndices;
        // Symbol: drake::examples::acrobot::AcrobotParams
        struct /* AcrobotParams */ {
          // Source: drake/examples/acrobot/acrobot_params.h
          const char* doc =
R"""(Specializes BasicVector with specific getters and setters.)""";
          // Symbol: drake::examples::acrobot::AcrobotParams::AcrobotParams<T>
          struct /* ctor */ {
            // Source: drake/examples/acrobot/acrobot_params.h
            const char* doc =
R"""(Default constructor. Sets all rows to their default value:

* ``m1`` defaults to 1.0 kg.

* ``m2`` defaults to 1.0 kg.

* ``l1`` defaults to 1.0 m.

* ``l2`` defaults to 2.0 m.

* ``lc1`` defaults to 0.5 m.

* ``lc2`` defaults to 1.0 m.

* ``Ic1`` defaults to 0.083 kg*m^2.

* ``Ic2`` defaults to 0.33 kg*m^2.

* ``b1`` defaults to 0.1 kg*m^2/s.

* ``b2`` defaults to 0.1 kg*m^2/s.

* ``gravity`` defaults to 9.81 m/s^2.)""";
          } ctor;
          // Symbol: drake::examples::acrobot::AcrobotParams::DoClone
          struct /* DoClone */ {
            // Source: drake/examples/acrobot/acrobot_params.h
            const char* doc = R"""()""";
          } DoClone;
          // Symbol: drake::examples::acrobot::AcrobotParams::GetCoordinateNames
          struct /* GetCoordinateNames */ {
            // Source: drake/examples/acrobot/acrobot_params.h
            const char* doc =
R"""(See AcrobotParamsIndices∷GetCoordinateNames().)""";
          } GetCoordinateNames;
          // Symbol: drake::examples::acrobot::AcrobotParams::GetElementBounds
          struct /* GetElementBounds */ {
            // Source: drake/examples/acrobot/acrobot_params.h
            const char* doc = R"""()""";
          } GetElementBounds;
          // Symbol: drake::examples::acrobot::AcrobotParams::Ic1
          struct /* Ic1 */ {
            // Source: drake/examples/acrobot/acrobot_params.h
            const char* doc =
R"""(Inertia of link 1 about the center of mass of link 1.

Note:
    ``Ic1`` is expressed in units of kg*m^2.

Note:
    ``Ic1`` has a limited domain of [0.0, +Inf].)""";
          } Ic1;
          // Symbol: drake::examples::acrobot::AcrobotParams::Ic2
          struct /* Ic2 */ {
            // Source: drake/examples/acrobot/acrobot_params.h
            const char* doc =
R"""(Inertia of link 2 about the center of mass of link 2.

Note:
    ``Ic2`` is expressed in units of kg*m^2.

Note:
    ``Ic2`` has a limited domain of [0.0, +Inf].)""";
          } Ic2;
          // Symbol: drake::examples::acrobot::AcrobotParams::IsValid
          struct /* IsValid */ {
            // Source: drake/examples/acrobot/acrobot_params.h
            const char* doc =
R"""(Returns whether the current values of this vector are well-formed.)""";
          } IsValid;
          // Symbol: drake::examples::acrobot::AcrobotParams::K
          struct /* K */ {
            // Source: drake/examples/acrobot/acrobot_params.h
            const char* doc =
R"""(An abbreviation for our row index constants.)""";
          } K;
          // Symbol: drake::examples::acrobot::AcrobotParams::Serialize
          struct /* Serialize */ {
            // Source: drake/examples/acrobot/acrobot_params.h
            const char* doc =
R"""(Visit each field of this named vector, passing them (in order) to the
given Archive. The archive can read and/or write to the vector values.
One common use of Serialize is the //common/yaml tools.)""";
          } Serialize;
          // Symbol: drake::examples::acrobot::AcrobotParams::SetToNamedVariables
          struct /* SetToNamedVariables */ {
            // Source: drake/examples/acrobot/acrobot_params.h
            const char* doc =
R"""(Create a symbolic∷Variable for each element with the known variable
name. This is only available for T == symbolic∷Expression.)""";
          } SetToNamedVariables;
          // Symbol: drake::examples::acrobot::AcrobotParams::b1
          struct /* b1 */ {
            // Source: drake/examples/acrobot/acrobot_params.h
            const char* doc =
R"""(Damping coefficient of the shoulder joint.

Note:
    ``b1`` is expressed in units of kg*m^2/s.

Note:
    ``b1`` has a limited domain of [0.0, +Inf].)""";
          } b1;
          // Symbol: drake::examples::acrobot::AcrobotParams::b2
          struct /* b2 */ {
            // Source: drake/examples/acrobot/acrobot_params.h
            const char* doc =
R"""(Damping coefficient of the elbow joint.

Note:
    ``b2`` is expressed in units of kg*m^2/s.

Note:
    ``b2`` has a limited domain of [0.0, +Inf].)""";
          } b2;
          // Symbol: drake::examples::acrobot::AcrobotParams::gravity
          struct /* gravity */ {
            // Source: drake/examples/acrobot/acrobot_params.h
            const char* doc =
R"""(Gravitational constant.

Note:
    ``gravity`` is expressed in units of m/s^2.

Note:
    ``gravity`` has a limited domain of [0.0, +Inf].)""";
          } gravity;
          // Symbol: drake::examples::acrobot::AcrobotParams::l1
          struct /* l1 */ {
            // Source: drake/examples/acrobot/acrobot_params.h
            const char* doc =
R"""(Length of link 1.

Note:
    ``l1`` is expressed in units of m.

Note:
    ``l1`` has a limited domain of [0.0, +Inf].)""";
          } l1;
          // Symbol: drake::examples::acrobot::AcrobotParams::l2
          struct /* l2 */ {
            // Source: drake/examples/acrobot/acrobot_params.h
            const char* doc =
R"""(Length of link 2.

Note:
    ``l2`` is expressed in units of m.

Note:
    ``l2`` has a limited domain of [0.0, +Inf].)""";
          } l2;
          // Symbol: drake::examples::acrobot::AcrobotParams::lc1
          struct /* lc1 */ {
            // Source: drake/examples/acrobot/acrobot_params.h
            const char* doc =
R"""(Vertical distance from shoulder joint to center of mass of link 1.

Note:
    ``lc1`` is expressed in units of m.

Note:
    ``lc1`` has a limited domain of [0.0, +Inf].)""";
          } lc1;
          // Symbol: drake::examples::acrobot::AcrobotParams::lc2
          struct /* lc2 */ {
            // Source: drake/examples/acrobot/acrobot_params.h
            const char* doc =
R"""(Vertical distance from elbow joint to center of mass of link 1.

Note:
    ``lc2`` is expressed in units of m.

Note:
    ``lc2`` has a limited domain of [0.0, +Inf].)""";
          } lc2;
          // Symbol: drake::examples::acrobot::AcrobotParams::m1
          struct /* m1 */ {
            // Source: drake/examples/acrobot/acrobot_params.h
            const char* doc =
R"""(Mass of link 1.

Note:
    ``m1`` is expressed in units of kg.

Note:
    ``m1`` has a limited domain of [0.0, +Inf].)""";
          } m1;
          // Symbol: drake::examples::acrobot::AcrobotParams::m2
          struct /* m2 */ {
            // Source: drake/examples/acrobot/acrobot_params.h
            const char* doc =
R"""(Mass of link 2.

Note:
    ``m2`` is expressed in units of kg.

Note:
    ``m2`` has a limited domain of [0.0, +Inf].)""";
          } m2;
          // Symbol: drake::examples::acrobot::AcrobotParams::set_Ic1
          struct /* set_Ic1 */ {
            // Source: drake/examples/acrobot/acrobot_params.h
            const char* doc = R"""(Setter that matches Ic1().)""";
          } set_Ic1;
          // Symbol: drake::examples::acrobot::AcrobotParams::set_Ic2
          struct /* set_Ic2 */ {
            // Source: drake/examples/acrobot/acrobot_params.h
            const char* doc = R"""(Setter that matches Ic2().)""";
          } set_Ic2;
          // Symbol: drake::examples::acrobot::AcrobotParams::set_b1
          struct /* set_b1 */ {
            // Source: drake/examples/acrobot/acrobot_params.h
            const char* doc = R"""(Setter that matches b1().)""";
          } set_b1;
          // Symbol: drake::examples::acrobot::AcrobotParams::set_b2
          struct /* set_b2 */ {
            // Source: drake/examples/acrobot/acrobot_params.h
            const char* doc = R"""(Setter that matches b2().)""";
          } set_b2;
          // Symbol: drake::examples::acrobot::AcrobotParams::set_gravity
          struct /* set_gravity */ {
            // Source: drake/examples/acrobot/acrobot_params.h
            const char* doc = R"""(Setter that matches gravity().)""";
          } set_gravity;
          // Symbol: drake::examples::acrobot::AcrobotParams::set_l1
          struct /* set_l1 */ {
            // Source: drake/examples/acrobot/acrobot_params.h
            const char* doc = R"""(Setter that matches l1().)""";
          } set_l1;
          // Symbol: drake::examples::acrobot::AcrobotParams::set_l2
          struct /* set_l2 */ {
            // Source: drake/examples/acrobot/acrobot_params.h
            const char* doc = R"""(Setter that matches l2().)""";
          } set_l2;
          // Symbol: drake::examples::acrobot::AcrobotParams::set_lc1
          struct /* set_lc1 */ {
            // Source: drake/examples/acrobot/acrobot_params.h
            const char* doc = R"""(Setter that matches lc1().)""";
          } set_lc1;
          // Symbol: drake::examples::acrobot::AcrobotParams::set_lc2
          struct /* set_lc2 */ {
            // Source: drake/examples/acrobot/acrobot_params.h
            const char* doc = R"""(Setter that matches lc2().)""";
          } set_lc2;
          // Symbol: drake::examples::acrobot::AcrobotParams::set_m1
          struct /* set_m1 */ {
            // Source: drake/examples/acrobot/acrobot_params.h
            const char* doc = R"""(Setter that matches m1().)""";
          } set_m1;
          // Symbol: drake::examples::acrobot::AcrobotParams::set_m2
          struct /* set_m2 */ {
            // Source: drake/examples/acrobot/acrobot_params.h
            const char* doc = R"""(Setter that matches m2().)""";
          } set_m2;
          // Symbol: drake::examples::acrobot::AcrobotParams::with_Ic1
          struct /* with_Ic1 */ {
            // Source: drake/examples/acrobot/acrobot_params.h
            const char* doc =
R"""(Fluent setter that matches Ic1(). Returns a copy of ``this`` with Ic1
set to a new value.)""";
          } with_Ic1;
          // Symbol: drake::examples::acrobot::AcrobotParams::with_Ic2
          struct /* with_Ic2 */ {
            // Source: drake/examples/acrobot/acrobot_params.h
            const char* doc =
R"""(Fluent setter that matches Ic2(). Returns a copy of ``this`` with Ic2
set to a new value.)""";
          } with_Ic2;
          // Symbol: drake::examples::acrobot::AcrobotParams::with_b1
          struct /* with_b1 */ {
            // Source: drake/examples/acrobot/acrobot_params.h
            const char* doc =
R"""(Fluent setter that matches b1(). Returns a copy of ``this`` with b1
set to a new value.)""";
          } with_b1;
          // Symbol: drake::examples::acrobot::AcrobotParams::with_b2
          struct /* with_b2 */ {
            // Source: drake/examples/acrobot/acrobot_params.h
            const char* doc =
R"""(Fluent setter that matches b2(). Returns a copy of ``this`` with b2
set to a new value.)""";
          } with_b2;
          // Symbol: drake::examples::acrobot::AcrobotParams::with_gravity
          struct /* with_gravity */ {
            // Source: drake/examples/acrobot/acrobot_params.h
            const char* doc =
R"""(Fluent setter that matches gravity(). Returns a copy of ``this`` with
gravity set to a new value.)""";
          } with_gravity;
          // Symbol: drake::examples::acrobot::AcrobotParams::with_l1
          struct /* with_l1 */ {
            // Source: drake/examples/acrobot/acrobot_params.h
            const char* doc =
R"""(Fluent setter that matches l1(). Returns a copy of ``this`` with l1
set to a new value.)""";
          } with_l1;
          // Symbol: drake::examples::acrobot::AcrobotParams::with_l2
          struct /* with_l2 */ {
            // Source: drake/examples/acrobot/acrobot_params.h
            const char* doc =
R"""(Fluent setter that matches l2(). Returns a copy of ``this`` with l2
set to a new value.)""";
          } with_l2;
          // Symbol: drake::examples::acrobot::AcrobotParams::with_lc1
          struct /* with_lc1 */ {
            // Source: drake/examples/acrobot/acrobot_params.h
            const char* doc =
R"""(Fluent setter that matches lc1(). Returns a copy of ``this`` with lc1
set to a new value.)""";
          } with_lc1;
          // Symbol: drake::examples::acrobot::AcrobotParams::with_lc2
          struct /* with_lc2 */ {
            // Source: drake/examples/acrobot/acrobot_params.h
            const char* doc =
R"""(Fluent setter that matches lc2(). Returns a copy of ``this`` with lc2
set to a new value.)""";
          } with_lc2;
          // Symbol: drake::examples::acrobot::AcrobotParams::with_m1
          struct /* with_m1 */ {
            // Source: drake/examples/acrobot/acrobot_params.h
            const char* doc =
R"""(Fluent setter that matches m1(). Returns a copy of ``this`` with m1
set to a new value.)""";
          } with_m1;
          // Symbol: drake::examples::acrobot::AcrobotParams::with_m2
          struct /* with_m2 */ {
            // Source: drake/examples/acrobot/acrobot_params.h
            const char* doc =
R"""(Fluent setter that matches m2(). Returns a copy of ``this`` with m2
set to a new value.)""";
          } with_m2;
        } AcrobotParams;
        // Symbol: drake::examples::acrobot::AcrobotParamsIndices
        struct /* AcrobotParamsIndices */ {
          // Source: drake/examples/acrobot/acrobot_params.h
          const char* doc =
R"""(Describes the row indices of a AcrobotParams.)""";
          // Symbol: drake::examples::acrobot::AcrobotParamsIndices::GetCoordinateNames
          struct /* GetCoordinateNames */ {
            // Source: drake/examples/acrobot/acrobot_params.h
            const char* doc =
R"""(Returns a vector containing the names of each coordinate within this
class. The indices within the returned vector matches that of this
class. In other words,
``AcrobotParamsIndices∷GetCoordinateNames()[i]`` is the name for
``BasicVector∷GetAtIndex(i)``.)""";
          } GetCoordinateNames;
        } AcrobotParamsIndices;
        // Symbol: drake::examples::acrobot::AcrobotPlant
        struct /* AcrobotPlant */ {
          // Source: drake/examples/acrobot/acrobot_plant.h
          const char* doc =
R"""(The Acrobot - a canonical underactuated system as described in <a
href="http://underactuated.mit.edu/underactuated.html?chapter=3">Chapter
3 of Underactuated Robotics</a>.

.. pydrake_system::

    name: AcrobotPlant
    input_ports:
    - elbow_torque (optional)
    output_ports:
    - acrobot_state

Note: If the elbow_torque input port is not connected, then the torque
is taken to be zero.)""";
          // Symbol: drake::examples::acrobot::AcrobotPlant::AcrobotPlant<T>
          struct /* ctor */ {
            // Source: drake/examples/acrobot/acrobot_plant.h
            const char* doc =
R"""(Constructs the plant. The parameters of the system are stored as
Parameters in the Context (see acrobot_params.h).)""";
            // Source: drake/examples/acrobot/acrobot_plant.h
            const char* doc_copyconvert =
R"""(Scalar-converting copy constructor. See system_scalar_conversion.)""";
          } ctor;
          // Symbol: drake::examples::acrobot::AcrobotPlant::DynamicsBiasTerm
          struct /* DynamicsBiasTerm */ {
            // Source: drake/examples/acrobot/acrobot_plant.h
            const char* doc =
R"""(Manipulator equation of Acrobot: M(q)q̈ + bias(q,q̇) = B*u.

- M[2x2] is the mass matrix. - bias[2x1] includes the Coriolis term,
gravity term and the damping term, i.e. bias[2x1] = C(q,v)*v - τ_g(q)
+ [b1*q̇₁;b2*q̇₂].)""";
          } DynamicsBiasTerm;
          // Symbol: drake::examples::acrobot::AcrobotPlant::MassMatrix
          struct /* MassMatrix */ {
            // Source: drake/examples/acrobot/acrobot_plant.h
            const char* doc = R"""()""";
          } MassMatrix;
          // Symbol: drake::examples::acrobot::AcrobotPlant::SetMitAcrobotParameters
          struct /* SetMitAcrobotParameters */ {
            // Source: drake/examples/acrobot/acrobot_plant.h
            const char* doc =
R"""(Sets the parameters to describe MIT Robot Locomotion Group's hardware
acrobot.)""";
          } SetMitAcrobotParameters;
          // Symbol: drake::examples::acrobot::AcrobotPlant::get_mutable_parameters
          struct /* get_mutable_parameters */ {
            // Source: drake/examples/acrobot/acrobot_plant.h
            const char* doc = R"""()""";
          } get_mutable_parameters;
          // Symbol: drake::examples::acrobot::AcrobotPlant::get_mutable_state
          struct /* get_mutable_state */ {
            // Source: drake/examples/acrobot/acrobot_plant.h
            const char* doc = R"""()""";
          } get_mutable_state;
          // Symbol: drake::examples::acrobot::AcrobotPlant::get_parameters
          struct /* get_parameters */ {
            // Source: drake/examples/acrobot/acrobot_plant.h
            const char* doc = R"""()""";
          } get_parameters;
          // Symbol: drake::examples::acrobot::AcrobotPlant::get_state
          struct /* get_state */ {
            // Source: drake/examples/acrobot/acrobot_plant.h
            const char* doc = R"""()""";
          } get_state;
          // Symbol: drake::examples::acrobot::AcrobotPlant::get_tau
          struct /* get_tau */ {
            // Source: drake/examples/acrobot/acrobot_plant.h
            const char* doc =
R"""(Evaluates the input port and returns the scalar value of the commanded
torque. If the input port is not connected, then the torque is taken
to be zero.)""";
          } get_tau;
        } AcrobotPlant;
        // Symbol: drake::examples::acrobot::AcrobotSpongController
        struct /* AcrobotSpongController */ {
          // Source: drake/examples/acrobot/spong_controller.h
          const char* doc =
R"""(The Spong acrobot swing-up controller as described in: Spong, Mark W.
"Swing up control of the acrobot." Robotics and Automation, 1994.
Proceedings., 1994 IEEE International Conference on. IEEE, 1994.

Note that the Spong controller works well on the default set of
parameters, which Spong used in his paper. In contrast, it is
difficult to find a functional set of gains to stabilize the robot
about its upright fixed point using the parameters of the physical
robot we have in lab.

.. pydrake_system::

    name: AcrobotSpongController
    input_ports:
    - acrobot_state
    output_ports:
    - elbow_torque)""";
          // Symbol: drake::examples::acrobot::AcrobotSpongController::AcrobotSpongController<T>
          struct /* ctor */ {
            // Source: drake/examples/acrobot/spong_controller.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::examples::acrobot::AcrobotSpongController::CalcControlTorque
          struct /* CalcControlTorque */ {
            // Source: drake/examples/acrobot/spong_controller.h
            const char* doc = R"""()""";
          } CalcControlTorque;
          // Symbol: drake::examples::acrobot::AcrobotSpongController::get_mutable_parameters
          struct /* get_mutable_parameters */ {
            // Source: drake/examples/acrobot/spong_controller.h
            const char* doc = R"""()""";
          } get_mutable_parameters;
          // Symbol: drake::examples::acrobot::AcrobotSpongController::get_parameters
          struct /* get_parameters */ {
            // Source: drake/examples/acrobot/spong_controller.h
            const char* doc = R"""()""";
          } get_parameters;
        } AcrobotSpongController;
        // Symbol: drake::examples::acrobot::AcrobotState
        struct /* AcrobotState */ {
          // Source: drake/examples/acrobot/acrobot_state.h
          const char* doc =
R"""(Specializes BasicVector with specific getters and setters.)""";
          // Symbol: drake::examples::acrobot::AcrobotState::AcrobotState<T>
          struct /* ctor */ {
            // Source: drake/examples/acrobot/acrobot_state.h
            const char* doc =
R"""(Default constructor. Sets all rows to their default value:

* ``theta1`` defaults to 0.0 rad.

* ``theta2`` defaults to 0.0 rad.

* ``theta1dot`` defaults to 0.0 rad/s.

* ``theta2dot`` defaults to 0.0 rad/s.)""";
          } ctor;
          // Symbol: drake::examples::acrobot::AcrobotState::DoClone
          struct /* DoClone */ {
            // Source: drake/examples/acrobot/acrobot_state.h
            const char* doc = R"""()""";
          } DoClone;
          // Symbol: drake::examples::acrobot::AcrobotState::GetCoordinateNames
          struct /* GetCoordinateNames */ {
            // Source: drake/examples/acrobot/acrobot_state.h
            const char* doc =
R"""(See AcrobotStateIndices∷GetCoordinateNames().)""";
          } GetCoordinateNames;
          // Symbol: drake::examples::acrobot::AcrobotState::IsValid
          struct /* IsValid */ {
            // Source: drake/examples/acrobot/acrobot_state.h
            const char* doc =
R"""(Returns whether the current values of this vector are well-formed.)""";
          } IsValid;
          // Symbol: drake::examples::acrobot::AcrobotState::K
          struct /* K */ {
            // Source: drake/examples/acrobot/acrobot_state.h
            const char* doc =
R"""(An abbreviation for our row index constants.)""";
          } K;
          // Symbol: drake::examples::acrobot::AcrobotState::Serialize
          struct /* Serialize */ {
            // Source: drake/examples/acrobot/acrobot_state.h
            const char* doc =
R"""(Visit each field of this named vector, passing them (in order) to the
given Archive. The archive can read and/or write to the vector values.
One common use of Serialize is the //common/yaml tools.)""";
          } Serialize;
          // Symbol: drake::examples::acrobot::AcrobotState::SetToNamedVariables
          struct /* SetToNamedVariables */ {
            // Source: drake/examples/acrobot/acrobot_state.h
            const char* doc =
R"""(Create a symbolic∷Variable for each element with the known variable
name. This is only available for T == symbolic∷Expression.)""";
          } SetToNamedVariables;
          // Symbol: drake::examples::acrobot::AcrobotState::set_theta1
          struct /* set_theta1 */ {
            // Source: drake/examples/acrobot/acrobot_state.h
            const char* doc = R"""(Setter that matches theta1().)""";
          } set_theta1;
          // Symbol: drake::examples::acrobot::AcrobotState::set_theta1dot
          struct /* set_theta1dot */ {
            // Source: drake/examples/acrobot/acrobot_state.h
            const char* doc = R"""(Setter that matches theta1dot().)""";
          } set_theta1dot;
          // Symbol: drake::examples::acrobot::AcrobotState::set_theta2
          struct /* set_theta2 */ {
            // Source: drake/examples/acrobot/acrobot_state.h
            const char* doc = R"""(Setter that matches theta2().)""";
          } set_theta2;
          // Symbol: drake::examples::acrobot::AcrobotState::set_theta2dot
          struct /* set_theta2dot */ {
            // Source: drake/examples/acrobot/acrobot_state.h
            const char* doc = R"""(Setter that matches theta2dot().)""";
          } set_theta2dot;
          // Symbol: drake::examples::acrobot::AcrobotState::theta1
          struct /* theta1 */ {
            // Source: drake/examples/acrobot/acrobot_state.h
            const char* doc =
R"""(The shoulder joint angle

Note:
    ``theta1`` is expressed in units of rad.)""";
          } theta1;
          // Symbol: drake::examples::acrobot::AcrobotState::theta1dot
          struct /* theta1dot */ {
            // Source: drake/examples/acrobot/acrobot_state.h
            const char* doc =
R"""(The shoulder joint velocity

Note:
    ``theta1dot`` is expressed in units of rad/s.)""";
          } theta1dot;
          // Symbol: drake::examples::acrobot::AcrobotState::theta2
          struct /* theta2 */ {
            // Source: drake/examples/acrobot/acrobot_state.h
            const char* doc =
R"""(The elbow joint angle

Note:
    ``theta2`` is expressed in units of rad.)""";
          } theta2;
          // Symbol: drake::examples::acrobot::AcrobotState::theta2dot
          struct /* theta2dot */ {
            // Source: drake/examples/acrobot/acrobot_state.h
            const char* doc =
R"""(The elbow joint velocity

Note:
    ``theta2dot`` is expressed in units of rad/s.)""";
          } theta2dot;
          // Symbol: drake::examples::acrobot::AcrobotState::with_theta1
          struct /* with_theta1 */ {
            // Source: drake/examples/acrobot/acrobot_state.h
            const char* doc =
R"""(Fluent setter that matches theta1(). Returns a copy of ``this`` with
theta1 set to a new value.)""";
          } with_theta1;
          // Symbol: drake::examples::acrobot::AcrobotState::with_theta1dot
          struct /* with_theta1dot */ {
            // Source: drake/examples/acrobot/acrobot_state.h
            const char* doc =
R"""(Fluent setter that matches theta1dot(). Returns a copy of ``this``
with theta1dot set to a new value.)""";
          } with_theta1dot;
          // Symbol: drake::examples::acrobot::AcrobotState::with_theta2
          struct /* with_theta2 */ {
            // Source: drake/examples/acrobot/acrobot_state.h
            const char* doc =
R"""(Fluent setter that matches theta2(). Returns a copy of ``this`` with
theta2 set to a new value.)""";
          } with_theta2;
          // Symbol: drake::examples::acrobot::AcrobotState::with_theta2dot
          struct /* with_theta2dot */ {
            // Source: drake/examples/acrobot/acrobot_state.h
            const char* doc =
R"""(Fluent setter that matches theta2dot(). Returns a copy of ``this``
with theta2dot set to a new value.)""";
          } with_theta2dot;
        } AcrobotState;
        // Symbol: drake::examples::acrobot::AcrobotStateIndices
        struct /* AcrobotStateIndices */ {
          // Source: drake/examples/acrobot/acrobot_state.h
          const char* doc =
R"""(Describes the row indices of a AcrobotState.)""";
          // Symbol: drake::examples::acrobot::AcrobotStateIndices::GetCoordinateNames
          struct /* GetCoordinateNames */ {
            // Source: drake/examples/acrobot/acrobot_state.h
            const char* doc =
R"""(Returns a vector containing the names of each coordinate within this
class. The indices within the returned vector matches that of this
class. In other words, ``AcrobotStateIndices∷GetCoordinateNames()[i]``
is the name for ``BasicVector∷GetAtIndex(i)``.)""";
          } GetCoordinateNames;
        } AcrobotStateIndices;
        // Symbol: drake::examples::acrobot::AcrobotWEncoder
        struct /* AcrobotWEncoder */ {
          // Source: drake/examples/acrobot/acrobot_plant.h
          const char* doc =
R"""(Constructs the Acrobot with (only) encoder outputs.

.. pydrake_system::

    name: AcrobotWEncoder
    input_ports:
    - elbow_torque
    output_ports:
    - measured_joint_positions
    - <span style="color:gray">acrobot_state</span>

The ``acrobot_state`` output port is present only if the construction
parameter ``acrobot_state_as_second_output`` is true.)""";
          // Symbol: drake::examples::acrobot::AcrobotWEncoder::AcrobotWEncoder<T>
          struct /* ctor */ {
            // Source: drake/examples/acrobot/acrobot_plant.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::examples::acrobot::AcrobotWEncoder::acrobot_plant
          struct /* acrobot_plant */ {
            // Source: drake/examples/acrobot/acrobot_plant.h
            const char* doc = R"""()""";
          } acrobot_plant;
          // Symbol: drake::examples::acrobot::AcrobotWEncoder::get_mutable_acrobot_state
          struct /* get_mutable_acrobot_state */ {
            // Source: drake/examples/acrobot/acrobot_plant.h
            const char* doc = R"""()""";
          } get_mutable_acrobot_state;
        } AcrobotWEncoder;
        // Symbol: drake::examples::acrobot::BalancingLQRController
        struct /* BalancingLQRController */ {
          // Source: drake/examples/acrobot/acrobot_plant.h
          const char* doc =
R"""(Constructs the LQR controller for stabilizing the upright fixed point
using default LQR cost matrices which have been tested for this
system.)""";
        } BalancingLQRController;
        // Symbol: drake::examples::acrobot::SpongControllerParams
        struct /* SpongControllerParams */ {
          // Source: drake/examples/acrobot/spong_controller_params.h
          const char* doc =
R"""(Specializes BasicVector with specific getters and setters.)""";
          // Symbol: drake::examples::acrobot::SpongControllerParams::DoClone
          struct /* DoClone */ {
            // Source: drake/examples/acrobot/spong_controller_params.h
            const char* doc = R"""()""";
          } DoClone;
          // Symbol: drake::examples::acrobot::SpongControllerParams::GetCoordinateNames
          struct /* GetCoordinateNames */ {
            // Source: drake/examples/acrobot/spong_controller_params.h
            const char* doc =
R"""(See SpongControllerParamsIndices∷GetCoordinateNames().)""";
          } GetCoordinateNames;
          // Symbol: drake::examples::acrobot::SpongControllerParams::GetElementBounds
          struct /* GetElementBounds */ {
            // Source: drake/examples/acrobot/spong_controller_params.h
            const char* doc = R"""()""";
          } GetElementBounds;
          // Symbol: drake::examples::acrobot::SpongControllerParams::IsValid
          struct /* IsValid */ {
            // Source: drake/examples/acrobot/spong_controller_params.h
            const char* doc =
R"""(Returns whether the current values of this vector are well-formed.)""";
          } IsValid;
          // Symbol: drake::examples::acrobot::SpongControllerParams::K
          struct /* K */ {
            // Source: drake/examples/acrobot/spong_controller_params.h
            const char* doc =
R"""(An abbreviation for our row index constants.)""";
          } K;
          // Symbol: drake::examples::acrobot::SpongControllerParams::Serialize
          struct /* Serialize */ {
            // Source: drake/examples/acrobot/spong_controller_params.h
            const char* doc =
R"""(Visit each field of this named vector, passing them (in order) to the
given Archive. The archive can read and/or write to the vector values.
One common use of Serialize is the //common/yaml tools.)""";
          } Serialize;
          // Symbol: drake::examples::acrobot::SpongControllerParams::SetToNamedVariables
          struct /* SetToNamedVariables */ {
            // Source: drake/examples/acrobot/spong_controller_params.h
            const char* doc =
R"""(Create a symbolic∷Variable for each element with the known variable
name. This is only available for T == symbolic∷Expression.)""";
          } SetToNamedVariables;
          // Symbol: drake::examples::acrobot::SpongControllerParams::SpongControllerParams<T>
          struct /* ctor */ {
            // Source: drake/examples/acrobot/spong_controller_params.h
            const char* doc =
R"""(Default constructor. Sets all rows to their default value:

* ``k_e`` defaults to 5.0 s.

* ``k_p`` defaults to 50.0 s^-2.

* ``k_d`` defaults to 5.0 s^-1.

* ``balancing_threshold`` defaults to 1e3 None.)""";
          } ctor;
          // Symbol: drake::examples::acrobot::SpongControllerParams::balancing_threshold
          struct /* balancing_threshold */ {
            // Source: drake/examples/acrobot/spong_controller_params.h
            const char* doc =
R"""(Cost value at which to switch from swing up to balancing.

Note:
    ``balancing_threshold`` is expressed in units of None.

Note:
    ``balancing_threshold`` has a limited domain of [0.0, +Inf].)""";
          } balancing_threshold;
          // Symbol: drake::examples::acrobot::SpongControllerParams::k_d
          struct /* k_d */ {
            // Source: drake/examples/acrobot/spong_controller_params.h
            const char* doc =
R"""(Partial feedback linearization derivative gain.

Note:
    ``k_d`` is expressed in units of s^-1.

Note:
    ``k_d`` has a limited domain of [0.0, +Inf].)""";
          } k_d;
          // Symbol: drake::examples::acrobot::SpongControllerParams::k_e
          struct /* k_e */ {
            // Source: drake/examples/acrobot/spong_controller_params.h
            const char* doc =
R"""(Energy shaping gain.

Note:
    ``k_e`` is expressed in units of s.

Note:
    ``k_e`` has a limited domain of [0.0, +Inf].)""";
          } k_e;
          // Symbol: drake::examples::acrobot::SpongControllerParams::k_p
          struct /* k_p */ {
            // Source: drake/examples/acrobot/spong_controller_params.h
            const char* doc =
R"""(Partial feedback linearization proportional gain.

Note:
    ``k_p`` is expressed in units of s^-2.

Note:
    ``k_p`` has a limited domain of [0.0, +Inf].)""";
          } k_p;
          // Symbol: drake::examples::acrobot::SpongControllerParams::set_balancing_threshold
          struct /* set_balancing_threshold */ {
            // Source: drake/examples/acrobot/spong_controller_params.h
            const char* doc =
R"""(Setter that matches balancing_threshold().)""";
          } set_balancing_threshold;
          // Symbol: drake::examples::acrobot::SpongControllerParams::set_k_d
          struct /* set_k_d */ {
            // Source: drake/examples/acrobot/spong_controller_params.h
            const char* doc = R"""(Setter that matches k_d().)""";
          } set_k_d;
          // Symbol: drake::examples::acrobot::SpongControllerParams::set_k_e
          struct /* set_k_e */ {
            // Source: drake/examples/acrobot/spong_controller_params.h
            const char* doc = R"""(Setter that matches k_e().)""";
          } set_k_e;
          // Symbol: drake::examples::acrobot::SpongControllerParams::set_k_p
          struct /* set_k_p */ {
            // Source: drake/examples/acrobot/spong_controller_params.h
            const char* doc = R"""(Setter that matches k_p().)""";
          } set_k_p;
          // Symbol: drake::examples::acrobot::SpongControllerParams::with_balancing_threshold
          struct /* with_balancing_threshold */ {
            // Source: drake/examples/acrobot/spong_controller_params.h
            const char* doc =
R"""(Fluent setter that matches balancing_threshold(). Returns a copy of
``this`` with balancing_threshold set to a new value.)""";
          } with_balancing_threshold;
          // Symbol: drake::examples::acrobot::SpongControllerParams::with_k_d
          struct /* with_k_d */ {
            // Source: drake/examples/acrobot/spong_controller_params.h
            const char* doc =
R"""(Fluent setter that matches k_d(). Returns a copy of ``this`` with k_d
set to a new value.)""";
          } with_k_d;
          // Symbol: drake::examples::acrobot::SpongControllerParams::with_k_e
          struct /* with_k_e */ {
            // Source: drake/examples/acrobot/spong_controller_params.h
            const char* doc =
R"""(Fluent setter that matches k_e(). Returns a copy of ``this`` with k_e
set to a new value.)""";
          } with_k_e;
          // Symbol: drake::examples::acrobot::SpongControllerParams::with_k_p
          struct /* with_k_p */ {
            // Source: drake/examples/acrobot/spong_controller_params.h
            const char* doc =
R"""(Fluent setter that matches k_p(). Returns a copy of ``this`` with k_p
set to a new value.)""";
          } with_k_p;
        } SpongControllerParams;
        // Symbol: drake::examples::acrobot::SpongControllerParamsIndices
        struct /* SpongControllerParamsIndices */ {
          // Source: drake/examples/acrobot/spong_controller_params.h
          const char* doc =
R"""(Describes the row indices of a SpongControllerParams.)""";
          // Symbol: drake::examples::acrobot::SpongControllerParamsIndices::GetCoordinateNames
          struct /* GetCoordinateNames */ {
            // Source: drake/examples/acrobot/spong_controller_params.h
            const char* doc =
R"""(Returns a vector containing the names of each coordinate within this
class. The indices within the returned vector matches that of this
class. In other words,
``SpongControllerParamsIndices∷GetCoordinateNames()[i]`` is the name
for ``BasicVector∷GetAtIndex(i)``.)""";
          } GetCoordinateNames;
        } SpongControllerParamsIndices;
      } acrobot;
    } examples;
  } drake;
} pydrake_doc_examples_acrobot;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
