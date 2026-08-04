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

// #include "drake/manipulation/schunk_wsg/build_schunk_wsg_control.h"
// #include "drake/manipulation/schunk_wsg/schunk_wsg_constants.h"
// #include "drake/manipulation/schunk_wsg/schunk_wsg_controller.h"
// #include "drake/manipulation/schunk_wsg/schunk_wsg_driver.h"
// #include "drake/manipulation/schunk_wsg/schunk_wsg_driver_functions.h"
// #include "drake/manipulation/schunk_wsg/schunk_wsg_lcm.h"
// #include "drake/manipulation/schunk_wsg/schunk_wsg_plain_controller.h"
// #include "drake/manipulation/schunk_wsg/schunk_wsg_position_controller.h"
// #include "drake/manipulation/schunk_wsg/schunk_wsg_trajectory_generator.h"
// #include "drake/manipulation/schunk_wsg/schunk_wsg_trajectory_generator_state_vector.h"

// Symbol: pydrake_doc_manipulation_schunk_wsg
constexpr struct /* pydrake_doc_manipulation_schunk_wsg */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::manipulation
    struct /* manipulation */ {
      // Symbol: drake::manipulation::schunk_wsg
      struct /* schunk_wsg */ {
        // Symbol: drake::manipulation::schunk_wsg::ApplyDriverConfig
        struct /* ApplyDriverConfig */ {
          // Source: drake/manipulation/schunk_wsg/schunk_wsg_driver_functions.h
          const char* doc =
R"""(Wires up Drake systems between an LCM interface and the actuation
input ports of a MultibodyPlant. This simulates the role that driver
software and robot firmware would take in real life.)""";
        } ApplyDriverConfig;
        // Symbol: drake::manipulation::schunk_wsg::BuildSchunkWsgControl
        struct /* BuildSchunkWsgControl */ {
          // Source: drake/manipulation/schunk_wsg/build_schunk_wsg_control.h
          const char* doc =
R"""(Builds (into ``builder)`` the WSG control and sensing systems for the
wsg model in ``plant`` indicated by ``wsg_instance``; hooks those
systems up to ``lcm`` and the relevant MultibodyPlant ports in the
diagram. ``pid_gains`` can be used to override the default sim PID
gains.)""";
        } BuildSchunkWsgControl;
        // Symbol: drake::manipulation::schunk_wsg::ControlMode
        struct /* ControlMode */ {
          // Source: drake/manipulation/schunk_wsg/schunk_wsg_plain_controller.h
          const char* doc = R"""()""";
          // Symbol: drake::manipulation::schunk_wsg::ControlMode::kForce
          struct /* kForce */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_plain_controller.h
            const char* doc = R"""()""";
          } kForce;
          // Symbol: drake::manipulation::schunk_wsg::ControlMode::kPosition
          struct /* kPosition */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_plain_controller.h
            const char* doc = R"""()""";
          } kPosition;
        } ControlMode;
        // Symbol: drake::manipulation::schunk_wsg::GetSchunkWsgOpenPosition
        struct /* GetSchunkWsgOpenPosition */ {
          // Source: drake/manipulation/schunk_wsg/schunk_wsg_constants.h
          const char* doc =
R"""(Returns the position vector corresponding to the open position of the
gripper.)""";
        } GetSchunkWsgOpenPosition;
        // Symbol: drake::manipulation::schunk_wsg::MakeMultibodyForceToWsgForceSystem
        struct /* MakeMultibodyForceToWsgForceSystem */ {
          // Source: drake/manipulation/schunk_wsg/schunk_wsg_constants.h
          const char* doc =
R"""(Helper method to create a MultibodyForceToWsgForceSystem.)""";
        } MakeMultibodyForceToWsgForceSystem;
        // Symbol: drake::manipulation::schunk_wsg::MakeMultibodyStateToWsgStateSystem
        struct /* MakeMultibodyStateToWsgStateSystem */ {
          // Source: drake/manipulation/schunk_wsg/schunk_wsg_constants.h
          const char* doc =
R"""(Extract the distance between the fingers (and its time derivative) out
of the plant model which pretends the two fingers are independent.)""";
        } MakeMultibodyStateToWsgStateSystem;
        // Symbol: drake::manipulation::schunk_wsg::MultibodyForceToWsgForceSystem
        struct /* MultibodyForceToWsgForceSystem */ {
          // Source: drake/manipulation/schunk_wsg/schunk_wsg_constants.h
          const char* doc =
R"""(Extract the gripper measured force from the generalized forces on the
two fingers.

.. pydrake_system::

    name: MultibodyForceToWsgForceSystem
    input_ports:
    - u0
    output_ports:
    - y0)""";
          // Symbol: drake::manipulation::schunk_wsg::MultibodyForceToWsgForceSystem::DoCalcVectorOutput
          struct /* DoCalcVectorOutput */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_constants.h
            const char* doc = R"""()""";
          } DoCalcVectorOutput;
          // Symbol: drake::manipulation::schunk_wsg::MultibodyForceToWsgForceSystem::MultibodyForceToWsgForceSystem<T>
          struct /* ctor */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_constants.h
            const char* doc = R"""()""";
          } ctor;
        } MultibodyForceToWsgForceSystem;
        // Symbol: drake::manipulation::schunk_wsg::SchunkWsgCommandReceiver
        struct /* SchunkWsgCommandReceiver */ {
          // Source: drake/manipulation/schunk_wsg/schunk_wsg_lcm.h
          const char* doc =
R"""(Handles the command for the Schunk WSG gripper from a
LcmSubscriberSystem.

It has one input port: "command_message" for lcmt_schunk_wsg_command
abstract values.

It has two output ports: one for the commanded finger position
represented as the desired distance between the fingers in meters, and
one for the commanded force limit. The commanded position and force
limit are scalars (BasicVector<double> of size 1).

.. pydrake_system::

    name: SchunkWsgCommandReceiver
    input_ports:
    - command_message
    output_ports:
    - position
    - force_limit)""";
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgCommandReceiver::SchunkWsgCommandReceiver
          struct /* ctor */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_lcm.h
            const char* doc =
R"""(Parameter ``initial_position``:
    the commanded position to output if no LCM message has been
    received yet.

Parameter ``initial_force``:
    the commanded force limit to output if no LCM message has been
    received yet.)""";
          } ctor;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgCommandReceiver::get_force_limit_output_port
          struct /* get_force_limit_output_port */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_lcm.h
            const char* doc = R"""()""";
          } get_force_limit_output_port;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgCommandReceiver::get_position_output_port
          struct /* get_position_output_port */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_lcm.h
            const char* doc = R"""()""";
          } get_position_output_port;
        } SchunkWsgCommandReceiver;
        // Symbol: drake::manipulation::schunk_wsg::SchunkWsgCommandSender
        struct /* SchunkWsgCommandSender */ {
          // Source: drake/manipulation/schunk_wsg/schunk_wsg_lcm.h
          const char* doc =
R"""(Send lcmt_schunk_wsg_command messages for a Schunk WSG gripper. Has
two input ports: one for the commanded finger position represented as
the desired signed distance between the fingers in meters, and one for
the commanded force limit. The commanded position and force limit are
scalars (BasicVector<double> of size 1).

.. pydrake_system::

    name: SchunkWsgCommandSender
    input_ports:
    - position
    - force_limit
    output_ports:
    - lcmt_schunk_wsg_command

The ``force_limit`` input port can be left unconnected; in this case,
the ``default_force_limit`` value given at construction time will be
used.)""";
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgCommandSender::SchunkWsgCommandSender
          struct /* ctor */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_lcm.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgCommandSender::get_command_output_port
          struct /* get_command_output_port */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_lcm.h
            const char* doc = R"""()""";
          } get_command_output_port;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgCommandSender::get_force_limit_input_port
          struct /* get_force_limit_input_port */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_lcm.h
            const char* doc = R"""()""";
          } get_force_limit_input_port;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgCommandSender::get_position_input_port
          struct /* get_position_input_port */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_lcm.h
            const char* doc = R"""()""";
          } get_position_input_port;
        } SchunkWsgCommandSender;
        // Symbol: drake::manipulation::schunk_wsg::SchunkWsgController
        struct /* SchunkWsgController */ {
          // Source: drake/manipulation/schunk_wsg/schunk_wsg_controller.h
          const char* doc =
R"""(This class implements a controller for a Schunk WSG gripper. It has
two input ports: lcmt_schunk_wsg_command message and the current
state, and an output port which emits the target force for the
actuated finger. Note, only one of the command input ports should be
connected, However, if both are connected, the message input will be
ignored. The internal implementation consists of a PID controller
(which controls the target position from the command message) combined
with a saturation block (which applies the force control from the
command message).

.. pydrake_system::

    name: SchunkWsgController
    input_ports:
    - state
    - command_message
    output_ports:
    - force)""";
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgController::SchunkWsgController
          struct /* ctor */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_controller.h
            const char* doc = R"""()""";
          } ctor;
        } SchunkWsgController;
        // Symbol: drake::manipulation::schunk_wsg::SchunkWsgDriver
        struct /* SchunkWsgDriver */ {
          // Source: drake/manipulation/schunk_wsg/schunk_wsg_driver.h
          const char* doc =
R"""(This config struct specifies how to wire up Drake systems between an
LCM interface and the actuation input ports of a MultibodyPlant. This
simulates the role that driver software and control cabinets would
take in real life.

It creates an LCM publisher on the ``SCHUNK_WSG_STATUS`` channel and
an LCM subscriber on the ``SCHUNK_WSG_COMMAND`` channel.)""";
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgDriver::Serialize
          struct /* Serialize */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_driver.h
            const char* doc = R"""()""";
          } Serialize;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgDriver::lcm_bus
          struct /* lcm_bus */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_driver.h
            const char* doc = R"""()""";
          } lcm_bus;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgDriver::pid_gains
          struct /* pid_gains */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_driver.h
            const char* doc =
R"""(Gains to apply to the the WSG fingers. The p term corresponds
approximately to the elastic modulus of the belt, the d term to the
viscous friction of the geartrain. The i term is nonphysical.)""";
          } pid_gains;
          auto Serialize__fields() const {
            return std::array{
              std::make_pair("lcm_bus", lcm_bus.doc),
              std::make_pair("pid_gains", pid_gains.doc),
            };
          }
        } SchunkWsgDriver;
        // Symbol: drake::manipulation::schunk_wsg::SchunkWsgPdController
        struct /* SchunkWsgPdController */ {
          // Source: drake/manipulation/schunk_wsg/schunk_wsg_position_controller.h
          const char* doc =
R"""(This class implements a controller for a Schunk WSG gripper in
position control mode. It assumes that the gripper is modeled in the
plant as two independent prismatic joints for the fingers.

Note: This is intended as a simpler single-system implementation that
can replace the SchunkWsgController when using position control mode.
We anticipate a single-system SchunkWsgForceController implementation
to (soon) replace the other mode, and then will deprecate
SchunkWsgController.

Call the positions of the prismatic joints q₀ and q₁. q₀ = q₁ = 0 is
the configuration where the fingers are touching in the center. When
the gripper is open, q₀ < 0 and q₁ > 0.

The physical gripper mechanically imposes that -q₀ = q₁, and
implements a controller to track -q₀ = q₁ = q_d/2 (q_d is the desired
position, which is the signed distance between the two fingers). We
model that here with two PD controllers -- one that implements the
physical constraint (keeping the fingers centered): f₀+f₁ =
-kp_constraint*(q₀+q₁) - kd_constraint*(v₀+v₁), and another to
implement the controller (opening/closing the fingers): -f₀+f₁ =
sat(kp_command*(q_d + q₀ - q₁) + kd_command*(v_d + v₀ - v₁)), where
sat() saturates the command to be in the range [-force_limit,
force_limit]. The expectation is that kp_constraint ≫ kp_command.

.. pydrake_system::

    name: SchunkWSGPdController
    input_ports:
    - desired_state
    - force_limit
    - state
    output_ports:
    - generalized_force
    - grip_force

The ``force_limit`` input port can be left unconnected; in this case,
the ``default_force_limit`` value given at construction time will be
used.

The desired_state is a BasicVector<double> of size 2 (position and
velocity of the distance between the fingers). The force_limit is a
scalar (BasicVector<double> of size 1) and is optional; if the input
port is not connected then the constant value passed into the
constructor is used. The state is a BasicVector<double> of size 4
(positions and velocities of the two fingers). The output
generalized_force is a BasicVector<double> of size 2 (generalized
force inputs to the two fingers). The output grip_force is a scalar
surrogate for the force measurement from the driver, f = abs(f₀-f₁)
which, like the gripper itself, only reports a positive force.)""";
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgPdController::SchunkWsgPdController
          struct /* ctor */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_position_controller.h
            const char* doc =
R"""(Initialize the controller. The gain parameters are set based limited
tuning in simulation with a kuka picking up small objects.)""";
          } ctor;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgPdController::get_desired_state_input_port
          struct /* get_desired_state_input_port */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_position_controller.h
            const char* doc = R"""()""";
          } get_desired_state_input_port;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgPdController::get_force_limit_input_port
          struct /* get_force_limit_input_port */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_position_controller.h
            const char* doc = R"""()""";
          } get_force_limit_input_port;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgPdController::get_generalized_force_output_port
          struct /* get_generalized_force_output_port */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_position_controller.h
            const char* doc = R"""()""";
          } get_generalized_force_output_port;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgPdController::get_grip_force_output_port
          struct /* get_grip_force_output_port */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_position_controller.h
            const char* doc = R"""()""";
          } get_grip_force_output_port;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgPdController::get_state_input_port
          struct /* get_state_input_port */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_position_controller.h
            const char* doc = R"""()""";
          } get_state_input_port;
        } SchunkWsgPdController;
        // Symbol: drake::manipulation::schunk_wsg::SchunkWsgPlainController
        struct /* SchunkWsgPlainController */ {
          // Source: drake/manipulation/schunk_wsg/schunk_wsg_plain_controller.h
          const char* doc =
R"""(This class implements a controller for a Schunk WSG gripper as a
``systems∷Diagram``. The composition of this diagram is determined by
the control mode specified for the controller, which can be either
ControlMode∷kPosition or ControlMode∷kForce. In both cases, the
overall layout of the diagram is:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    ┌─────────────┐
    joint       │Joint State  │   ┌──────────┐
    state ─────▶│To Control   ├──▶│          │
                │State        │   │          │
                └─────────────┘   │PID       │   ╔════════════╗
                ╔═════════════╗   │Controller├──▶║            ╟─────┐
    desired     ║Generate     ║   │          │   ║            ║     │
    grip ──────▶║Desired      ╟──▶│          │   ║Handle      ║     │
    state       ║Control State║   └──────────┘   ║Feed-Forward║     │
                ╚═════════════╝                  ║Force       ║     │
    feed                                         ║            ║     │
    forward ────────────────────────────────────▶║            ╟──┐  │
    force                                        ╚════════════╝  │  │
                                                                 │  │
                              ┌──────────────────────────────────┘  │
                              │              ┌──────────────────────┘
                              │              │
                              │              │   ┌───────────┐
                              │              │   │Mean Finger│   ┌───┐
                              │              └──▶│Force To   ├──▶│   │
                              │                  │Joint Force│   │   │
                              │                  └───────────┘   │   │
                              │                                  │ + ├──▶ control
                              │   ┌──────────┐   ┌───────────┐   │   │
                    ┌─────────│──▶│          │   │Grip Force │   │   │
                    │   ┌──┐  └──▶│Saturation├──▶│To Joint   ├──▶│   │
    max force / 2 ──┴──▶│-1├─────▶│          │   │Force      │   └───┘
                        └──┘      └──────────┘   └───────────┘

.. raw:: html

    </details>

The blocks with double outlines (══) differ between the two control
modes:

- Generate Desired Control State
  - ControlMode∷kPosition



.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    ┌───────────┐
           │Desired    │
           │Mean Finger├──▶█
           │State      │   █   ┌─────────────┐
           └───────────┘   █   │Muxed States │    desired
                           █──▶│To Control   ├──▶ control
                           █   │State        │    state
            desired        █   └─────────────┘
            grip   ───────▶█
            state

.. raw:: html

    </details>

- ControlMode∷kForce


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    ┌───────────┐
           │Desired    │                          desired
           │Mean Finger├────────────────────────▶ control
           │State      │                          state
           └───────────┘
    
            desired        ┌────────┐
            grip   ───────▶│IGNORED │
            state          └────────┘

.. raw:: html

    </details>

- Handle Feed-Forward Force - ControlMode∷kPosition


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    █────▶ mean finger force
            pid                         █
            controller ────────────────▶█
            output                      █
                                        █────▶ grip force
            feed           ┌────────┐
            forward ──────▶│IGNORED │
            force          └────────┘

.. raw:: html

    </details>

- ControlMode∷kForce


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    pid
            controller ──────────────────────▶ mean finger force
            output
    
            feed
            forward ─────────────────────────▶ grip force
            force

.. raw:: html

    </details>

The remaining blocks differ only in their numerical parameters.

Note that the "feed forward force" input is ignored for
ControlMode∷kPosition and the "desired grip state" input is ignored
for ControlMode∷kPosition.

.. pydrake_system::

    name: SchunkWsgPlainController
    input_ports:
    - joint_state
    - max_force
    - <span style="color:gray">desired_grip_state</span>
    - <span style="color:gray">feed_forward_force</span>
    output_ports:
    - control

The ``desired_grip_state`` port is present only when the control mode
is ``kPosition``; the ``feed_forward_force`` port is present only when
the control mode is ``kForce``.)""";
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgPlainController::SchunkWsgPlainController
          struct /* ctor */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_plain_controller.h
            const char* doc =
R"""(Specify control gains and mode. Mode defaults to position control.)""";
          } ctor;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgPlainController::get_input_port_desired_state
          struct /* get_input_port_desired_state */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_plain_controller.h
            const char* doc =
R"""(Returns the desired grip state input port.

Precondition:
    ``this`` was constructed with ``control_mode`` set to
    ``ControlMode∷kPosition``.)""";
          } get_input_port_desired_state;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgPlainController::get_input_port_estimated_state
          struct /* get_input_port_estimated_state */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_plain_controller.h
            const char* doc = R"""()""";
          } get_input_port_estimated_state;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgPlainController::get_input_port_feed_forward_force
          struct /* get_input_port_feed_forward_force */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_plain_controller.h
            const char* doc =
R"""(Returns the feed-forward force input port.

Precondition:
    ``this`` was constructed with ``control_mode`` set to
    ``ControlMode∷kForce``.)""";
          } get_input_port_feed_forward_force;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgPlainController::get_input_port_max_force
          struct /* get_input_port_max_force */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_plain_controller.h
            const char* doc = R"""()""";
          } get_input_port_max_force;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgPlainController::get_output_port_control
          struct /* get_output_port_control */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_plain_controller.h
            const char* doc = R"""()""";
          } get_output_port_control;
        } SchunkWsgPlainController;
        // Symbol: drake::manipulation::schunk_wsg::SchunkWsgPositionController
        struct /* SchunkWsgPositionController */ {
          // Source: drake/manipulation/schunk_wsg/schunk_wsg_position_controller.h
          const char* doc =
R"""(This class implements a controller for a Schunk WSG gripper in
position control mode adding a discrete-derivative to estimate the
desired velocity from the desired position commands. It is a thin
wrapper around SchunkWsgPdController.

.. pydrake_system::

    name: SchunkWSGPositionController
    input_ports:
    - desired_position
    - force_limit
    - state
    output_ports:
    - generalized_force
    - grip_force

The ``force_limit`` input port can be left unconnected; in this case,
the ``default_force_limit`` value given at construction time will be
used.

See also:
    SchunkWsgPdController)""";
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgPositionController::SchunkWsgPositionController
          struct /* ctor */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_position_controller.h
            const char* doc =
R"""(Initialize the controller. The default ``time_step`` is set to match
the update rate of the wsg firmware. The gain parameters are set based
limited tuning in simulation with a kuka picking up small objects.

See also:
    SchunkWsgPdController∷SchunkWsgPdController())""";
          } ctor;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgPositionController::get_desired_position_input_port
          struct /* get_desired_position_input_port */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_position_controller.h
            const char* doc = R"""()""";
          } get_desired_position_input_port;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgPositionController::get_force_limit_input_port
          struct /* get_force_limit_input_port */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_position_controller.h
            const char* doc = R"""()""";
          } get_force_limit_input_port;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgPositionController::get_generalized_force_output_port
          struct /* get_generalized_force_output_port */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_position_controller.h
            const char* doc = R"""()""";
          } get_generalized_force_output_port;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgPositionController::get_grip_force_output_port
          struct /* get_grip_force_output_port */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_position_controller.h
            const char* doc = R"""()""";
          } get_grip_force_output_port;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgPositionController::get_state_input_port
          struct /* get_state_input_port */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_position_controller.h
            const char* doc = R"""()""";
          } get_state_input_port;
        } SchunkWsgPositionController;
        // Symbol: drake::manipulation::schunk_wsg::SchunkWsgStatusReceiver
        struct /* SchunkWsgStatusReceiver */ {
          // Source: drake/manipulation/schunk_wsg/schunk_wsg_lcm.h
          const char* doc =
R"""(Handles lcmt_schunk_wsg_status messages from a LcmSubscriberSystem.
Has two output ports: one for the measured state of the gripper,
represented as the signed distance between the fingers in meters and
its corresponding velocity, and one for the measured force.

.. pydrake_system::

    name: SchunkWsgStatusReceiver
    input_ports:
    - lcmt_schunk_wsg_status
    output_ports:
    - state
    - force)""";
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgStatusReceiver::SchunkWsgStatusReceiver
          struct /* ctor */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_lcm.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgStatusReceiver::get_force_output_port
          struct /* get_force_output_port */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_lcm.h
            const char* doc = R"""()""";
          } get_force_output_port;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgStatusReceiver::get_state_output_port
          struct /* get_state_output_port */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_lcm.h
            const char* doc = R"""()""";
          } get_state_output_port;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgStatusReceiver::get_status_input_port
          struct /* get_status_input_port */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_lcm.h
            const char* doc = R"""()""";
          } get_status_input_port;
        } SchunkWsgStatusReceiver;
        // Symbol: drake::manipulation::schunk_wsg::SchunkWsgStatusSender
        struct /* SchunkWsgStatusSender */ {
          // Source: drake/manipulation/schunk_wsg/schunk_wsg_lcm.h
          const char* doc =
R"""(Sends lcmt_schunk_wsg_status messages for a Schunk WSG. This system
has one input port for the current state of the WSG, and one optional
input port for the measured gripping force.

.. pydrake_system::

    name: SchunkStatusSender
    input_ports:
    - state
    - force
    output_ports:
    - lcmt_schunk_wsg_status

The state input is a BasicVector<double> of size 2 -- with one
position and one velocity -- representing the distance between the
fingers (positive implies non-penetration).)""";
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgStatusSender::SchunkWsgStatusSender
          struct /* ctor */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_lcm.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgStatusSender::get_force_input_port
          struct /* get_force_input_port */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_lcm.h
            const char* doc = R"""()""";
          } get_force_input_port;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgStatusSender::get_state_input_port
          struct /* get_state_input_port */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_lcm.h
            const char* doc = R"""()""";
          } get_state_input_port;
        } SchunkWsgStatusSender;
        // Symbol: drake::manipulation::schunk_wsg::SchunkWsgTrajectoryGenerator
        struct /* SchunkWsgTrajectoryGenerator */ {
          // Source: drake/manipulation/schunk_wsg/schunk_wsg_trajectory_generator.h
          const char* doc =
R"""(This system defines input ports for the desired finger position
represented as the desired distance between the fingers in meters and
the desired force limit in newtons, and emits target position/velocity
for the actuated finger to reach the commanded target, expressed as
the negative of the distance between the two fingers in meters. The
force portion of the command message is passed through this system,
but does not affect the generated trajectory. The desired_position and
force_limit are scalars (BasicVector<double> of size 1).

.. pydrake_system::

    name: SchunkWsgTrajectoryGenerator
    input_ports:
    - desired_position
    - force_limit
    - <span style="color:gray">u2</span>
    output_ports:
    - y0
    - <span style="color:gray">y1</span>

Port ``u2`` accepts state. Port ``y0`` emits target position/velocity.
Port ``y1`` emits max force.

Note that the ``force_limit`` input port and ``y1`` output port (max
force) can be opted-out with a constructor argument.)""";
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgTrajectoryGenerator::SchunkWsgTrajectoryGenerator
          struct /* ctor */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_trajectory_generator.h
            const char* doc =
R"""(Parameter ``input_size``:
    The size of the state input port to create (one reason this may
    vary is passing in the entire state of a rigid body tree vs.
    having already demultiplexed the actuated finger).

Parameter ``position_index``:
    The index in the state input vector which contains the position of
    the actuated finger.

Parameter ``use_force_limit``:
    when false, the ``force_limit`` and ``y1`` ports will be omitted.)""";
          } ctor;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgTrajectoryGenerator::get_desired_position_input_port
          struct /* get_desired_position_input_port */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_trajectory_generator.h
            const char* doc = R"""()""";
          } get_desired_position_input_port;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgTrajectoryGenerator::get_force_limit_input_port
          struct /* get_force_limit_input_port */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_trajectory_generator.h
            const char* doc =
R"""(Precondition:
    The constructor argument use_force_limit was set to true.)""";
          } get_force_limit_input_port;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgTrajectoryGenerator::get_max_force_output_port
          struct /* get_max_force_output_port */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_trajectory_generator.h
            const char* doc =
R"""(Precondition:
    The constructor argument use_force_limit was set to true.)""";
          } get_max_force_output_port;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgTrajectoryGenerator::get_state_input_port
          struct /* get_state_input_port */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_trajectory_generator.h
            const char* doc = R"""()""";
          } get_state_input_port;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgTrajectoryGenerator::get_target_output_port
          struct /* get_target_output_port */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_trajectory_generator.h
            const char* doc = R"""()""";
          } get_target_output_port;
        } SchunkWsgTrajectoryGenerator;
        // Symbol: drake::manipulation::schunk_wsg::SchunkWsgTrajectoryGeneratorStateVector
        struct /* SchunkWsgTrajectoryGeneratorStateVector */ {
          // Source: drake/manipulation/schunk_wsg/schunk_wsg_trajectory_generator_state_vector.h
          const char* doc =
R"""(Specializes BasicVector with specific getters and setters.)""";
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgTrajectoryGeneratorStateVector::DoClone
          struct /* DoClone */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_trajectory_generator_state_vector.h
            const char* doc = R"""()""";
          } DoClone;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgTrajectoryGeneratorStateVector::GetCoordinateNames
          struct /* GetCoordinateNames */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_trajectory_generator_state_vector.h
            const char* doc =
R"""(See
SchunkWsgTrajectoryGeneratorStateVectorIndices∷GetCoordinateNames().)""";
          } GetCoordinateNames;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgTrajectoryGeneratorStateVector::IsValid
          struct /* IsValid */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_trajectory_generator_state_vector.h
            const char* doc =
R"""(Returns whether the current values of this vector are well-formed.)""";
          } IsValid;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgTrajectoryGeneratorStateVector::K
          struct /* K */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_trajectory_generator_state_vector.h
            const char* doc =
R"""(An abbreviation for our row index constants.)""";
          } K;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgTrajectoryGeneratorStateVector::SchunkWsgTrajectoryGeneratorStateVector<T>
          struct /* ctor */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_trajectory_generator_state_vector.h
            const char* doc =
R"""(Default constructor. Sets all rows to their default value:

* ``last_target_position`` defaults to 0.0 with unknown units.

* ``trajectory_start_time`` defaults to 0.0 with unknown units.

* ``last_position`` defaults to 0.0 with unknown units.

* ``max_force`` defaults to 0.0 with unknown units.)""";
          } ctor;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgTrajectoryGeneratorStateVector::Serialize
          struct /* Serialize */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_trajectory_generator_state_vector.h
            const char* doc =
R"""(Visit each field of this named vector, passing them (in order) to the
given Archive. The archive can read and/or write to the vector values.
One common use of Serialize is the //common/yaml tools.)""";
          } Serialize;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgTrajectoryGeneratorStateVector::SetToNamedVariables
          struct /* SetToNamedVariables */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_trajectory_generator_state_vector.h
            const char* doc =
R"""(Create a symbolic∷Variable for each element with the known variable
name. This is only available for T == symbolic∷Expression.)""";
          } SetToNamedVariables;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgTrajectoryGeneratorStateVector::last_position
          struct /* last_position */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_trajectory_generator_state_vector.h
            const char* doc = R"""(last_position)""";
          } last_position;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgTrajectoryGeneratorStateVector::last_target_position
          struct /* last_target_position */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_trajectory_generator_state_vector.h
            const char* doc = R"""(last_target_position)""";
          } last_target_position;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgTrajectoryGeneratorStateVector::max_force
          struct /* max_force */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_trajectory_generator_state_vector.h
            const char* doc = R"""(max_force)""";
          } max_force;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgTrajectoryGeneratorStateVector::set_last_position
          struct /* set_last_position */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_trajectory_generator_state_vector.h
            const char* doc = R"""(Setter that matches last_position().)""";
          } set_last_position;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgTrajectoryGeneratorStateVector::set_last_target_position
          struct /* set_last_target_position */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_trajectory_generator_state_vector.h
            const char* doc =
R"""(Setter that matches last_target_position().)""";
          } set_last_target_position;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgTrajectoryGeneratorStateVector::set_max_force
          struct /* set_max_force */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_trajectory_generator_state_vector.h
            const char* doc = R"""(Setter that matches max_force().)""";
          } set_max_force;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgTrajectoryGeneratorStateVector::set_trajectory_start_time
          struct /* set_trajectory_start_time */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_trajectory_generator_state_vector.h
            const char* doc =
R"""(Setter that matches trajectory_start_time().)""";
          } set_trajectory_start_time;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgTrajectoryGeneratorStateVector::trajectory_start_time
          struct /* trajectory_start_time */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_trajectory_generator_state_vector.h
            const char* doc = R"""(trajectory_start_time)""";
          } trajectory_start_time;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgTrajectoryGeneratorStateVector::with_last_position
          struct /* with_last_position */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_trajectory_generator_state_vector.h
            const char* doc =
R"""(Fluent setter that matches last_position(). Returns a copy of ``this``
with last_position set to a new value.)""";
          } with_last_position;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgTrajectoryGeneratorStateVector::with_last_target_position
          struct /* with_last_target_position */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_trajectory_generator_state_vector.h
            const char* doc =
R"""(Fluent setter that matches last_target_position(). Returns a copy of
``this`` with last_target_position set to a new value.)""";
          } with_last_target_position;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgTrajectoryGeneratorStateVector::with_max_force
          struct /* with_max_force */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_trajectory_generator_state_vector.h
            const char* doc =
R"""(Fluent setter that matches max_force(). Returns a copy of ``this``
with max_force set to a new value.)""";
          } with_max_force;
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgTrajectoryGeneratorStateVector::with_trajectory_start_time
          struct /* with_trajectory_start_time */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_trajectory_generator_state_vector.h
            const char* doc =
R"""(Fluent setter that matches trajectory_start_time(). Returns a copy of
``this`` with trajectory_start_time set to a new value.)""";
          } with_trajectory_start_time;
        } SchunkWsgTrajectoryGeneratorStateVector;
        // Symbol: drake::manipulation::schunk_wsg::SchunkWsgTrajectoryGeneratorStateVectorIndices
        struct /* SchunkWsgTrajectoryGeneratorStateVectorIndices */ {
          // Source: drake/manipulation/schunk_wsg/schunk_wsg_trajectory_generator_state_vector.h
          const char* doc =
R"""(Describes the row indices of a
SchunkWsgTrajectoryGeneratorStateVector.)""";
          // Symbol: drake::manipulation::schunk_wsg::SchunkWsgTrajectoryGeneratorStateVectorIndices::GetCoordinateNames
          struct /* GetCoordinateNames */ {
            // Source: drake/manipulation/schunk_wsg/schunk_wsg_trajectory_generator_state_vector.h
            const char* doc =
R"""(Returns a vector containing the names of each coordinate within this
class. The indices within the returned vector matches that of this
class. In other words,
``SchunkWsgTrajectoryGeneratorStateVectorIndices∷GetCoordinateNames()[i]``
is the name for ``BasicVector∷GetAtIndex(i)``.)""";
          } GetCoordinateNames;
        } SchunkWsgTrajectoryGeneratorStateVectorIndices;
      } schunk_wsg;
    } manipulation;
  } drake;
} pydrake_doc_manipulation_schunk_wsg;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
