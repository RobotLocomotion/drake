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

// #include "drake/manipulation/kuka_iiwa/build_iiwa_control.h"
// #include "drake/manipulation/kuka_iiwa/iiwa_command_receiver.h"
// #include "drake/manipulation/kuka_iiwa/iiwa_command_sender.h"
// #include "drake/manipulation/kuka_iiwa/iiwa_constants.h"
// #include "drake/manipulation/kuka_iiwa/iiwa_driver.h"
// #include "drake/manipulation/kuka_iiwa/iiwa_driver_functions.h"
// #include "drake/manipulation/kuka_iiwa/iiwa_status_receiver.h"
// #include "drake/manipulation/kuka_iiwa/iiwa_status_sender.h"
// #include "drake/manipulation/kuka_iiwa/sim_iiwa_driver.h"

// Symbol: pydrake_doc_manipulation_kuka_iiwa
constexpr struct /* pydrake_doc_manipulation_kuka_iiwa */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::manipulation
    struct /* manipulation */ {
      // Symbol: drake::manipulation::kuka_iiwa
      struct /* kuka_iiwa */ {
        // Symbol: drake::manipulation::kuka_iiwa::ApplyDriverConfig
        struct /* ApplyDriverConfig */ {
          // Source: drake/manipulation/kuka_iiwa/iiwa_driver_functions.h
          const char* doc =
R"""(Wires up Drake systems between an LCM interface and the actuation
input ports of a MultibodyPlant. This simulates the role that driver
software and control cabinets would take in real life.

Precondition:
    model_instance_name is in models_from_directives.

Precondition:
    driver_config.hand_model_name is in models_from_directives.)""";
        } ApplyDriverConfig;
        // Symbol: drake::manipulation::kuka_iiwa::BuildIiwaControl
        struct /* BuildIiwaControl */ {
          // Source: drake/manipulation/kuka_iiwa/build_iiwa_control.h
          const char* doc =
R"""(Given a ``plant`` (and associated ``iiwa_instance``) and a`builder`,
installs in that builder the systems necessary to control and monitor
an Iiwa described by ``controller_plant`` in that plant.

The installed plant will communicate over the LCM interface ``lcm``.

The installed plant will connect itself to the actuation input port
and state output ports in ``plant`` corresponding to the Iiwa model.

The values in ``driver_config`` further specify Diagram behavior; see
IiwaDriver for more details.

Note: The Diagram will maintain an internal reference to
``controller_plant``, so you must ensure that ``controller_plant`` has
a longer lifetime than the Diagram.

The torque values in the published LCM status messages will follow the
IIWA-specific LCM conventions outlined in manipulation/README.)""";
        } BuildIiwaControl;
        // Symbol: drake::manipulation::kuka_iiwa::BuildSimplifiedIiwaControl
        struct /* BuildSimplifiedIiwaControl */ {
          // Source: drake/manipulation/kuka_iiwa/build_iiwa_control.h
          const char* doc =
R"""(A simplified Iiwa controller builder to construct an
InverseDynamicsController without adding LCM I/O systems.

See also:
    BuildIiwaControl())""";
        } BuildSimplifiedIiwaControl;
        // Symbol: drake::manipulation::kuka_iiwa::FormatIiwaControlMode
        struct /* FormatIiwaControlMode */ {
          // Source: drake/manipulation/kuka_iiwa/iiwa_constants.h
          const char* doc =
R"""(Formats control mode to a string using the mappping shown above.)""";
        } FormatIiwaControlMode;
        // Symbol: drake::manipulation::kuka_iiwa::IiwaCommandReceiver
        struct /* IiwaCommandReceiver */ {
          // Source: drake/manipulation/kuka_iiwa/iiwa_command_receiver.h
          const char* doc =
R"""(Handles lcmt_iiwa_command message from a LcmSubscriberSystem.

Note that this system does not actually subscribe to an LCM channel.
To receive the message, the input of this system should be connected
to a LcmSubscriberSystem∷Make<drake∷lcmt_iiwa_command>().

It has one required input port, "lcmt_iiwa_command".

It has three output ports: one for the commanded position for each
joint, one for commanded additional feedforward joint torque, and one
for the timestamp in the most recently received message.

.. pydrake_system::

    name: IiwaCommandReceiver
    input_ports:
    - lcmt_iiwa_command
    - position_measured (optional)
    output_ports:
    - position
    - torque
    - time

@par Output prior to receiving a valid lcmt_iiwa_command message: The
"position" output initially feeds through from the "position_measured"
input port -- or if not connected, outputs zero. When discrete update
events are enabled (e.g., during a simulation), the system latches the
"position_measured" input into state during the first event, and the
"position" output comes from the latched state, no longer fed through
from the "position" input. Alternatively, the LatchInitialPosition()
method is available to achieve the same effect without using events.
The "torque" output will be a vector of zeros, and the "time" output
will be a vector of a single zero.)""";
          // Symbol: drake::manipulation::kuka_iiwa::IiwaCommandReceiver::IiwaCommandReceiver
          struct /* ctor */ {
            // Source: drake/manipulation/kuka_iiwa/iiwa_command_receiver.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::manipulation::kuka_iiwa::IiwaCommandReceiver::LatchInitialPosition
          struct /* LatchInitialPosition */ {
            // Source: drake/manipulation/kuka_iiwa/iiwa_command_receiver.h
            const char* doc =
R"""((Advanced.) Copies the current "position_measured" input (or zero if
not connected) into Context state, and changes the behavior of the
"position" output to produce the latched state if no message has been
received yet. The latching already happens automatically during the
first discrete update event (e.g., when using a Simulator); this
method exists for use when not already using a Simulator or other
special cases.)""";
          } LatchInitialPosition;
          // Symbol: drake::manipulation::kuka_iiwa::IiwaCommandReceiver::get_commanded_position_output_port
          struct /* get_commanded_position_output_port */ {
            // Source: drake/manipulation/kuka_iiwa/iiwa_command_receiver.h
            const char* doc =
R"""(Raises:
    RuntimeError if control_mode does not include position control.)""";
          } get_commanded_position_output_port;
          // Symbol: drake::manipulation::kuka_iiwa::IiwaCommandReceiver::get_commanded_torque_output_port
          struct /* get_commanded_torque_output_port */ {
            // Source: drake/manipulation/kuka_iiwa/iiwa_command_receiver.h
            const char* doc =
R"""(Raises:
    RuntimeError if control_mode does not include torque control.)""";
          } get_commanded_torque_output_port;
          // Symbol: drake::manipulation::kuka_iiwa::IiwaCommandReceiver::get_message_input_port
          struct /* get_message_input_port */ {
            // Source: drake/manipulation/kuka_iiwa/iiwa_command_receiver.h
            const char* doc = R"""()""";
          } get_message_input_port;
          // Symbol: drake::manipulation::kuka_iiwa::IiwaCommandReceiver::get_position_measured_input_port
          struct /* get_position_measured_input_port */ {
            // Source: drake/manipulation/kuka_iiwa/iiwa_command_receiver.h
            const char* doc = R"""()""";
          } get_position_measured_input_port;
          // Symbol: drake::manipulation::kuka_iiwa::IiwaCommandReceiver::get_time_output_port
          struct /* get_time_output_port */ {
            // Source: drake/manipulation/kuka_iiwa/iiwa_command_receiver.h
            const char* doc = R"""()""";
          } get_time_output_port;
        } IiwaCommandReceiver;
        // Symbol: drake::manipulation::kuka_iiwa::IiwaCommandSender
        struct /* IiwaCommandSender */ {
          // Source: drake/manipulation/kuka_iiwa/iiwa_command_sender.h
          const char* doc =
R"""(Creates and outputs lcmt_iiwa_command messages.

Note that this system does not actually send the message on an LCM
channel. To send the message, the output of this system should be
connected to a
systems∷lcm∷LcmPublisherSystem∷Make<lcmt_iiwa_command>().

This system has three vector-valued input ports:

- one for the commanded position, which must be connected if position mode is
specified,
- one for commanded torque, which is optional if position and torque mode is
specified (for backwards compatibility), but must be connected if it is
torque only, and
- one for the time to use, in seconds, for the message timestamp, which is
optional.

If position and torque mode is specified, the torque input port can
remain unconnected; the message will contain torque values of size
zero. If position mode is not specified, the message will contain
position values of size zero.

If the time input port is not connected, the context time will be used
for message timestamp.

This system has one abstract-valued output port of type
lcmt_iiwa_command.

.. pydrake_system::

    name: IiwaCommandSender
    input_ports:
    - position (required if using position mode)
    - torque (optional if using position mode, required in torque mode)
    - time (optional)
    output_ports:
    - lcmt_iiwa_command

See also:
    ``lcmt_iiwa_command.lcm`` for additional documentation.)""";
          // Symbol: drake::manipulation::kuka_iiwa::IiwaCommandSender::IiwaCommandSender
          struct /* ctor */ {
            // Source: drake/manipulation/kuka_iiwa/iiwa_command_sender.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::manipulation::kuka_iiwa::IiwaCommandSender::get_position_input_port
          struct /* get_position_input_port */ {
            // Source: drake/manipulation/kuka_iiwa/iiwa_command_sender.h
            const char* doc = R"""()""";
          } get_position_input_port;
          // Symbol: drake::manipulation::kuka_iiwa::IiwaCommandSender::get_time_input_port
          struct /* get_time_input_port */ {
            // Source: drake/manipulation/kuka_iiwa/iiwa_command_sender.h
            const char* doc = R"""()""";
          } get_time_input_port;
          // Symbol: drake::manipulation::kuka_iiwa::IiwaCommandSender::get_torque_input_port
          struct /* get_torque_input_port */ {
            // Source: drake/manipulation/kuka_iiwa/iiwa_command_sender.h
            const char* doc = R"""()""";
          } get_torque_input_port;
        } IiwaCommandSender;
        // Symbol: drake::manipulation::kuka_iiwa::IiwaControlMode
        struct /* IiwaControlMode */ {
          // Source: drake/manipulation/kuka_iiwa/iiwa_constants.h
          const char* doc = R"""(Enumeration for control modes.)""";
          // Symbol: drake::manipulation::kuka_iiwa::IiwaControlMode::kPositionAndTorque
          struct /* kPositionAndTorque */ {
            // Source: drake/manipulation/kuka_iiwa/iiwa_constants.h
            const char* doc = R"""()""";
          } kPositionAndTorque;
          // Symbol: drake::manipulation::kuka_iiwa::IiwaControlMode::kPositionOnly
          struct /* kPositionOnly */ {
            // Source: drake/manipulation/kuka_iiwa/iiwa_constants.h
            const char* doc = R"""()""";
          } kPositionOnly;
          // Symbol: drake::manipulation::kuka_iiwa::IiwaControlMode::kTorqueOnly
          struct /* kTorqueOnly */ {
            // Source: drake/manipulation/kuka_iiwa/iiwa_constants.h
            const char* doc = R"""()""";
          } kTorqueOnly;
        } IiwaControlMode;
        // Symbol: drake::manipulation::kuka_iiwa::IiwaControlPorts
        struct /* IiwaControlPorts */ {
          // Source: drake/manipulation/kuka_iiwa/build_iiwa_control.h
          const char* doc =
R"""(The return type of BuildSimplifiedIiwaControl(). Depending on the
``control_mode``, some of the input ports might be null. The output
ports are never null.

These follow the general conventions for torque as outlined in
manipulation/README.)""";
          // Symbol: drake::manipulation::kuka_iiwa::IiwaControlPorts::commanded_positions
          struct /* commanded_positions */ {
            // Source: drake/manipulation/kuka_iiwa/build_iiwa_control.h
            const char* doc =
R"""(This will be non-null iff the control_mode denotes commanded
positions.)""";
          } commanded_positions;
          // Symbol: drake::manipulation::kuka_iiwa::IiwaControlPorts::commanded_torque
          struct /* commanded_torque */ {
            // Source: drake/manipulation/kuka_iiwa/build_iiwa_control.h
            const char* doc =
R"""(This will be non-null iff the control_mode denotes commanded torques.)""";
          } commanded_torque;
          // Symbol: drake::manipulation::kuka_iiwa::IiwaControlPorts::external_torque
          struct /* external_torque */ {
            // Source: drake/manipulation/kuka_iiwa/build_iiwa_control.h
            const char* doc = R"""()""";
          } external_torque;
          // Symbol: drake::manipulation::kuka_iiwa::IiwaControlPorts::joint_torque
          struct /* joint_torque */ {
            // Source: drake/manipulation/kuka_iiwa/build_iiwa_control.h
            const char* doc = R"""()""";
          } joint_torque;
          // Symbol: drake::manipulation::kuka_iiwa::IiwaControlPorts::position_commanded
          struct /* position_commanded */ {
            // Source: drake/manipulation/kuka_iiwa/build_iiwa_control.h
            const char* doc = R"""()""";
          } position_commanded;
          // Symbol: drake::manipulation::kuka_iiwa::IiwaControlPorts::position_measured
          struct /* position_measured */ {
            // Source: drake/manipulation/kuka_iiwa/build_iiwa_control.h
            const char* doc = R"""()""";
          } position_measured;
          // Symbol: drake::manipulation::kuka_iiwa::IiwaControlPorts::torque_measured
          struct /* torque_measured */ {
            // Source: drake/manipulation/kuka_iiwa/build_iiwa_control.h
            const char* doc = R"""()""";
          } torque_measured;
          // Symbol: drake::manipulation::kuka_iiwa::IiwaControlPorts::velocity_estimated
          struct /* velocity_estimated */ {
            // Source: drake/manipulation/kuka_iiwa/build_iiwa_control.h
            const char* doc = R"""()""";
          } velocity_estimated;
        } IiwaControlPorts;
        // Symbol: drake::manipulation::kuka_iiwa::IiwaDriver
        struct /* IiwaDriver */ {
          // Source: drake/manipulation/kuka_iiwa/iiwa_driver.h
          const char* doc =
R"""(This config struct specifies how to wire up Drake systems between an
LCM interface and the actuation input ports of a MultibodyPlant. This
simulates the role that driver software and control cabinets would
take in real life.

It creates an LCM publisher on the ``IIWA_STATUS`` channel and an LCM
subscriber on the ``IIWA_COMMAND`` channel.)""";
          // Symbol: drake::manipulation::kuka_iiwa::IiwaDriver::Serialize
          struct /* Serialize */ {
            // Source: drake/manipulation/kuka_iiwa/iiwa_driver.h
            const char* doc = R"""()""";
          } Serialize;
          // Symbol: drake::manipulation::kuka_iiwa::IiwaDriver::arm_child_frame_name
          struct /* arm_child_frame_name */ {
            // Source: drake/manipulation/kuka_iiwa/iiwa_driver.h
            const char* doc =
R"""(Optionally give an alternative frame on the arm model for its weld
point to the world. If not supplied, the ``child_frame_name`` in the
arm's ModelInstanceInfo will be used.)""";
          } arm_child_frame_name;
          // Symbol: drake::manipulation::kuka_iiwa::IiwaDriver::control_mode
          struct /* control_mode */ {
            // Source: drake/manipulation/kuka_iiwa/iiwa_driver.h
            const char* doc =
R"""(The driver's control mode. Valid options (per ParseIiwaControlMode)
are: - "position_only" - "position_and_torque" (default) -
"torque_only")""";
          } control_mode;
          // Symbol: drake::manipulation::kuka_iiwa::IiwaDriver::desired_kp_gains
          struct /* desired_kp_gains */ {
            // Source: drake/manipulation/kuka_iiwa/iiwa_driver.h
            const char* doc =
R"""(Optionally pass in gains corresponding to the Iiwa Dof (7) in the
controller. If no value is passed, the gains derived from hardware
will be used instead (hardcoded within the implementations of
functions accepting this struct). These gains must be nullopt if
``control_mode`` does not include position control.)""";
          } desired_kp_gains;
          // Symbol: drake::manipulation::kuka_iiwa::IiwaDriver::ext_joint_filter_tau
          struct /* ext_joint_filter_tau */ {
            // Source: drake/manipulation/kuka_iiwa/iiwa_driver.h
            const char* doc =
R"""(A time constant used to low-pass filter external torque inputs.)""";
          } ext_joint_filter_tau;
          // Symbol: drake::manipulation::kuka_iiwa::IiwaDriver::gripper_parent_frame_name
          struct /* gripper_parent_frame_name */ {
            // Source: drake/manipulation/kuka_iiwa/iiwa_driver.h
            const char* doc =
R"""(Optionally give an alternative frame on the arm model for its weld
point to the gripper. If not supplied, the ``parent_frame_name`` in
the gripper's ModelInstanceInfo will be used.)""";
          } gripper_parent_frame_name;
          // Symbol: drake::manipulation::kuka_iiwa::IiwaDriver::hand_model_name
          struct /* hand_model_name */ {
            // Source: drake/manipulation/kuka_iiwa/iiwa_driver.h
            const char* doc =
R"""(The name of the model (``name`` element of the ``add_model``
directive) in the simulation that the driver will analyze to compute
end effector inertia for its copy of the arm in inverse dynamics.)""";
          } hand_model_name;
          // Symbol: drake::manipulation::kuka_iiwa::IiwaDriver::lcm_bus
          struct /* lcm_bus */ {
            // Source: drake/manipulation/kuka_iiwa/iiwa_driver.h
            const char* doc = R"""()""";
          } lcm_bus;
          // Symbol: drake::manipulation::kuka_iiwa::IiwaDriver::lcm_status_period
          struct /* lcm_status_period */ {
            // Source: drake/manipulation/kuka_iiwa/iiwa_driver.h
            const char* doc =
R"""(The period in seconds at which status reports are expected.)""";
          } lcm_status_period;
          auto Serialize__fields() const {
            return std::array{
              std::make_pair("arm_child_frame_name", arm_child_frame_name.doc),
              std::make_pair("control_mode", control_mode.doc),
              std::make_pair("desired_kp_gains", desired_kp_gains.doc),
              std::make_pair("ext_joint_filter_tau", ext_joint_filter_tau.doc),
              std::make_pair("gripper_parent_frame_name", gripper_parent_frame_name.doc),
              std::make_pair("hand_model_name", hand_model_name.doc),
              std::make_pair("lcm_bus", lcm_bus.doc),
              std::make_pair("lcm_status_period", lcm_status_period.doc),
            };
          }
        } IiwaDriver;
        // Symbol: drake::manipulation::kuka_iiwa::IiwaStatusReceiver
        struct /* IiwaStatusReceiver */ {
          // Source: drake/manipulation/kuka_iiwa/iiwa_status_receiver.h
          const char* doc =
R"""(@system name: IiwaStatusReceiver input_ports: - lcmt_iiwa_status
output_ports: - time_measured - position_commanded - position_measured
- velocity_estimated - torque_commanded - torque_measured -
torque_external @endsystem

See also:
    ``lcmt_iiwa_status.lcm`` for additional documentation.)""";
          // Symbol: drake::manipulation::kuka_iiwa::IiwaStatusReceiver::IiwaStatusReceiver
          struct /* ctor */ {
            // Source: drake/manipulation/kuka_iiwa/iiwa_status_receiver.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::manipulation::kuka_iiwa::IiwaStatusReceiver::get_position_commanded_output_port
          struct /* get_position_commanded_output_port */ {
            // Source: drake/manipulation/kuka_iiwa/iiwa_status_receiver.h
            const char* doc = R"""()""";
          } get_position_commanded_output_port;
          // Symbol: drake::manipulation::kuka_iiwa::IiwaStatusReceiver::get_position_measured_output_port
          struct /* get_position_measured_output_port */ {
            // Source: drake/manipulation/kuka_iiwa/iiwa_status_receiver.h
            const char* doc = R"""()""";
          } get_position_measured_output_port;
          // Symbol: drake::manipulation::kuka_iiwa::IiwaStatusReceiver::get_time_measured_output_port
          struct /* get_time_measured_output_port */ {
            // Source: drake/manipulation/kuka_iiwa/iiwa_status_receiver.h
            const char* doc = R"""()""";
          } get_time_measured_output_port;
          // Symbol: drake::manipulation::kuka_iiwa::IiwaStatusReceiver::get_torque_commanded_output_port
          struct /* get_torque_commanded_output_port */ {
            // Source: drake/manipulation/kuka_iiwa/iiwa_status_receiver.h
            const char* doc = R"""()""";
          } get_torque_commanded_output_port;
          // Symbol: drake::manipulation::kuka_iiwa::IiwaStatusReceiver::get_torque_external_output_port
          struct /* get_torque_external_output_port */ {
            // Source: drake/manipulation/kuka_iiwa/iiwa_status_receiver.h
            const char* doc = R"""()""";
          } get_torque_external_output_port;
          // Symbol: drake::manipulation::kuka_iiwa::IiwaStatusReceiver::get_torque_measured_output_port
          struct /* get_torque_measured_output_port */ {
            // Source: drake/manipulation/kuka_iiwa/iiwa_status_receiver.h
            const char* doc = R"""()""";
          } get_torque_measured_output_port;
          // Symbol: drake::manipulation::kuka_iiwa::IiwaStatusReceiver::get_velocity_estimated_output_port
          struct /* get_velocity_estimated_output_port */ {
            // Source: drake/manipulation/kuka_iiwa/iiwa_status_receiver.h
            const char* doc = R"""()""";
          } get_velocity_estimated_output_port;
        } IiwaStatusReceiver;
        // Symbol: drake::manipulation::kuka_iiwa::IiwaStatusSender
        struct /* IiwaStatusSender */ {
          // Source: drake/manipulation/kuka_iiwa/iiwa_status_sender.h
          const char* doc =
R"""(Creates and outputs lcmt_iiwa_status messages.

Note that this system does not actually send the message an LCM
channel. To send the message, the output of this system should be
connected to a
systems∷lcm∷LcmPublisherSystem∷Make<lcmt_iiwa_status>().

This system has many vector-valued input ports, most of which have
exactly num_joints elements. The exception is ``time_measured`` which
is the one-dimensional time in seconds to set as the message timestamp
(i.e. the time inputted will be converted to microseconds and sent to
the hardware). It is optional and if unset, the context time is used.

- ``position_commanded``: the most recently received position command.
- ``position_measured``: the plant's current position.
- ``velocity_estimated`` (optional): the plant's current velocity (this
    should be a low-pass filter of the position's derivative; see detailed
    comments in ``lcmt_iiwa_status.lcm``); when absent, the output message
    will use zeros.
- ``torque_commanded``: the most recently received joint torque command.
- ``torque_measured`` (optional): the plant's measured joint torque; when
    absent, the output message will duplicate torque_commanded.
- ``torque_external`` (optional): the plant's external joint torque; when
    absent, the output message will use zeros.

This system has one abstract-valued output port of type
lcmt_iiwa_status.

This system is presently only used in simulation. The robot hardware
drivers publish directly to LCM and do not make use of this system.

.. pydrake_system::

    name: IiwaStatusSender
    input_ports:
    - position_commanded
    - position_measured
    - velocity_estimated
    - torque_commanded
    - torque_measured
    - torque_external
    - time_measured
    output_ports:
    - lcmt_iiwa_status

The ports ``velocity_estimated``, `torque_measured`,
``torque_external``, and ``time_measured`` may be left unconnected, as
detailed above.

See also:
    ``lcmt_iiwa_status.lcm`` for additional documentation.)""";
          // Symbol: drake::manipulation::kuka_iiwa::IiwaStatusSender::IiwaStatusSender
          struct /* ctor */ {
            // Source: drake/manipulation/kuka_iiwa/iiwa_status_sender.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::manipulation::kuka_iiwa::IiwaStatusSender::get_position_commanded_input_port
          struct /* get_position_commanded_input_port */ {
            // Source: drake/manipulation/kuka_iiwa/iiwa_status_sender.h
            const char* doc = R"""()""";
          } get_position_commanded_input_port;
          // Symbol: drake::manipulation::kuka_iiwa::IiwaStatusSender::get_position_measured_input_port
          struct /* get_position_measured_input_port */ {
            // Source: drake/manipulation/kuka_iiwa/iiwa_status_sender.h
            const char* doc = R"""()""";
          } get_position_measured_input_port;
          // Symbol: drake::manipulation::kuka_iiwa::IiwaStatusSender::get_time_measured_input_port
          struct /* get_time_measured_input_port */ {
            // Source: drake/manipulation/kuka_iiwa/iiwa_status_sender.h
            const char* doc = R"""()""";
          } get_time_measured_input_port;
          // Symbol: drake::manipulation::kuka_iiwa::IiwaStatusSender::get_torque_commanded_input_port
          struct /* get_torque_commanded_input_port */ {
            // Source: drake/manipulation/kuka_iiwa/iiwa_status_sender.h
            const char* doc = R"""()""";
          } get_torque_commanded_input_port;
          // Symbol: drake::manipulation::kuka_iiwa::IiwaStatusSender::get_torque_external_input_port
          struct /* get_torque_external_input_port */ {
            // Source: drake/manipulation/kuka_iiwa/iiwa_status_sender.h
            const char* doc = R"""()""";
          } get_torque_external_input_port;
          // Symbol: drake::manipulation::kuka_iiwa::IiwaStatusSender::get_torque_measured_input_port
          struct /* get_torque_measured_input_port */ {
            // Source: drake/manipulation/kuka_iiwa/iiwa_status_sender.h
            const char* doc = R"""()""";
          } get_torque_measured_input_port;
          // Symbol: drake::manipulation::kuka_iiwa::IiwaStatusSender::get_velocity_estimated_input_port
          struct /* get_velocity_estimated_input_port */ {
            // Source: drake/manipulation/kuka_iiwa/iiwa_status_sender.h
            const char* doc = R"""()""";
          } get_velocity_estimated_input_port;
        } IiwaStatusSender;
        // Symbol: drake::manipulation::kuka_iiwa::ParseIiwaControlMode
        struct /* ParseIiwaControlMode */ {
          // Source: drake/manipulation/kuka_iiwa/iiwa_constants.h
          const char* doc =
R"""(Parses control mode with the following mapping: - "position_only":
kPositionOnly - "torque_only": kTorqueOnly - "position_and_torque":
kPositionAndTorque)""";
        } ParseIiwaControlMode;
        // Symbol: drake::manipulation::kuka_iiwa::SimIiwaDriver
        struct /* SimIiwaDriver */ {
          // Source: drake/manipulation/kuka_iiwa/sim_iiwa_driver.h
          const char* doc =
R"""(SimIiwaDriver simulates the IIWA control and status interface using a
MultibodyPlant.

Warning:
    This feature is considered to be **experimental** and may change
    or be removed at any time, without any deprecation notice ahead of
    time.

.. pydrake_system::

    name: SimIiwaDriver
    input_ports:
    - <b style="color:orange">state</b>
    - <b style="color:orange">generalized_contact_forces</b>
    - position (in kPositionOnly or kPositionAndTorque mode)
    - torque (in kTorqueOnly or kPositionAndTorque mode)
    output_ports:
    - <b style="color:orange">actuation</b>
    - position_commanded
    - position_measured
    - velocity_estimated
    - state_estimated
    - torque_commanded
    - torque_measured
    - torque_external
    - velocity_commanded (in kPositionOnly or kPositionAndTorque mode)

Ports shown in <b style="color:orange">orange</b> are intended to
connect to the MultibodyPlant's per-model-instance ports of the same
name. All other ports are intended to mimic the LCM command and status
message fields.)""";
          // Symbol: drake::manipulation::kuka_iiwa::SimIiwaDriver::AddToBuilder
          struct /* AddToBuilder */ {
            // Source: drake/manipulation/kuka_iiwa/sim_iiwa_driver.h
            const char* doc =
R"""(Given a ``plant`` (and associated ``iiwa_instance``) and a
``builder``, installs in that builder the ``SimIiwaDriver`` system to
control and monitor an iiwa described by ``controller_plant``.

The added ``SimIiwaDriver`` system is connected to the actuation input
port, state and generalized contact forces output ports in ``plant``
corresponding to the iiwa model.

Returns the newly-added ``SimIiwaDriver`` System.

Note: The Diagram will maintain an internal reference to
``controller_plant``, so you must ensure that ``controller_plant`` has
a longer lifetime than the Diagram.)""";
          } AddToBuilder;
          // Symbol: drake::manipulation::kuka_iiwa::SimIiwaDriver::SimIiwaDriver<T>
          struct /* ctor */ {
            // Source: drake/manipulation/kuka_iiwa/sim_iiwa_driver.h
            const char* doc =
R"""(Constructs a diagram with the given ``driver_config``. A reference to
the ``controller_plant`` is retained by this system, so the
``controller_plant`` must outlive ``this``.)""";
            // Source: drake/manipulation/kuka_iiwa/sim_iiwa_driver.h
            const char* doc_copyconvert =
R"""(Scalar-converting copy constructor. See system_scalar_conversion.)""";
          } ctor;
        } SimIiwaDriver;
        // Symbol: drake::manipulation::kuka_iiwa::get_iiwa_max_joint_velocities
        struct /* get_iiwa_max_joint_velocities */ {
          // Source: drake/manipulation/kuka_iiwa/iiwa_constants.h
          const char* doc =
R"""(Returns the maximum joint velocities (rad/s) provided by Kuka.)""";
        } get_iiwa_max_joint_velocities;
        // Symbol: drake::manipulation::kuka_iiwa::position_enabled
        struct /* position_enabled */ {
          // Source: drake/manipulation/kuka_iiwa/iiwa_constants.h
          const char* doc =
R"""(Reports if the given control ``mode`` includes positions.)""";
        } position_enabled;
        // Symbol: drake::manipulation::kuka_iiwa::torque_enabled
        struct /* torque_enabled */ {
          // Source: drake/manipulation/kuka_iiwa/iiwa_constants.h
          const char* doc =
R"""(Reports if the given control ``mode`` includes torques.)""";
        } torque_enabled;
      } kuka_iiwa;
    } manipulation;
  } drake;
} pydrake_doc_manipulation_kuka_iiwa;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
