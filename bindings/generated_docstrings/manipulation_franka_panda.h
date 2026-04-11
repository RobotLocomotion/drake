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

// #include "drake/manipulation/franka_panda/panda_command_receiver.h"
// #include "drake/manipulation/franka_panda/panda_command_sender.h"
// #include "drake/manipulation/franka_panda/panda_constants.h"
// #include "drake/manipulation/franka_panda/panda_status_receiver.h"
// #include "drake/manipulation/franka_panda/panda_status_sender.h"

// Symbol: pydrake_doc_manipulation_franka_panda
constexpr struct /* pydrake_doc_manipulation_franka_panda */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::manipulation
    struct /* manipulation */ {
      // Symbol: drake::manipulation::franka_panda
      struct /* franka_panda */ {
        // Symbol: drake::manipulation::franka_panda::PandaCommandReceiver
        struct /* PandaCommandReceiver */ {
          // Source: drake/manipulation/franka_panda/panda_command_receiver.h
          const char* doc =
R"""(Handles lcmt_panda_command message from a LcmSubscriberSystem.

Note that this system does not actually subscribe to an LCM channel.
To receive the message, the input of this system should be connected
to a LcmSubscriberSystem∷Make<drake∷lcmt_panda_command>().

It has one required input port, "lcmt_panda_command". It has several
output ports, each one of size num_joints.

.. pydrake_system::

    name: PandaCommandReceiver
    input_ports:
    - lcmt_panda_command
    - position_measured
    output_ports:
    - position (*)
    - velocity (*)
    - torque   (*)

(*) Each output port is present iff the control_mode passed to the
constructor set the corresponding CONTROL_MODE bit.

Prior to receiving a valid lcmt_panda_command message, the "position"
output (if present) initially feeds through from the
"position_measured" input, and both the "velocity" and "torque"
outputs (if present) are zero.

If discrete update events are enabled (e.g., during simulation), the
system latches the "position_measured" input into state during the
first event, and the "position" output (if present) comes from the
latched state, not the input.

The lcmt_panda_command input must match the num_joints and
control_mode that were passed to the constructor: the message's
control_mode_expected must be set to the same value as the
constructor's control_mode, and the message's position, velocity,
torque vectors must be sized to match the constructor's num_joint iff
the corresponding bit of the control_mode is set or must be zero-sized
otherwise. TODO(jeremy.nimmer) The control_mode_expected is not
actually validated yet at runtime, but will be in the future (once we
fix our code to stop setting it incorrectly).)""";
          // Symbol: drake::manipulation::franka_panda::PandaCommandReceiver::LatchInitialPosition
          struct /* LatchInitialPosition */ {
            // Source: drake/manipulation/franka_panda/panda_command_receiver.h
            const char* doc =
R"""((Advanced.) Copies the current "position_measured" input into Context
state, and changes the behavior of the "position" output to produce
the latched state if no message has been received yet. The latching
already happens automatically during the first discrete update event
(e.g., when using a Simulator); this method exists for use when not
already using a Simulator or other special cases.

Precondition:
    the control_mode has the POSITION bit set)""";
          } LatchInitialPosition;
          // Symbol: drake::manipulation::franka_panda::PandaCommandReceiver::PandaCommandReceiver
          struct /* ctor */ {
            // Source: drake/manipulation/franka_panda/panda_command_receiver.h
            const char* doc =
R"""(Parameter ``control_mode``:
    is a bitset of one or more control mode constants defined in
    PandaControlModes namespace. Use bitwise OR to combine modes.)""";
          } ctor;
          // Symbol: drake::manipulation::franka_panda::PandaCommandReceiver::get_commanded_position_output_port
          struct /* get_commanded_position_output_port */ {
            // Source: drake/manipulation/franka_panda/panda_command_receiver.h
            const char* doc = R"""()""";
          } get_commanded_position_output_port;
          // Symbol: drake::manipulation::franka_panda::PandaCommandReceiver::get_commanded_torque_output_port
          struct /* get_commanded_torque_output_port */ {
            // Source: drake/manipulation/franka_panda/panda_command_receiver.h
            const char* doc = R"""()""";
          } get_commanded_torque_output_port;
          // Symbol: drake::manipulation::franka_panda::PandaCommandReceiver::get_commanded_velocity_output_port
          struct /* get_commanded_velocity_output_port */ {
            // Source: drake/manipulation/franka_panda/panda_command_receiver.h
            const char* doc = R"""()""";
          } get_commanded_velocity_output_port;
          // Symbol: drake::manipulation::franka_panda::PandaCommandReceiver::get_message_input_port
          struct /* get_message_input_port */ {
            // Source: drake/manipulation/franka_panda/panda_command_receiver.h
            const char* doc = R"""()""";
          } get_message_input_port;
          // Symbol: drake::manipulation::franka_panda::PandaCommandReceiver::get_position_measured_input_port
          struct /* get_position_measured_input_port */ {
            // Source: drake/manipulation/franka_panda/panda_command_receiver.h
            const char* doc = R"""()""";
          } get_position_measured_input_port;
        } PandaCommandReceiver;
        // Symbol: drake::manipulation::franka_panda::PandaCommandSender
        struct /* PandaCommandSender */ {
          // Source: drake/manipulation/franka_panda/panda_command_sender.h
          const char* doc =
R"""(Creates and outputs lcmt_panda_command messages.

Note that this system does not actually send the message an LCM
channel. To send the message, the output of this system should be
connected to a LcmPublisherSystem∷Make<lcmt_panda_command>().

This system has vector-valued input ports, each one of size
num_joints.

This system has one abstract-valued output port of type
lcmt_panda_command.

.. pydrake_system::

    name: PandaCommandSender
    input_ports:
    - position (*)
    - velocity (*)
    - torque (*)
    output_ports:
    - lcmt_panda_command

(*) Each input port is present if it's relevant to the control_mode
passed to the constructor.

See also:
    ``lcmt_panda_command.lcm`` for additional documentation.)""";
          // Symbol: drake::manipulation::franka_panda::PandaCommandSender::PandaCommandSender
          struct /* ctor */ {
            // Source: drake/manipulation/franka_panda/panda_command_sender.h
            const char* doc =
R"""(Parameter ``control_mode``:
    is a bitset of one or more control mode constants defined in
    PandaControlModes namespace. Use bitwise OR to combine modes.)""";
          } ctor;
          // Symbol: drake::manipulation::franka_panda::PandaCommandSender::get_position_input_port
          struct /* get_position_input_port */ {
            // Source: drake/manipulation/franka_panda/panda_command_sender.h
            const char* doc = R"""()""";
          } get_position_input_port;
          // Symbol: drake::manipulation::franka_panda::PandaCommandSender::get_torque_input_port
          struct /* get_torque_input_port */ {
            // Source: drake/manipulation/franka_panda/panda_command_sender.h
            const char* doc = R"""()""";
          } get_torque_input_port;
          // Symbol: drake::manipulation::franka_panda::PandaCommandSender::get_velocity_input_port
          struct /* get_velocity_input_port */ {
            // Source: drake/manipulation/franka_panda/panda_command_sender.h
            const char* doc = R"""()""";
          } get_velocity_input_port;
        } PandaCommandSender;
        // Symbol: drake::manipulation::franka_panda::PandaControlMode
        struct /* PandaControlMode */ {
          // Source: drake/manipulation/franka_panda/panda_constants.h
          const char* doc =
R"""(Type alias for Panda control mode bitfields. Control modes can be
bitwise OR'd together to enable multiple control modes simultaneously.
Values match lcmt_panda_status∷CONTROL_MODE_* constants.)""";
        } PandaControlMode;
        // Symbol: drake::manipulation::franka_panda::PandaControlModes
        struct /* PandaControlModes */ {
        } PandaControlModes;
        // Symbol: drake::manipulation::franka_panda::PandaStatusReceiver
        struct /* PandaStatusReceiver */ {
          // Source: drake/manipulation/franka_panda/panda_status_receiver.h
          const char* doc =
R"""(@system name: PandaStatusReceiver input_ports: - lcmt_panda_status
output_ports: - position_commanded - position_measured -
velocity_commanded - velocity_measured - acceleration_commanded -
torque_commanded - torque_measured - torque_external @endsystem

See also:
    ``lcmt_panda_status.lcm`` for additional documentation.)""";
          // Symbol: drake::manipulation::franka_panda::PandaStatusReceiver::OutputPort
          struct /* OutputPort */ {
            // Source: drake/manipulation/franka_panda/panda_status_receiver.h
            const char* doc = R"""()""";
          } OutputPort;
          // Symbol: drake::manipulation::franka_panda::PandaStatusReceiver::PandaStatusReceiver
          struct /* ctor */ {
            // Source: drake/manipulation/franka_panda/panda_status_receiver.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::manipulation::franka_panda::PandaStatusReceiver::get_acceleration_commanded_output_port
          struct /* get_acceleration_commanded_output_port */ {
            // Source: drake/manipulation/franka_panda/panda_status_receiver.h
            const char* doc = R"""()""";
          } get_acceleration_commanded_output_port;
          // Symbol: drake::manipulation::franka_panda::PandaStatusReceiver::get_position_commanded_output_port
          struct /* get_position_commanded_output_port */ {
            // Source: drake/manipulation/franka_panda/panda_status_receiver.h
            const char* doc = R"""()""";
          } get_position_commanded_output_port;
          // Symbol: drake::manipulation::franka_panda::PandaStatusReceiver::get_position_measured_output_port
          struct /* get_position_measured_output_port */ {
            // Source: drake/manipulation/franka_panda/panda_status_receiver.h
            const char* doc = R"""()""";
          } get_position_measured_output_port;
          // Symbol: drake::manipulation::franka_panda::PandaStatusReceiver::get_torque_commanded_output_port
          struct /* get_torque_commanded_output_port */ {
            // Source: drake/manipulation/franka_panda/panda_status_receiver.h
            const char* doc = R"""()""";
          } get_torque_commanded_output_port;
          // Symbol: drake::manipulation::franka_panda::PandaStatusReceiver::get_torque_external_output_port
          struct /* get_torque_external_output_port */ {
            // Source: drake/manipulation/franka_panda/panda_status_receiver.h
            const char* doc = R"""()""";
          } get_torque_external_output_port;
          // Symbol: drake::manipulation::franka_panda::PandaStatusReceiver::get_torque_measured_output_port
          struct /* get_torque_measured_output_port */ {
            // Source: drake/manipulation/franka_panda/panda_status_receiver.h
            const char* doc = R"""()""";
          } get_torque_measured_output_port;
          // Symbol: drake::manipulation::franka_panda::PandaStatusReceiver::get_velocity_commanded_output_port
          struct /* get_velocity_commanded_output_port */ {
            // Source: drake/manipulation/franka_panda/panda_status_receiver.h
            const char* doc = R"""()""";
          } get_velocity_commanded_output_port;
          // Symbol: drake::manipulation::franka_panda::PandaStatusReceiver::get_velocity_measured_output_port
          struct /* get_velocity_measured_output_port */ {
            // Source: drake/manipulation/franka_panda/panda_status_receiver.h
            const char* doc = R"""()""";
          } get_velocity_measured_output_port;
        } PandaStatusReceiver;
        // Symbol: drake::manipulation::franka_panda::PandaStatusSender
        struct /* PandaStatusSender */ {
          // Source: drake/manipulation/franka_panda/panda_status_sender.h
          const char* doc =
R"""(Creates and outputs lcmt_panda_status messages.

Note that this system does not actually send the message an LCM
channel. To send the message, the output of this system should be
connected to a LcmPublisherSystem∷Make<lcmt_panda_status>().

This system has many vector-valued input ports, each of which has
exactly num_joints elements.

This system has one abstract-valued output port of type
lcmt_panda_status.

This system is presently only used in simulation. The robot hardware
drivers publish directly to LCM and do not make use of this system.

.. pydrake_system::

    name: PandaStatusSender
    input_ports:
    - position_commanded (optional)
    - position_measured
    - velocity_commanded (optional)
    - velocity_measured (optional)
    - acceleration_commanded (optional)
    - torque_commanded
    - torque_measured (optional)
    - torque_external (optional)
    output_ports:
    - lcmt_panda_status

See also:
    ``lcmt_panda_status.lcm`` for additional documentation.)""";
          // Symbol: drake::manipulation::franka_panda::PandaStatusSender::InputPort
          struct /* InputPort */ {
            // Source: drake/manipulation/franka_panda/panda_status_sender.h
            const char* doc = R"""()""";
          } InputPort;
          // Symbol: drake::manipulation::franka_panda::PandaStatusSender::PandaStatusSender
          struct /* ctor */ {
            // Source: drake/manipulation/franka_panda/panda_status_sender.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::manipulation::franka_panda::PandaStatusSender::get_acceleration_commanded_input_port
          struct /* get_acceleration_commanded_input_port */ {
            // Source: drake/manipulation/franka_panda/panda_status_sender.h
            const char* doc = R"""()""";
          } get_acceleration_commanded_input_port;
          // Symbol: drake::manipulation::franka_panda::PandaStatusSender::get_position_commanded_input_port
          struct /* get_position_commanded_input_port */ {
            // Source: drake/manipulation/franka_panda/panda_status_sender.h
            const char* doc = R"""()""";
          } get_position_commanded_input_port;
          // Symbol: drake::manipulation::franka_panda::PandaStatusSender::get_position_measured_input_port
          struct /* get_position_measured_input_port */ {
            // Source: drake/manipulation/franka_panda/panda_status_sender.h
            const char* doc = R"""()""";
          } get_position_measured_input_port;
          // Symbol: drake::manipulation::franka_panda::PandaStatusSender::get_torque_commanded_input_port
          struct /* get_torque_commanded_input_port */ {
            // Source: drake/manipulation/franka_panda/panda_status_sender.h
            const char* doc = R"""()""";
          } get_torque_commanded_input_port;
          // Symbol: drake::manipulation::franka_panda::PandaStatusSender::get_torque_external_input_port
          struct /* get_torque_external_input_port */ {
            // Source: drake/manipulation/franka_panda/panda_status_sender.h
            const char* doc = R"""()""";
          } get_torque_external_input_port;
          // Symbol: drake::manipulation::franka_panda::PandaStatusSender::get_torque_measured_input_port
          struct /* get_torque_measured_input_port */ {
            // Source: drake/manipulation/franka_panda/panda_status_sender.h
            const char* doc = R"""()""";
          } get_torque_measured_input_port;
          // Symbol: drake::manipulation::franka_panda::PandaStatusSender::get_velocity_commanded_input_port
          struct /* get_velocity_commanded_input_port */ {
            // Source: drake/manipulation/franka_panda/panda_status_sender.h
            const char* doc = R"""()""";
          } get_velocity_commanded_input_port;
          // Symbol: drake::manipulation::franka_panda::PandaStatusSender::get_velocity_measured_input_port
          struct /* get_velocity_measured_input_port */ {
            // Source: drake/manipulation/franka_panda/panda_status_sender.h
            const char* doc = R"""()""";
          } get_velocity_measured_input_port;
        } PandaStatusSender;
        // Symbol: drake::manipulation::franka_panda::to_int
        struct /* to_int */ {
          // Source: drake/manipulation/franka_panda/panda_constants.h
          const char* doc = R"""()""";
        } to_int;
      } franka_panda;
    } manipulation;
  } drake;
} pydrake_doc_manipulation_franka_panda;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
