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

// #include "drake/manipulation/kinova_jaco/jaco_command_receiver.h"
// #include "drake/manipulation/kinova_jaco/jaco_command_sender.h"
// #include "drake/manipulation/kinova_jaco/jaco_constants.h"
// #include "drake/manipulation/kinova_jaco/jaco_status_receiver.h"
// #include "drake/manipulation/kinova_jaco/jaco_status_sender.h"

// Symbol: pydrake_doc_manipulation_kinova_jaco
constexpr struct /* pydrake_doc_manipulation_kinova_jaco */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::manipulation
    struct /* manipulation */ {
      // Symbol: drake::manipulation::kinova_jaco
      struct /* kinova_jaco */ {
        // Symbol: drake::manipulation::kinova_jaco::JacoCommandReceiver
        struct /* JacoCommandReceiver */ {
          // Source: drake/manipulation/kinova_jaco/jaco_command_receiver.h:51
          const char* doc =
R"""(Handles lcmt_jaco_command message from a LcmSubscriberSystem.

Note that this system does not actually subscribe to an LCM channel.
To receive the message, the input of this system should be connected
to a LcmSubscriberSystem::Make<drake::lcmt_jaco_command>().

It has one required input port, "lcmt_jaco_command".

This system has three output ports: one each for the commanded
position and velocity of the arm+finger joints, and one for the
timestamp in the most recently received message. Finger velocities
will be translated from the values used by the Kinova SDK to values
appropriate for the finger joints in the Jaco description (see
jaco_constants.h).

.. pydrake_system::

    name: JacoCommandReceiver
    input_ports:
    - lcmt_jaco_command
    - position_measured (optional)
    output_ports:
    - position
    - velocity
    - time

@par Output prior to receiving a valid lcmt_jaco_command message: The
"position" output initially feeds through from the "position_measured"
input port -- or if not connected, outputs zero. When discrete update
events are enabled (e.g., during a simulation), the system latches the
"position_measured" input into state during the first event, and the
"position" output comes from the latched state, no longer fed through
from the "position" input. Alternatively, the LatchInitialPosition()
method is available to achieve the same effect without using events.
The "time" output will be a vector of a single zero.)""";
          // Symbol: drake::manipulation::kinova_jaco::JacoCommandReceiver::JacoCommandReceiver
          struct /* ctor */ {
            // Source: drake/manipulation/kinova_jaco/jaco_command_receiver.h:53
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::manipulation::kinova_jaco::JacoCommandReceiver::LatchInitialPosition
          struct /* LatchInitialPosition */ {
            // Source: drake/manipulation/kinova_jaco/jaco_command_receiver.h:64
            const char* doc =
R"""((Advanced) Copies the current "position_measured" input (or zero if
not connected) into Context state, and changes the behavior of the
"position" output to produce the latched state if no message has been
received yet. The latching already happens automatically during the
first discrete update event (e.g., when using a Simulator); this
method exists for use when not already using a Simulator or other
special cases.)""";
          } LatchInitialPosition;
          // Symbol: drake::manipulation::kinova_jaco::JacoCommandReceiver::get_commanded_position_output_port
          struct /* get_commanded_position_output_port */ {
            // Source: drake/manipulation/kinova_jaco/jaco_command_receiver.h:74
            const char* doc = R"""()""";
          } get_commanded_position_output_port;
          // Symbol: drake::manipulation::kinova_jaco::JacoCommandReceiver::get_commanded_velocity_output_port
          struct /* get_commanded_velocity_output_port */ {
            // Source: drake/manipulation/kinova_jaco/jaco_command_receiver.h:78
            const char* doc = R"""()""";
          } get_commanded_velocity_output_port;
          // Symbol: drake::manipulation::kinova_jaco::JacoCommandReceiver::get_message_input_port
          struct /* get_message_input_port */ {
            // Source: drake/manipulation/kinova_jaco/jaco_command_receiver.h:68
            const char* doc = R"""()""";
          } get_message_input_port;
          // Symbol: drake::manipulation::kinova_jaco::JacoCommandReceiver::get_position_measured_input_port
          struct /* get_position_measured_input_port */ {
            // Source: drake/manipulation/kinova_jaco/jaco_command_receiver.h:71
            const char* doc = R"""()""";
          } get_position_measured_input_port;
          // Symbol: drake::manipulation::kinova_jaco::JacoCommandReceiver::get_time_output_port
          struct /* get_time_output_port */ {
            // Source: drake/manipulation/kinova_jaco/jaco_command_receiver.h:82
            const char* doc = R"""()""";
          } get_time_output_port;
        } JacoCommandReceiver;
        // Symbol: drake::manipulation::kinova_jaco::JacoCommandSender
        struct /* JacoCommandSender */ {
          // Source: drake/manipulation/kinova_jaco/jaco_command_sender.h:39
          const char* doc =
R"""(Creates and outputs lcmt_jaco_command messages.

Note that this system does not actually send the message to an LCM
channel. To send the message, the output of this system should be
connected to a
systems::lcm::LcmPublisherSystem::Make<lcmt_jaco_command>().

This system has two mandatory vector-valued input ports containing the
desired position and velocity, and an optional vector-valued input
port for the command timestamp. If the time input port is not
connected, the context time will be used. Finger velocities will be
translated to the values used by the Kinova SDK from values
appropriate for the finger joints in the Jaco description (see
jaco_constants.h).

This system has one abstract-valued output port of type
lcmt_jaco_command.

.. pydrake_system::

    name: JacoCommandSender
    input_ports:
    - position
    - velocity
    - time (optional)
    output_ports:
    - lcmt_jaco_command

See also:
    ``lcmt_jaco_command.lcm`` for additional documentation.)""";
          // Symbol: drake::manipulation::kinova_jaco::JacoCommandSender::JacoCommandSender
          struct /* ctor */ {
            // Source: drake/manipulation/kinova_jaco/jaco_command_sender.h:41
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::manipulation::kinova_jaco::JacoCommandSender::get_position_input_port
          struct /* get_position_input_port */ {
            // Source: drake/manipulation/kinova_jaco/jaco_command_sender.h:46
            const char* doc = R"""()""";
          } get_position_input_port;
          // Symbol: drake::manipulation::kinova_jaco::JacoCommandSender::get_time_input_port
          struct /* get_time_input_port */ {
            // Source: drake/manipulation/kinova_jaco/jaco_command_sender.h:53
            const char* doc = R"""()""";
          } get_time_input_port;
          // Symbol: drake::manipulation::kinova_jaco::JacoCommandSender::get_velocity_input_port
          struct /* get_velocity_input_port */ {
            // Source: drake/manipulation/kinova_jaco/jaco_command_sender.h:49
            const char* doc = R"""()""";
          } get_velocity_input_port;
        } JacoCommandSender;
        // Symbol: drake::manipulation::kinova_jaco::JacoStatusReceiver
        struct /* JacoStatusReceiver */ {
          // Source: drake/manipulation/kinova_jaco/jaco_status_receiver.h:44
          const char* doc =
R"""(@system name: JacoStatusReceiver input_ports: - lcmt_jaco_status
output_ports: - position_measured - velocity_measured -
torque_measured - torque_external - current - time_measured @endsystem

See also:
    ``lcmt_jaco_status.lcm`` for additional documentation.)""";
          // Symbol: drake::manipulation::kinova_jaco::JacoStatusReceiver::JacoStatusReceiver
          struct /* ctor */ {
            // Source: drake/manipulation/kinova_jaco/jaco_status_receiver.h:46
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::manipulation::kinova_jaco::JacoStatusReceiver::get_current_output_port
          struct /* get_current_output_port */ {
            // Source: drake/manipulation/kinova_jaco/jaco_status_receiver.h:65
            const char* doc = R"""()""";
          } get_current_output_port;
          // Symbol: drake::manipulation::kinova_jaco::JacoStatusReceiver::get_position_measured_output_port
          struct /* get_position_measured_output_port */ {
            // Source: drake/manipulation/kinova_jaco/jaco_status_receiver.h:53
            const char* doc = R"""()""";
          } get_position_measured_output_port;
          // Symbol: drake::manipulation::kinova_jaco::JacoStatusReceiver::get_time_measured_output_port
          struct /* get_time_measured_output_port */ {
            // Source: drake/manipulation/kinova_jaco/jaco_status_receiver.h:68
            const char* doc = R"""()""";
          } get_time_measured_output_port;
          // Symbol: drake::manipulation::kinova_jaco::JacoStatusReceiver::get_torque_external_output_port
          struct /* get_torque_external_output_port */ {
            // Source: drake/manipulation/kinova_jaco/jaco_status_receiver.h:62
            const char* doc = R"""()""";
          } get_torque_external_output_port;
          // Symbol: drake::manipulation::kinova_jaco::JacoStatusReceiver::get_torque_measured_output_port
          struct /* get_torque_measured_output_port */ {
            // Source: drake/manipulation/kinova_jaco/jaco_status_receiver.h:59
            const char* doc = R"""()""";
          } get_torque_measured_output_port;
          // Symbol: drake::manipulation::kinova_jaco::JacoStatusReceiver::get_velocity_measured_output_port
          struct /* get_velocity_measured_output_port */ {
            // Source: drake/manipulation/kinova_jaco/jaco_status_receiver.h:56
            const char* doc = R"""()""";
          } get_velocity_measured_output_port;
        } JacoStatusReceiver;
        // Symbol: drake::manipulation::kinova_jaco::JacoStatusSender
        struct /* JacoStatusSender */ {
          // Source: drake/manipulation/kinova_jaco/jaco_status_sender.h:50
          const char* doc =
R"""(Creates and outputs lcmt_jaco_status messages.

Note that this system does not actually send the message to an LCM
channel. To send the message, the output of this system should be
connected to a
systems::lcm::LcmPublisherSystem::Make<lcmt_jaco_status>().

This system has many vector-valued input ports. Most input ports are
of size num_joints + num_fingers. The exception is ``time_measured``
which is the one-dimensional time in seconds to set as the message
timestamp (i.e. the time inputted will be converted to microseconds
and sent to the hardware). It is optional and if unset, the context
time is used. The elements in the ports are the joints of the arm from
the base to the tip, followed by the fingers in the same order as used
by the Kinova SDK (consult the URDF model for a visual example). If
the torque, torque_external, or current input ports are not connected,
the output message will use zeros. Finger velocities will be
translated to the values used by the Kinova SDK from values
appropriate for the finger joints in the Jaco description (see
jaco_constants.h).

This system has one abstract-valued output port of type
lcmt_jaco_status.

This system is presently only used in simulation. The robot hardware
drivers publish directly to LCM and do not make use of this system.

.. pydrake_system::

    name: JacoStatusSender
    input_ports:
    - position
    - velocity
    - torque (optional)
    - torque_external (optional)
    - current (optional)
    - time_measured (optional)
    output_ports:
    - lcmt_jaco_status

See also:
    ``lcmt_jaco_status.lcm`` for additional documentation.)""";
          // Symbol: drake::manipulation::kinova_jaco::JacoStatusSender::JacoStatusSender
          struct /* ctor */ {
            // Source: drake/manipulation/kinova_jaco/jaco_status_sender.h:52
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::manipulation::kinova_jaco::JacoStatusSender::get_current_input_port
          struct /* get_current_input_port */ {
            // Source: drake/manipulation/kinova_jaco/jaco_status_sender.h:74
            const char* doc = R"""()""";
          } get_current_input_port;
          // Symbol: drake::manipulation::kinova_jaco::JacoStatusSender::get_position_input_port
          struct /* get_position_input_port */ {
            // Source: drake/manipulation/kinova_jaco/jaco_status_sender.h:62
            const char* doc = R"""()""";
          } get_position_input_port;
          // Symbol: drake::manipulation::kinova_jaco::JacoStatusSender::get_time_measured_input_port
          struct /* get_time_measured_input_port */ {
            // Source: drake/manipulation/kinova_jaco/jaco_status_sender.h:59
            const char* doc = R"""()""";
          } get_time_measured_input_port;
          // Symbol: drake::manipulation::kinova_jaco::JacoStatusSender::get_torque_external_input_port
          struct /* get_torque_external_input_port */ {
            // Source: drake/manipulation/kinova_jaco/jaco_status_sender.h:71
            const char* doc = R"""()""";
          } get_torque_external_input_port;
          // Symbol: drake::manipulation::kinova_jaco::JacoStatusSender::get_torque_input_port
          struct /* get_torque_input_port */ {
            // Source: drake/manipulation/kinova_jaco/jaco_status_sender.h:68
            const char* doc = R"""()""";
          } get_torque_input_port;
          // Symbol: drake::manipulation::kinova_jaco::JacoStatusSender::get_velocity_input_port
          struct /* get_velocity_input_port */ {
            // Source: drake/manipulation/kinova_jaco/jaco_status_sender.h:65
            const char* doc = R"""()""";
          } get_velocity_input_port;
        } JacoStatusSender;
      } kinova_jaco;
    } manipulation;
  } drake;
} pydrake_doc_manipulation_kinova_jaco;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
