"""
Runs the manipulation_station example with a simple tcl/tk joint slider ui for
directly tele-operating the joints.
"""

import argparse

import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.examples.manipulation_station import \
    (ManipulationStation, ManipulationStationHardwareInterface)
from pydrake.geometry import ConnectDrakeVisualizer
from pydrake.manipulation.simple_ui import JointSliders, SchunkWsgButtons
from pydrake.multibody.multibody_tree.parsing import AddModelFromSdfFile
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator
from pydrake.systems.meshcat_visualizer import MeshcatVisualizer
from pydrake.systems.primitives import FirstOrderLowPassFilter
from pydrake.util.eigen_geometry import Isometry3

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument(
    "--target_realtime_rate", type=float, default=1.0,
    help="Desired rate relative to real time.  See documentation for "
         "Simulator::set_target_realtime_rate() for details.")
parser.add_argument(
    "--duration", type=float, default=np.inf,
    help="Desired duration of the simulation in seconds.")
parser.add_argument(
    "--hardware", action='store_true',
    help="Use the ManipulationStationHardwareInterface instead of an "
         "in-process simulation.")
parser.add_argument(
    "--test", action='store_true',
    help="Disable opening the gui window for testing.")
MeshcatVisualizer.add_argparse_argument(parser)
args = parser.parse_args()

builder = DiagramBuilder()

if args.hardware:
    # TODO(russt): Replace this hard-coded camera serial number with a config
    # file.
    camera_ids = ["805212060544"]
    station = builder.AddSystem(ManipulationStationHardwareInterface(
        camera_ids))
    station.Connect(wait_for_cameras=False)
else:
    station = builder.AddSystem(ManipulationStation())
    station.SetupDefaultStation()
    object = AddModelFromSdfFile(FindResourceOrThrow(
        "drake/examples/manipulation_station/models/061_foam_brick.sdf"),
        "object", station.get_mutable_multibody_plant(),
        station.get_mutable_scene_graph())
    station.Finalize()

    ConnectDrakeVisualizer(builder, station.get_scene_graph(),
                           station.GetOutputPort("pose_bundle"))
    if args.meshcat:
        meshcat = builder.AddSystem(MeshcatVisualizer(
                station.get_scene_graph(), zmq_url=args.meshcat))
        builder.Connect(station.GetOutputPort("pose_bundle"),
                        meshcat.get_input_port(0))

teleop = builder.AddSystem(JointSliders(station.get_controller_plant(),
                                        length=800))
if args.test:
    teleop.window.withdraw()  # Don't display the window when testing.

filter = builder.AddSystem(FirstOrderLowPassFilter(time_constant=2.0,
                                                   size=7))
builder.Connect(teleop.get_output_port(0), filter.get_input_port(0))
builder.Connect(filter.get_output_port(0),
                station.GetInputPort("iiwa_position"))

wsg_buttons = builder.AddSystem(SchunkWsgButtons(teleop.window))
builder.Connect(wsg_buttons.GetOutputPort("position"), station.GetInputPort(
    "wsg_position"))
builder.Connect(wsg_buttons.GetOutputPort("force_limit"),
                station.GetInputPort("wsg_force_limit"))

diagram = builder.Build()
simulator = Simulator(diagram)

station_context = diagram.GetMutableSubsystemContext(
    station, simulator.get_mutable_context())

station_context.FixInputPort(station.GetInputPort(
    "iiwa_feedforward_torque").get_index(), np.zeros(7))

if not args.hardware:
    # Set the initial positions of the IIWA to a comfortable configuration
    # inside the workspace of the station.
    q0 = [0, 0.6, 0, -1.75, 0, 1.0, 0]
    station.SetIiwaPosition(q0, station_context)
    station.SetIiwaVelocity(np.zeros(7), station_context)

    # Set the initial configuration of the gripper to open.
    station.SetWsgPosition(0.1, station_context)
    station.SetWsgVelocity(0, station_context)

    # Place the object in the middle of the workspace.
    X_WObject = Isometry3.Identity()
    X_WObject.set_translation([.6, 0, 0])
    station.get_multibody_plant().tree().SetFreeBodyPoseOrThrow(
        station.get_multibody_plant().GetBodyByName("base_link",
                                                    object),
        X_WObject, station.GetMutableSubsystemContext(
            station.get_multibody_plant(),
            station_context))

# Eval the output port once to read the initial positions of the IIWA.
q0 = station.GetOutputPort("iiwa_position_measured").Eval(
    station_context).get_value()
teleop.set_position(q0)
filter.set_initial_output_value(diagram.GetMutableSubsystemContext(
    filter, simulator.get_mutable_context()), q0)

# This is important to avoid duplicate publishes to the hardware interface:
simulator.set_publish_every_time_step(False)

simulator.set_target_realtime_rate(args.target_realtime_rate)
simulator.StepTo(args.duration)
