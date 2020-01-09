"""
Runs the manipulation_station example with a simple tcl/tk joint slider ui for
directly tele-operating the joints.
"""

import argparse

import numpy as np

from pydrake.examples.manipulation_station import (
    ManipulationStation, ManipulationStationHardwareInterface,
    CreateClutterClearingYcbObjectList)
from pydrake.geometry import ConnectDrakeVisualizer
from pydrake.manipulation.simple_ui import JointSliders, SchunkWsgButtons
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator
from pydrake.systems.meshcat_visualizer import MeshcatVisualizer
from pydrake.systems.primitives import FirstOrderLowPassFilter
from pydrake.systems.planar_scenegraph_visualizer import \
    PlanarSceneGraphVisualizer

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
parser.add_argument(
    '--setup', type=str, default='manipulation_class',
    help="The manipulation station setup to simulate. ",
    choices=['manipulation_class', 'clutter_clearing', 'planar'])
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

    # Initializes the chosen station type.
    if args.setup == 'manipulation_class':
        station.SetupManipulationClassStation()
        station.AddManipulandFromFile(
            "drake/examples/manipulation_station/models/061_foam_brick.sdf",
            RigidTransform(RotationMatrix.Identity(), [0.6, 0, 0]))
    elif args.setup == 'clutter_clearing':
        station.SetupClutterClearingStation()
        ycb_objects = CreateClutterClearingYcbObjectList()
        for model_file, X_WObject in ycb_objects:
            station.AddManipulandFromFile(model_file, X_WObject)
    elif args.setup == 'planar':
        station.SetupPlanarIiwaStation()
        station.AddManipulandFromFile(
            "drake/examples/manipulation_station/models/061_foam_brick.sdf",
            RigidTransform(RotationMatrix.Identity(), [0.6, 0, 0]))

    station.Finalize()

    ConnectDrakeVisualizer(builder, station.get_scene_graph(),
                           station.GetOutputPort("pose_bundle"))
    if args.meshcat:
        meshcat = builder.AddSystem(MeshcatVisualizer(
                station.get_scene_graph(), zmq_url=args.meshcat,
                open_browser=args.open_browser))
        builder.Connect(station.GetOutputPort("pose_bundle"),
                        meshcat.get_input_port(0))
    if args.setup == 'planar':
        pyplot_visualizer = builder.AddSystem(PlanarSceneGraphVisualizer(
            station.get_scene_graph()))
        builder.Connect(station.GetOutputPort("pose_bundle"),
                        pyplot_visualizer.get_input_port(0))

teleop = builder.AddSystem(JointSliders(station.get_controller_plant(),
                                        length=800))
if args.test:
    teleop.window.withdraw()  # Don't display the window when testing.

filter = builder.AddSystem(FirstOrderLowPassFilter(
    time_constant=2.0, size=station.num_iiwa_joints()))
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

# This is important to avoid duplicate publishes to the hardware interface:
simulator.set_publish_every_time_step(False)

station_context = diagram.GetMutableSubsystemContext(
    station, simulator.get_mutable_context())

station.GetInputPort("iiwa_feedforward_torque").FixValue(
    station_context, np.zeros(station.num_iiwa_joints()))

# Eval the output port once to read the initial positions of the IIWA.
simulator.AdvanceTo(1e-6)
q0 = station.GetOutputPort("iiwa_position_measured").Eval(
    station_context)
teleop.set_position(q0)
filter.set_initial_output_value(diagram.GetMutableSubsystemContext(
    filter, simulator.get_mutable_context()), q0)

simulator.set_target_realtime_rate(args.target_realtime_rate)
simulator.AdvanceTo(args.duration)
