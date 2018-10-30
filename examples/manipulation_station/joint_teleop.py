"""
Runs the manipulation_station example with a simple tcl/tk joint slider ui for
directly tele-operating the joints.
"""

import argparse

import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.examples.manipulation_station import ManipulationStation
from pydrake.geometry import ConnectDrakeVisualizer
from pydrake.manipulation.simple_ui import JointSliders, SchunkWsgButtons
from pydrake.multibody.multibody_tree.parsing import AddModelFromSdfFile
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator
from pydrake.util.eigen_geometry import Isometry3

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument(
    "--target_realtime_rate", type=float, default=1.0,
    help="Desired rate relative to real time.  See documentation for "
         "Simulator::set_target_realtime_rate() for details.")
parser.add_argument(
    "--duration", type=float, default=np.inf,
    help="Desired duration of the simulation in seconds.")
args = parser.parse_args()

builder = DiagramBuilder()

station = builder.AddSystem(ManipulationStation())
station.AddCupboard()
object = AddModelFromSdfFile(FindResourceOrThrow(
    "drake/examples/manipulation_station/models/061_foam_brick.sdf"),
    "object", station.get_mutable_multibody_plant(),
    station.get_mutable_scene_graph())
station.Finalize()

ConnectDrakeVisualizer(builder, station.get_mutable_scene_graph(),
                       station.GetOutputPort("pose_bundle"))

teleop = builder.AddSystem(JointSliders(station.get_controller_plant(),
                                        length=800))
builder.Connect(teleop.get_output_port(0), station.GetInputPort(
    "iiwa_position"))

wsg_buttons = builder.AddSystem(SchunkWsgButtons(teleop.window))
builder.Connect(wsg_buttons.GetOutputPort("position"), station.GetInputPort(
    "wsg_position"))
builder.Connect(wsg_buttons.GetOutputPort("force_limit"),
                station.GetInputPort("wsg_force_limit"))

diagram = builder.Build()
simulator = Simulator(diagram)

context = diagram.GetMutableSubsystemContext(station,
                                             simulator.get_mutable_context())

# Set up the context for the simulator:
q0 = [0, 0.6, 0, -1.75, 0, 1.0, 0]
station.SetIiwaPosition(q0, context)
station.SetIiwaVelocity(np.zeros(7), context)
station.SetWsgPosition(0.1, context)
station.SetWsgVelocity(0, context)
X_WObject = Isometry3.Identity()
X_WObject.set_translation([.6, 0, 0])
station.get_mutable_multibody_plant().tree().SetFreeBodyPoseOrThrow(
    station.get_mutable_multibody_plant().GetBodyByName("base_link", object),
    X_WObject, station.GetMutableSubsystemContext(
        station.get_mutable_multibody_plant(),
        context))

teleop.set(q0)
context.FixInputPort(station.GetInputPort(
    "iiwa_feedforward_torque").get_index(), np.zeros(7))

simulator.set_target_realtime_rate(args.target_realtime_rate)
simulator.StepTo(args.duration)
