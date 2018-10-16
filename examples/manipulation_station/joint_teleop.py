"""
Runs the manipulation_station example with a simple tcl/tk joint slider ui for
directly tele-operating the joints.
"""
import Tkinter as tk
import numpy as np

from pydrake.examples.manipulation_station import StationSimulation
from pydrake.geometry import ConnectDrakeVisualizer
from pydrake.manipulation.simple_ui import JointSliders
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator


builder = DiagramBuilder()

station = builder.AddSystem(StationSimulation())
station.Finalize()

teleop = builder.AddSystem(JointSliders(station.get_controller_plant()))
builder.Connect(teleop.get_output_port(0), station.GetInputPort(
    "iiwa_position"))

ConnectDrakeVisualizer(builder, station.get_mutable_scene_graph(),
                       station.GetOutputPort("pose_bundle"))

diagram = builder.Build()
simulator = Simulator(diagram)

context = diagram.GetMutableSubsystemContext(station,
                                             simulator.get_mutable_context())

q0 = [0, 0.6, 0, -1.0, 0, 1.0, 0]
station.SetIiwaPosition(q0, context)
station.SetIiwaVelocity(np.zeros(7), context)
teleop.set(q0)

context.FixInputPort(station.GetInputPort(
    "iiwa_feedforward_torque").get_index(), np.zeros(7))

simulator.set_target_realtime_rate(1.0)
simulator.StepTo(np.inf)
