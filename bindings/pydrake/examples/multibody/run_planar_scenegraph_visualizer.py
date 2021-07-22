"""
Run examples of PlanarSceneGraphVisualizer, e.g. to visualize a pendulum.
Usage demo: load a URDF, rig it up with a constant torque input, and draw it.
"""

import argparse

import numpy as np

from pydrake.common import FindResourceOrThrow, temp_directory
from pydrake.examples.manipulation_station import ManipulationStation
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.planar_scenegraph_visualizer import (
    ConnectPlanarSceneGraphVisualizer)


def run_pendulum_example(args):
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
    parser = Parser(plant)
    parser.AddModelFromFile(FindResourceOrThrow(
        "drake/examples/pendulum/Pendulum.urdf"))
    plant.Finalize()

    T_VW = np.array([[1., 0., 0., 0.],
                     [0., 0., 1., 0.],
                     [0., 0., 0., 1.]])
    visualizer = ConnectPlanarSceneGraphVisualizer(
        builder, scene_graph, T_VW=T_VW, xlim=[-1.2, 1.2], ylim=[-1.2, 1.2])

    if args.playback:
        visualizer.start_recording()

    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.0)

    # Fix the input port to zero.
    plant_context = diagram.GetMutableSubsystemContext(
        plant, simulator.get_mutable_context())
    plant.get_actuation_input_port().FixValue(
        plant_context, np.zeros(plant.num_actuators()))
    plant_context.SetContinuousState([0.5, 0.1])
    simulator.AdvanceTo(args.duration)

    if args.playback:
        visualizer.stop_recording()
        ani = visualizer.get_recording_as_animation()
        # Playback the recording and save the output.
        ani.save("{}/pend_playback.mp4".format(temp_directory()), fps=30)


def run_manipulation_example(args):
    builder = DiagramBuilder()
    station = builder.AddSystem(ManipulationStation(time_step=0.005))
    station.SetupManipulationClassStation()
    station.Finalize()

    plant = station.get_multibody_plant()
    scene_graph = station.get_scene_graph()
    query_object_output_port = station.GetOutputPort("geometry_query")

    # Side-on view of the station.
    T_VW = np.array([[1., 0., 0., 0.],
                     [0., 0., 1., 0.],
                     [0., 0., 0., 1.]])
    ConnectPlanarSceneGraphVisualizer(
        builder, scene_graph, query_object_output_port, T_VW=T_VW,
        xlim=[-0.5, 1.0], ylim=[-1.2, 1.2], draw_period=0.1)

    if args.playback:
        visualizer.start_recording()

    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.0)

    # Fix the control inputs to zero.
    station_context = diagram.GetMutableSubsystemContext(
        station, simulator.get_mutable_context())
    station.GetInputPort("iiwa_position").FixValue(
        station_context, station.GetIiwaPosition(station_context))
    station.GetInputPort("iiwa_feedforward_torque").FixValue(
        station_context, np.zeros(7))
    station.GetInputPort("wsg_position").FixValue(
        station_context, np.zeros(1))
    station.GetInputPort("wsg_force_limit").FixValue(
        station_context, [40.0])
    simulator.AdvanceTo(args.duration)

    if args.playback:
        visualizer.stop_recording()
        ani = visualizer.get_recording_as_animation()
        # Playback the recording and save the output.
        ani.save("{}/manip_playback.mp4".format(temp_directory()), fps=30)


def main():
    np.set_printoptions(precision=5, suppress=True)
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("-T", "--duration",
                        type=float,
                        help="Duration to run sim in seconds.",
                        default=1.0)
    parser.add_argument("-m", "--models",
                        type=str,
                        nargs="*",
                        help="Models to run, at least one of [pend, manip]",
                        default=["pend"])
    parser.add_argument("-p", "--playback",
                        type=bool,
                        help="Whether to record and playback the animation",
                        default=False)
    args = parser.parse_args()

    for model in args.models:
        if model == "pend":
            run_pendulum_example(args)
        elif model == "manip":
            run_manipulation_example(args)
        else:
            print("Unrecognized model %s." % model)
            parser.print_usage()
            exit(1)


if __name__ == "__main__":
    main()
