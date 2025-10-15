"""
Run examples of PlanarSceneGraphVisualizer, e.g. to visualize a pendulum.
Usage demo: load a URDF, rig it up with a constant torque input, and draw it.
"""

import argparse

import numpy as np

from pydrake.common import configure_logging, temp_directory
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.planar_scenegraph_visualizer import (
    ConnectPlanarSceneGraphVisualizer,
)


def run_pendulum_example(args):
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
    parser = Parser(builder)
    parser.AddModelsFromUrl(
        url="package://drake/examples/pendulum/Pendulum.urdf"
    )
    plant.Finalize()

    T_VW = np.array(
        [
            [1.0, 0.0, 0.0, 0.0],  # BR
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    visualizer = ConnectPlanarSceneGraphVisualizer(
        builder, scene_graph, T_VW=T_VW, xlim=[-1.2, 1.2], ylim=[-1.2, 1.2]
    )

    if args.playback:
        visualizer.start_recording()

    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.0)

    # Fix the input port to zero.
    plant_context = diagram.GetMutableSubsystemContext(
        plant, simulator.get_mutable_context()
    )
    plant.get_actuation_input_port().FixValue(
        plant_context, np.zeros(plant.num_actuators())
    )
    plant_context.SetContinuousState([0.5, 0.1])
    simulator.AdvanceTo(args.duration)

    if args.playback:
        visualizer.stop_recording()
        ani = visualizer.get_recording_as_animation()
        # Playback the recording and save the output.
        ani.save("{}/pend_playback.mp4".format(temp_directory()), fps=30)


def main():
    configure_logging()
    np.set_printoptions(precision=5, suppress=True)
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "-T",
        "--duration",
        type=float,
        help="Duration to run sim in seconds.",
        default=1.0,
    )
    parser.add_argument(
        "-p",
        "--playback",
        type=bool,
        help="Whether to record and playback the animation",
        default=False,
    )
    args = parser.parse_args()
    run_pendulum_example(args)


if __name__ == "__main__":
    main()
