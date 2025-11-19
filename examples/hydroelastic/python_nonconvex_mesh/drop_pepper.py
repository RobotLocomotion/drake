"""
This is an example of using hydroelastic contact model through pydrake with
non-convex meshes. It reads SDFormat files of:
- a non-convex mesh of a yellow bell pepper with compliant-hydroelastic
  properties,
- a non-convex mesh of a bowl with rigid-hydroelastic properties,
- and a table top (anchored to the World) represented as a box primitive with
  compliant-hydroelastic properties.
"""

import argparse

import numpy as np

from pydrake.math import RigidTransform
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlant, MultibodyPlantConfig
from pydrake.systems.analysis import (
    ApplySimulatorConfig,
    PrintSimulatorStatistics,
    Simulator,
    SimulatorConfig,
)
from pydrake.systems.framework import DiagramBuilder
from pydrake.visualization import AddDefaultVisualization


def make_pepper_bowl_table(contact_model, time_step):
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlant(
        MultibodyPlantConfig(
            time_step=time_step,
            contact_model=contact_model,
            contact_surface_representation="polygon",
        ),
        builder,
    )

    parser = Parser(builder)
    parser.AddModels(
        url="package://drake_models/veggies/yellow_bell_pepper_no_stem_low.sdf"
    )
    parser.AddModels(url="package://drake_models/dishes/evo_bowl.sdf")
    parser.AddModels(
        url="package://drake/examples/hydroelastic/python_nonconvex_mesh/"
        "table.sdf"
    )

    # We pose the table with its top surface on World's X-Y plane.
    # Intuitively we push it down 1 cm because the box is 2 cm thick.
    p_WTable_fixed = RigidTransform(np.array([0, 0, -0.01]))
    plant.WeldFrames(
        frame_on_parent_F=plant.world_frame(),
        frame_on_child_M=plant.GetFrameByName("table"),
        X_FM=p_WTable_fixed,
    )
    plant.Finalize()

    AddDefaultVisualization(builder=builder)
    diagram = builder.Build()
    return diagram, plant


def simulate_diagram(
    diagram,
    plant,
    pepper_position,
    pepper_wz,
    bowl_position,
    simulation_time,
    target_realtime_rate,
):
    simulator = Simulator(diagram)
    ApplySimulatorConfig(
        SimulatorConfig(
            target_realtime_rate=target_realtime_rate,
            publish_every_time_step=True,
        ),
        simulator,
    )

    q_init_val = np.hstack(
        (
            np.array([1, 0, 0, 0]),
            pepper_position,
            np.array([1, 0, 0, 0]),
            bowl_position,
        )
    )
    v_init_val = np.hstack(
        (np.array([0, 0, pepper_wz]), np.zeros(3), np.zeros(3), np.zeros(3))
    )
    plant.SetPositionsAndVelocities(
        diagram.GetSubsystemContext(plant, simulator.get_context()),
        np.concatenate((q_init_val, v_init_val)),
    )

    simulator.get_mutable_context().SetTime(0)
    simulator.Initialize()
    simulator.AdvanceTo(boundary_time=simulation_time)
    PrintSimulatorStatistics(simulator)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--simulation_time",
        type=float,
        default=2,
        help="Desired duration of the simulation in seconds. "
        "Default %(default)s.",
    )
    parser.add_argument(
        "--contact_model",
        type=str,
        default="hydroelastic_with_fallback",
        help="Contact model. Options are: 'point', 'hydroelastic', "
        "'hydroelastic_with_fallback'. Default %(default)s.",
    )
    parser.add_argument(
        "--time_step",
        type=float,
        default=0.01,
        help="The fixed time step period (in seconds) of discrete updates "
        "for the multibody plant modeled as a discrete system. "
        "Strictly positive. Default %(default)s.",
    )
    parser.add_argument(
        "--pepper_position",
        nargs=3,
        metavar=("x", "y", "z"),
        default=[0, -0.15, 0.10],
        help="Pepper's initial position of the bottom of the pepper: "
        "x, y, z (in meters) in World frame. Default %(default)s.",
    )
    parser.add_argument(
        "--pepper_wz",
        type=float,
        default=150,
        help="Pepper's initial angular velocity in the z-axis in rad/s. "
        "Default %(default)s.",
    )
    parser.add_argument(
        "--bowl_position",
        nargs=3,
        metavar=("x", "y", "z"),
        default=[0, -0.07, 0.061],
        help="Bowl's initial position of its center: "
        "x, y, z (in meters) in World frame. Default %(default)s.",
    )
    parser.add_argument(
        "--target_realtime_rate",
        type=float,
        default=1.0,
        help="Target realtime rate. Set to 0 to run as fast as it can. "
        "Default %(default)s.",
    )
    args = parser.parse_args()

    diagram, plant = make_pepper_bowl_table(args.contact_model, args.time_step)
    simulate_diagram(
        diagram,
        plant,
        np.array(args.pepper_position),
        args.pepper_wz,
        np.array(args.bowl_position),
        args.simulation_time,
        args.target_realtime_rate,
    )
