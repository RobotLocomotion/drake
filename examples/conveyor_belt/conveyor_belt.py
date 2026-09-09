"""Simulates objects moving on a loop of conveyor belts."""

import argparse

from pydrake.common.value import Value
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.tree import BodyIndex
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import BusValue, DiagramBuilder
from pydrake.systems.primitives import ConstantValueSource
from pydrake.visualization import AddDefaultVisualization


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--sim_time_step",
        type=float,
        default=1e-2,
        help="Simulation time step used for the integrator.",
    )
    parser.add_argument(
        "--simulation_time",
        type=float,
        default=100.0,
        help="Duration of the simulation in seconds.",
    )
    parser.add_argument(
        "--realtime_rate",
        "--target_realtime_rate",
        dest="realtime_rate",
        type=float,
        default=1.0,
        help="Target realtime rate.",
    )
    parser.add_argument(
        "--belt_speed",
        type=float,
        default=0.5,
        help=(
            "Surface speed (m/s) commanded for every body that declares a "
            "surface velocity axis."
        ),
    )
    args = parser.parse_args()

    builder = DiagramBuilder()
    plant, _ = AddMultibodyPlantSceneGraph(
        builder=builder, time_step=args.sim_time_step
    )
    Parser(builder=builder).AddModelsFromUrl(
        url=("package://drake/examples/conveyor_belt/conveyor_example.dmd.yaml")
    )
    plant.Finalize()

    # Command a constant surface speed for every body that declared a surface
    # velocity axis. Each signal on the "surface_speeds" bus is keyed by the
    # body's scoped name.
    speeds = BusValue()
    for i in range(plant.num_bodies()):
        body = plant.get_body(BodyIndex(i))
        if plant.GetSurfaceVelocityAxis(body) is not None:
            speeds.Set(
                body.scoped_name().to_string(), Value[float](args.belt_speed)
            )
    speed_source = builder.AddSystem(
        ConstantValueSource(Value[BusValue](speeds))
    )
    builder.Connect(
        speed_source.get_output_port(), plant.get_surface_speeds_input_port()
    )

    AddDefaultVisualization(builder=builder)
    diagram = builder.Build()

    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(args.realtime_rate)
    simulator.Initialize()
    simulator.AdvanceTo(args.simulation_time)


if __name__ == "__main__":
    main()
