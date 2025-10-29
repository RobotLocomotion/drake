"""
This is an example for using hydroelastic contact model through pydrake.
It reads two simple SDFormat files of a compliant hydroelastic ball and
a compliant hydroelastic paddle.
The ball is dropped on an edge of the paddle and bounces off.
"""

import argparse

import numpy as np

from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlant, MultibodyPlantConfig
from pydrake.systems.analysis import (
    ApplySimulatorConfig,
    PrintSimulatorStatistics,
    Simulator,
    SimulatorConfig,
)
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import VectorLogSink
from pydrake.visualization import AddDefaultVisualization


def make_ball_paddle(contact_model, contact_surface_representation, time_step):
    multibody_plant_config = MultibodyPlantConfig(
        time_step=time_step,
        contact_model=contact_model,
        contact_surface_representation=contact_surface_representation,
    )
    # We pose the paddle, so that its top surface is on World's X-Y plane.
    # Intuitively we push it down 1 cm because the box is 2 cm thick.
    p_WPaddle_fixed = RigidTransform(
        RollPitchYaw(0, 0, 0), np.array([0, 0, -0.01])
    )
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlant(multibody_plant_config, builder)

    parser = Parser(builder)
    paddle_sdf_url = (
        "package://drake/examples/hydroelastic/python_ball_paddle/paddle.sdf"
    )
    (paddle,) = parser.AddModels(url=paddle_sdf_url)
    plant.WeldFrames(
        frame_on_parent_F=plant.world_frame(),
        frame_on_child_M=plant.GetFrameByName("paddle", paddle),
        X_FM=p_WPaddle_fixed,
    )
    ball_sdf_url = (
        "package://drake/examples/hydroelastic/python_ball_paddle/ball.sdf"
    )
    parser.AddModels(url=ball_sdf_url)
    # TODO(DamrongGuoy): Let users override hydroelastic modulus, dissipation,
    #  and resolution hint from the two SDF files above.

    plant.Finalize()

    AddDefaultVisualization(builder=builder)

    nx = plant.num_positions() + plant.num_velocities()
    state_logger = builder.AddSystem(VectorLogSink(nx))
    builder.Connect(
        plant.get_state_output_port(), state_logger.get_input_port()
    )

    diagram = builder.Build()
    return diagram, plant, state_logger


def simulate_diagram(
    diagram,
    ball_paddle_plant,
    state_logger,
    ball_init_position,
    ball_init_velocity,
    simulation_time,
    target_realtime_rate,
):
    q_init_val = np.hstack(
        (
            np.array([1, 0, 0, 0]),
            ball_init_position,
        )
    )
    v_init_val = np.hstack((np.zeros(3), ball_init_velocity))
    qv_init_val = np.concatenate((q_init_val, v_init_val))

    simulator_config = SimulatorConfig(
        target_realtime_rate=target_realtime_rate, publish_every_time_step=True
    )
    simulator = Simulator(diagram)
    ApplySimulatorConfig(simulator_config, simulator)

    plant_context = diagram.GetSubsystemContext(
        ball_paddle_plant, simulator.get_context()
    )
    ball_paddle_plant.SetPositionsAndVelocities(plant_context, qv_init_val)
    simulator.get_mutable_context().SetTime(0)
    state_log = state_logger.FindMutableLog(simulator.get_mutable_context())
    state_log.Clear()
    simulator.Initialize()
    simulator.AdvanceTo(boundary_time=simulation_time)
    PrintSimulatorStatistics(simulator)
    return state_log.sample_times(), state_log.data()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--simulation_time",
        type=float,
        default=0.5,
        help="Desired duration of the simulation in seconds. Default 0.5.",
    )
    parser.add_argument(
        "--contact_model",
        type=str,
        default="hydroelastic_with_fallback",
        help="Contact model. Options are: 'point', 'hydroelastic', "
        "'hydroelastic_with_fallback'. "
        "Default 'hydroelastic_with_fallback'",
    )
    parser.add_argument(
        "--contact_surface_representation",
        type=str,
        default="polygon",
        help="Contact-surface representation for hydroelastics. "
        "Options are: 'triangle' or 'polygon'. Default 'polygon'.",
    )
    parser.add_argument(
        "--time_step",
        type=float,
        default=0.001,
        help="The fixed time step period (in seconds) of discrete updates "
        "for the multibody plant modeled as a discrete system. "
        "If zero, we will use an integrator for a continuous system. "
        "Non-negative. Default 0.001.",
    )
    parser.add_argument(
        "--ball_initial_position",
        nargs=3,
        metavar=("x", "y", "z"),
        default=[0, 0, 0.1],
        help="Ball's initial position: x, y, z (in meters) in World frame. "
        "Default: 0 0 0.1",
    )
    parser.add_argument(
        "--target_realtime_rate",
        type=float,
        default=1.0,
        help="Target realtime rate. Default 1.0.",
    )
    args = parser.parse_args()

    diagram, ball_paddle_plant, state_logger = make_ball_paddle(
        args.contact_model, args.contact_surface_representation, args.time_step
    )
    time_samples, state_samples = simulate_diagram(
        diagram,
        ball_paddle_plant,
        state_logger,
        np.array(args.ball_initial_position),
        np.array([0.0, 0.0, 0.0]),
        args.simulation_time,
        args.target_realtime_rate,
    )
    print("\nFinal state variables:")
    print(state_samples[:, -1])
