"""
This is an example for using hydroelastic contact model through pydrake.
It reads two simple SDFormat files of a compliant hydroelastic pepper and
a compliant hydroelastic table.
The pepper is dropped on the table and bounces off.
"""
import argparse
import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.math import RigidTransform
from pydrake.math import RollPitchYaw
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlant
from pydrake.multibody.plant import MultibodyPlantConfig
from pydrake.systems.analysis import ApplySimulatorConfig
from pydrake.systems.analysis import Simulator
from pydrake.systems.analysis import SimulatorConfig
from pydrake.systems.analysis import PrintSimulatorStatistics
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import VectorLogSink
from pydrake.visualization import AddDefaultVisualization


def make_pepper_table(contact_model, contact_surface_representation,
                      time_step):
    multibody_plant_config = \
        MultibodyPlantConfig(
            time_step=time_step,
            contact_model=contact_model,
            contact_surface_representation=contact_surface_representation)
# We pose the table, so that its top surface is on World's X-Y plane.
    # Intuitively we push it down 1 cm because the box is 2 cm thick.
    p_WTable_fixed = RigidTransform(RollPitchYaw(0, 0, 0),
                                    np.array([0, 0, -0.01]))
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlant(multibody_plant_config, builder)

    parser = Parser(plant)
    table_sdf_file_name = \
        FindResourceOrThrow(
            "drake/examples/hydroelastic/python_nonconvex_compliant_mesh/"
            "table.sdf")
    table = parser.AddModelFromFile(table_sdf_file_name, model_name="table")
    plant.WeldFrames(
        frame_on_parent_F=plant.world_frame(),
        frame_on_child_M=plant.GetFrameByName("table", table),
        X_FM=p_WTable_fixed
    )
    pepper_sdf_file_name = \
        FindResourceOrThrow(
            "drake/examples/hydroelastic/python_nonconvex_compliant_mesh/"
            "pepper.sdf")
    parser.AddModelFromFile(pepper_sdf_file_name)
    # TODO(DamrongGuoy): Let users override hydroelastic modulus, dissipation,
    #  and resolution hint from the two SDF files above.

    plant.Finalize()

    AddDefaultVisualization(builder=builder)

    nx = plant.num_positions() + plant.num_velocities()
    state_logger = builder.AddSystem(VectorLogSink(nx))
    builder.Connect(plant.get_state_output_port(),
                    state_logger.get_input_port())

    diagram = builder.Build()
    return diagram, plant, state_logger


def simulate_diagram(diagram, pepper_table_plant, state_logger,
                     pepper_init_position, pepper_init_velocity,
                     simulation_time, target_realtime_rate):
    q_init_val = np.array([
        1, 0, 0, 0, pepper_init_position[0], pepper_init_position[1],
        pepper_init_position[2]
    ])
    v_init_val = np.hstack((np.zeros(3), pepper_init_velocity))
    qv_init_val = np.concatenate((q_init_val, v_init_val))

    simulator_config = SimulatorConfig(
                           target_realtime_rate=target_realtime_rate,
                           publish_every_time_step=True)
    simulator = Simulator(diagram)
    ApplySimulatorConfig(simulator_config, simulator)

    plant_context = diagram.GetSubsystemContext(pepper_table_plant,
                                                simulator.get_context())
    pepper_table_plant.SetPositionsAndVelocities(plant_context,
                                                 qv_init_val)
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
        "--simulation_time", type=float, default=1.0,
        help="Desired duration of the simulation in seconds. "
             "Default 1.0.")
    parser.add_argument(
        "--contact_model", type=str, default="hydroelastic_with_fallback",
        help="Contact model. Options are: 'point', 'hydroelastic', "
             "'hydroelastic_with_fallback'. "
             "Default 'hydroelastic_with_fallback'")
    parser.add_argument(
        "--contact_surface_representation", type=str, default="polygon",
        help="Contact-surface representation for hydroelastics. "
             "Options are: 'triangle' or 'polygon'. Default 'polygon'.")
    parser.add_argument(
        "--time_step", type=float, default=0.001,
        help="The fixed time step period (in seconds) of discrete updates "
             "for the multibody plant modeled as a discrete system. "
             "If zero, we will use an integrator for a continuous system. "
             "Non-negative. Default 0.001.")
    parser.add_argument(
        "--pepper_initial_position", nargs=3, metavar=('x', 'y', 'z'),
        default=[0, 0, 0.1],
        help="Pepper's initial position: x, y, z (in meters) in World frame. "
             "Default: 0 0 0.1")
    parser.add_argument(
        "--target_realtime_rate", type=float, default=0.1,
        help="Target realtime rate. Default 0.1.")
    args = parser.parse_args()

    diagram, pepper_table_plant, state_logger = make_pepper_table(
        args.contact_model, args.contact_surface_representation,
        args.time_step)
    time_samples, state_samples = simulate_diagram(
        diagram, pepper_table_plant, state_logger,
        np.array(args.pepper_initial_position),
        np.array([0., 0., 0.]),
        args.simulation_time, args.target_realtime_rate)
    print("\nFinal state variables:")
    print(state_samples[:, -1])
