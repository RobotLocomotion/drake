import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.geometry import DrakeVisualizer
from pydrake.geometry import DrakeVisualizerParams
from pydrake.math import RigidTransform
from pydrake.math import RollPitchYaw
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.plant import ConnectContactResultsToDrakeVisualizer
from pydrake.systems.analysis import Simulator
from pydrake.systems.analysis import PrintSimulatorStatistics
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import VectorLogSink


def make_ball_paddle():
    dt = 0.001
    p_WPaddle_fixed = RigidTransform(RollPitchYaw(0, 0, 0),
                                     np.array([0.1, 0, -0.01]))
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, dt)

    parser = Parser(plant)
    paddle_sdf_file_name = \
        FindResourceOrThrow("drake/examples/ball_paddle/paddle.sdf")
    paddle = parser.AddModelFromFile(paddle_sdf_file_name, model_name="paddle")
    plant.WeldFrames(
        frame_on_parent_P=plant.world_frame(),
        frame_on_child_C=plant.GetFrameByName("paddle", paddle),
        X_PC=p_WPaddle_fixed
    )

    ball_sdf_file_name = \
        FindResourceOrThrow("drake/examples/ball_paddle/ball.sdf")
    parser.AddModelFromFile(ball_sdf_file_name)

    plant.Finalize()

    # TODO(DamrongGuoy) Figure out why we need to set publish period to dt.
    #  Otherwise, the animation looks very lagging.
    drake_visualizer_params = DrakeVisualizerParams()
    drake_visualizer_params.publish_period = dt
    DrakeVisualizer.AddToBuilder(builder=builder, scene_graph=scene_graph,
                                 params=drake_visualizer_params)
    ConnectContactResultsToDrakeVisualizer(builder=builder, plant=plant,
                                           scene_graph=scene_graph,
                                           publish_period=dt)

    nx = plant.num_positions() + plant.num_velocities()
    state_logger = builder.AddSystem(VectorLogSink(nx, dt))
    builder.Connect(plant.get_state_output_port(),
                    state_logger.get_input_port())

    diagram = builder.Build()
    return diagram, plant, state_logger


def simulate_diagram(diagram, ball_paddle_plant, state_logger,
                     ball_init_position,
                     ball_init_velocity):
    q_init_val = np.array([
        1, 0, 0, 0, ball_init_position[0], ball_init_position[1],
        ball_init_position[2]
    ])
    v_init_val = np.hstack((np.zeros(3), ball_init_velocity))
    qv_init_val = np.concatenate((q_init_val, v_init_val))
    simulator = Simulator(diagram)

    plant_context = diagram.GetSubsystemContext(ball_paddle_plant,
                                                simulator.get_context())
    ball_paddle_plant.SetPositionsAndVelocities(plant_context,
                                                qv_init_val)
    simulator.get_mutable_context().SetTime(0)
    state_log = state_logger.FindMutableLog(simulator.get_mutable_context())
    state_log.Clear()
    target_realtime_rate = 0.02
    simulator.set_target_realtime_rate(target_realtime_rate)
    simulator.Initialize()
    simulator.AdvanceTo(boundary_time=0.1)
    print()
    PrintSimulatorStatistics(simulator)
    return state_log.sample_times(), state_log.data()


if __name__ == "__main__":
    diagram, ball_paddle_plant, state_logger = make_ball_paddle()
    time_samples, state_samples = simulate_diagram(
        diagram, ball_paddle_plant, state_logger, np.array([-5E-4, 0, 0.05]),
        np.array([0., 0., -np.sqrt(2 * 9.81 * 0.95)]))
    print("\nFinal state variables:")
    print(state_samples[:, -1])
    pass
