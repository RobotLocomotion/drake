import pydrake.examples.ball_paddle as mut

from pydrake.multibody.plant import (
    MultibodyPlant_, )
from pydrake.autodiffutils import (
    AutoDiffXd,
    initializeAutoDiffGivenGradientMatrix,
)

from pydrake.geometry import (
    SceneGraph_, )

from pydrake.math import (
    RigidTransform,
    RollPitchYaw,
)

import numpy as np
from pydrake.systems.framework import DiagramBuilder_
from pydrake.systems.primitives import (
    VectorLogSink_, )
from pydrake.systems.analysis import Simulator_


def construct_ball_paddle_diagram():
    T = float
    dt = 0.000
    p_WPaddle_fixed = RigidTransform(RollPitchYaw(0, 0, 0),
                                     np.array([0.5, 0, -0.2]))
    ball_paddle = mut.BallPaddle_[T](dt, p_WPaddle_fixed)
    builder = DiagramBuilder_[T]()
    ball_paddle.AddToBuilder(builder)
    nx = ball_paddle.plant().num_positions() + ball_paddle.plant(
    ).num_velocities()
    state_logger = builder.AddSystem(VectorLogSink_[T](nx, dt))
    builder.Connect(ball_paddle.plant().get_state_output_port(),
                    state_logger.get_input_port())
    diagram = builder.Build()
    return diagram, ball_paddle, state_logger


def simulate_diagram(diagram, ball_paddle, state_logger, ball_init_position,
                     ball_init_velocity):
    T = float
    q_init_val = np.array([
        1, 0, 0, 0, ball_init_position[0], ball_init_position[1],
        ball_init_position[2]
    ])
    v_init_val = np.hstack((np.zeros(3), ball_init_velocity))
    qv_init_val = np.concatenate((q_init_val, v_init_val))
    # Take the gradient of the initial ball position.
    q_init_gradient = np.vstack((np.zeros((4, 3)), np.eye(3)))
    q_init_ad = initializeAutoDiffGivenGradientMatrix(q_init_val,
                                                      q_init_gradient)
    v_init_ad = np.array([AutoDiffXd(v_init_val[i]) for i in range(3)])
    qv_init_ad = np.concatenate((q_init_ad, v_init_ad))
    simulator = Simulator_[T](diagram)

    plant_context = diagram.GetSubsystemContext(ball_paddle.plant(),
                                                simulator.get_context())
    if T == AutoDiffXd:
        ball_paddle.plant().SetPositionsAndVelocities(plant_context,
                                                      qv_init_ad)
    elif T == float:
        ball_paddle.plant().SetPositionsAndVelocities(plant_context,
                                                      qv_init_val)
    simulator.get_mutable_context().SetTime(T(0.))
    state_log = state_logger.FindMutableLog(simulator.get_mutable_context())
    state_log.Clear()
    target_realtime_rate = 0.05
    simulator.set_target_realtime_rate(target_realtime_rate)
    simulator.Initialize()
    simulator.AdvanceTo(boundary_time=T(0.2))
    return state_log.sample_times(), state_log.data()


if __name__ == "__main__":
    diagram, ball_paddle, state_logger = construct_ball_paddle_diagram()
    time_samples, state_samples = simulate_diagram(
        diagram, ball_paddle, state_logger, np.array([-5E-4, 0, 0.05]),
        np.array([0., 0., -np.sqrt(2 * 9.81 * 0.95)]))
    print(state_samples[:, -1])
    pass
    #num_samples = 500
    #x_init = np.linspace(-0.04, 0.04, num_samples)
    #state_trajs = [None] * num_samples
    #for i in range(num_samples):
    #    time_samples, state_trajs[i]= simulate_diagram(diagram, ball_paddle, state_logger, np.array([x_init[i], 0., 1.]))
    #pass
