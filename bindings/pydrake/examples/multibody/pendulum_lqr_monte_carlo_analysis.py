# -*- coding: utf-8 -*-

from __future__ import print_function

import argparse

import numpy as np

from pydrake.common import (
    RandomGenerator,
    FindResourceOrThrow
)
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import UniformGravityFieldElement
from pydrake.symbolic import (
    Expression,
    Variable
)
from pydrake.systems.analysis import (
    MonteCarloSimulation,
    RandomSimulationResult,
    RandomSimulation,
    Simulator
)
from pydrake.systems.controllers import LinearQuadraticRegulator
from pydrake.systems.framework import (
    Context,
    Diagram,
    DiagramBuilder
)
from pydrake.systems.primitives import (
    ConstantVectorSource,
    Linearize,
    Saturation
)

if __name__ == "__main__":
    # This example demonstrates using Monte Carlo verification to analyze
    # task performance, for the task of stabilizing an inverted torque-limited
    # pendulum with LQR.

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--num_samples", type=int, default=50,
        help="Number of Monte Carlo samples to use to estimate performance.")
    parser.add_argument(
        "--torque_limit", type=float, default=1.0,
        help="Torque limit of the pendulum.")
    args = parser.parse_args()

    # Assemble the Pendulum plant.
    builder = DiagramBuilder()
    pendulum = builder.AddSystem(MultibodyPlant())
    file_name = FindResourceOrThrow(
        "drake/examples/pendulum/Pendulum.urdf")
    Parser(pendulum).AddModelFromFile(file_name)
    pendulum.AddForceElement(UniformGravityFieldElement())
    pendulum.WeldFrames(pendulum.world_frame(),
                        pendulum.GetFrameByName("base_part2"))
    pendulum.Finalize()

    # Set the pendulum to start at uniformly random
    # positions (but always zero velocity).
    elbow = pendulum.GetMutableJointByName("theta")
    upright_theta = np.pi
    theta_expression = Variable(
        name="theta",
        type=Variable.Type.RANDOM_UNIFORM)*2*np.pi
    elbow.set_random_angle_distribution(theta_expression)

    # Set up LQR, with high position gains to ensure the
    # ROA is close to the theoretical torque-limited limit.
    Q = np.identity(2)*10.
    R = np.identity(1)
    linearize_context = pendulum.CreateDefaultContext()
    linearize_context.SetContinuousState(
        np.array([upright_theta, 0.]))
    actuation_port_index = pendulum.get_actuation_input_port().get_index()
    linearize_context.FixInputPort(
        actuation_port_index, np.zeros(1))
    controller = builder.AddSystem(
        LinearQuadraticRegulator(
            pendulum, linearize_context, Q, R,
            np.zeros(0), actuation_port_index))

    # Apply the torque limit.
    torque_limit = args.torque_limit
    torque_limiter = builder.AddSystem(
        Saturation(min_value=np.array([-torque_limit]),
                   max_value=np.array([torque_limit])))

    builder.Connect(controller.get_output_port(0),
                    torque_limiter.get_input_port(0))
    builder.Connect(torque_limiter.get_output_port(0),
                    pendulum.get_actuation_input_port())
    builder.Connect(pendulum.get_output_port(0),
                    controller.get_input_port(0))
    diagram = builder.Build()

    # Perform the Monte Carlo simulation.
    def make_simulator(generator):
        ''' Create a simulator for the system
            using the given generator. '''
        simulator = Simulator(diagram)
        simulator.set_target_realtime_rate(0)
        simulator.Initialize()
        return simulator

    def calc_wrapped_error(system, context):
        ''' Given a context from the end of the simulation,
            calculate an error -- which for this stabilizing
            task is the distance from the
            fixed point. '''
        state = diagram.GetSubsystemContext(
            pendulum, context).get_continuous_state_vector()
        error = state.GetAtIndex(0) - upright_theta + np.pi
        # Wrap error to [-pi, pi].
        return (error) % (2*np.pi) - np.pi

    num_samples = args.num_samples
    results = MonteCarloSimulation(
        make_simulator=make_simulator, output=calc_wrapped_error,
        final_time=1.0, num_samples=num_samples, generator=RandomGenerator())

    # Compute results.
    # The acceptably region is fairly large since some "stabilized" trials
    # may still be oscillating around the fixed point. Failed examples are
    # consistently much farther from the fixed point than this.
    binary_results = np.array([abs(res.output) < 0.05 for res in results])
    passing_ratio = float(sum(binary_results)) / len(results)
    # 95% confidence interval for the passing ratio.
    passing_ratio_var = 1.96 * np.sqrt(
        passing_ratio*(1. - passing_ratio)/len(results))

    print("Monte-Carlo estimated performance across %d samples: %f +/- %f" % (
        num_samples, passing_ratio, passing_ratio_var))

    # Analytically compute the best possible ROA, for comparison, but
    # calculating where the torque needed to lift the pendulum exceeds
    # the torque limit.
    arm_radius = 0.5
    arm_mass = 1.0
    # torque = r x f = r * (m * 9.81 * sin(theta))
    # theta = asin(torque / (r * m))
    roa_half_width = np.arcsin(torque_limit /
                               (arm_radius * arm_mass * 9.81))
    print("Max possible ROA: %f / 2*pi = %f%% of state space, which should"
          " match with the above estimate." % (
            roa_half_width*2, roa_half_width / np.pi))
