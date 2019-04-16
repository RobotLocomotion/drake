# -*- coding: utf-8 -*-

from __future__ import print_function

import copy
import time
import unittest
import warnings

import numpy as np

from pydrake.common import (
    RandomGenerator,
    FindResourceOrThrow
)
from pydrake.examples.pendulum import PendulumPlant
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


class TestMonteCarlo(unittest.TestCase):
    def test_minimal_simulation(self):
        # Create a simple system.
        system = ConstantVectorSource([1.])

        def make_simulator(generator):
            simulator = Simulator(system)
            simulator.Initialize()
            simulator.set_target_realtime_rate(0)
            return simulator

        def calc_output(system, context):
            return 42.

        result = RandomSimulation(
            make_simulator=make_simulator, output=calc_output,
            final_time=1.0, generator=RandomGenerator())

        self.assertEqual(result, 42.)

        result = MonteCarloSimulation(
            make_simulator=make_simulator, output=calc_output,
            final_time=1.0, num_samples=10, generator=RandomGenerator())
        self.assertIsInstance(result, list)
        self.assertEqual(len(result), 10)
        self.assertIsInstance(result[0], RandomSimulationResult)
        self.assertIsInstance(result[0].generator_snapshot, RandomGenerator)
        self.assertEqual(result[0].output, 42.)
        for i in range(1, len(result)):
            self.assertNotEqual(result[0].generator_snapshot,
                                result[i].generator_snapshot)

    def test_pendulum(self):
        # This tests the Monte Carlo method's ability to estimate
        # expected LQR performance balancing a torque-limited single
        # pendulum by comparing the estimated performance across
        # a number of samples to the analytical expected performance.

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
        torque_limit = 1.0
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
            simulator = Simulator(diagram)
            simulator.set_target_realtime_rate(0)
            simulator.Initialize()
            return simulator

        def calc_wrapped_error(system, context):
            state = diagram.GetSubsystemContext(
                pendulum, context).get_continuous_state_vector()
            error = state.GetAtIndex(0) - upright_theta + np.pi
            # Wrap error to [-pi, pi].
            return (error) % (2*np.pi) - np.pi

        results = MonteCarloSimulation(
            make_simulator=make_simulator, output=calc_wrapped_error,
            final_time=1.0, num_samples=50, generator=RandomGenerator())

        # The acceptably region is fairly large since some "stabilized" trials
        # may still be oscillating around the fixed point. Failed examples are
        # consistently much farther from the fixed point than this.
        binary_results = np.array([abs(res.output) < 0.05 for res in results])
        passing_ratio = float(sum(binary_results)) / len(results)
        # 95% confidence interval for the passing ratio.
        passing_ratio_var = 1.96 * np.sqrt(
            passing_ratio*(1. - passing_ratio)/len(results))

        # Calculate where the torque exceeds the torque limit to find
        # the expected ROA.
        arm_radius = 0.5
        arm_mass = 1.0
        # torque = r x f = r * (m * 9.81 * sin(theta))
        # theta = asin(torque / (r * m))
        roa_half_width = np.arcsin(torque_limit /
                                   (arm_radius * arm_mass * 9.81))
        roa_as_fraction_of_state_space = roa_half_width / np.pi

        self.assertTrue(passing_ratio - passing_ratio_var <=
                        roa_as_fraction_of_state_space <=
                        passing_ratio + passing_ratio_var)
