import math
import unittest

import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.examples.pendulum import PendulumPlant
from pydrake.multibody.tree import MultibodyForces
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.parsing import Parser
from pydrake.systems.analysis import Simulator
from pydrake.systems.controllers import (
    DiscreteTimeLinearQuadraticRegulator,
    DynamicProgrammingOptions,
    FiniteHorizonLinearQuadraticRegulator,
    FiniteHorizonLinearQuadraticRegulatorOptions,
    FiniteHorizonLinearQuadraticRegulatorResult,
    FittedValueIteration,
    InverseDynamicsController,
    InverseDynamics,
    LinearQuadraticRegulator,
    LinearProgrammingApproximateDynamicProgramming,
    MakeFiniteHorizonLinearQuadraticRegulator,
    PeriodicBoundaryCondition,
    PidControlledSystem,
    PidController,
)
from pydrake.systems.framework import DiagramBuilder, InputPortSelection
from pydrake.systems.primitives import Integrator, LinearSystem
from pydrake.trajectories import Trajectory


class TestControllers(unittest.TestCase):
    def test_fitted_value_iteration_pendulum(self):
        plant = PendulumPlant()
        simulator = Simulator(plant)

        def quadratic_regulator_cost(context):
            x = context.get_continuous_state_vector().CopyToVector()
            x[0] = x[0] - math.pi
            u = plant.EvalVectorInput(context, 0).CopyToVector()
            return x.dot(x) + u.dot(u)

        # Note: intentionally under-sampled to keep the problem small
        qbins = np.linspace(0., 2.*math.pi, 11)
        qdotbins = np.linspace(-10., 10., 11)
        state_grid = [set(qbins), set(qdotbins)]

        input_limit = 2.
        input_mesh = [set(np.linspace(-input_limit, input_limit, 5))]
        timestep = 0.01

        num_callbacks = [0]

        def callback(iteration, mesh, cost_to_go, policy):
            # Drawing is slow, don't draw every frame.
            num_callbacks[0] += 1

        options = DynamicProgrammingOptions()
        options.convergence_tol = 1.
        options.periodic_boundary_conditions = [
            PeriodicBoundaryCondition(0, 0., 2.*math.pi)
        ]
        options.visualization_callback = callback
        options.input_port_index = InputPortSelection.kUseFirstInputIfItExists
        options.assume_non_continuous_states_are_fixed = False

        policy, cost_to_go = FittedValueIteration(simulator,
                                                  quadratic_regulator_cost,
                                                  state_grid, input_mesh,
                                                  timestep, options)

        self.assertGreater(num_callbacks[0], 0)

    def test_linear_programming_approximate_dynamic_programming(self):
        integrator = Integrator(1)
        simulator = Simulator(integrator)

        # minimum time cost function (1 for all non-zero states).
        def cost_function(context):
            x = context.get_continuous_state_vector().CopyToVector()
            if (math.fabs(x[0]) > 0.1):
                return 1.
            else:
                return 0.

        def cost_to_go_function(state, parameters):
            return parameters[0] * math.fabs(state[0])

        state_samples = np.array([[-4., -3., -2., -1., 0., 1., 2., 3., 4.]])
        input_samples = np.array([[-1., 0., 1.]])

        timestep = 1.0
        options = DynamicProgrammingOptions()
        options.discount_factor = 1.

        J = LinearProgrammingApproximateDynamicProgramming(
            simulator, cost_function, cost_to_go_function, 1,
            state_samples, input_samples, timestep, options)

        self.assertAlmostEqual(J[0], 1., delta=1e-6)

    def test_inverse_dynamics(self):
        sdf_path = FindResourceOrThrow(
            "drake/manipulation/models/"
            "iiwa_description/sdf/iiwa14_no_collision.sdf")

        plant = MultibodyPlant(time_step=0.01)
        Parser(plant).AddModelFromFile(sdf_path)
        plant.WeldFrames(plant.world_frame(),
                         plant.GetFrameByName("iiwa_link_0"))
        plant.Finalize()

        # Just test that the constructor doesn't throw.
        controller = InverseDynamics(
            plant=plant,
            mode=InverseDynamics.InverseDynamicsMode.kGravityCompensation)

    def test_inverse_dynamics_controller(self):
        sdf_path = FindResourceOrThrow(
            "drake/manipulation/models/"
            "iiwa_description/sdf/iiwa14_no_collision.sdf")

        plant = MultibodyPlant(time_step=0.01)
        Parser(plant).AddModelFromFile(sdf_path)
        plant.WeldFrames(plant.world_frame(),
                         plant.GetFrameByName("iiwa_link_0"))
        plant.mutable_gravity_field().set_gravity_vector([0.0, 0.0, 0.0])
        plant.Finalize()

        # We verify the (known) size of the model.
        kNumPositions = 7
        kNumVelocities = 7
        kNumActuators = 7
        kStateSize = kNumPositions + kNumVelocities
        self.assertEqual(plant.num_positions(), kNumPositions)
        self.assertEqual(plant.num_velocities(), kNumVelocities)
        self.assertEqual(plant.num_actuators(), kNumActuators)

        kp = np.array([1., 2., 3., 4., 5., 6., 7.])
        ki = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])
        kd = np.array([.5, 1., 1.5, 2., 2.5, 3., 3.5])

        controller = InverseDynamicsController(robot=plant,
                                               kp=kp,
                                               ki=ki,
                                               kd=kd,
                                               has_reference_acceleration=True)
        context = controller.CreateDefaultContext()
        output = controller.AllocateOutput()

        estimated_state_port = controller.get_input_port(0)
        desired_state_port = controller.get_input_port(1)
        desired_acceleration_port = controller.get_input_port(2)
        control_port = controller.get_output_port(0)

        self.assertEqual(desired_acceleration_port.size(), kNumVelocities)
        self.assertEqual(estimated_state_port.size(), kStateSize)
        self.assertEqual(desired_state_port.size(), kStateSize)
        self.assertEqual(control_port.size(), kNumVelocities)

        # Current state.
        q = np.array([-0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3])
        v = np.array([-0.9, -0.6, -0.3, 0.0, 0.3, 0.6, 0.9])
        x = np.concatenate([q, v])

        # Reference state and acceleration.
        q_r = q + 0.1*np.ones_like(q)
        v_r = v + 0.1*np.ones_like(v)
        x_r = np.concatenate([q_r, v_r])
        vd_r = np.array([1., 2., 3., 4., 5., 6., 7.])

        integral_term = np.array([-1., -2., -3., -4., -5., -6., -7.])

        vd_d = vd_r + kp*(q_r-q) + kd*(v_r-v) + ki*integral_term

        estimated_state_port.FixValue(context, x)
        desired_state_port.FixValue(context, x_r)
        desired_acceleration_port.FixValue(context, vd_r)
        controller.set_integral_value(context, integral_term)

        # Set the plant's context.
        plant_context = plant.CreateDefaultContext()
        x_plant = plant.GetMutablePositionsAndVelocities(plant_context)
        x_plant[:] = x

        # Compute the expected value of the generalized forces using
        # inverse dynamics.
        tau_id = plant.CalcInverseDynamics(
            plant_context, vd_d, MultibodyForces(plant))

        # Verify the result.
        controller.CalcOutput(context, output)
        self.assertTrue(np.allclose(output.get_vector_data(0).CopyToVector(),
                                    tau_id))

    def test_issue14355(self):
        """
        DiagramBuilder.AddSystem() may not propagate keep alive relationships.
        We use this test to show resolution at a known concrete point of
        failure.
        https://github.com/RobotLocomotion/drake/issues/14355
        """

        def make_diagram():
            # Use a nested function to ensure that all locals get garbage
            # collected quickly.

            # Construct a trivial plant and ID controller.
            # N.B. We explicitly do *not* add this plant to the diagram.
            controller_plant = MultibodyPlant(time_step=0.002)
            controller_plant.Finalize()
            builder = DiagramBuilder()
            controller = builder.AddSystem(
                InverseDynamicsController(
                    controller_plant,
                    kp=[],
                    ki=[],
                    kd=[],
                    has_reference_acceleration=False,
                )
            )
            # Forward ports for ease of testing.
            builder.ExportInput(
                controller.get_input_port_estimated_state(), "x_estimated")
            builder.ExportInput(
                controller.get_input_port_desired_state(), "x_desired")
            builder.ExportOutput(controller.get_output_port_control(), "u")
            diagram = builder.Build()
            return diagram

        diagram = make_diagram()
        # N.B. Without the workaround for #14355, we get a segfault when
        # creating the context.
        context = diagram.CreateDefaultContext()
        diagram.GetInputPort("x_estimated").FixValue(context, [])
        diagram.GetInputPort("x_desired").FixValue(context, [])
        u = diagram.GetOutputPort("u").Eval(context)
        np.testing.assert_equal(u, [])

    def test_pid_controlled_system(self):
        controllers = [
            PidControlledSystem(plant=PendulumPlant(), kp=1., ki=0.,
                                kd=2., state_output_port_index=0,
                                plant_input_port_index=0),
            PidControlledSystem(plant=PendulumPlant(), kp=[0], ki=[1],
                                kd=[2], state_output_port_index=0,
                                plant_input_port_index=0),
            PidControlledSystem(plant=PendulumPlant(),
                                feedback_selector=np.eye(2), kp=1.,
                                ki=0., kd=2.,
                                state_output_port_index=0,
                                plant_input_port_index=0),
            PidControlledSystem(plant=PendulumPlant(),
                                feedback_selector=np.eye(2),
                                kp=[0], ki=[1], kd=[2],
                                state_output_port_index=0,
                                plant_input_port_index=0),
        ]

        for controller in controllers:
            self.assertIsNotNone(controller.get_control_input_port())
            self.assertIsNotNone(controller.get_state_input_port())
            self.assertIsNotNone(controller.get_state_output_port())

    def test_pid_controller(self):
        controllers = [
            PidController(kp=np.ones(3), ki=np.zeros(3),
                          kd=[1, 2, 3]),
            PidController(state_projection=np.ones((6, 4)),
                          kp=np.ones(3), ki=np.zeros(3),
                          kd=[1, 2, 3]),
            PidController(state_projection=np.ones((6, 4)),
                          output_projection=np.ones((4, 3)),
                          kp=np.ones(3), ki=np.zeros(3),
                          kd=[1, 2, 3]),
        ]

        for controller in controllers:
            self.assertEqual(controller.num_input_ports(), 2)
            self.assertEqual(len(controller.get_Kp_vector()), 3)
            self.assertEqual(len(controller.get_Ki_vector()), 3)
            self.assertEqual(len(controller.get_Kd_vector()), 3)
            self.assertIsNotNone(controller.get_input_port_estimated_state())
            self.assertIsNotNone(controller.get_input_port_desired_state())
            self.assertIsNotNone(controller.get_output_port_control())

    def test_linear_quadratic_regulator(self):
        A = np.array([[0, 1], [0, 0]])
        B = np.array([[0], [1]])
        C = np.identity(2)
        D = np.array([[0], [0]])
        double_integrator = LinearSystem(A, B, C, D)

        Q = np.identity(2)
        R = np.identity(1)
        K_expected = np.array([[1, math.sqrt(3.)]])
        S_expected = np.array([[math.sqrt(3), 1.], [1., math.sqrt(3)]])

        (K, S) = LinearQuadraticRegulator(A, B, Q, R)
        np.testing.assert_almost_equal(K, K_expected)
        np.testing.assert_almost_equal(S, S_expected)

        controller = LinearQuadraticRegulator(double_integrator, Q, R)
        np.testing.assert_almost_equal(controller.D(), -K_expected)

        context = double_integrator.CreateDefaultContext()
        double_integrator.get_input_port(0).FixValue(context, [0])
        controller = LinearQuadraticRegulator(double_integrator, context, Q, R)
        np.testing.assert_almost_equal(controller.D(), -K_expected)

    def test_discrete_time_linear_quadratic_regulator(self):
        A = np.array([[1, 1], [0, 1]])
        B = np.array([[0], [1]])
        Q = np.identity(2)
        R = np.identity(1)
        (K, S) = DiscreteTimeLinearQuadraticRegulator(A, B, Q, R)
        self.assertEqual(K.shape, (1, 2))
        self.assertEqual(S.shape, (2, 2))

    def test_finite_horizon_linear_quadratic_regulator(self):
        A = np.array([[0, 1], [0, 0]])
        B = np.array([[0], [1]])
        C = np.identity(2)
        D = np.array([[0], [0]])
        double_integrator = LinearSystem(A, B, C, D)

        Q = np.identity(2)
        R = np.identity(1)

        options = FiniteHorizonLinearQuadraticRegulatorOptions()
        options.Qf = Q
        self.assertIsNone(options.N)
        self.assertIsNone(options.x0)
        self.assertIsNone(options.u0)
        self.assertIsNone(options.xd)
        self.assertIsNone(options.ud)
        self.assertEqual(options.input_port_index,
                         InputPortSelection.kUseFirstInputIfItExists)
        self.assertRegex(repr(options), "".join([
            r"FiniteHorizonLinearQuadraticRegulatorOptions\(",
            # Don't be particular about numpy's whitespace in Qf.
            r"Qf=\[\[ *1\. *0\.\]\s*\[ *0\. *1\.\]\], "
            r"N=None, ",
            r"input_port_index=",
            r"InputPortSelection.kUseFirstInputIfItExists\)"]))

        context = double_integrator.CreateDefaultContext()
        double_integrator.get_input_port(0).FixValue(context, 0.0)

        result = FiniteHorizonLinearQuadraticRegulator(
            system=double_integrator,
            context=context,
            t0=0,
            tf=0.1,
            Q=Q,
            R=R,
            options=options)

        self.assertIsInstance(result,
                              FiniteHorizonLinearQuadraticRegulatorResult)

        self.assertIsInstance(result.x0, Trajectory)
        self.assertEqual(result.x0.value(0).shape, (2, 1))
        self.assertIsInstance(result.u0, Trajectory)
        self.assertEqual(result.u0.value(0).shape, (1, 1))
        self.assertIsInstance(result.K, Trajectory)
        self.assertEqual(result.K.value(0).shape, (1, 2))
        self.assertIsInstance(result.S, Trajectory)
        self.assertEqual(result.S.value(0).shape, (2, 2))
        self.assertIsInstance(result.k0, Trajectory)
        self.assertEqual(result.k0.value(0).shape, (1, 1))
        self.assertIsInstance(result.sx, Trajectory)
        self.assertEqual(result.sx.value(0).shape, (2, 1))
        self.assertIsInstance(result.s0, Trajectory)
        self.assertEqual(result.s0.value(0).shape, (1, 1))

        regulator = MakeFiniteHorizonLinearQuadraticRegulator(
            system=double_integrator,
            context=context,
            t0=0,
            tf=0.1,
            Q=Q,
            R=R,
            options=options)
        self.assertEqual(regulator.get_input_port(0).size(), 2)
        self.assertEqual(regulator.get_output_port(0).size(), 1)
