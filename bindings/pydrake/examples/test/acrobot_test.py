import unittest

from pydrake.examples.acrobot import (
    AcrobotGeometry, AcrobotInput, AcrobotParams, AcrobotPlant,
    AcrobotSpongController, AcrobotState, SpongControllerParams
    )
from pydrake.geometry import SceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder


class TestAcrobot(unittest.TestCase):
    def test_input(self):
        input = AcrobotInput()
        input.set_tau(1.)
        self.assertEqual(input.tau(), 1.)

    def test_params(self):
        params = AcrobotParams()
        params.set_m1(1.)
        params.set_m2(2.)
        params.set_l1(3.)
        params.set_Ic1(4.)
        params.set_Ic2(5.)
        params.set_b1(6.)
        params.set_b2(7.)
        params.set_gravity(8.)
        self.assertEqual(params.m1(), 1.)
        self.assertEqual(params.m2(), 2.)
        self.assertEqual(params.l1(), 3.)
        self.assertEqual(params.Ic1(), 4.)
        self.assertEqual(params.Ic2(), 5.)
        self.assertEqual(params.b1(), 6.)
        self.assertEqual(params.b2(), 7.)
        self.assertEqual(params.gravity(), 8.)

    def test_state(self):
        state = AcrobotState()
        state.set_theta1(1.)
        state.set_theta1dot(2.)
        state.set_theta2(3.)
        state.set_theta2dot(4.)
        self.assertEqual(state.theta1(), 1.)
        self.assertEqual(state.theta1dot(), 2.)
        self.assertEqual(state.theta2(), 3.)
        self.assertEqual(state.theta2dot(), 4.)

    def test_geometry(self):
        builder = DiagramBuilder()
        plant = builder.AddSystem(AcrobotPlant())
        scene_graph = builder.AddSystem(SceneGraph())
        geom = AcrobotGeometry.AddToBuilder(
            builder=builder, acrobot_state_port=plant.get_output_port(0),
            scene_graph=scene_graph)
        builder.Build()
        self.assertIsInstance(geom, AcrobotGeometry)

    def test_geometry_with_params(self):
        builder = DiagramBuilder()
        plant = builder.AddSystem(AcrobotPlant())
        scene_graph = builder.AddSystem(SceneGraph())
        geom = AcrobotGeometry.AddToBuilder(
            builder=builder, acrobot_state_port=plant.get_output_port(0),
            acrobot_params=AcrobotParams(), scene_graph=scene_graph)
        builder.Build()
        self.assertIsInstance(geom, AcrobotGeometry)

    def test_simulation(self):
        # Basic constant-torque acrobot simulation.
        acrobot = AcrobotPlant()

        # Create the simulator.
        simulator = Simulator(acrobot)
        context = simulator.get_mutable_context()

        # Set an input torque.
        input = AcrobotInput()
        input.set_tau(1.)
        acrobot.GetInputPort("elbow_torque").FixValue(context, input)

        # Set the initial state.
        state = context.get_mutable_continuous_state_vector()
        state.set_theta1(1.)
        state.set_theta1dot(0.)
        state.set_theta2(0.)
        state.set_theta2dot(0.)

        self.assertIsInstance(acrobot.get_state(context=context), AcrobotState)
        self.assertIsInstance(acrobot.get_mutable_state(context=context),
                              AcrobotState)
        self.assertIsInstance(acrobot.get_parameters(context=context),
                              AcrobotParams)
        self.assertIsInstance(acrobot.get_mutable_parameters(context=context),
                              AcrobotParams)

        self.assertTrue(acrobot.DynamicsBiasTerm(context).shape == (2,))
        self.assertTrue(acrobot.MassMatrix(context).shape == (2, 2))
        initial_total_energy = acrobot.EvalPotentialEnergy(context) + \
            acrobot.EvalKineticEnergy(context)

        # Simulate (and make sure the state actually changes).
        initial_state = state.CopyToVector()
        simulator.AdvanceTo(1.0)

        self.assertLessEqual(acrobot.EvalPotentialEnergy(context)
                             + acrobot.EvalKineticEnergy(context),
                             initial_total_energy)


class TestAcrobotSpongController(unittest.TestCase):

    def test_default_parameters(self):
        controller = AcrobotSpongController()
        context = controller.CreateDefaultContext()
        expected_parameters = SpongControllerParams()
        actual_parameters = controller.get_parameters(context)
        self.assertEqual(actual_parameters.k_e(), expected_parameters.k_e())
        self.assertEqual(actual_parameters.k_p(), expected_parameters.k_p())
        self.assertEqual(actual_parameters.k_d(), expected_parameters.k_d())
        self.assertEqual(actual_parameters.balancing_threshold(),
                         expected_parameters.balancing_threshold())

    def test_param_accessors(self):
        controller = AcrobotSpongController()
        context = controller.CreateDefaultContext()
        controller.get_mutable_parameters(context).set_k_e(1.)
        actual_parameters = controller.get_parameters(context)
        self.assertEqual(actual_parameters.k_e(), 1.)


class TestSpongControllerParams(unittest.TestCase):
    def test_param_accessors(self):
        params = SpongControllerParams()
        params.set_k_e(1.)
        params.set_k_p(2.)
        params.set_k_d(3.)
        params.set_balancing_threshold(4.)
        self.assertEqual(params.k_e(), 1.)
        self.assertEqual(params.k_p(), 2.)
        self.assertEqual(params.k_d(), 3.)
        self.assertEqual(params.balancing_threshold(), 4.)

    def test_param_defaults(self):
        params = SpongControllerParams()
        self.assertEqual(params.k_e(), 5.)
        self.assertEqual(params.k_p(), 50.)
        self.assertEqual(params.k_d(), 5.)
        self.assertEqual(params.balancing_threshold(), 1000.)
