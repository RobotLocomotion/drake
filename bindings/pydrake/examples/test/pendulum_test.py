import unittest

from pydrake.examples.pendulum import (
    PendulumGeometry, PendulumInput, PendulumParams, PendulumPlant,
    PendulumState
    )
from pydrake.geometry import SceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder


class TestPendulum(unittest.TestCase):
    def test_plant(self):
        pendulum = PendulumPlant()
        self.assertEqual(pendulum.get_input_port(), pendulum.get_input_port(0))
        self.assertEqual(pendulum.get_state_output_port(),
                         pendulum.get_output_port(0))

    def test_input(self):
        input = PendulumInput()
        input.set_tau(1.)
        self.assertEqual(input.tau(), 1.)

    def test_params(self):
        params = PendulumParams()
        params.set_mass(1.)
        params.set_length(2.)
        params.set_damping(3.)
        params.set_gravity(4.)
        self.assertEqual(params.mass(), 1.)
        self.assertEqual(params.length(), 2.)
        self.assertEqual(params.damping(), 3.)
        self.assertEqual(params.gravity(), 4.)

    def test_state(self):
        state = PendulumState()
        state.set_theta(1.)
        state.set_thetadot(2.)
        self.assertEqual(state.theta(), 1.)
        self.assertEqual(state.thetadot(), 2.)

    def test_geometry(self):
        builder = DiagramBuilder()
        plant = builder.AddSystem(PendulumPlant())
        scene_graph = builder.AddSystem(SceneGraph())
        geom = PendulumGeometry.AddToBuilder(
            builder=builder, pendulum_state_port=plant.get_output_port(0),
            scene_graph=scene_graph)
        builder.Build()
        self.assertIsInstance(geom, PendulumGeometry)

    def test_simulation(self):
        # Basic constant-torque pendulum simulation.
        pendulum = PendulumPlant()

        # Create the simulator.
        simulator = Simulator(pendulum)
        context = simulator.get_mutable_context()

        # Set an input torque.
        input = PendulumInput()
        input.set_tau(1.)
        pendulum.get_input_port().FixValue(context, input)

        # Set the initial state.
        state = context.get_mutable_continuous_state_vector()
        state.set_theta(1.)
        state.set_thetadot(0.)

        # Simulate (and make sure the state actually changes).
        initial_state = state.CopyToVector()
        simulator.AdvanceTo(1.0)
        self.assertFalse((state.CopyToVector() == initial_state).any())
