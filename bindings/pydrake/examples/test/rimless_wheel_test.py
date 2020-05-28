import numpy as np
import unittest

from pydrake.examples.rimless_wheel import (
    RimlessWheel, RimlessWheelContinuousState,
    RimlessWheelGeometry, RimlessWheelParams
    )
from pydrake.geometry import SceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder


class TestRimlessWheel(unittest.TestCase):
    def test_rimless_wheel(self):
        plant = RimlessWheel()
        # Confirm the spelling on the output ports.
        plant.get_minimal_state_output_port()
        plant.get_floating_base_state_output_port()

    def test_params(self):
        params = RimlessWheelParams()
        params.set_mass(1.)
        params.set_length(2.)
        params.set_gravity(4.)
        params.set_number_of_spokes(7)
        params.set_slope(.15)
        self.assertEqual(params.mass(), 1.)
        self.assertEqual(params.length(), 2.)
        self.assertEqual(params.gravity(), 4.)
        self.assertEqual(params.number_of_spokes(), 7)
        self.assertEqual(params.slope(), .15)
        self.assertEqual(
            RimlessWheel.calc_alpha(params=params),
            np.pi / params.number_of_spokes())

    def test_state(self):
        state = RimlessWheelContinuousState()
        state.set_theta(1.)
        state.set_thetadot(2.)
        self.assertEqual(state.theta(), 1.)
        self.assertEqual(state.thetadot(), 2.)

    def test_geometry(self):
        builder = DiagramBuilder()
        plant = builder.AddSystem(RimlessWheel())
        scene_graph = builder.AddSystem(SceneGraph())
        geom = RimlessWheelGeometry.AddToBuilder(
            builder=builder,
            floating_base_state_port=plant.get_floating_base_state_output_port(),  # noqa
            scene_graph=scene_graph)
        # Confirming that the resulting diagram builds.
        builder.Build()
        self.assertIsInstance(geom, RimlessWheelGeometry)

    def test_geometry_with_params(self):
        builder = DiagramBuilder()
        plant = builder.AddSystem(RimlessWheel())
        scene_graph = builder.AddSystem(SceneGraph())
        geom = RimlessWheelGeometry.AddToBuilder(
            builder=builder,
            floating_base_state_port=plant.get_floating_base_state_output_port(),  # noqa
            rimless_wheel_params=RimlessWheelParams(), scene_graph=scene_graph)
        # Confirming that the resulting diagram builds.
        builder.Build()
        self.assertIsInstance(geom, RimlessWheelGeometry)

    def test_simulation(self):
        # Basic rimless_wheel simulation.
        rimless_wheel = RimlessWheel()

        # Create the simulator.
        simulator = Simulator(rimless_wheel)
        context = simulator.get_mutable_context()
        context.SetAccuracy(1e-8)

        # Set the initial state.
        state = context.get_mutable_continuous_state_vector()
        state.set_theta(0.)
        state.set_thetadot(4.)

        # Simulate (and make sure the state actually changes).
        initial_state = state.CopyToVector()
        simulator.AdvanceTo(1.0)
        self.assertFalse((state.CopyToVector() == initial_state).any())
