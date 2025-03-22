import pydrake.systems.estimators as mut

import unittest

import numpy as np

from pydrake.examples import PendulumPlant
from pydrake.systems.primitives import LinearSystem


class TestEstimators(unittest.TestCase):

    def test_luenberger_observer(self):
        plant = PendulumPlant()
        context = plant.CreateDefaultContext()
        L = np.eye(2)
        observer = mut.LuenbergerObserver(
            observed_system=plant,
            observed_system_context=context,
            observer_gain=L)
        port = observer.get_observed_system_input_input_port()
        self.assertEqual(port.size(), 1)
        port = observer.get_observed_system_output_input_port()
        self.assertEqual(port.size(), 2)
        port = observer.get_estimated_state_output_port()
        self.assertEqual(port.size(), 2)
        np.testing.assert_array_equal(L, observer.observer_gain())
        np.testing.assert_array_equal(L, observer.L())

    def test_steady_state_kalman_filter(self):
        A = np.array([[0., 1.], [-10., -0.1]])
        C = np.eye(2)
        W = np.eye(2)
        V = np.eye(2)
        L = mut.SteadyStateKalmanFilter(A=A, C=C, W=W, V=V)
        self.assertEqual(L.shape, (2, 2))

        L = mut.DiscreteTimeSteadyStateKalmanFilter(A=A, C=C, W=W, V=V)
        self.assertEqual(L.shape, (2, 2))

        plant = LinearSystem(A=A, C=C)
        filter = mut.SteadyStateKalmanFilter(system=plant, W=W, V=V)
        self.assertIsInstance(filter, mut.LuenbergerObserver)

        plant = PendulumPlant()
        context = plant.CreateDefaultContext()
        plant.get_input_port().FixValue(context, [0.])
        filter = mut.SteadyStateKalmanFilter(
            system=plant, context=context, W=W, V=V)
        self.assertIsInstance(filter, mut.LuenbergerObserver)
