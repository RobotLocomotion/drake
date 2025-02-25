import unittest
import copy
import gc
import numpy as np
import weakref

from pydrake.examples import PendulumPlant
from pydrake.systems.estimators import (
    DiscreteTimeSteadyStateKalmanFilter,
    ExtendedKalmanFilter,
    ExtendedKalmanFilterOptions,
    LuenbergerObserver,
    SteadyStateKalmanFilter,
)
from pydrake.systems.framework import (
    DiagramBuilder,
    InputPortSelection,
    OutputPortSelection,
)
from pydrake.systems.primitives import LinearSystem
from pydrake.systems.estimators import (
    DiscreteTimeSteadyStateKalmanFilter,
    LuenbergerObserver,
    SteadyStateKalmanFilter,
)


class TestEstimators(unittest.TestCase):

    def test_luenberger_observer(self):
        plant = PendulumPlant()
        context = plant.CreateDefaultContext()
        L = np.eye(2)
        observer = LuenbergerObserver(
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

        observer.Clone()
        copy.copy(observer)
        copy.deepcopy(observer)

    def test_steady_state_kalman_filter(self):
        A = np.array([[0., 1.], [-10., -0.1]])
        C = np.eye(2)
        W = np.eye(2)
        V = np.eye(2)
        L = SteadyStateKalmanFilter(A=A, C=C, W=W, V=V)
        self.assertEqual(L.shape, (2, 2))

        L = DiscreteTimeSteadyStateKalmanFilter(A=A, C=C, W=W, V=V)
        self.assertEqual(L.shape, (2, 2))

        plant = LinearSystem(A=A, C=C)
        filter = SteadyStateKalmanFilter(system=plant, W=W, V=V)
        self.assertIsInstance(filter, LuenbergerObserver)

        plant = PendulumPlant()
        context = plant.CreateDefaultContext()
        plant.get_input_port().FixValue(context, [0.])
        filter = SteadyStateKalmanFilter(
            system=plant, context=context, W=W, V=V)
        self.assertIsInstance(filter, LuenbergerObserver)

    def test_extended_kalman_filter(self):
        options = ExtendedKalmanFilterOptions()
        self.assertIsNone(options.initial_state_estimate)
        self.assertIsNone(options.initial_state_covariance)
        self.assertEqual(options.actuation_input_port_index,
                         InputPortSelection.kUseFirstInputIfItExists)
        self.assertEqual(options.measurement_output_port_index,
                         OutputPortSelection.kUseFirstOutputIfItExists)
        self.assertIsNone(options.process_noise_input_port_index)
        self.assertIsNone(options.measurement_noise_input_port_index)
        self.assertEqual(options.use_square_root_method, False)
        self.assertIsNone(options.discrete_measurement_time_period)
        self.assertEqual(options.discrete_measurement_time_offset, 0.0)

        plant = LinearSystem(
            np.array([[0, 1], [0, 0]]), np.array([[0], [1]]),
            np.array([[1, 0]]), np.array([[0]]), 0.01)
        W = np.eye(2)
        V = np.eye(1)
        # Overload taking System_[double].
        observer = ExtendedKalmanFilter(
            observed_system=plant,
            observed_system_context=plant.CreateDefaultContext(),
            W=W, V=V, options=options)
        # Overload taking System_[AutoDiffXd].
        plant = plant.ToAutoDiffXd()
        observer = ExtendedKalmanFilter(
            observed_system=plant,
            observed_system_context=plant.CreateDefaultContext(),
            W=W, V=V, options=options)

        self.assertTrue(
            observer.get_observed_system_input_input_port().size(), 1)
        self.assertTrue(
            observer.get_observed_system_output_input_port().size(), 1)
        self.assertTrue(observer.get_estimated_state_output_port().size(), 2)

        context = observer.CreateDefaultContext()
        xhat = np.array([1, 2])
        Phat = np.eye(2)
        observer.SetStateEstimateAndCovariance(context, xhat, Phat)
        np.testing.assert_array_equal(observer.GetStateEstimate(context), xhat)
        np.testing.assert_array_equal(
            observer.GetStateCovariance(context), Phat)

        spy = weakref.finalize(observer, lambda: None)
        builder = DiagramBuilder()
        builder.AddSystem(observer)
        del observer
        gc.collect()
        self.assertTrue(spy.alive)
        diagram = builder.Build()
        del builder
        gc.collect()
        self.assertTrue(spy.alive)
