import unittest

import numpy as np
from numpy.testing import assert_allclose

from pydrake.systems.analysis import Simulator
from pydrake.systems._resample_log_interp1d import _resample_log_interp1d
from pydrake.systems.framework import DiagramBuilder, VectorSystem
from pydrake.systems.primitives import VectorLogSink


class SimpleContinuousTimeSystem(VectorSystem):
    def __init__(self):
        VectorSystem.__init__(self,
                              0,        # Zero inputs.
                              1)        # One output.
        self.DeclareContinuousState(1)  # One state variable.

    # xdot(t) = -x(t) + x^3(t)
    def DoCalcVectorTimeDerivatives(self, context, u, x, xdot):
        xdot[:] = -x + x**3

    # y(t) = x(t)
    def DoCalcVectorOutput(self, context, u, x, y):
        y[:] = x


class TestResampleLogInterp1d(unittest.TestCase):

    def test_resample_log_interp1d(self):
        builder = DiagramBuilder()
        system = builder.AddSystem(SimpleContinuousTimeSystem())
        logger = builder.AddSystem(VectorLogSink(1))
        builder.Connect(system.get_output_port(0), logger.get_input_port(0))
        diagram = builder.Build()

        context = diagram.CreateDefaultContext()
        context.SetContinuousState([0.9])

        simulator = Simulator(diagram, context)
        simulator.AdvanceTo(.1)

        # Get the log and make sure its original values are as expected.
        # A low tolerance is used since we just want to roughly validate.
        log = logger.FindLog(context)
        expected_input_times = np.array([
            0.0, 0.0001, 0.0006, 0.0031, 0.0156, 0.0781, 0.1])
        expected_input_data = np.array([[
            0.9, 0.899983, 0.899897, 0.899469, 0.897303, 0.885885, 0.881648]])
        tolerance = 1e-3
        assert_allclose(
            log.sample_times(), expected_input_times, rtol=tolerance,
            err_msg="Expected simulation input times not equivalent.")
        assert_allclose(
            log.data(), expected_input_data, rtol=tolerance,
            err_msg="Expected simulation input data not equivalent.")

        # Resample [0, .1] by step=0.03125 => 4 data points.
        t_expected = np.array([0.0, 0.03125, 0.0625, 0.09375])
        x_expected = np.array([[0.9, 0.894444, 0.888735, 0.882857]])
        self.validate_resample(log, 0.03125, t_expected, x_expected, tolerance)

        # Resample [0, .1] by step=0.01 => 10 data points.
        t_expected = np.array([
            0.0, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09])
        x_expected = np.array([[
            0.9, 0.898273, 0.896499, 0.894672, 0.892845, 0.891018, 0.889192,
            0.887365, 0.885517, 0.883582]])
        self.validate_resample(log, 0.01, t_expected, x_expected, tolerance)

        # Final test: make sure un-sorted data gets sorted.  Use a proxy to
        # reverse the original log, only sample_times and data methods needed.
        class ReverseLog:

            def sample_times(self):
                return np.flip(np.array(log.sample_times(), copy=True), axis=0)

            def data(self):
                return np.flip(np.array(log.data(), copy=True), axis=1)

        # Re-use the previous test's expected t and x.
        r_log = ReverseLog()
        self.validate_resample(r_log, 0.01, t_expected, x_expected, tolerance)

    def validate_resample(self, log, timestep, t_expected, x_expected,
                          tolerance):
        """Perform the resampling and validate with the provided values."""
        t, x = _resample_log_interp1d(log, timestep)
        self.assertTrue(
            t.shape[0] == x.shape[1],
            msg=f"Expected t.shape={t.shape} to match x.shape={x.shape}.")
        assert_allclose(
            t, t_expected, rtol=tolerance,
            err_msg="Resampled times are not as expected.")
        assert_allclose(
            x, x_expected, rtol=tolerance,
            err_msg="Resampled data are not as expected.")
