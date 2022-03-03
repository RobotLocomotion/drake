import unittest

import numpy as np
from numpy.testing import assert_allclose

from pydrake.systems.analysis import Simulator
from pydrake.systems._resample_interp1d import _resample_interp1d
from pydrake.systems.framework import DiagramBuilder, VectorSystem
from pydrake.systems.primitives import VectorLogSink


class SimpleContinuousTimeSystem(VectorSystem):
    def __init__(self):
        self.output_size = 1
        VectorSystem.__init__(self,
                              0,                 # Zero inputs.
                              self.output_size)  # One output.
        self.DeclareContinuousState(1)           # One state variable.

    # xdot(t) = -x(t) + x^3(t)
    def DoCalcVectorTimeDerivatives(self, context, u, x, xdot):
        xdot[:] = -x + x**3

    # y(t) = x(t)
    def DoCalcVectorOutput(self, context, u, x, y):
        y[:] = x


class MultiDimensionalTimeSystem(VectorSystem):
    def __init__(self):
        self.output_size = 3
        VectorSystem.__init__(self,
                              0,                 # Zero inputs.
                              self.output_size)  # One output.
        self.DeclareContinuousState(1)           # One state variable.

    # dx/dt = 2 * t -> x(t) = t^2 + const.
    def DoCalcVectorTimeDerivatives(self, context, u, x, xdot):
        t = context.get_time()
        xdot[:] = 2 * t

    # y(t) = [x(t), 2*x(t), sqrt(x(t))]
    def DoCalcVectorOutput(self, context, u, x, y):
        y[:] = np.array([x[0], 2*x[0], np.sqrt(x[0])])


class TestResampleLogInterp1d(unittest.TestCase):
    def test_resample_log_oned_interp1d(self):
        vector_system = SimpleContinuousTimeSystem()
        simulator, log, context = self.create_log(vector_system)

        context.SetContinuousState([0.9])
        simulator.AdvanceTo(0.1)

        # Check data generated by the VectorSystem
        expected_t = np.array([
            0.0, 0.0001, 0.0006, 0.0031, 0.0156, 0.0781, 0.1])
        expected_y = np.array([[
            0.9, 0.899983, 0.899897, 0.899469, 0.897303, 0.885885, 0.881648]])
        self._check_input_data(log, expected_t, expected_y)

        # Resample [0, .1] by step=0.03125 => 4 data points.
        expected_t = np.array([0.0, 0.03125, 0.0625, 0.09375])
        expected_y = np.array([[0.9, 0.894444, 0.888735, 0.882857]])
        self._check_resample(log, 0.03125, expected_t, expected_y)

        # Resample [0, .1] by step=0.01 => 10 data points.
        expected_t = np.array([
            0.0, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09])
        expected_y = np.array([[
            0.9, 0.898273, 0.896499, 0.894672, 0.892845, 0.891018, 0.889192,
            0.887365, 0.885517, 0.883582]])
        self._check_resample(log, 0.01, expected_t, expected_y)

    def test_resample_log_multid_interp1d(self):
        vector_system = MultiDimensionalTimeSystem()
        simulator, log, context = self.create_log(vector_system)

        context.SetContinuousState([0.])
        simulator.AdvanceTo(0.1)

        # Check data generated by the VectorSystem
        expected_t = np.array([
            0.0, 0.0001, 0.0006, 0.0031, 0.0156, 0.0781, 0.1])
        expected_y = np.array([
            [0.000000e+00, 1.000000e-08, 3.600000e-07, 9.610000e-06,
             2.433600e-04, 6.099610e-03, 1.000000e-02],
            [0.000000e+00, 2.000000e-08, 7.200000e-07, 1.922000e-05,
             4.867200e-04, 1.219922e-02, 2.000000e-02],
            [0.000000e+00, 1.000000e-04, 6.000000e-04, 3.100000e-03,
             1.560000e-02, 7.810000e-02, 1.000000e-01],
        ])
        self._check_input_data(log, expected_t, expected_y)

        # Resample [0, .1] by step=0.03125 => 4 data points.
        expected_t = np.array([0.0, 0.03125, 0.0625, 0.09375])
        expected_y = np.array([
            [0.0000000000, 0.0017097650, 0.0046378900, 0.0088868750],
            [0.0000000000, 0.0034195300, 0.0092757800, 0.0177737500],
            [0.0000000000, 0.0312500000, 0.0625000000, 0.0937500000],
        ])
        self._check_resample(log, 0.03125, expected_t, expected_y)

        # Resample [0, .1] by step=0.01 => 10 data points.
        expected_t = np.array([
            0.0, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09])
        expected_y = np.array([
            [0.0000000000, 0.0001386400, 0.0006556400, 0.0015926400,
             0.0025296400, 0.0034666400, 0.0044036400, 0.0053406400,
             0.0064380000, 0.0082190000],
            [0.0000000000, 0.0002772800, 0.0013112800, 0.0031852800,
             0.0050592800, 0.0069332800, 0.0088072800, 0.0106812800,
             0.0128760000, 0.0164380000],
            [0.0000000000, 0.0100000000, 0.0200000000, 0.0300000000,
             0.0400000000, 0.0500000000, 0.0600000000, 0.0700000000,
             0.0800000000, 0.0900000000],
        ])
        self._check_resample(log, 0.01, expected_t, expected_y)

    def create_log(self, vector_system):
        builder = DiagramBuilder()
        system = builder.AddSystem(vector_system)
        logger = builder.AddSystem(VectorLogSink(vector_system.output_size))
        builder.Connect(system.get_output_port(0), logger.get_input_port(0))
        diagram = builder.Build()

        context = diagram.CreateDefaultContext()

        simulator = Simulator(diagram, context)

        # Get the log and make sure its original values are as expected.
        # A low tolerance is used since we just want to roughly validate.
        log = logger.FindLog(context)

        return simulator, log, context

    def _check_input_data(self, log, expected_t, expected_y):
        tolerance = 1e-6

        assert_allclose(
            log.sample_times(), expected_t, rtol=tolerance,
            err_msg="Expected simulation input times not equivalent.")
        assert_allclose(
            log.data(), expected_y, rtol=tolerance,
            err_msg="Expected simulation input data not equivalent.")

    def _check_resample(self, log, step, expected_t, expected_y):
        tolerance = 1e-6

        self.validate_resample(log, step, expected_t, expected_y, tolerance)

        # Final test: make sure un-sorted data gets sorted.  Use a proxy to
        # reverse the original log, only sample_times and data methods needed.
        class ReverseLog:

            def sample_times(self):
                return np.flip(np.array(log.sample_times(), copy=True), axis=0)

            def data(self):
                return np.flip(np.array(log.data(), copy=True), axis=1)

        # Re-use the previous test's expected t and x.
        r_log = ReverseLog()
        self.validate_resample(r_log, step, expected_t, expected_y, tolerance)

    def validate_resample(self, log, timestep, t_expected, x_expected,
                          tolerance):
        """Perform the resampling and validate with the provided values."""
        t, x = log.sample_times(), log.data()
        t, x = _resample_interp1d(t, x, timestep)
        self.assertTrue(
            t.shape[0] == x.shape[1],
            msg=f"Expected t.shape={t.shape} to match x.shape={x.shape}.")
        assert_allclose(
            t, t_expected, rtol=tolerance,
            err_msg="Resampled times are not as expected.")
        assert_allclose(
            x, x_expected, rtol=tolerance,
            err_msg="Resampled data are not as expected.")
