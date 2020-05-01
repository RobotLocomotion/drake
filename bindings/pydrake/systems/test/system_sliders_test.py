# -*- coding: utf-8 -*-

import unittest

import numpy as np
try:
    import tkinter as tk
except ImportError:
    import Tkinter as tk

from pydrake.systems.system_sliders import SystemSliders


class TestSystemSliders(unittest.TestCase):
    def test_multibody_plant_slider(self):
        # Tests that the UI loads and produces the output, the
        # size of the output port is correct, and appropriate
        # errors are thrown for incorrect usage.
        port_size = 3
        slider = SystemSliders(port_size,
                               slider_names=["x", "y", "z"], lower_limit=-5.,
                               upper_limit=[5., 1., 10.], resolution=0.001,
                               update_period_sec=0.01, title='test',
                               length=300)
        slider.window.withdraw()  # Don't open a window during testing.
        context = slider.CreateDefaultContext()
        output = slider.AllocateOutput()

        # Input size matches output size.
        self.assertEqual(
            slider.get_output_port(0).size(),
            port_size
        )

        # Valid position.
        q = [.1, 0, 5]
        slider.set_position(q)
        slider.CalcOutput(context, output)
        np.testing.assert_array_equal(output.get_vector_data(0).get_value(), q)

        # Outside of upper limits.
        q = [10, 10, 10]
        slider.set_position(q)
        slider.CalcOutput(context, output)
        np.testing.assert_array_equal(
            output.get_vector_data(0).get_value(), [5., 1., 10.])

        # Outside of lower limits.
        # We set lower limit to -5 by scalar value and we indirectly confirm
        # that all slider lower limits got set to that scalar by setting all
        # sliders to a value lower than that lower limit and observing that
        # they get clamped to the same value.
        q = [-10, -10, -10]
        slider.set_position(q)
        slider.CalcOutput(context, output)
        np.testing.assert_array_equal(
            output.get_vector_data(0).get_value(), [-5, -5, -5])

        # Incorrect size of q when setting slider positions.
        with self.assertRaisesRegex(
                ValueError,
                r"Size of q \(2\) doesn't match input port size \(3\)"):
            slider.set_position([2, 4])

        # Incorrect size of passed in slider names.
        with self.assertRaisesRegex(
                ValueError,
                r"Slider names size \(2\) doesn't match port size \(3\)"):
            SystemSliders(port_size, slider_names=["a", "b"])

        # Incorrect size of passed in lower limits.
        with self.assertRaisesRegex(
                ValueError,
                r"Size of lower_limit \(4\) doesn't match port size \(3\)"):
            SystemSliders(port_size, lower_limit=[2, 3, 4, 5])

        # Incorrect size of passed in upper limits.
        with self.assertRaisesRegex(
                ValueError,
                r"Size of upper_limit \(0\) doesn't match port size \(3\)"):
            SystemSliders(port_size, upper_limit=[])

        # Incorrect size of passed in resolutions.
        with self.assertRaisesRegex(
                ValueError,
                r"Size of resolution \(5\) doesn't match port size \(3\)"):
            SystemSliders(port_size, resolution=[1, 2, 3, 4, 5])
