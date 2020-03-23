# -*- coding: utf-8 -*-

import unittest

import numpy as np
try:
    import tkinter as tk
except ImportError:
    import Tkinter as tk

from pydrake.common import FindResourceOrThrow
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.parsing import Parser
from pydrake.systems.primitives import SignalLogger
from pydrake.systems.system_sliders import SystemSliders


class TestSystemSliders(unittest.TestCase):
    def test_multibody_plant_slider(self):
        # Tests that the UI loads and produces the output, the
        # size of the output port is correct, and appropriate
        # errors are thrown for incorrect usage
        file_name = FindResourceOrThrow(
            "drake/multibody/benchmarks/acrobot/acrobot.sdf")
        plant = MultibodyPlant(0.0)
        Parser(plant).AddModelFromFile(file_name)
        plant.Finalize()

        slider = SystemSliders(input_port=plant.get_actuation_input_port(),
                               slider_names=["elbow"], lower_limit=-5.,
                               upper_limit=5., resolution=0.001,
                               update_period_sec=0.01, title='test',
                               length=300)
        slider.window.withdraw()  # Don't open a window during testing.
        context = slider.CreateDefaultContext()
        output = slider.AllocateOutput()

        # input size matches output size
        self.assertEqual(
            slider.get_output_port(0).size(),
            plant.get_actuation_input_port().size()
        )

        # valid position
        q = [.1]
        slider.set_position(q)
        slider.CalcOutput(context, output)
        np.testing.assert_array_equal(output.get_vector_data(0).get_value(), q)

        # outside of limits
        q = [10]
        slider.set_position(q)
        slider.CalcOutput(context, output)
        np.testing.assert_array_equal(
            output.get_vector_data(0).get_value(), [5])

        # check that bad inputs raise a ValueError in constructor
        with self.assertRaises(ValueError):
            slider.set_position([2, 4])

        with self.assertRaises(ValueError):
            SystemSliders(input_port=plant.get_actuation_input_port(),
                          slider_names=["a", "b"])
        with self.assertRaises(ValueError):
            SystemSliders(input_port=plant.get_actuation_input_port(),
                          lower_limit=[2, 3, 4])

    def test_signal_logger(self):
        system_dim = 4
        logger = SignalLogger(system_dim)

        slider = SystemSliders(input_port=logger.get_input_port(0),
                               lower_limit=-10., upper_limit=10.,
                               resolution=0.001, update_period_sec=0.01,
                               title='test_signal_logger', length=300)
        slider.window.withdraw()  # Don't open a window during testing
        context = slider.CreateDefaultContext()
        output = slider.AllocateOutput()

        q = [5, 3, -10, 10]
        slider.set_position(q)
        slider.CalcOutput(context, output)

        np.testing.assert_array_equal(output.get_vector_data(0).get_value(), q)
