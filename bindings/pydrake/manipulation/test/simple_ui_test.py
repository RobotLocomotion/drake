# -*- coding: utf-8 -*-

from pydrake.manipulation.simple_ui import JointSliders, SchunkWsgButtons

import unittest

import numpy as np
try:
    import tkinter as tk
except ImportError:
    import Tkinter as tk

from pydrake.common import FindResourceOrThrow
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.parsing import Parser
from pydrake.systems.framework import BasicVector


class TestSimpleUI(unittest.TestCase):
    def test_joint_slider(self):
        # Simply test to make sure that the UI loads and produces the output.
        file_name = FindResourceOrThrow(
            "drake/multibody/benchmarks/acrobot/acrobot.sdf")
        plant = MultibodyPlant(0.0)
        Parser(plant).AddModelFromFile(file_name)
        plant.Finalize()

        slider = JointSliders(robot=plant, lower_limit=-5., upper_limit=5.,
                              resolution=0.001, update_period_sec=0.01,
                              title='test', length=300)
        slider.window.withdraw()  # Don't open a window during testing.
        context = slider.CreateDefaultContext()
        output = slider.AllocateOutput()

        q = [.1, .2]
        slider.set_position(q)
        slider.set_joint_position(q)
        slider.CalcOutput(context, output)

        np.testing.assert_array_equal(output.get_vector_data(0).get_value(), q)

    def test_schunk_wsg_buttons(self):
        window = tk.Tk()
        window.withdraw()  # Don't open a window during testing.
        wsg_buttons = SchunkWsgButtons(window, closed_position=0.008,
                                       open_position=0.05, force_limit=50,
                                       update_period_sec=1.0)
        context = wsg_buttons.CreateDefaultContext()
        output = wsg_buttons.AllocateOutput()

        # Check the port names.
        wsg_buttons.GetOutputPort("position")
        wsg_buttons.GetOutputPort("force_limit")

        wsg_buttons.open()
        wsg_buttons.CalcOutput(context, output)
        np.testing.assert_array_equal(output.get_vector_data(0).get_value(),
                                      [0.05])
        np.testing.assert_array_equal(output.get_vector_data(1).get_value(),
                                      [50.])

        wsg_buttons.close()
        wsg_buttons.CalcOutput(context, output)
        np.testing.assert_array_equal(output.get_vector_data(0).get_value(),
                                      [0.008])
        np.testing.assert_array_equal(output.get_vector_data(1).get_value(),
                                      [50.])
