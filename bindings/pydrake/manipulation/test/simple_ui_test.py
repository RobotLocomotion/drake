# -*- coding: utf-8 -*-

from pydrake.manipulation.simple_ui import JointSliders

import unittest
import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.multibody.multibody_tree.multibody_plant import MultibodyPlant
from pydrake.multibody.multibody_tree.parsing import AddModelFromSdfFile


class TestSimpleUI(unittest.TestCase):
    def test_joint_slider(self):
        # Simply test to make sure that the UI loads and produces the output.
        file_name = FindResourceOrThrow(
            "drake/multibody/benchmarks/acrobot/acrobot.sdf")
        plant = MultibodyPlant()
        AddModelFromSdfFile(file_name=file_name, plant=plant, scene_graph=None)
        plant.Finalize()

        slider = JointSliders(robot=plant, lower_limit=-5., upper_limit=5.,
                              resolution=0.001, update_period_sec=0.01,
                              title='test')
        context = slider.CreateDefaultContext()
        output = slider.AllocateOutput()

        q = [.1, .2]
        slider.set(q)
        slider.CalcOutput(context, output)

        np.testing.assert_array_equal(output.get_vector_data(0).get_value(), q)
