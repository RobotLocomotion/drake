# -*- coding: utf-8 -*-

import unittest

import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.geometry import SceneGraph
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.systems.rendering import (
    MultibodyPositionToGeometryPose,
)


def normalized(x):
    return x / np.linalg.norm(x)


class TestRendering(unittest.TestCase):
    def testMultibodyPositionToGeometryPose(self):
        file_name = FindResourceOrThrow(
            "drake/multibody/benchmarks/acrobot/acrobot.sdf"
        )
        plant = MultibodyPlant(time_step=0.01)
        Parser(plant).AddModels(file_name)
        scene_graph = SceneGraph()
        plant.RegisterAsSourceForSceneGraph(scene_graph)
        plant.Finalize()

        to_pose = MultibodyPositionToGeometryPose(
            plant=plant, input_multibody_state=False
        )

        # Check the size of the input.
        self.assertEqual(to_pose.get_input_port().size(), 2)

        # Just check the spelling of the output port (size is not meaningful
        # for Abstract-valued ports).
        to_pose.get_output_port()
