# -*- coding: utf-8 -*-

from pydrake.systems.rendering import (
    MultibodyPositionToGeometryPose,
)

import copy
import unittest
import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.common.test_utilities.deprecation import catch_drake_warnings
from pydrake.common.value import AbstractValue
from pydrake.geometry import SceneGraph
from pydrake.math import RigidTransform
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.math import (
    SpatialVelocity,
)
from pydrake.multibody.parsing import Parser
from pydrake.systems.framework import (
    BasicVector,
    PortDataType,
)
from pydrake.common.eigen_geometry import Quaternion


def normalized(x):
    return x / np.linalg.norm(x)


class TestRendering(unittest.TestCase):
    def testMultibodyPositionToGeometryPose(self):
        file_name = FindResourceOrThrow(
            "drake/multibody/benchmarks/acrobot/acrobot.sdf")
        plant = MultibodyPlant(time_step=0.01)
        model_instance = Parser(plant).AddModelFromFile(file_name)
        scene_graph = SceneGraph()
        plant.RegisterAsSourceForSceneGraph(scene_graph)
        plant.Finalize()

        to_pose = MultibodyPositionToGeometryPose(
            plant=plant, input_multibody_state=False)

        # Check the size of the input.
        self.assertEqual(to_pose.get_input_port().size(), 2)

        # Just check the spelling of the output port (size is not meaningful
        # for Abstract-valued ports).
        to_pose.get_output_port()
