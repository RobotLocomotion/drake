from __future__ import print_function

import unittest
import numpy as np

from pydrake.geometry import SceneGraph
from pydrake.examples.quadrotor import QuadrotorPlant, StabilizingLQRController


class TestQuadrotor(unittest.TestCase):

    def test_basics(self):
        # call default constructor
        QuadrotorPlant()

        quadrotor = QuadrotorPlant(m_arg=1, L_arg=2, I_arg=np.eye(3),
                                   kF_arg=1., kM_arg=1.)
        self.assertEqual(quadrotor.m(), 1)
        self.assertEqual(quadrotor.g(), 9.81)

        scene_graph = SceneGraph()
        quadrotor.RegisterGeometry(scene_graph)

        self.assertTrue(quadrotor.source_id().is_valid())
        quadrotor.get_geometry_pose_output_port()

        StabilizingLQRController(quadrotor, np.zeros(3))
