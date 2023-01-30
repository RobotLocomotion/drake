import unittest

import numpy as np

from pydrake.geometry import SceneGraph
from pydrake.examples import (
    QuadrotorGeometry,
    QuadrotorPlant,
    StabilizingLQRController,
)
from pydrake.systems.framework import DiagramBuilder


class TestQuadrotor(unittest.TestCase):

    def test_basics(self):
        # call default constructor
        QuadrotorPlant()

        quadrotor = QuadrotorPlant(m_arg=1, L_arg=2, I_arg=np.eye(3),
                                   kF_arg=1., kM_arg=1.)
        self.assertEqual(quadrotor.m(), 1)
        self.assertEqual(quadrotor.g(), 9.81)
        self.assertEqual(quadrotor.length(), 2)
        self.assertEqual(quadrotor.force_constant(), 1)
        self.assertEqual(quadrotor.moment_constant(), 1.)
        np.testing.assert_array_equal(quadrotor.inertia(), np.eye(3))

        StabilizingLQRController(quadrotor, np.zeros(3))

    def test_geometry(self):
        builder = DiagramBuilder()
        quadrotor = builder.AddSystem(QuadrotorPlant())
        scene_graph = builder.AddSystem(SceneGraph())
        state_port = quadrotor.get_output_port(0)
        geom = QuadrotorGeometry.AddToBuilder(builder, state_port, scene_graph)
        self.assertTrue(geom.get_frame_id().is_valid())
