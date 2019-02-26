from __future__ import print_function

import unittest
import numpy as np

import pydrake.systems.framework as framework
from pydrake.systems.primitives import AffineSystem
from pydrake.examples.quadrotor import QuadrotorPlant, StabilizingLQRController


class TestQuadrotor(unittest.TestCase):

    def test_basics(self):
        # call default constructor
        QuadrotorPlant()

        quadrotor = QuadrotorPlant(m=1, L=2, inertia=np.eye(3), kF=1., kM=1.)
        self.assertEqual(quadrotor.m(), 1)
        self.assertEqual(quadrotor.g(), 9.81)

        StabilizingLQRController(quadrotor, np.zeros(3))
