import pydrake.planning as mut  # ruff: isort: skip

import unittest

import numpy as np

from pydrake.multibody.plant import MultibodyPlant


class TestJointLimits(unittest.TestCase):
    def test_default_ctor(self):
        dut = mut.JointLimits()
        self.assertEqual(dut.num_positions(), 0)
        self.assertEqual(dut.num_velocities(), 0)
        self.assertEqual(dut.num_accelerations(), 0)

    def test_empty_plant_ctor(self):
        plant = MultibodyPlant(0.01)
        plant.Finalize()
        dut = mut.JointLimits(
            plant=plant,
            require_finite_positions=True,
            require_finite_velocities=True,
            require_finite_accelerations=True,
        )
        self.assertEqual(dut.num_positions(), 0)
        self.assertEqual(dut.num_velocities(), 0)
        self.assertEqual(dut.num_accelerations(), 0)

    def test_selecting_plant_ctor(self):
        no_dof = mut.DofMask(0, True)
        plant = MultibodyPlant(0.01)
        plant.Finalize()
        dut = mut.JointLimits(
            plant=plant,
            active_dof=no_dof,
            require_finite_positions=True,
            require_finite_velocities=True,
            require_finite_accelerations=True,
        )
        self.assertEqual(dut.num_positions(), 0)
        self.assertEqual(dut.num_velocities(), 0)
        self.assertEqual(dut.num_accelerations(), 0)

    def test_limits_api(self):
        lo = np.array([0.0, 0.0, 0.0])
        hi = np.array([1.0, 1.0, 1.0])
        dut = mut.JointLimits(
            position_lower=lo,
            position_upper=hi,
            velocity_lower=lo,
            velocity_upper=hi,
            acceleration_lower=lo,
            acceleration_upper=hi,
            require_finite_positions=True,
            require_finite_velocities=True,
            require_finite_accelerations=True,
        )
        self.assertEqual(dut.num_positions(), 3)
        self.assertEqual(dut.num_velocities(), 3)
        self.assertEqual(dut.num_accelerations(), 3)
        self.assertEqual(dut.position_lower()[0], 0.0)
        self.assertEqual(dut.position_upper()[0], 1.0)
        self.assertEqual(dut.velocity_lower()[0], 0.0)
        self.assertEqual(dut.velocity_upper()[0], 1.0)
        self.assertEqual(dut.acceleration_lower()[0], 0.0)
        self.assertEqual(dut.acceleration_upper()[0], 1.0)
        self.assertTrue(dut.CheckInPositionLimits(position=lo, tolerance=0.0))
        self.assertTrue(dut.CheckInVelocityLimits(velocity=lo, tolerance=0.0))
        self.assertTrue(
            dut.CheckInAccelerationLimits(acceleration=lo, tolerance=0.0)
        )

        dut2 = mut.JointLimits(dut, mut.DofMask([True, False, True]))
        self.assertEqual(dut2.num_positions(), 2)
