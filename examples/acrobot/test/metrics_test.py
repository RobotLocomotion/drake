import unittest

import numpy as np

from drake.examples.acrobot.metrics import (
    deviation_from_upright_equilibrium,
    ensemble_cost,
    final_state_cost,
    is_success,
    success_rate,
)


class TestMetrics(unittest.TestCase):
    def setUp(self):
        self.x_equilibrium = [np.pi, 0, 0, 0]
        self.x_wrapped_equilibrium = [-np.pi, 2 * np.pi, 0, 0]

        self.equilibrium_tapes = 2 * [np.zeros([4, 10])]
        self.equilibrium_tapes[0][:, -1] = self.x_equilibrium
        self.equilibrium_tapes[1][:, -1] = self.x_wrapped_equilibrium

        self.nonequilibrium_tapes = 6 * [np.zeros([4, 10])]
        self.nonequilibrium_tapes[0][:, -1] = [0, 0, 0, 0]
        self.nonequilibrium_tapes[1][:, -1] = [np.pi, np.pi / 2, 0, 0]
        self.nonequilibrium_tapes[2][:, -1] = [np.pi, 0, 1, 0]
        self.nonequilibrium_tapes[3][:, -1] = [np.pi, 0, 0, 1]
        self.nonequilibrium_tapes[4][:, -1] = [-np.pi, -np.pi / 2, -1, -1]
        self.nonequilibrium_tapes[5][:, -1] = [-0.1, 0.2, -0.3, 0.4]

    def test_deviation_from_upright_equilibrium(self):
        # Verify that the deviation is 0 at the upright equilibrium.
        self.assertTrue(
            np.linalg.norm(
                deviation_from_upright_equilibrium(self.x_equilibrium)) == 0)
        # Verify that the deviation is 0 if we add or subtract 2 * pi
        self.assertTrue(
            np.linalg.norm(
                deviation_from_upright_equilibrium(self.x_wrapped_equilibrium))
            == 0)
        # Verify a few other values.
        self.assertTrue((deviation_from_upright_equilibrium(
            [np.pi, np.pi / 2, 0, 0]) == np.array([0, np.pi / 2, 0, 0])).all())
        self.assertTrue((deviation_from_upright_equilibrium(
            [np.pi, 0, 1, 0]) == np.array([0, 0, 1, 0])).all())
        self.assertTrue((deviation_from_upright_equilibrium(
            [np.pi, 0, 0, 1]) == np.array([0, 0, 0, 1])).all())
        self.assertTrue(np.allclose(deviation_from_upright_equilibrium(
            [3 * np.pi + 0.01, -2 * np.pi - 0.02, 0.03, -0.04]),
                                    np.array([0.01, -0.02, 0.03, -0.04])))

    def test_final_state_cost(self):
        # Verify that tapes ending at equilibrium have zero cost.
        for x_tape in self.equilibrium_tapes:
            self.assertTrue(final_state_cost(x_tape) == 0)
        self.assertEqual(ensemble_cost(self.equilibrium_tapes), 0)
        # Verify that tapes ending at non-equilibrium points have positive
        # cost.
        for x_tape in self.nonequilibrium_tapes:
            self.assertGreater(final_state_cost(x_tape), 0)
        self.assertGreater(ensemble_cost(self.nonequilibrium_tapes), 0)

    def test_is_success(self):
        # Verify that ending at the upright equilibrium yields true.
        for x_tape in self.equilibrium_tapes:
            self.assertTrue(is_success(x_tape))
        # Verify that ending at non-equilibrium points yields false.
        for x_tape in self.nonequilibrium_tapes:
            self.assertFalse(is_success(x_tape))

    def test_success_rate(self):
        self.assertEqual(success_rate(self.nonequilibrium_tapes), 0.)
        self.assertEqual(
            success_rate(self.nonequilibrium_tapes
                         + (self.equilibrium_tapes * 3)),
            0.5)
        self.assertEqual(success_rate(self.equilibrium_tapes), 1.)
