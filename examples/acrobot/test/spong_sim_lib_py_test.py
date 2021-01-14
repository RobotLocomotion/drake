import unittest

from drake.examples.acrobot.spong_sim import simulate


class TestSpongControlledAcrobot(unittest.TestCase):

    def test_simulate(self):
        x_tape = simulate(
            initial_state=[0.01, 0.02, 0.03, 0.04],
            controller_params=[0.001, 0.002, 0.003, 0.004],
            t_final=30.0, tape_period=0.05)
        # 4 states x 30 seconds of samples at 20 Hz per sample_scenario.
        self.assertEqual(x_tape.shape, (4, 601))
