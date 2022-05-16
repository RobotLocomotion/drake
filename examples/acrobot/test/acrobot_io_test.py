import numpy as np
import unittest

from pydrake.common import FindResourceOrThrow
from drake.examples.acrobot.acrobot_io import (
    load_scenario, save_scenario,
    load_output, save_output)


class TestIo(unittest.TestCase):

    def setUp(self):
        self.maxDiff = None
        self.example = FindResourceOrThrow(
            "drake/examples/acrobot/test/example_scenario.yaml")
        # When saving, everything comes out as floats (not `int`, etc.).
        self.expected_save = """\
controller_params: [5.0, 50.0, 5.0, 1000.0]
initial_state: [1.2, 0.0, 0.0, 0.0]
t_final: 30.0
tape_period: 0.05
"""

    def test_load_scenario(self):
        scenario = load_scenario(filename=self.example)
        expected = ", ".join([
            "{'controller_params': [5, 50, 5, '1e3']",
            "'initial_state': [1.2, 0, 0, 0]",
            "'t_final': 30.0",
            "'tape_period': 0.05}"])
        self.assertEqual(str(scenario), expected)

    def test_save_scenario(self):
        scenario = load_scenario(filename=self.example)
        actual = save_scenario(scenario=scenario)
        self.assertEqual(actual, self.expected_save)

    def test_save_scenario_numpy(self):
        scenario = {
            "controller_params": np.array([5, 50, 5, 1e3]),
            "initial_state": np.array([1.2, 0, 0, 0]),
            "t_final": 30.0,
            "tape_period": 0.05,
        }
        actual = save_scenario(scenario=scenario)
        self.assertEqual(actual, self.expected_save)

    def test_save_scenario_stochastic(self):
        scenario = {
            "controller_params": {
                "max": np.array([5, 50, 5, 1e3]),
                "min": np.array([1, 10, 1, 1e2]),
            },
            "initial_state": np.array([1.2, 0, 0, 0]),
            "t_final": 30.0,
            "tape_period": 0.05,
        }
        actual = save_scenario(scenario=scenario)
        self.assertEqual(actual, """\
controller_params:
  max: [5.0, 50.0, 5.0, 1000.0]
  min: [1.0, 10.0, 1.0, 100.0]
initial_state: [1.2, 0.0, 0.0, 0.0]
t_final: 30.0
tape_period: 0.05
""")

    def test_save_output_and_load_output(self):
        values = [
            [0, 1, 2, 3, 4, 5, 6, 7],
            [10, 11, 12, 13, 14, 15, 16, 17],
            [20, 21, 22, 23, 24, 25, 26, 27],
            [30, 31, 32, 33, 34, 35, 36, 37],
        ]
        x_tape = np.array(values)
        actual = save_output(x_tape=x_tape)
        expected = """x_tape:
- [0, 1, 2, 3, 4, 5, 6, 7]
- [10, 11, 12, 13, 14, 15, 16, 17]
- [20, 21, 22, 23, 24, 25, 26, 27]
- [30, 31, 32, 33, 34, 35, 36, 37]
"""
        self.assertEqual(actual, expected)
        readback = load_output(data=expected)
        self.assertEqual(x_tape.tolist(), values)
