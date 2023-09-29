import os.path
import subprocess
import unittest

from bazel_tools.tools.python.runfiles.runfiles import Create as CreateRunfiles
import yaml

from drake.examples.hardware_sim.robot_commander import (
    COMMANDER_LCM,
    IIWA_INITIAL_POSITION,
    WSG_INITIAL_POSITION,
)
from pydrake.common.yaml import yaml_load


class RobotCommanderTest(unittest.TestCase):

    def setUp(self):
        runfiles = CreateRunfiles()
        self._robot_commander = runfiles.Rlocation(
            "drake/examples/hardware_sim/robot_commander"
        )
        self._example_scenarios = runfiles.Rlocation(
            "drake/examples/hardware_sim/example_scenarios.yaml"
        )

    def test_scenario_parameters(self):
        """Checks the parameters in `robot_commander.py` are consistent with
        the ones in `example_scenarios.yaml`.
        """
        example_scenarios = yaml_load(filename=self._example_scenarios)
        self.assertEqual(
            IIWA_INITIAL_POSITION.tolist(),
            [-0.2, 0.79, 0.32, -1.76, -0.36, 0.64, -0.73],
        )
        self.assertEqual(WSG_INITIAL_POSITION, 20)
        driver_url = example_scenarios["Demo"]["lcm_buses"]["driver_traffic"]
        self.assertEqual(
            COMMANDER_LCM.get_lcm_url(), driver_url["lcm_url"]
        )

    def test_smoke(self):
        """Runs `robot_commander` to send the command once and checks it
        doesn't crash.
        """
        self.assertTrue(os.path.exists(self._robot_commander))
        subprocess.run([self._robot_commander, "--unit_test"], check=True)
