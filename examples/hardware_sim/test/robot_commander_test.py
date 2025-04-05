import unittest

from python.runfiles import Create as CreateRunfiles

from pydrake.common.yaml import yaml_load

import examples.hardware_sim.robot_commander as mut


class RobotCommanderTest(unittest.TestCase):

    def setUp(self):
        runfiles = CreateRunfiles()
        self._example_scenarios = runfiles.Rlocation(
            "drake/examples/hardware_sim/example_scenarios.yaml"
        )

    def test_scenario_parameters(self):
        """Checks the parameters in `robot_commander.py` are consistent with
        the ones in `example_scenarios.yaml`.
        """
        example_scenarios = yaml_load(filename=self._example_scenarios)
        demo = example_scenarios["Demo"]

        iiwa_directive = demo["directives"][2]["add_model"]
        self.assertEqual(iiwa_directive["name"], "iiwa")
        iiwa_q0 = []
        for _, qs in iiwa_directive["default_joint_positions"].items():
            iiwa_q0.extend(qs)
        self.assertListEqual(mut.IIWA_Q0.tolist(), iiwa_q0)

        wsg_directive = demo["directives"][5]["add_model"]
        self.assertEqual(wsg_directive["name"], "wsg")
        wsg_q0 = []
        for _, qs in demo["initial_position"]["wsg"].items():
            wsg_q0.extend(qs)
        self.assertListEqual([-mut.WSG_Q0, mut.WSG_Q0], wsg_q0)

        lcm_url = demo["lcm_buses"]["driver_traffic"]["lcm_url"]
        self.assertEqual(mut.LCM_URL, lcm_url)

    def test_smoke(self):
        """Runs `robot_commander.main` to check that it doesn't crash. Uses a
        nerfed URL to avoid sending network robot commands from a unit test.
        """
        old_url = mut.LCM_URL
        try:
            mut.LCM_URL = "memq://"
            mut.main()
        finally:
            # Set `LCM_URL` back as other test cases depend on it.
            mut.LCM_URL = old_url
