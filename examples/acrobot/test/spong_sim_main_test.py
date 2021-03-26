import os
import subprocess
import sys
import unittest

from pydrake.common import FindResourceOrThrow

from drake.examples.acrobot.acrobot_io import load_output, load_scenario


_backend = "py"


class TestRunSpongControlledAcrobot(unittest.TestCase):

    def setUp(self):
        self.dut = FindResourceOrThrow(
            f"drake/examples/acrobot/spong_sim_main_{_backend}")
        # 4 states x 30 seconds of samples at 20 Hz per sample_scenario.
        self.nominal_x_tape_shape = (4, 601)

    def test_help(self):
        result = subprocess.run(
            [self.dut, "--help"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            encoding="utf-8")
        self.assertEqual(result.stderr, "")
        self.assertIn("spong-controlled acrobot", result.stdout)
        self.assertEqual(result.returncode, 0 if _backend == "py" else 1)

    def test_example_scenario(self):
        scenario = FindResourceOrThrow(
            "drake/examples/acrobot/test/example_scenario.yaml")
        output = os.path.join(os.environ["TEST_TMPDIR"], "output.yaml")
        subprocess.check_call([
            self.dut, "--scenario", scenario, "--output", output])
        x_tape = load_output(filename=output)
        self.assertEqual(x_tape.shape, self.nominal_x_tape_shape)

    def test_stochastic_scenario(self):
        if _backend == "py":
            return  # Python backend does not support stochastic scenarios.

        scenario = FindResourceOrThrow(
            "drake/examples/acrobot/test/example_stochastic_scenario.yaml")
        output = os.path.join(os.environ["TEST_TMPDIR"], "output.yaml")
        dump_file = os.path.join(os.environ["TEST_TMPDIR"],
                                 "scenario_out.yaml")
        subprocess.check_call([
            self.dut, "--scenario", scenario, "--output", output,
            "--dump_scenario", dump_file])
        x_tape = load_output(filename=output)
        # 4 states x 30 seconds of samples at 20 Hz per sample_scenario.
        self.assertEqual(x_tape.shape, (4, 601))

        # Load the scenario dump; don't bother checking exact distribution
        # semantics; just check that we have the right data shape and type.
        dump = load_scenario(filename=dump_file)
        self.assertEqual(len(dump["controller_params"]), 4)
        self.assertTrue(all(type(x) == float
                            for x in dump["controller_params"]))
        self.assertEqual(len(dump["initial_state"]), 4)
        self.assertTrue(all(type(x) == float
                            for x in dump["initial_state"]))


if __name__ == "__main__":
    if "--cc" in sys.argv:
        sys.argv.remove("--cc")
        _backend = "cc"
    unittest.main()
