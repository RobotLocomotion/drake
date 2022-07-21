import copy
import re
import shlex
import subprocess
import sys
import unittest

import yaml

from bazel_tools.tools.python.runfiles.runfiles import Create as CreateRunfiles


class HardwareSimTest(unittest.TestCase):
    """Smoke test for our config file."""

    def _find_resource(self, respath):
        runfiles = CreateRunfiles()
        result = runfiles.Rlocation(respath)
        self.assertTrue(result)
        return result

    def setUp(self):
        self._simulator = self._find_resource(
            "drake/manipulation/hardware_sim/hardware_sim")
        self._scenario_file = self._find_resource(
            "drake/manipulation/hardware_sim/test/test_scenarios.yaml")
        self._default_extra = {
            # For our smoke test, exit fairly quickly.
            "simulation_duration": 1.0,
        }

    def _dict_to_single_line_yaml(self, *, data):
        result = yaml.dump(data, default_flow_style=True)
        result = result.replace("\n", " ").strip()
        result = re.sub(r"  *", " ", result)
        return result

    def _run(self, scenario_name, extra=None):
        merged = copy.deepcopy(self._default_extra)
        merged.update(extra or {})
        scenario_text = self._dict_to_single_line_yaml(data=merged)
        args = [
            self._simulator,
            f"--scenario_file={self._scenario_file}",
            f"--scenario_name={scenario_name}",
            f"--scenario_text={scenario_text}",
        ]
        printable_args = " ".join([
            shlex.quote(re.sub(r"[^=]*\.runfiles/", "", x))
            for x in args
        ])
        print(f"== Running {printable_args}", file=sys.stderr, flush=True)
        subprocess.run(args, check=True)

    def test_Defaults(self):
        """Smoke test that the scenario defaults are always valid."""
        self._run("Defaults")

    def test_OneOfEverything(self):
        """Smoke test for the OneOfEverything scenario."""
        self._run("OneOfEverything")
