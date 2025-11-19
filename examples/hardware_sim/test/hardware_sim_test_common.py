"""This file contains smoke tests for our hardware_sim program and its sample
config files. The tests are reused for both the C++ and Python regression
tests.

Note that this file is only ever imported; it is never run as a unittest
program itself; we'll use wrapper files named hardware_sim_cc_test and
hardware_sim_py_test to actually run it.
"""

import copy
import os.path
import re
import shlex
import subprocess
import sys

from python.runfiles import Create as CreateRunfiles
import yaml


class HardwareSimTest:
    def _find_resource(self, respath):
        runfiles = CreateRunfiles()
        result = runfiles.Rlocation(respath)
        self.assertTrue(result, respath)
        self.assertTrue(os.path.exists(result), respath)
        return result

    def setUp(self):
        """Prior to anything calling this function, our subclass must set our
        self._sim_lang to either "cc" or "py".
        """
        lang = self._sim_lang
        self._simulator = self._find_resource(
            f"drake/examples/hardware_sim/hardware_sim_{lang}"
        )
        self._example_scenarios = self._find_resource(
            "drake/examples/hardware_sim/example_scenarios.yaml"
        )
        self._test_scenarios = self._find_resource(
            "drake/examples/hardware_sim/test/test_scenarios.yaml"
        )
        self._default_extra = {
            # For our smoke test, exit fairly quickly.
            "simulation_duration": 0.0625,
        }

    def _dict_to_single_line_yaml(self, *, data):
        """Given a dictionary, returns it as a YAML one-liner."""
        result = yaml.dump(data, default_flow_style=True)
        result = result.replace("\n", " ").strip()
        result = re.sub(r"  *", " ", result)
        return result

    def _run(self, scenario_file, scenario_name, extra=None, graphviz=None):
        """Runs the given scenario for 1 second, checking that it does not
        crash. Allows overriding scenario options using the optional `extra`
        dictionary"""
        merged = copy.deepcopy(self._default_extra)
        merged.update(extra or {})
        scenario_text = self._dict_to_single_line_yaml(data=merged)
        args = [
            self._simulator,
            f"--scenario_file={scenario_file}",
            f"--scenario_name={scenario_name}",
            f"--scenario_text={scenario_text}",
        ]
        if graphviz is not None:
            args.append(f"--graphviz={graphviz}")
        printable_args = " ".join(
            [shlex.quote(re.sub(r"[^=]*\.runfiles/", "", x)) for x in args]
        )
        print(f"== Running {printable_args}", file=sys.stderr, flush=True)
        subprocess.run(args, check=True)

    def test_Defaults(self):
        """Tests that the scenario defaults are always valid."""
        self._run(self._test_scenarios, "Defaults")

    def test_OneOfEverything(self):
        """Tests the OneOfEverything test scenario."""
        self._run(self._test_scenarios, "OneOfEverything")

    def test_Demo(self):
        """Tests the Demo example."""
        self._run(self._example_scenarios, "Demo")

    def test_graphviz(self):
        out_file = f"{os.environ['TEST_TMPDIR']}/graph.dot"
        self.assertFalse(os.path.exists(out_file))
        self._run(self._test_scenarios, "Defaults", graphviz=out_file)
        with open(out_file, encoding="utf-8") as f:
            content = f.read()
        self.assertIn("(split)", content)
