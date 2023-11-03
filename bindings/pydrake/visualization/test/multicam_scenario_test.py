import os.path
import subprocess
import sys
import unittest

from bazel_tools.tools.python.runfiles.runfiles import Create as CreateRunfiles


class TestMulticamScenario(unittest.TestCase):
    def _find_resource(self, respath):
        runfiles = CreateRunfiles()
        result = runfiles.Rlocation(respath)
        self.assertTrue(result, respath)
        self.assertTrue(os.path.exists(result), respath)
        return result

    @unittest.skipIf(sys.platform == "darwin", "Missing RenderEngineGl")
    def test_smoke(self):
        """Runs `multicam_scenario.yaml` and checks it doesn't crash."""
        simulator = self._find_resource(
            "drake/examples/hardware_sim/hardware_sim_py"
        )
        test_scenarios = self._find_resource(
            "drake/bindings/pydrake/visualization/test/multicam_scenario.yaml"
        )
        run_args = [
            simulator,
            f"--scenario_file={test_scenarios}",
            "--scenario_name=Multicam",
            # For the smoke test, exit fairly quickly.
            "--scenario_text={simulation_duration: 0.0625}",
        ]
        subprocess.run(run_args, check=True)
