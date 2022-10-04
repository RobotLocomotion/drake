import subprocess
import unittest

from bazel_tools.tools.python.runfiles import runfiles


class TestShowModel(unittest.TestCase):
    def test_show(self):
        """Test that show_model doesn't crash."""
        manifest = runfiles.Create()
        model_runpaths = [
            # Simple SDFormat file.
            "drake/multibody/benchmarks/acrobot/acrobot.sdf",
            # Simple URDF file.
            "drake/multibody/benchmarks/acrobot/acrobot.urdf",
            # Nested SDFormat file.
            "drake/manipulation/util/test/simple_nested_model.sdf",
            # SDFormat world file with multiple models.
            "drake/manipulation/util/test/simple_world_with_two_models.sdf",
        ]
        bin = manifest.Rlocation("drake/manipulation/util/show_model")
        for model_runpath in model_runpaths:
            print(model_runpath)
            model_file = manifest.Rlocation(model_runpath)
            subprocess.check_call([bin, model_file, "--loop_once"])
