import subprocess
import unittest


class TestShowModel(unittest.TestCase):
    def test_show(self):
        """Test that show_model doesn't crash."""
        # Test both an SDF and an URDF.
        models = [
            "multibody/benchmarks/acrobot/acrobot.sdf",
            "multibody/benchmarks/acrobot/acrobot.urdf"
        ]
        for model in models:
            print("model: {}".format(model))
            subprocess.check_call(
                ["manipulation/util/show_model", model])
