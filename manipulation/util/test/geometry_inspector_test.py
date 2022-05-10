import subprocess
import unittest


class TestGeometryInspector(unittest.TestCase):
    def test_geometry_inspector(self):
        """Test that the binary doesn't crash."""
        # Test both an SDF and an URDF.
        models = [
            "multibody/benchmarks/acrobot/acrobot.sdf",
            "multibody/benchmarks/acrobot/acrobot.urdf"
        ]
        for model in models:
            print("model: {}".format(model))
            subprocess.check_call(
                ["manipulation/util/geometry_inspector",
                 model, "--test", "--position", "0.1", "0.2"])

    def test_package_path(self):
        # Test that the binary doesn't crash when fed a package path argument.
        subprocess.check_call(
            ["manipulation/util/geometry_inspector",
             "manipulation/models/iiwa_description/sdf/"
             "iiwa14_no_collision.sdf",
             "--test",
             "--visualize_collisions",
             "--package_path=."])

    def test_pyplot(self):
        subprocess.check_call(
            ["manipulation/util/geometry_inspector",
             "manipulation/models/iiwa_description/sdf/"
             "iiwa14_no_collision.sdf",
             "--test",
             "--pyplot"])
