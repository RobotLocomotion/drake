import os
import subprocess
import unittest


class TestShowModel(unittest.TestCase):

    def test_show(self):
        """Test that show_model doesn't crash."""

        # Use a bogus LCM URL so that we don't clobber people's desktop
        # visualizers when they run the unit tests.
        env = dict(os.environ)
        env["LCM_DEFAULT_URL"] = "udpm://239.87.65.43:2109?ttl=0"

        # Check a random SDF.
        subprocess.check_call([
            "manipulation/util/show_model",
            "examples/acrobot/Acrobot.sdf"
        ], env=env)

        # Check a random URDF.
        subprocess.check_call([
            "manipulation/util/show_model",
            "examples/pendulum/Pendulum.urdf"
        ], env=env)
