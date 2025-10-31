import os
from pathlib import Path
import subprocess
import unittest


class ScriptsTest(unittest.TestCase):
    def test_ideal_stiction(self):
        source = Path(".").absolute()
        temp = Path(os.environ["TEST_TMPDIR"])
        subprocess.check_call(
            [source / "multibody/plant/images/ideal_stiction"],
            cwd=temp,
        )
        self.assertTrue((temp / "ideal_stiction.png").exists())

    def test_stiction(self):
        source = Path(".").absolute()
        temp = Path(os.environ["TEST_TMPDIR"])
        subprocess.check_call(
            [source / "multibody/plant/images/stiction"],
            cwd=temp,
        )
        self.assertTrue((temp / "stribeck.png").exists())
