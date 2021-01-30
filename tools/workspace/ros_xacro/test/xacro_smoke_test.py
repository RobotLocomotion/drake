import subprocess
import unittest


class XacroSmokeTest(unittest.TestCase):

    def _diff_file(self, filename):
        self.maxDiff = None
        with open(filename) as f:
            actual = f.read()
        with open(filename + ".expected") as f:
            expected = f.read()
        # We use an atypical (expected, actual) order here so that the diff
        # shown during failure has the correct sense of added vs removed lines.
        self.assertMultiLineEqual(expected, actual)

    def test_samples(self):
        """Check use of xacro_filegroup and the expected result.
        """
        self._diff_file("tools/workspace/ros_xacro/test/sample1.xml")
        self._diff_file("tools/workspace/ros_xacro/test/sample2.xml")
