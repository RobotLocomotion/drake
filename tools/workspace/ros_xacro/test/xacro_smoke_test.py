import subprocess
import unittest


class XacroSmokeTest(unittest.TestCase):

    def test_simple_file(self):
        """Check that we can run xacro on a simple file and get the expected
        post-processed result.
        """
        self.maxDiff = None
        proc = subprocess.run([
            "external/ros_xacro/xacro",
            "tools/workspace/ros_xacro/test/sample.xacro",
        ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.assertEqual(proc.returncode, 0, proc.stderr.decode("utf-8"))
        actual = proc.stdout.decode("utf-8")
        with open("tools/workspace/ros_xacro/test/sample.xml.expected") as f:
            expected = f.read()
        # We use an atypical (expected, actual) order here so that the diff
        # shown during failure has the correct sense of added vs removed lines.
        self.assertMultiLineEqual(expected.strip(), actual.strip())
