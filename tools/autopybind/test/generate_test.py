import os
from subprocess import run
import unittest


class TestGenerate(unittest.TestCase):
    def test_generate(self):
        generate_binary = "tools/autopybind/generate"
        output_dir = os.path.join(os.environ["TEST_TMPDIR"], "output")

        print(os.getcwd())
        run([generate_binary, "--output_dir", output_dir], check=True)
        self.assertTrue(os.path.isdir(output_dir), output_dir)

        # TODO(eric.cousineau): Add some quick text checks on the generated
        # source code.
