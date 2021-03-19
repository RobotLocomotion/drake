import os
from subprocess import run
import sys
import unittest


def get_isolated_env():
    home = "/do_not_read_or_write"
    if sys.platform == "darwin":
        path = "/usr/local/bin:/usr/bin:/bin"
    else:
        path = "/usr/bin:/bin"
    env = {
        "HOME": home,
        "LANG": "en_US.UTF-8",
        "PATH": path,
    }
    return env


class TestGenerate(unittest.TestCase):
    def test_generate(self):
        generate_binary = "tools/autopybind/generate"
        output_dir = os.path.join(os.environ["TEST_TMPDIR"], "output")

        run(
            [generate_binary, "--output_dir", output_dir],
            env=get_isolated_env(),
            check=True,
        )
        self.assertTrue(os.path.isdir(output_dir), output_dir)

        # TODO(eric.cousineau): Add some quick text checks on the generated
        # source code.
