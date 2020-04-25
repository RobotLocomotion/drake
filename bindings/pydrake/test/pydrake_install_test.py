"""
Test `pydrake` in install tree.
"""

import os
from os.path import abspath
import subprocess
import sys
from textwrap import dedent
import unittest

import install_test_helper

# N.B. `pydrake` no longer supports Python 2, so it will never be the actual
# interpreter.
PY_BAD_BIN = "/usr/bin/python2"


class TestPydrakeInstall(unittest.TestCase):
    def setUp(self):
        # Get install directory.
        self.install_dir = install_test_helper.get_install_dir()
        # Override PYTHONPATH to only use the installed `pydrake` module.
        self.env = install_test_helper.get_install_env()
        self.env["PYTHONPATH"] = \
            install_test_helper.get_python_site_packages_dir(self.install_dir)

    def test_all(self):
        # Ensure we can import all user-visible modules.
        script = "import pydrake.all"
        subprocess.check_call(
            [install_test_helper.get_python_executable(), "-c", script],
            env=self.env,
        )

    def test_find_resource(self):
        data_folder = os.path.join(self.install_dir, "share", "drake")
        # N.B. Call function twice to verify that there is no memory allocation
        # issue in the C code.
        script = dedent("""\
            from pydrake.common import FindResourceOrThrow
            FindResourceOrThrow("drake/common/resource_tool")
            print(FindResourceOrThrow("drake/common/resource_tool"))
            """)
        output_path = subprocess.check_output(
            [install_test_helper.get_python_executable(), "-c", script],
            env=self.env,
            encoding="utf8",
        ).strip()
        # N.B. Normalize path.
        self.assertIn(abspath(data_folder), abspath(output_path))

    @unittest.skipIf(sys.platform == "darwin", f"Only tested on Ubuntu")
    def test_bad_interpreter(self):
        self.assertTrue(os.path.isfile(PY_BAD_BIN))
        # Ensure we fail fast with a bad interpreter on any type of import.
        script = "import pydrake.geometry"
        stat = subprocess.run(
            [PY_BAD_BIN, "-c", script],
            env=self.env,
            encoding="utf8",
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        )
        self.assertNotEqual(stat.returncode, 0)
        self.assertIn("does not match compiled version", stat.stdout)


if __name__ == '__main__':
    unittest.main()
