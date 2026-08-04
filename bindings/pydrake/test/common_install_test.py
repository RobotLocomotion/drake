import os
import sys
import unittest

import install_test_helper


class TestCommonInstall(unittest.TestCase):
    def testDrakeFindResourceOrThrowInInstall(self):
        install_dir = install_test_helper.get_install_dir()

        # Override PYTHONPATH to only use the installed `pydrake` module, plus
        # the drake.venv if it exists.
        paths = [install_test_helper.get_python_site_packages_dir(install_dir)]
        for item in sys.path:
            if "/venv.drake/" in item:
                paths.append(item)
        tool_env = dict(os.environ)
        tool_env["PYTHONPATH"] = ":".join(paths)

        # Calling `pydrake.getDrakePath()` twice verifies that there
        # is no memory allocation issue in the C code.
        output_path = install_test_helper.check_output(
            [
                install_test_helper.get_python_executable(),
                "-c",
                "import pydrake; print(pydrake.getDrakePath());\
                 import pydrake; print(pydrake.getDrakePath())",
            ],
            env=tool_env,
        ).strip()
        data_folder = os.path.join(install_dir, "share", "drake")
        self.assertIn(data_folder, output_path)


if __name__ == "__main__":
    unittest.main()
