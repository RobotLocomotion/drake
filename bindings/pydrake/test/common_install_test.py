import os
import unittest

import install_test_helper


class TestCommonInstall(unittest.TestCase):
    def testDrakeFindResourceOrThrowInInstall(self):
        # Override PYTHONPATH to only use the installed `pydrake` module.
        install_dir = install_test_helper.get_install_dir()
        data_folder = os.path.join(install_dir, "share", "drake")
        # Calling `pydrake.getDrakePath()` twice verifies that there
        # is no memory allocation issue in the C code.
        output_path = install_test_helper.check_output(
            [install_test_helper.get_python_executable(),
             "-c", "import pydrake; print(pydrake.getDrakePath());\
             import pydrake; print(pydrake.getDrakePath())"
             ],
            env=install_test_helper.get_python_env(install_dir),
            ).strip()
        found_install_path = (data_folder in output_path)
        self.assertTrue(found_install_path)


if __name__ == '__main__':
    unittest.main()
