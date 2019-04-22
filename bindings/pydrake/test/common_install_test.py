import os
import install_test_helper
import unittest


class TestCommonInstall(unittest.TestCase):
    def testDrakeFindResourceOrThrowInInstall(self):
        # Override PYTHONPATH to only use the installed `pydrake` module.
        install_dir = install_test_helper.get_install_dir()
        env_python_path = "PYTHONPATH"
        tool_env = dict(os.environ)
        tool_env[env_python_path] = \
            install_test_helper.get_python_site_packages_dir(install_dir)
        data_folder = os.path.join(install_dir, "share", "drake")
        # Calling `pydrake.getDrakePath()` twice verifies that there
        # is no memory allocation issue in the C code.
        output_path = install_test_helper.check_output(
            [install_test_helper.get_python_executable(),
             "-c", "import pydrake; print(pydrake.getDrakePath());\
             import pydrake; print(pydrake.getDrakePath())"
             ],
            env=tool_env,
            ).strip()
        self.assertIn(data_folder, output_path)


if __name__ == '__main__':
    unittest.main()
