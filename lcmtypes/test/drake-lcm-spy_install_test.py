import os
import subprocess
import unittest
import install_test_helper


class TestLcmSpy(unittest.TestCase):
    def test_install(self):
        # Get install directory.
        install_dir = install_test_helper.get_install_dir()
        executable_folder = os.path.join(install_dir, "bin")
        try:
            install_test_helper.check_output(
                [os.path.join(executable_folder, "drake-lcm-spy"), "--help"],
                stderr=subprocess.STDOUT
                )
            # If the process doesn't fail, we cannot test the string
            # returned in the exception output. Since this should not
            # happen, we fail here. If lcm is updated to return 0 when
            # "--help" is called, this test will fail and will need to be
            # updated.
            self.fail(
                "drake-lcm-spy execution passed instead of failing. Update test"  # noqa
                )
        except subprocess.CalledProcessError as e:
            self.assertIn("usage: lcm-spy [options]", e.output.decode("utf8"))


if __name__ == '__main__':
    unittest.main()
