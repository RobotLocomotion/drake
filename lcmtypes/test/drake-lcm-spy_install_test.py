import os
import subprocess
import unittest
import install_test_helper


class TestCommonInstall(unittest.TestCase):
    def testInstall(self):
        tmp_folder = "tmp"
        result = install_test_helper.install(tmp_folder,
                                             ['bin', 'lib', 'share'])
        self.assertEqual(None, result)
        executable_folder = os.path.join(tmp_folder, "bin")
        try:
            subprocess.check_output(
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
            self.assertIn("usage: lcm-spy [options]", e.output)


if __name__ == '__main__':
    unittest.main()
