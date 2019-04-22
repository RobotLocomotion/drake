import os
import sys
import install_test_helper
import unittest


class TestInstall(unittest.TestCase):
    def test_install(self):
        # Fail (on purpose) if not given exactly one command line argument.
        self.assertEqual(len(sys.argv), 2)
        with open(sys.argv[1], 'r') as f:
            lines = f.readlines()

        # Install into bazel read-only temporary directory. The temporary
        # directory does not need to be removed as bazel tests are run in a
        # scratch space.
        install_test_helper.install()
        installation_folder = install_test_helper.get_install_dir()
        self.assertTrue(os.path.isdir(installation_folder))

        # Verify install directory content.
        content = set(os.listdir(installation_folder))
        self.assertSetEqual(set(['bin', 'include', 'lib', 'share']), content)

        # Our launched processes should be independent, not inherit their
        # runfiles from the install_test.py runner.
        cmd_env = dict(os.environ)
        for key in ["RUNFILES_MANIFEST_FILE", "RUNFILES_DIR", "TEST_SRCDIR"]:
            if key in cmd_env:
                del cmd_env[key]

        # Execute the install actions.
        for cmd in lines:
            cmd = cmd.strip()
            print("+ {}".format(cmd))
            install_test_helper.check_call(
                [os.path.join(os.getcwd(), cmd)],
                env=cmd_env)


if __name__ == '__main__':
    unittest.main(argv=["TestInstall"])
