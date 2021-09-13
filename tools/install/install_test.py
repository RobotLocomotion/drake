import argparse
import functools
import os
import re
import sys
import unittest

import install_test_helper


class InstallTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # Install into bazel read-only temporary directory.  We expect this
        # method to be called exactly once, so we assert that the install
        # directory must not exist beforehand, but must exist afterward.
        cls._installation_folder = install_test_helper.get_install_dir()
        assert not os.path.exists(cls._installation_folder)
        install_test_helper.install()
        assert os.path.isdir(cls._installation_folder)

    def test_basic_paths(self):
        # Verify install directory content.
        content = set(os.listdir(self._installation_folder))
        self.assertSetEqual(set(['bin', 'include', 'lib', 'share']), content)

    def _run_one_command(self, test_command):
        # Our launched processes should be independent, not inherit their
        # runfiles from the install_test.py runner.
        env = dict(os.environ)
        for key in ["RUNFILES_MANIFEST_FILE", "RUNFILES_DIR", "TEST_SRCDIR"]:
            if key in env:
                del env[key]

        # Execute the test_command.
        print("+ {}".format(test_command), file=sys.stderr)
        install_test_helper.check_call(
            [os.path.join(os.getcwd(), test_command)],
            env=env)


def _convert_test_command_to_test_case_name(test_command):
    program = test_command.split()[0]
    basename = os.path.basename(program)
    bare_name = os.path.splitext(basename)[0]
    identifier = re.sub("[^0-9a-zA-Z]+", "_", bare_name)
    if identifier.startswith("test_"):
        test_case_name = identifier
    else:
        test_case_name = "test_" + identifier
    return test_case_name


def main():
    # Locate the command-line argument that provides the list of test commands.
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--install_tests_filename', required=True)
    args, unparsed = parser.parse_known_args()
    new_argv = ["install_test"] + unparsed

    # Read the list of tests.
    with open(args.install_tests_filename, 'r') as f:
        lines = f.readlines()

    # Add them as individual tests.
    for one_line in lines:
        test_command = one_line.strip()
        test_case_name = _convert_test_command_to_test_case_name(test_command)
        setattr(InstallTest, test_case_name, functools.partialmethod(
            InstallTest._run_one_command, test_command=test_command))

    # Delegate to unittest.
    unittest.main(argv=new_argv)


if __name__ == '__main__':
    main()
