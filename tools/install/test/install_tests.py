#!/usr/bin/env python

import os
import unittest
import sys
import install_test_helper


class TestCommonInstall(unittest.TestCase):
    def testDrakeInstall(self):
        # Install into bazel read-only temporary directory. The temporary
        # directory does not need to be removed as bazel tests are run in a
        # scratch space.
        install_test_helper.install()
        installation_folder = install_test_helper.get_install_dir()
        self.assertTrue(os.path.isdir(installation_folder))
        # Verify install directory content.
        content = os.listdir(installation_folder)
        self.assertSetEqual(set(['bin', 'include', 'lib', 'plugins', 'share']),
                            set(content))
        # Execute the install actions.
        sys.stderr.write(str(sys.argv)+str('\n'))
        if len(sys.argv) <= 1:
            return
        # Will fail (on purpose) if more than one file.
        cmds, = sys.argv[1:]

        with open(cmds, "r") as f:
            lines = f.readlines()

        for cmd in lines:
            cmd = cmd.split('#')[0].strip()
            install_test_helper.check_call(os.path.join(os.getcwd(), cmd))


if __name__ == '__main__':
    unittest.main(argv=sys.argv[1:] if len(sys.argv) > 1 else None)
