#!/usr/bin/env python

# This file is configured automatically.

import os
import unittest
import install_test_helper


cmds = []


class TestCommonInstall(unittest.TestCase):
    def testDrakeInstall(self):
        # Install into bazel read-only temporary directory. The temporary
        # directory does not need to be removed as bazel tests are run in a
        # scratch space.
        install_test_helper.install()
        installation_folder = install_test_helper.get_install_dir()
        self.assertTrue(os.path.isdir(installation_folder))
        # Execute the install actions.
        for cmd in cmds:
            install_test_helper.check_call(os.path.join(os.getcwd(), cmd))
        content = os.listdir(installation_folder)
        self.assertSetEqual(set(['bin', 'include', 'lib', 'plugins', 'share']),
                            set(content))


if __name__ == '__main__':
    unittest.main()
