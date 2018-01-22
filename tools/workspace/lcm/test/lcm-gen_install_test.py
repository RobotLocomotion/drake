#!/usr/bin/env python

import os
import subprocess
import unittest
import install_test_helper


class TestLcmGen(unittest.TestCase):
    def test_install(self):
        # Get install directory
        install_dir = install_test_helper.get_install_dir()
        executable_folder = os.path.join(install_dir, "bin")
        subprocess.check_call(
            [os.path.join(executable_folder, "lcm-gen"), "--help"],
            )


if __name__ == '__main__':
    unittest.main()
