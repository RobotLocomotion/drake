from __future__ import absolute_import, division, print_function

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
        subprocess.check_call([os.path.join(executable_folder, "lcm-gen")])


if __name__ == '__main__':
    unittest.main()
