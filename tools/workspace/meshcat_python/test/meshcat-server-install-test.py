import unittest

import install_test_helper


class TestMeshcatServerInstall(unittest.TestCase):
    def test_run_meshcat_server_and_kill(self):
        install_test_helper.run_and_kill(['bin/meshcat-server'])


if __name__ == '__main__':
    unittest.main()
