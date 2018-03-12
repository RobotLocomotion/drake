import unittest

from pydrake.lcm import DrakeLcm, DrakeLcmInterface, DrakeMockLcm


class TestLcm(unittest.TestCase):
    def _test_lcm_interface(self, lcm):
        self.assertIsInstance(lcm, DrakeLcmInterface)
        # Quickly start and stop the receiving thread.
        lcm.StartReceiveThread()
        lcm.StopReceiveThread()

    def test_lcm(self):
        self._test_lcm_interface(DrakeLcm())

    def test_mock_lcm(self):
        self._test_lcm_interface(DrakeMockLcm())
