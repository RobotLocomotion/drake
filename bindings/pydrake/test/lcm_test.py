import unittest

from pydrake.lcm import DrakeLcm, DrakeLcmInterface, DrakeMockLcm


class TestLcm(unittest.TestCase):
    def test_lcm(self):
        dut = DrakeLcm()
        self.assertIsInstance(dut, DrakeLcmInterface)
        # Quickly start and stop the receiving thread.
        dut.StartReceiveThread()
        dut.StopReceiveThread()

    def test_mock_lcm(self):
        dut = DrakeMockLcm()
        self.assertIsInstance(dut, DrakeLcmInterface)
