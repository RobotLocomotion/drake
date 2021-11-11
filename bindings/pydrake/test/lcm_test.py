import unittest

from pydrake.common.test_utilities.deprecation import catch_drake_warnings
from pydrake.lcm import DrakeLcm, DrakeLcmInterface, DrakeMockLcm, Subscriber

from drake import lcmt_quaternion


class TestLcm(unittest.TestCase):
    def setUp(self):
        # Create a simple ground-truth message.
        self.wxyz = (1, 2, 3, 4)
        quat = lcmt_quaternion()
        quat.w, quat.x, quat.y, quat.z = self.wxyz
        self.quat = quat
        self.count = 0

    def test_lcm(self):
        dut = DrakeLcm()
        self.assertIsInstance(dut, DrakeLcmInterface)
        DrakeLcm(lcm_url="")
        DrakeLcm(lcm_url="", defer_initialization=True)
        # Test virtual function names.
        dut.Publish
        dut.HandleSubscriptions

    def _handler(self, raw):
        quat = lcmt_quaternion.decode(raw)
        self.assertTupleEqual((quat.w, quat.x, quat.y, quat.z), self.wxyz)
        self.count += 1

    def test_mock_lcm_roundtrip(self):
        with catch_drake_warnings(expected_count=1):
            dut = DrakeMockLcm()
        self.assertIsInstance(dut, DrakeLcmInterface)
        dut.Subscribe(channel="TEST_CHANNEL", handler=self._handler)
        dut.Publish(channel="TEST_CHANNEL", buffer=self.quat.encode())
        self.assertEqual(self.count, 0)
        dut.HandleSubscriptions(0)
        self.assertEqual(self.count, 1)

    def test_subscriber(self):
        lcm = DrakeLcm()
        dut = Subscriber(lcm=lcm, channel="CHANNEL", lcm_type=lcmt_quaternion)
        lcm.Publish(channel="CHANNEL", buffer=self.quat.encode())
        self.assertEqual(dut.count, 0)
        self.assertEqual(len(dut.raw), 0)
        self.assertEqual(dut.message.w, 0)
        self.assertEqual(dut.message.x, 0)
        self.assertEqual(dut.message.y, 0)
        self.assertEqual(dut.message.z, 0)
        lcm.HandleSubscriptions(0)
        self.assertEqual(dut.count, 1)
        self.assertEqual(dut.raw, self.quat.encode())
        self.assertEqual(dut.message.w, self.quat.w)
        self.assertEqual(dut.message.x, self.quat.x)
        self.assertEqual(dut.message.y, self.quat.y)
        self.assertEqual(dut.message.z, self.quat.z)
