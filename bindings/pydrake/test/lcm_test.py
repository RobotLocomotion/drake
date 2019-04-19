import unittest

from pydrake.lcm import DrakeLcm, DrakeLcmInterface, DrakeMockLcm, Subscriber

from robotlocomotion import quaternion_t

from pydrake.common.test_utilities.deprecation import catch_drake_warnings


class TestLcm(unittest.TestCase):
    def setUp(self):
        # Create a simple ground-truth message.
        self.wxyz = (1, 2, 3, 4)
        quat = quaternion_t()
        quat.w, quat.x, quat.y, quat.z = self.wxyz
        self.quat = quat
        self.count = 0

    def test_lcm(self):
        dut = DrakeLcm()
        self.assertIsInstance(dut, DrakeLcmInterface)
        # Quickly start and stop the receiving thread.
        with catch_drake_warnings(expected_count=2):
            dut.StartReceiveThread()
            dut.StopReceiveThread()
        # Test virtual function names.
        dut.Publish
        dut.HandleSubscriptions

    def _handler(self, raw):
        quat = quaternion_t.decode(raw)
        self.assertTupleEqual((quat.w, quat.x, quat.y, quat.z), self.wxyz)
        self.count += 1

    def test_mock_lcm_roundtrip(self):
        dut = DrakeMockLcm()
        self.assertIsInstance(dut, DrakeLcmInterface)
        dut.Subscribe(channel="TEST_CHANNEL", handler=self._handler)
        dut.Publish(channel="TEST_CHANNEL", buffer=self.quat.encode())
        self.assertEqual(self.count, 0)
        dut.HandleSubscriptions(0)
        self.assertEqual(self.count, 1)

    def test_subscriber(self):
        mock = DrakeMockLcm()
        dut = Subscriber(lcm=mock, channel="CHANNEL", lcm_type=quaternion_t)
        mock.Publish(channel="CHANNEL", buffer=self.quat.encode())
        self.assertEqual(dut.count, 0)
        self.assertEqual(len(dut.raw), 0)
        self.assertEqual(dut.message.w, 0)
        self.assertEqual(dut.message.x, 0)
        self.assertEqual(dut.message.y, 0)
        self.assertEqual(dut.message.z, 0)
        mock.HandleSubscriptions(0)
        self.assertEqual(dut.count, 1)
        self.assertEqual(dut.raw, self.quat.encode())
        self.assertEqual(dut.message.w, self.quat.w)
        self.assertEqual(dut.message.x, self.quat.x)
        self.assertEqual(dut.message.y, self.quat.y)
        self.assertEqual(dut.message.z, self.quat.z)

    def test_mock_lcm_get_last_published_message_deprecated(self):
        dut = DrakeMockLcm()
        dut.Publish(channel="TEST_CHANNEL", buffer=self.quat.encode())
        with catch_drake_warnings(expected_count=1):
            raw = dut.get_last_published_message("TEST_CHANNEL")
            self.assertEqual(raw, self.quat.encode())

    def test_mock_lcm_induce_subscriber_callback_deprecated(self):
        dut = DrakeMockLcm()
        dut.Subscribe(channel="TEST_CHANNEL", handler=self._handler)
        with catch_drake_warnings(expected_count=1):
            dut.InduceSubscriberCallback(
                channel="TEST_CHANNEL", buffer=self.quat.encode())
        self.assertEqual(self.count, 1)
