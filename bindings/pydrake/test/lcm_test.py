import unittest

from pydrake.lcm import DrakeLcm, DrakeLcmInterface, DrakeMockLcm

from robotlocomotion import quaternion_t


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

    def test_mock_lcm_get_last_published_message(self):
        dut = DrakeMockLcm()
        dut.Publish(channel="TEST_CHANNEL", buffer=self.quat.encode())
        raw = dut.get_last_published_message("TEST_CHANNEL")
        self.assertEqual(raw, self.quat.encode())

    def test_mock_lcm_induce_subscriber_callback(self):
        dut = DrakeMockLcm()
        dut.Subscribe(channel="TEST_CHANNEL", handler=self._handler)
        dut.InduceSubscriberCallback(
            channel="TEST_CHANNEL", buffer=self.quat.encode())
        self.assertEqual(self.count, 1)
