import copy
import unittest

from pydrake.common.test_utilities.deprecation import catch_drake_warnings
from pydrake.lcm import DrakeLcm, DrakeLcmInterface, DrakeLcmParams, Subscriber

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
        dut.get_lcm_url()
        DrakeLcm(lcm_url="")
        # Test virtual function names.
        dut.Publish
        dut.Subscribe
        dut.SubscribeMultichannel
        dut.SubscribeAllChannels
        dut.HandleSubscriptions

    def test_params(self):
        dut = DrakeLcmParams()
        dut.lcm_url = "memq://123"
        dut.channel_suffix = "_foo"
        dut.defer_initialization = True
        instance = DrakeLcm(params=dut)
        self.assertTrue(instance.get_lcm_url(), "memq://123")
        self.assertIn("lcm_url", repr(dut))
        copy.copy(dut)

    def _handler(self, raw):
        quat = lcmt_quaternion.decode(raw)
        self.assertTupleEqual((quat.w, quat.x, quat.y, quat.z), self.wxyz)
        self.count += 1

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

    def test_round_trip_single_channel(self):
        dut = DrakeLcm()
        channel = "ROUND_TRIP_SINGLE"

        received = []
        dut.Subscribe(
            channel=channel,
            handler=lambda data: received.append(data))
        for _ in range(10):
            dut.Publish(channel=channel, buffer=self.quat.encode())
            dut.HandleSubscriptions(timeout_millis=100)
            if len(received) > 0:
                break

        data = received[0]
        self.assertEqual(data, self.quat.encode())

    def test_round_trip_multi_channel(self):
        dut = DrakeLcm()
        channel = "ROUND_TRIP_MULTI"

        received = []
        dut.SubscribeMultichannel(
            regex="ROUND.*MULTI",
            handler=lambda name, data: received.append((name, data)))
        for _ in range(10):
            dut.Publish(channel=channel, buffer=self.quat.encode())
            dut.HandleSubscriptions(timeout_millis=100)
            if len(received) > 0:
                break

        name, data = received[0]
        self.assertEqual(name, channel)
        self.assertEqual(data, self.quat.encode())

    def test_round_trip_all_channels(self):
        dut = DrakeLcm()
        channel = "ROUND_TRIP_ALL"

        received = []
        dut.SubscribeAllChannels(
            handler=lambda name, data: received.append((name, data)))
        for _ in range(10):
            dut.Publish(channel=channel, buffer=self.quat.encode())
            dut.HandleSubscriptions(timeout_millis=100)
            if len(received) > 0:
                break

        name, data = received[0]
        self.assertEqual(name, channel)
        self.assertEqual(data, self.quat.encode())
