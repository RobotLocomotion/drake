import copy
import dataclasses
import unittest

from drake import lcmt_quaternion
from pydrake.lcm import DrakeLcm, DrakeLcmInterface, DrakeLcmParams, Subscriber


class TestLcm(unittest.TestCase):
    def setUp(self):
        # Create a simple ground-truth message.
        self.wxyz = (1, 2, 3, 4)
        quat = lcmt_quaternion()
        quat.w, quat.x, quat.y, quat.z = self.wxyz
        self.quat = quat
        self.count = 0

    def test_lcm(self):
        self.assertTrue(DrakeLcm.available())
        dut = DrakeLcm()
        self.assertIsInstance(dut, DrakeLcmInterface)
        dut.get_lcm_url()
        DrakeLcm(lcm_url="")
        # Test virtual function names.
        dut.Publish  # noqa: B018
        dut.Subscribe  # noqa: B018
        dut.SubscribeMultichannel  # noqa: B018
        dut.SubscribeAllChannels  # noqa: B018
        dut.HandleSubscriptions  # noqa: B018

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
            channel=channel, handler=lambda data: received.append(data)
        )
        for _ in range(10):
            dut.Publish(channel=channel, buffer=self.quat.encode())
            dut.HandleSubscriptions(timeout_millis=100)
            if len(received) > 0:
                break

        data = received[0]
        self.assertEqual(data, self.quat.encode())

    def test_unhashable_handlers(self):
        """The handlers are borrowed by C++ and kept alive by the DrakeLcm
        (see #24162). A callable need not be hashable to be a handler.
        """

        @dataclasses.dataclass
        class Handler:
            received: list

            def __call__(self, *args):
                self.received.append(args)

        # Dataclasses with eq=True (the default) are not hashable.
        self.assertIsNone(Handler.__hash__)

        dut = DrakeLcm()
        single = Handler([])
        multi = Handler([])
        every = Handler([])
        dut.Subscribe(channel="UNHASHABLE_SINGLE", handler=single)
        dut.SubscribeMultichannel(regex="UNHASHABLE_MULTI", handler=multi)
        dut.SubscribeAllChannels(handler=every)
        del single, multi, every  # DrakeLcm must keep the handlers alive.

        # Look the handlers up again via the peers the DrakeLcm retains.
        for channel in ["UNHASHABLE_SINGLE", "UNHASHABLE_MULTI"]:
            for _ in range(10):
                dut.Publish(channel=channel, buffer=self.quat.encode())
                dut.HandleSubscriptions(timeout_millis=100)
        peers = list(dut._pydrake_internal_ref_cycle_peers.values())
        handlers = [x for x in peers if isinstance(x, Handler)]
        self.assertEqual(len(handlers), 3)
        self.assertTrue(all(len(h.received) > 0 for h in handlers))

    def test_round_trip_multi_channel(self):
        dut = DrakeLcm()
        channel = "ROUND_TRIP_MULTI"

        received = []
        dut.SubscribeMultichannel(
            regex="ROUND.*MULTI",
            handler=lambda name, data: received.append((name, data)),
        )
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
            handler=lambda name, data: received.append((name, data))
        )
        for _ in range(10):
            dut.Publish(channel=channel, buffer=self.quat.encode())
            dut.HandleSubscriptions(timeout_millis=100)
            if len(received) > 0:
                break

        name, data = received[0]
        self.assertEqual(name, channel)
        self.assertEqual(data, self.quat.encode())
