import unittest

from pydrake.lcm import DrakeLcm, DrakeLcmInterface, DrakeMockLcm

from robotlocomotion import quaternion_t


class TestLcm(unittest.TestCase):
    def test_lcm(self):
        dut = DrakeLcm()
        self.assertIsInstance(dut, DrakeLcmInterface)
        # Quickly start and stop the receiving thread.
        dut.StartReceiveThread()
        dut.StopReceiveThread()
        # Test virtual function names.
        dut.Publish

    def test_mock_lcm(self):
        dut = DrakeMockLcm()
        self.assertIsInstance(dut, DrakeLcmInterface)

        # Create a simple ground-truth message.
        msg = quaternion_t()
        wxyz = (1, 2, 3, 4)
        msg.w, msg.x, msg.y, msg.z = wxyz

        dut.Publish(channel="TEST_CHANNEL", buffer=msg.encode())
        raw = dut.get_last_published_message("TEST_CHANNEL")
        self.assertEqual(raw, msg.encode())

        # Use an array so that we can mutate the contained value.
        called = [False]

        def handler(raw):
            msg = quaternion_t.decode(raw)
            self.assertTupleEqual((msg.w, msg.x, msg.y, msg.z), wxyz)
            called[0] = True

        dut.Subscribe(channel="TEST_CHANNEL", handler=handler)
        dut.InduceSubscriberCallback(
            channel="TEST_CHANNEL", buffer=msg.encode())
        self.assertTrue(called[0])
