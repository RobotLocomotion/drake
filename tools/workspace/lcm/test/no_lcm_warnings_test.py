import os
import unittest
import warnings

from lcm import LCM, EventLog


class Test(unittest.TestCase):
    def test_publish(self):
        """
        Ensures that no warnings are issued using `lcm.publish()`.
        """
        lcm = LCM("memq://")
        with warnings.catch_warnings():
            warnings.simplefilter("error", DeprecationWarning)
            lcm.publish("TEST_CHANNEL", b"")

    def test_event_log(self):
        """
        Ensures no crashes on construction / destruction.
        """
        dut = EventLog(path=f"{os.environ['TEST_TMPDIR']}/lcm.log", mode="w")
        dut.close()
        del dut
