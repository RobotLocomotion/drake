import unittest
import warnings

from lcm import LCM


class Test(unittest.TestCase):

    def test_publish(self):
        """
        Ensures that no warnings are issued using `lcm.publish()`.
        """
        lcm = LCM("memq://")
        with warnings.catch_warnings():
            warnings.simplefilter("error", DeprecationWarning)
            lcm.publish("TEST_CHANNEL", b"")
