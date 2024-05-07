import unittest

from examples.hardware_sim.test.hardware_sim_test_common import HardwareSimTest


class HardwareSimCcTest(HardwareSimTest, unittest.TestCase):
    """Smoke test for our hardware_sim_cc program.

    All of the actual test cases are defined in our base class.
    """

    def __init__(self, *args, **kwargs):
        self._sim_lang = "cc"
        super().__init__(*args, **kwargs)
