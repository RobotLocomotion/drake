import unittest

from examples.hardware_sim.test.hardware_sim_test_common import HardwareSimTest


class HardwareSimPyTest(HardwareSimTest, unittest.TestCase):
    """Smoke test for our hardware_sim_py program.

    All of the actual test cases are defined in our base class.
    """

    def __init__(self, *args, **kwargs):
        self._sim_lang = "py"
        super().__init__(*args, **kwargs)
