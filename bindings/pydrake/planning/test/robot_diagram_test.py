import unittest

# import numpy as np

from pydrake.common.test_utilities import numpy_compare
import pydrake.planning as mut


class TestRobotDiagram(unittest.TestCase):
    @numpy_compare.check_all_types
    def test_robot_diagram(self, T):
        Class = mut.RobotDiagram_[T]
        dut = Class()
