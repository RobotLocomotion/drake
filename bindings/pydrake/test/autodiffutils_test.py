from __future__ import absolute_import, division, print_function

import unittest
import numpy as np
from pydrake.autodiffutils import (AutoDiffXd)


class TestAutoDiffXd(unittest.TestCase):
    def test_div(self):
        x = AutoDiffXd(1, [1., 0])
        y = x/2.
        self.assertAlmostEquals(y.value(), .5)
        np.testing.assert_almost_equal(y.derivatives(), [.5, 0.])

    def test_pow(self):
        x = AutoDiffXd(1., [1., 0., 0.])
        y = x**2
        self.assertAlmostEquals(y.value(), 1.)
        np.testing.assert_almost_equal(y.derivatives(), [2., 0., 0.])
