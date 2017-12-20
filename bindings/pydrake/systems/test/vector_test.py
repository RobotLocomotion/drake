#!/usr/bin/env python
# -*- coding: utf8 -*-

from __future__ import print_function

import copy
import unittest
import numpy as np

from pydrake.systems.framework import (
    BasicVector,
    )

# TODO(eric.cousineau): Add negative test cases for AutoDiffXd and Symbolic
# once they are in the bindings.


class TestReference(unittest.TestCase):
    def test_basic_vector_double(self):
        # Ensure that we can get vectors templated on double by reference.
        init = [1., 2, 3]
        value_data = BasicVector(init)
        value = value_data.get_mutable_value()
        # TODO(eric.cousineau): Determine if there is a way to extract the
        # pointer referred to by the buffer (e.g. `value.data`).
        value[:] += 1
        expected = [2., 3, 4]
        self.assertTrue(np.allclose(value, expected))
        self.assertTrue(np.allclose(value_data.get_value(), expected))
        self.assertTrue(np.allclose(value_data.get_mutable_value(), expected))
        expected = [5., 6, 7]
        value_data.SetFromVector(expected)
        self.assertTrue(np.allclose(value, expected))
        self.assertTrue(np.allclose(value_data.get_value(), expected))
        self.assertTrue(np.allclose(value_data.get_mutable_value(), expected))


if __name__ == '__main__':
    unittest.main()
