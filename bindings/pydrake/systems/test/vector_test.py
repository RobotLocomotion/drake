#!/usr/bin/env python
# -*- coding: utf8 -*-

from __future__ import print_function

import copy
import unittest
import numpy as np

from pydrake.systems.framework import (
    BasicVector,
    )


def pass_through(x):
    return x


# TODO(eric.cousineau): Add negative (or positive) test cases for AutoDiffXd
# and Symbolic once they are in the bindings.


class TestReference(unittest.TestCase):
    def test_basic_vector_double(self):
        # Test constructing vectors of sizes [0, 1, 2], and ensure that we can
        # consruct from both lists and `np.array` objects with no ambiguity.
        for n in range(3):
            init = range(n)
            if n > 0:
                init[0] = float(0)
            after_add = [x + 1 for x in init]
            after_set = [x + 10 for x in init]

            for func in [pass_through, np.array]:
                # Ensure that we can get vectors templated on double by
                # reference.
                init = func(init)
                value_data = BasicVector(init)
                value = value_data.get_mutable_value()
                # TODO(eric.cousineau): Determine if there is a way to extract
                # the pointer referred to by the buffer (e.g. `value.data`).
                value[:] += 1
                expected = func(after_add)
                self.assertTrue(np.allclose(value, expected))
                self.assertTrue(np.allclose(value_data.get_value(), expected))
                self.assertTrue(
                    np.allclose(value_data.get_mutable_value(), expected))
                expected = func(after_set)
                value_data.SetFromVector(expected)
                self.assertTrue(np.allclose(value, expected))
                self.assertTrue(np.allclose(value_data.get_value(), expected))
                self.assertTrue(
                    np.allclose(value_data.get_mutable_value(), expected))
                # Ensure we can construct from size.
                value_data = BasicVector(n)
                self.assertEquals(value_data.size(), n)


if __name__ == '__main__':
    unittest.main()
