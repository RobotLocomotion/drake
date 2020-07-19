"""
Tests `pybind11` version information available from Drake.
"""


import unittest

import pydrake.common.pybind11_version as mut


class TestPybind11Version(unittest.TestCase):
    def test_values(self):
        self.assertTrue(isinstance(mut.repository, str))
        self.assertTrue(isinstance(mut.commit, str))
        self.assertTrue(isinstance(mut.sha256, str))
