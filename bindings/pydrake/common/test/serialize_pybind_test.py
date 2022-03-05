import inspect
import unittest

from pydrake.common.test.serialize_test_util import (
    MyData,
    MyData2
)


class TestSerializePybind(unittest.TestCase):

    def test_structure(self):
        """Tests that the class properties were all bound correctly."""
        dut = MyData()

        self.assertEqual(dut.foo, 0.0)
        dut.foo = 1.0
        self.assertEqual(dut.foo, 1.0)

        self.assertListEqual(dut.bar, [])
        dut.bar = [1.0, 2.0]
        self.assertListEqual(dut.bar, [1.0, 2.0])

    def test_docs(self):
        self.assertEqual(inspect.getdoc(MyData.foo),
                         "This is the field docstring for foo.")
        self.assertEqual(inspect.getdoc(MyData.bar),
                         "This is the field docstring for bar.")

    def test_binding_with_no_docstrings(self):
        dut = MyData2()
        self.assertEqual(dut.quux, 0.0)
        dut.quux = 1.0
        self.assertEqual(dut.quux, 1.0)
        self.assertEqual(inspect.getdoc(MyData2.quux), "")
