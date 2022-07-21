import inspect
import unittest

from pydrake.common.test.serialize_test_util import (
    MyData,
    MyData2
)


class TestSerializePybind(unittest.TestCase):

    def test_attributes_using_serialize_structure(self):
        """Tests that DefAttributesUsingSerialize binds the class properties
        for read and write.
        """
        dut = MyData(foo=1.0, bar=[2.0, 3.0])

        self.assertEqual(dut.foo, 1.0)
        dut.foo = -1.0
        self.assertEqual(dut.foo, -1.0)

        self.assertListEqual(dut.bar, [2.0, 3.0])
        dut.bar = [-2.0, -3.0]
        self.assertListEqual(dut.bar, [-2.0, -3.0])

    def test_attributes_using_serialize_docs(self):
        """Tests that DefAttributesUsingSerialize adds docstrings to the bound
        fields.
        """
        self.assertEqual(inspect.getdoc(MyData.foo),
                         "This is the field docstring for foo.")
        self.assertEqual(inspect.getdoc(MyData.bar),
                         "This is the field docstring for bar.")

    def test_attributes_using_serialize_no_docstrings(self):
        """Sanity checks the DefAttributesUsingSerialize overload that does not
        use docstrings.
        """
        dut = MyData2(quux=1.0)
        self.assertEqual(dut.quux, 1.0)
        dut.quux = -1.0
        self.assertEqual(dut.quux, -1.0)
        self.assertEqual(inspect.getdoc(MyData2.quux), "")

    def test_repr_using_serialize(self):
        self.assertEqual(repr(MyData(foo=1.0, bar=[2.0, 3.0])),
                         "MyData(foo=1.0, bar=[2.0, 3.0])")
        self.assertEqual(repr(MyData2(quux=1.0)),
                         "MyData2(quux=1.0)")
