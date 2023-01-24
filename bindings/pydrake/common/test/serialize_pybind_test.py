import inspect
import typing
import unittest

import numpy as np

from pydrake.common.test.serialize_test_util import (
    MyData1,
    MyData2,
    MyData3,
)


class TestSerializePybind(unittest.TestCase):

    @staticmethod
    def _make_data1():
        return MyData1(quux=22.0)

    @staticmethod
    def _make_data2():
        return MyData2(
            some_bool=False,
            some_int=1,
            some_uint64=1 << 48,
            some_float=0.5,
            some_double=2.0,
            some_string="3",
            some_eigen=[4.4],
            some_optional=None,
            some_vector=[5.0, 6.0],
            some_map={'key': 7},
            some_variant=8.0)

    def test_attributes_using_serialize_no_docs(self):
        """Tests the DefAttributesUsingSerialize overload WITHOUT docstrings.
        """
        # Basic read / write.
        dut = MyData1(quux=1.0)
        self.assertEqual(dut.quux, 1.0)
        dut.quux = -1.0

        # No docs.
        self.assertEqual(dut.quux, -1.0)
        self.assertEqual(inspect.getdoc(MyData1.quux), "")

        # Test fields.
        fields = getattr(MyData1, "__fields__")
        self.assertSequenceEqual([(x.name, x.type) for x in fields], (
            ("quux", float),
        ))

    def test_attributes_using_serialize_with_docs(self):
        """Tests the DefAttributesUsingSerialize overload WITH docstrings.
        """
        # Basic read / write.
        dut = MyData2(some_double=1.0)
        self.assertEqual(dut.some_double, 1.0)
        dut.some_double = -1.0
        self.assertEqual(dut.some_double, -1.0)

        # We'll just spot-check a few of the docs; that should be enough.
        self.assertEqual(inspect.getdoc(MyData2.some_double),
                         "Field docstring for a double.")
        self.assertEqual(inspect.getdoc(MyData2.some_vector),
                         "Field docstring for a vector.")

        # N.B. Fields are tested below.

    def test_attributes_using_serialize_types(self):
        """Probes the details of DefAttributesUsingSerialize all of the
        possible types of fields.
        """
        dut = self._make_data2()

        # Reset all fields.
        dut.some_bool = True
        dut.some_int = 10
        dut.some_uint64 = 1 << 52
        dut.some_float = 5.0
        dut.some_double = 20.0
        dut.some_string = "30"
        dut.some_eigen = [44.4]
        dut.some_optional = -1.0
        dut.some_vector = [50.0, 60.0]
        dut.some_map = {'new_key': 70}
        dut.some_variant = 80.0

        # Read back all fields.
        self.assertEqual(dut.some_bool, True)
        self.assertEqual(dut.some_int, 10)
        self.assertEqual(dut.some_uint64, 1 << 52)
        self.assertEqual(dut.some_float, 5.0)
        self.assertEqual(dut.some_double, 20.0)
        self.assertEqual(dut.some_string, "30")
        self.assertEqual(dut.some_eigen, [44.4])
        self.assertEqual(dut.some_optional, -1.0)
        self.assertEqual(dut.some_vector, [50.0, 60.0])
        self.assertEqual(dut.some_map, {'new_key': 70})
        self.assertEqual(dut.some_variant, 80.0)

        try:
            # Use the PEP-585 types if supported (Python >= 3.9).
            expected_vector_type = list[float]
            expected_dict_type = dict[str, float]
        except TypeError:
            # Otherwise, use the Python 3.8 types.
            expected_vector_type = typing.List[float]
            expected_dict_type = typing.Dict[str, float]

        # Check all field types.
        fields = getattr(MyData2, "__fields__")
        self.assertSequenceEqual([(x.name, x.type) for x in fields], (
            ("some_bool", bool),
            ("some_int", int),
            ("some_uint64", int),
            ("some_float", float),
            ("some_double", float),
            ("some_string", str),
            ("some_eigen", np.ndarray),
            ("some_optional", typing.Optional[float]),
            ("some_vector", expected_vector_type),
            ("some_map", expected_dict_type),
            ("some_variant", typing.Union[float, MyData1])))

    def test_repr_using_serialize_no_docs(self):
        """Tests the repr() for a class bound WITHOUT docstrings.
        (The docstrings shouldn't make any difference one way or another.)
        """
        self.assertEqual(repr(self._make_data1()),
                         "MyData1(quux=22.0)")

    def test_repr_using_serialize_with_docs(self):
        """Tests the repr() for a class bound WITH docstrings.
        (The docstrings shouldn't make any difference one way or another.)
        """
        self.assertEqual(repr(self._make_data2()),
                         "MyData2("
                         "some_bool=False, "
                         "some_int=1, "
                         "some_uint64=281474976710656, "
                         "some_float=0.5, "
                         "some_double=2.0, "
                         "some_string='3', "
                         "some_eigen=array([[4.4]]), "
                         "some_optional=None, "
                         "some_vector=[5.0, 6.0], "
                         "some_map={'key': 7.0}, "
                         "some_variant=8.0)")

    def test_repr_with_templates(self):
        """Tests the automatically generated repr() of a templated class.
        """
        self.assertEqual(repr(MyData3[float]()), "MyData3[float](quux=0.0)")
        self.assertEqual(repr(MyData3[int]()), "MyData3[int](quux=0)")
