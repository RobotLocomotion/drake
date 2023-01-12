import dataclasses as dc
import math
from math import nan
from textwrap import dedent
import typing
import unittest

import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.common.test_utilities.meta import (
    ValueParameterizedTest,
    run_with_multiple_values,
)
from pydrake.common.yaml import yaml_load_typed


# To provide test coverage for all of the special cases of YAML loading, we'll
# define some dataclasses. These classes mimic
#  drake/common/yaml/test/example_structs.h
# and should be roughly kept in sync with the definitions in that file.


@dc.dataclass
class FloatStruct:
    value: float = nan


@dc.dataclass
class StringStruct:
    value: str = "nominal_string"


@dc.dataclass
class AllScalarsStruct:
    some_bool: bool = False
    some_float: float = nan
    some_int: int = 11
    some_str: str = "nominal_string"


@dc.dataclass
class ListStruct:
    value: typing.List[float] = dc.field(
        default_factory=lambda: list((nan,)))


@dc.dataclass
class MapStruct:
    value: typing.Dict[str, float] = dc.field(
        default_factory=lambda: dict(nominal_float=nan))


@dc.dataclass
class InnerStruct:
    inner_value: float = nan


@dc.dataclass
class OptionalStruct:
    value: typing.Optional[float] = nan


@dc.dataclass
class OptionalStructNoDefault:
    value: typing.Optional[float] = None


@dc.dataclass
class NumpyStruct:
    # TODO(jwnimmer-tri) Once we drop support for Ubuntu 20.04 "Focal", then we
    # can upgrade to numpy >= 1.21 as our minimum at which point we can use the
    # numpy.typing module here to constrain the shape and/or dtype.
    value: np.ndarray = dc.field(
        default_factory=lambda: np.array([nan]))


@dc.dataclass
class OuterStruct:
    outer_value: float = nan
    inner_struct: InnerStruct = dc.field(
        default_factory=lambda: InnerStruct())


@dc.dataclass
class BigMapStruct:
    value: typing.Mapping[str, OuterStruct] = dc.field(
        default_factory=lambda: dict(
            foo=OuterStruct(
                outer_value=1.0,
                inner_struct=InnerStruct(inner_value=2.0))))


class TestYamlTypedRead(unittest.TestCase,
                        metaclass=ValueParameterizedTest):
    """Detailed tests for the typed yaml_load function(s).

    This test class is the Python flavor of the C++ test suite at
     drake/common/yaml/test/yaml_read_archive_test.cc
    and should be roughly kept in sync with the test cases in that file.
    """

    def _all_typed_read_options(
            sweep_allow_yaml_with_no_schema=(True, False),
            sweep_allow_schema_with_no_yaml=(True, False),
            sweep_retain_map_defaults=(True, False)):
        """Returns the options matrix for our value-parameterized test cases.
        """
        result = []
        for i in sweep_allow_yaml_with_no_schema:
            for j in sweep_allow_schema_with_no_yaml:
                for k in sweep_retain_map_defaults:
                    result.append(dict(options=dict(
                        allow_yaml_with_no_schema=i,
                        allow_schema_with_no_yaml=j,
                        retain_map_defaults=k,
                    )))
        return result

    @run_with_multiple_values(_all_typed_read_options())
    def test_read_float(self, *, options):
        cases = [
            ("0", 0.0),
            ("1", 1.0),
            ("-1", -1.0),
            ("0.0", 0.0),
            ("1.2", 1.2),
            ("-1.2", -1.2),
            ("3e4", 3e4),
            ("3e-4", 3e-4),
            ("5.6e7", 5.6e7),
            ("5.6e-7", 5.6e-7),
            ("-5.6e7", -5.6e7),
            ("-5.6e-7", -5.6e-7),
            ("3E4", 3e4),
            ("3E-4", 3e-4),
            ("5.6E7", 5.6e7),
            ("5.6E-7", 5.6e-7),
            ("-5.6E7", -5.6e7),
            ("-5.6E-7", -5.6e-7),
        ]
        for value, expected in cases:
            data = f"value: {value}"
            x = yaml_load_typed(schema=FloatStruct, data=data, **options)
            self.assertEqual(x.value, expected)

    @run_with_multiple_values(_all_typed_read_options())
    def test_read_float_missing(self, *, options):
        if options["allow_schema_with_no_yaml"]:
            x = yaml_load_typed(schema=FloatStruct, data="{}",
                                **options)
            self.assertTrue(math.isnan(x.value), msg=repr(x.value))
        else:
            with self.assertRaisesRegex(RuntimeError, ".*missing.*"):
                yaml_load_typed(schema=FloatStruct, data="{}",
                                **options)

    @run_with_multiple_values(_all_typed_read_options())
    def test_read_all_scalars(self, *, options):
        data = dedent("""
        some_bool: true
        some_float: 101.0
        some_int: 102
        some_str: foo
        """)
        x = yaml_load_typed(schema=AllScalarsStruct, data=data, **options)
        self.assertEqual(x.some_bool, True)
        self.assertEqual(x.some_float, 101.0)
        self.assertEqual(x.some_int, 102)
        self.assertEqual(x.some_str, "foo")

    @run_with_multiple_values(_all_typed_read_options())
    def test_read_list(self, *, options):
        cases = [
            ("[1.0, 2.0, 3.0]", [1.0, 2.0, 3.0]),
        ]
        for value, expected in cases:
            data = f"value: {value}"
            x = yaml_load_typed(schema=ListStruct, data=data, **options)
            self.assertEqual(x.value, expected)

    @run_with_multiple_values(_all_typed_read_options())
    def test_read_list_missing(self, *, options):
        if options["allow_schema_with_no_yaml"]:
            x = yaml_load_typed(schema=ListStruct, data="{}", **options)
            self.assertTrue(len(x.value), 1)
            self.assertTrue(math.isnan(x.value[0]), msg=repr(x.value))
        else:
            with self.assertRaisesRegex(RuntimeError, ".*missing.*"):
                yaml_load_typed(schema=ListStruct, data="{}", **options)

    @run_with_multiple_values(_all_typed_read_options())
    def test_read_map(self, *, options):
        data = dedent("""
        value:
          foo: 0.0
          bar: 1.0
        """)
        x = yaml_load_typed(schema=MapStruct, data=data, **options)
        expected = dict(foo=0.0, bar=1.0)
        if options["retain_map_defaults"]:
            expected.update(nominal_float=nan)
        self.assertEqual(x.value, expected)

    @run_with_multiple_values(_all_typed_read_options())
    def test_read_big_map_append(self, *, options):
        data = dedent("""
        value:
          bar:
            outer_value: 3.0
            inner_struct:
              inner_value: 4.0
        """)
        x = yaml_load_typed(schema=BigMapStruct, data=data, **options)
        expected = dict(bar=OuterStruct(3.0, InnerStruct(4.0)))
        if options["retain_map_defaults"]:
            expected.update(foo=OuterStruct(1.0, InnerStruct(2.0)))
        self.assertEqual(x.value, expected)

    @run_with_multiple_values(_all_typed_read_options(
        # When False, the parser raises an exception not worth testing for.
        sweep_allow_schema_with_no_yaml=[True]))
    def test_read_big_map_merge_new_outer_value(self, *, options):
        data = dedent("""
        value:
          foo:
            outer_value: 3.0
        """)
        x = yaml_load_typed(schema=BigMapStruct, data=data, **options)
        expected = dict(foo=OuterStruct(3.0))
        if options["retain_map_defaults"]:
            expected["foo"].inner_struct.inner_value = 2.0
        self.assertEqual(x.value, expected)

    @run_with_multiple_values(_all_typed_read_options(
        # When False, the parser raises an exception not worth testing for.
        sweep_allow_schema_with_no_yaml=[True]))
    def test_read_big_map_merge_new_inner_value(self, *, options):
        data = dedent("""
        value:
          foo:
            inner_struct:
              inner_value: 4.0
        """)
        x = yaml_load_typed(schema=BigMapStruct, data=data, **options)
        expected = dict(foo=OuterStruct(inner_struct=InnerStruct(4.0)))
        if options["retain_map_defaults"]:
            expected["foo"].outer_value = 1.0
        self.assertEqual(x.value, expected)

    @run_with_multiple_values(_all_typed_read_options(
        # When False, the parser raises an exception not worth testing for.
        sweep_allow_schema_with_no_yaml=[True]))
    def test_read_big_map_merge_empty(self, *, options):
        data = dedent("""
        value:
          foo: {}
        """)
        x = yaml_load_typed(schema=BigMapStruct, data=data, **options)
        expected = dict(foo=OuterStruct())
        if options["retain_map_defaults"]:
            expected["foo"].outer_value = 1.0
            expected["foo"].inner_struct.inner_value = 2.0
        self.assertEqual(x.value, expected)

    @run_with_multiple_values(_all_typed_read_options())
    def test_read_map_missing(self, *, options):
        if options["allow_schema_with_no_yaml"]:
            x = yaml_load_typed(schema=MapStruct, data="{}", **options)
            self.assertEqual(x.value, dict(nominal_float=nan))
        else:
            with self.assertRaisesRegex(RuntimeError, ".*missing.*"):
                yaml_load_typed(schema=MapStruct, data="{}", **options)

    # TODO(jwnimmer-tri) Add test cases similar to StdMapWithMergeKeys
    # and StdMapWithBadMergeKey from the C++ YAML test suite.

    # TODO(jwnimmer-tri) Add test cases similar to StdMapDirectly and
    # StdMapDirectlyWithDefaults from the C++ YAML test suite.

    @run_with_multiple_values(_all_typed_read_options())
    def test_read_optional(self, *, options):
        # The test case numbers here (1..12) reference the specification as
        # documented in the C++ unit test yaml_read_archive_test.cc.
        for schema, data, expected in (
                (OptionalStructNoDefault, "value: 1.0", 1.0),    # Case 1, 2
                (OptionalStruct,          "value: 1.0", 1.0),    # Case 3, 4
                (OptionalStructNoDefault, "value:",     None),   # Case 5, 6
                (OptionalStruct,          "value:",     None),   # Case 7, 8
                (OptionalStructNoDefault, "{}",         None),   # Case 9, 10
                (OptionalStruct,          "{}", (
                    nan if options["allow_schema_with_no_yaml"]  # Case 12
                    else None)),                                 # Case 11
        ):
            with self.subTest(data=data, schema=schema):
                actual = yaml_load_typed(schema=schema, data=data, **options)
                self.assertEqual(actual, schema(expected))
                if options["allow_yaml_with_no_schema"]:
                    if "value:" in data:
                        amended_data = "foo: bar\n" + data
                    else:
                        amended_data = "foo: bar"
                    actual = yaml_load_typed(
                        schema=schema, data=amended_data, **options)
                    self.assertEqual(actual, schema(expected))

    # TODO(jwnimmer-tri) Add test cases similar to Variant and VariantMissing
    # from the C++ YAML test suite.

    @run_with_multiple_values(_all_typed_read_options())
    def test_read_np_vector(self, *, options):
        data = "value: [1.0, 2.0, 3.0]"
        expected = [1.0, 2.0, 3.0]
        x = yaml_load_typed(schema=NumpyStruct, data=data, **options)
        np.testing.assert_equal(x.value, np.array(expected), verbose=True)

        data = "value: [1.0]"
        expected = [1.0]
        x = yaml_load_typed(schema=NumpyStruct, data=data, **options)
        np.testing.assert_equal(x.value, np.array(expected), verbose=True)

        data = "value: []"
        expected = []
        x = yaml_load_typed(schema=NumpyStruct, data=data, **options)
        np.testing.assert_equal(x.value, np.array(expected), verbose=True)

    @run_with_multiple_values(_all_typed_read_options())
    def test_read_np_matrix(self, *, options):
        data = dedent("""
        value:
        - [0.0, 1.0, 2.0, 3.0]
        - [4.0, 5.0, 6.0, 7.0]
        - [8.0, 9.0, 10.0, 11.0]
        """)
        expected = [
            [0.0, 1.0, 2.0, 3.0],
            [4.0, 5.0, 6.0, 7.0],
            [8.0, 9.0, 10.0, 11.0],
        ]
        x = yaml_load_typed(schema=NumpyStruct, data=data, **options)
        np.testing.assert_equal(x.value, np.array(expected), verbose=True)

    @run_with_multiple_values(_all_typed_read_options())
    def test_read_np_missing(self, *, options):
        schema = NumpyStruct
        data = "{}"
        expected = [nan]
        if options["allow_schema_with_no_yaml"]:
            x = yaml_load_typed(schema=schema, data=data, **options)
            np.testing.assert_equal(x.value, np.array(expected), verbose=True)
        else:
            with self.assertRaisesRegex(RuntimeError, ".*missing.*"):
                yaml_load_typed(schema=schema, data=data, **options)

    @run_with_multiple_values(_all_typed_read_options())
    def test_read_nested(self, *, options):
        data = dedent("""
        outer_value: 1.0
        inner_struct:
          inner_value: 2.0
        """)
        x = yaml_load_typed(schema=OuterStruct, data=data, **options)
        expected = dict(foo=0.0, bar=1.0)
        self.assertEqual(x, OuterStruct(1.0, InnerStruct(2.0)))

    # TODO(jwnimmer-tri) Add a test case similar to NestedWithMergeKeys from
    # the C++ YAML test suite.

    # TODO(jwnimmer-tri) Add a test case similar to NestedWithBadMergeKey from
    # the C++ YAML test suite.

    # TODO(jwnimmer-tri) Add a test cases similar to these from the C++ YAML
    # test suite:
    # - VisitScalarFoundNothing
    # - VisitScalarFoundArray
    # - VisitScalarFoundStruct
    # - VisitArrayFoundNothing
    # - VisitArrayFoundScalar
    # - VisitArrayFoundStruct
    # - VisitVectorFoundNothing
    # - VisitVectorFoundScalar
    # - VisitVectorFoundStruct
    # - VisitOptionalScalarFoundSequence
    # - VisitVariantFoundNoTag
    # - VisitVariantFoundUnknownTag
    # - VisitEigenFoundNothing
    # - VisitEigenFoundScalar
    # - VisitEigenMatrixFoundOneDimensional
    # - VisitEigenMatrixFoundNonSquare
    # - VisitStructFoundNothing
    # - VisitStructFoundScalar
    # - VisitStructFoundArray


class TestYamlTypedReadAcceptance(unittest.TestCase):
    """Acceptance tests for the typed yaml_load function(s).

    This test class is the Python flavor of the C++ test suite at
     drake/common/yaml/test/yaml_io_test.cc
    and should be roughly kept in sync with the test cases in that file.
    """

    def test_load_string(self):
        data = dedent("""
        value:
          some_value
        """)
        result = yaml_load_typed(schema=StringStruct, data=data)
        self.assertEqual(result.value, "some_value")

    def test_load_string_child_name(self):
        data = dedent("""
        some_child_name:
          value:
            some_value
        """)
        result = yaml_load_typed(schema=StringStruct, data=data,
                                 child_name="some_child_name")
        self.assertEqual(result.value, "some_value")

        # When the requested child_name does not exist, that's an error.
        with self.assertRaisesRegex(KeyError, "wrong_child_name"):
            yaml_load_typed(schema=StringStruct, data=data,
                            child_name="wrong_child_name")

    def test_load_string_defaults(self):
        data = dedent("""
        value:
          some_key: 1.0
        """)
        defaults = MapStruct()

        # Merge the default map value(s).
        result = yaml_load_typed(
            schema=MapStruct, data=data, defaults=defaults)
        self.assertDictEqual(result.value, dict(
            nominal_float=nan,
            some_key=1.0))

        # Replace the default map value(s).
        result = yaml_load_typed(
            schema=MapStruct, data=data, defaults=defaults,
            retain_map_defaults=False)
        self.assertDictEqual(result.value, dict(some_key=1.0))

    def test_load_string_options(self):
        data = dedent("""
        value: some_value
        extra_junk: will_be_ignored
        """)
        result = yaml_load_typed(schema=StringStruct, data=data,
                                 allow_yaml_with_no_schema=True)
        self.assertEqual(result.value, "some_value")

        # Cross-check that the option actually was important.
        with self.assertRaisesRegex(RuntimeError, ".*extra_junk.*"):
            yaml_load_typed(schema=StringStruct, data=data)

    def test_load_file(self):
        filename = FindResourceOrThrow(
            "drake/common/yaml/test/yaml_io_test_input_1.yaml")
        result = yaml_load_typed(schema=StringStruct, filename=filename)
        self.assertEqual(result.value, "some_value_1")
