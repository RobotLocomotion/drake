import dataclasses as dc
import math
from math import nan
import os
from textwrap import dedent
import typing
import unittest

import numpy as np

from pydrake.common.schema import Gaussian
from pydrake.common.test_utilities.meta import (
    ValueParameterizedTest,
    run_with_multiple_values,
)
from pydrake.common.yaml import yaml_load_typed


@dc.dataclass
class FloatStruct:
    value: float = nan


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


class TestTypedYaml(unittest.TestCase,
                    metaclass=ValueParameterizedTest):
    """Tests for the typed yaml_load function(s)."""

    def _all_typed_read_options(
            sweep_allow_yaml_with_no_schema=(True, False),
            sweep_allow_schema_with_no_yaml=(True, False),
            sweep_retain_map_defaults=(True, False)):
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
        # The test case numberes here (1..12) reference the specification as
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
                        schema=schema, data=data, **options)
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
