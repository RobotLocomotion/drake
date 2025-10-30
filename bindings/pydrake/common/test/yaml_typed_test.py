import base64
import copy
import dataclasses as dc
import functools
import math
from math import inf, nan
import os
from pathlib import Path
from textwrap import dedent
import typing
import unittest

import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.common.test.serialize_test_util import MyData2
from pydrake.common.test_utilities.meta import (
    ValueParameterizedTest,
    run_with_multiple_values,
)
from pydrake.common.value import Value
from pydrake.common.yaml import yaml_dump_typed, yaml_load_typed

# To provide test coverage for all of the special cases of YAML loading, we'll
# define some dataclasses. These classes mimic
#  drake/common/yaml/test/example_structs.h
# and should be roughly kept in sync with the definitions in that file.


def _dataclass_eq(a, b):
    # Work around https://github.com/python/cpython/issues/128294.
    return dc.astuple(a) == dc.astuple(b)


@dc.dataclass
class FloatStruct:
    value: float = nan
    __eq__ = _dataclass_eq


@dc.dataclass
class IntStruct:
    value: int = -1
    __eq__ = _dataclass_eq


@dc.dataclass
class BoolStruct:
    value: bool = False
    __eq__ = _dataclass_eq


@dc.dataclass
class StringStruct:
    value: str = "nominal_string"
    __eq__ = _dataclass_eq


@dc.dataclass
class BytesStruct:
    value: bytes = b"\x00\x01\x02"
    __eq__ = _dataclass_eq


@dc.dataclass
class PathStruct:
    value: Path = "/path/to/nowhere"
    __eq__ = _dataclass_eq


@dc.dataclass
class AllScalarsStruct:
    some_bool: bool = False
    some_bytes: bytes = b"\x00\x01\x02"
    some_float: float = nan
    some_int: int = 11
    some_path: Path = "/path/to/nowhere"
    some_str: str = "nominal_string"
    __eq__ = _dataclass_eq


@dc.dataclass
class ListStruct:
    value: typing.List[float] = dc.field(default_factory=lambda: list((nan,)))
    __eq__ = _dataclass_eq


@dc.dataclass
class MapStruct:
    value: typing.Dict[str, float] = dc.field(
        default_factory=lambda: dict(nominal_float=nan)
    )
    __eq__ = _dataclass_eq


@dc.dataclass
class InnerStruct:
    inner_value: float = nan
    __eq__ = _dataclass_eq


@dc.dataclass
class OptionalByteStruct:
    value: bytes | None = b"\x02\x03\x04"
    __eq__ = _dataclass_eq


@dc.dataclass
class OptionalStruct:
    value: float | None = nan
    __eq__ = _dataclass_eq


@dc.dataclass
class OptionalStructNoDefault:
    value: float | None = None
    __eq__ = _dataclass_eq


@dc.dataclass
class LegacyOptionalStruct:
    # Here we write out typing.Optional (dispreferred), instead of `| None`.
    value: typing.Optional[float] = nan
    __eq__ = _dataclass_eq


@dc.dataclass
class LegacyOptionalStructNoDefault:
    # Here we write out typing.Optional (dispreferred), instead of `| None`.
    value: typing.Optional[float] = None
    __eq__ = _dataclass_eq


@dc.dataclass
class NumpyStruct:
    # TODO(jwnimmer-tri) We should use the numpy.typing module here to
    # constrain the shape and/or dtype.
    value: np.ndarray = dc.field(default_factory=lambda: np.array([nan]))
    __eq__ = _dataclass_eq


@dc.dataclass
class RejectGetattrNumpyStruct:
    value: np.ndarray = dc.field(default_factory=lambda: np.array([nan]))
    __eq__ = _dataclass_eq

    def __getattribute__(self, name):
        if name == "value":
            # When loading fields that do not support merging (i.e., lists),
            # yaml_load_typed is careful to not call getattr on data it doesn't
            # need. Check that invariant by rejecting such access here.
            raise NotImplementedError()
        return object.__getattribute__(self, name)

    def _value(self):
        return self.__dict__["value"]


@dc.dataclass
class VariantStruct:
    value: typing.Union[str, float, FloatStruct, NumpyStruct] = nan
    __eq__ = _dataclass_eq


@dc.dataclass
class NullableVariantStruct:
    value: typing.Union[None, FloatStruct, StringStruct] = None
    __eq__ = _dataclass_eq


@dc.dataclass
class PrimitiveVariantStruct:
    value: typing.Union[typing.List[float], bool, int, float, str, bytes] = nan
    __eq__ = _dataclass_eq


@dc.dataclass
class ListVariantStruct:
    value: typing.List[typing.Union[str, float, FloatStruct, NumpyStruct]] = (
        dc.field(default_factory=lambda: list([nan]))
    )
    __eq__ = _dataclass_eq


@dc.dataclass
class OuterStruct:
    outer_value: float = nan
    inner_struct: InnerStruct = dc.field(default_factory=lambda: InnerStruct())
    __eq__ = _dataclass_eq


@dc.dataclass
class OuterStructOpposite:
    # N.B. The opposite member order of OuterStruct.
    inner_struct: InnerStruct = dc.field(default_factory=lambda: InnerStruct())
    outer_value: float = nan
    __eq__ = _dataclass_eq


@dc.dataclass
class Blank:
    __eq__ = _dataclass_eq


@dc.dataclass
class OuterWithBlankInner:
    outer_value: float = nan
    inner_struct: Blank = dc.field(default_factory=lambda: Blank())
    __eq__ = _dataclass_eq


@dc.dataclass
class BigMapStruct:
    value: typing.Mapping[str, OuterStruct] = dc.field(
        default_factory=lambda: dict(
            foo=OuterStruct(
                outer_value=1.0, inner_struct=InnerStruct(inner_value=2.0)
            )
        )
    )
    __eq__ = _dataclass_eq


class TestYamlTypedRead(unittest.TestCase, metaclass=ValueParameterizedTest):
    """Detailed tests for the typed yaml_load function(s).

    This test class is the Python flavor of the C++ test suite at
     drake/common/yaml/test/yaml_read_archive_test.cc
    and should be roughly kept in sync with the test cases in that file.
    """

    def _all_typed_read_options(
        sweep_allow_yaml_with_no_schema=(True, False),
        sweep_allow_schema_with_no_yaml=(True, False),
        sweep_retain_map_defaults=(True, False),
    ):
        """Returns the options matrix for our value-parameterized test cases."""
        result = []
        for i in sweep_allow_yaml_with_no_schema:
            for j in sweep_allow_schema_with_no_yaml:
                for k in sweep_retain_map_defaults:
                    result.append(
                        dict(
                            options=dict(
                                allow_yaml_with_no_schema=i,
                                allow_schema_with_no_yaml=j,
                                retain_map_defaults=k,
                            )
                        )
                    )
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
            x = yaml_load_typed(schema=FloatStruct, data="{}", **options)
            self.assertTrue(math.isnan(x.value), msg=repr(x.value))
        else:
            with self.assertRaisesRegex(RuntimeError, ".*missing.*"):
                yaml_load_typed(schema=FloatStruct, data="{}", **options)

    @run_with_multiple_values(_all_typed_read_options())
    def test_read_int(self, *, options):
        cases = [
            # Plain scalars.
            ("0", 0),
            ("1", 1),
            ("-1", -1),
            # Strings.
            ("30000", 30000),
            ("'0'", 0),
            ("'1'", 1),
            ("'-1'", -1),
            ("'30000'", 30000),
            # Float scalars.
            ("0.0", 0),
            ("1.0", 1),
            ("-1.0", -1),
            ("3.0e+4", 30000),
        ]
        for value, expected in cases:
            data = f"value: {value}"
            x = yaml_load_typed(schema=IntStruct, data=data, **options)
            self.assertEqual(x.value, expected)

        with self.assertRaisesRegex(Exception, "Expected.*int.*"):
            yaml_load_typed(schema=IntStruct, data="value: 1.1", **options)

    @run_with_multiple_values(_all_typed_read_options())
    def test_read_bool(self, *, options):
        cases = [
            # Plain scalars in canonical form.
            ("true", True),
            ("false", False),
            # Plain scalars in non-canonical form.
            ("yes", True),
            ("no", False),
            # Using the canonical form keywords but typed as strings.
            ("'true'", True),
            ("'false'", False),
        ]
        for value, expected in cases:
            data = f"value: {value}"
            x = yaml_load_typed(schema=BoolStruct, data=data, **options)
            self.assertEqual(x.value, expected)

        # Yaml's insane non-canonical plain scalars. (This is not the complete
        # set, rather just a couple as a sanity check.)
        bad_cases = (
            # Using the non-canonical form keywords but typed as strings.
            "'yes'",
            "'no'",
        )
        for value in bad_cases:
            data = f"value: {value}"
            with self.assertRaisesRegex(Exception, "Expected.*bool.*"):
                yaml_load_typed(schema=BoolStruct, data=data, **options)

    @run_with_multiple_values(_all_typed_read_options())
    def test_read_string_scalar_type_mismatch(self, *, options):
        cases = [
            "0",
            "3.0e+4",
        ]
        for value in cases:
            data = f"value: {value}"
            with self.assertRaisesRegex(Exception, "Expected.*str.*"):
                yaml_load_typed(schema=StringStruct, data=data, **options)

    @run_with_multiple_values(_all_typed_read_options())
    def test_read_path(self, *, options):
        # The following should all turn into paths.
        cases = [
            # These plain strings are unchanged by parsing them into a Path.
            (".", "."),
            ("no_directory.txt", "no_directory.txt"),
            ("/absolute/path/file.txt", "/absolute/path/file.txt"),
            ('/quoted"/path', '/quoted"/path'),
            # These strings end up changing to a greater or lesser degree.
            ('""', "."),
            ("!!str", "."),
            ("/non_lexical//path", "/non_lexical/path"),
            ("'1234'", "1234"),
            ('"1234"', "1234"),
            ("!!str 1234", "1234"),
        ]
        for value, expected in cases:
            data = f"value: {value}"
            x = yaml_load_typed(schema=PathStruct, data=data, **options)
            self.assertEqual(x.value, Path(expected))

        # The following should *not* turn into paths; we can't instantiate
        # paths from these types.
        cases = [
            ("!!float 1234.5"),
            ("1234.5"),
            ("!!int 1234"),
            ("1234"),
            ("!!bool true"),
            ("true"),
        ]
        for value in cases:
            data = f"value: {value}"
            with self.assertRaises(Exception):
                yaml_load_typed(schema=PathStruct, data=data, **options)

    @run_with_multiple_values(_all_typed_read_options())
    def test_read_path_missing(self, *, options):
        if options["allow_schema_with_no_yaml"]:
            default_value = PathStruct()
            x = yaml_load_typed(schema=PathStruct, data="{}", **options)
            self.assertEqual(x.value, default_value.value, msg=repr(x.value))
        else:
            with self.assertRaisesRegex(RuntimeError, ".*missing.*"):
                yaml_load_typed(schema=PathStruct, data="{}", **options)

    @run_with_multiple_values(_all_typed_read_options())
    def test_read_bytes(self, *, options):
        # Using !!binary on a schema whose type is bytes.
        cases = [
            ("!!binary A3Rlc3Rfc3RyAw==", b"\x03test_str\x03"),
            ("!!binary |\n  A3Rlc3Rfc3RyAw==", b"\x03test_str\x03"),
            ("!!binary |\n  A3Rlc3Rf\n  c3RyAw====", b"\x03test_str\x03"),
            ("!!binary", b""),
        ]
        for value, expected in cases:
            data = f"value: {value}"
            x = yaml_load_typed(schema=BytesStruct, data=data, **options)
            self.assertEqual(x.value, expected)

        # Malformed base64 encoding.
        cases = [
            ("A3Rfc3RyAw=", "Incorrect padding"),
            ("A3Rfc*RyAw==", "Invalid base64-encoded string"),
        ]
        for value, error_regex in cases:
            data = f"value: !!binary {value}"
            with self.assertRaisesRegex(Exception, error_regex):
                yaml_load_typed(schema=BytesStruct, data=data, **options)

        # Assigning any other type to bytes is rejected.
        cases = [
            # String.
            ("test string", "Expected.*bytes.*str"),
            ("!!str 1234", "Expected.*bytes.*str"),
            # Int.
            ("12", "Expected.*bytes.*int"),
            ("!!int 12", "Expected.*bytes.*int"),
            ("0x3", "Expected.*bytes.*int"),
            # Pyyaml defect: 0o3 should be an int.
            ("0o3", "Expected.*bytes.*str"),
            # Pyyaml defect: 00:03 should be an int (value of 3).
            ("00:03", "Expected.*bytes.*str"),
            # Float.
            ("1234.5", "Expected.*bytes.*float"),
            ("!!float 1234.5", "Expected.*bytes.*float"),
            (".inf", "Expected.*bytes.*float"),
            ("00:03.3", "Expected.*bytes.*float"),
            # Null
            ("null", "Expected.*bytes.*NoneType"),
            ("", "Expected.*bytes.*NoneType"),
            # Bool
            ("true", "Expected.*bytes.*bool"),
            ("!!bool true", "Expected.*bytes.*bool"),
        ]
        for value, error_regex in cases:
            data = f"value: {value}"
            with self.assertRaisesRegex(
                RuntimeError, error_regex, msg=f"For value '{value}'"
            ):
                yaml_load_typed(schema=BytesStruct, data=data, **options)

        # Using !!binary and assigning it to non-bytes should throw.
        cases = [
            (b".inf", FloatStruct),
            (b"inf", FloatStruct),
            (b"1234.5", FloatStruct),
            (b"1234", IntStruct),
            (b"test/path", PathStruct),
            (b"a string", StringStruct),
        ]
        for byte_value, schema in cases:
            encoded = base64.b64encode(byte_value)
            data = f"value: !!binary {encoded.decode('utf-8')}"
            with self.assertRaisesRegex(
                RuntimeError,
                "Expected a .* value .* instead got "
                "yaml data of type <class 'bytes'>",
                msg=f"value: {byte_value}",
            ):
                yaml_load_typed(schema=schema, data=data, **options)

    @run_with_multiple_values(_all_typed_read_options())
    def test_read_all_scalars(self, *, options):
        data = dedent("""
        some_bool: true
        some_bytes: !!binary BQYH
        some_float: 101.0
        some_int: 102
        some_path: /alternative/path
        some_str: foo
        """)
        x = yaml_load_typed(schema=AllScalarsStruct, data=data, **options)
        self.assertEqual(x.some_bool, True)
        self.assertEqual(x.some_bytes, b"\x05\x06\x07")
        self.assertEqual(x.some_float, 101.0)
        self.assertEqual(x.some_int, 102)
        self.assertEqual(x.some_path, Path("/alternative/path"))
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

    @run_with_multiple_values(
        _all_typed_read_options(
            # When False, the parser raises an exception not worth testing for.
            sweep_allow_schema_with_no_yaml=[True]
        )
    )
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

    @run_with_multiple_values(
        _all_typed_read_options(
            # When False, the parser raises an exception not worth testing for.
            sweep_allow_schema_with_no_yaml=[True]
        )
    )
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

    @run_with_multiple_values(
        _all_typed_read_options(
            # When False, the parser raises an exception not worth testing for.
            sweep_allow_schema_with_no_yaml=[True]
        )
    )
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
        cases = [
            (OptionalStructNoDefault, "value: 1.0", 1.0),  # Case 1, 2
            (OptionalStruct, "value: 1.0", 1.0),  # Case 3, 4
            (OptionalStructNoDefault, "value:", None),  # Case 5, 6
            (OptionalStruct, "value:", None),  # Case 7, 8
            (OptionalStructNoDefault, "{}", None),  # Case 9, 10
            (
                OptionalStruct,
                "{}",
                (
                    nan
                    if options["allow_schema_with_no_yaml"]  # Case 12
                    else None
                ),
            ),  # Case 11
        ]
        respell = {
            OptionalStruct: LegacyOptionalStruct,
            OptionalStructNoDefault: LegacyOptionalStructNoDefault,
        }
        legacy_cases = [
            (respell[schema], data, expected)
            for schema, data, expected in cases
        ]
        for schema, data, expected in cases + legacy_cases:
            with self.subTest(data=data, schema=schema):
                actual = yaml_load_typed(schema=schema, data=data, **options)
                self.assertEqual(actual, schema(expected))
                if options["allow_yaml_with_no_schema"]:
                    if "value:" in data:
                        amended_data = "foo: bar\n" + data
                    else:
                        amended_data = "foo: bar"
                    actual = yaml_load_typed(
                        schema=schema, data=amended_data, **options
                    )
                    self.assertEqual(actual, schema(expected))

    def test_read_optional_bytes(self):
        """Smoke test for compatibility for the non-json scalar: binary. This
        is trivial in python, but awkward in C++. To maintain parity with the
        C++ tests, we simply mirror the C++ test.

        This skips the nuance of parsing configuration, assuming that is
        handled by the more general test_read_optional.
        """
        data = "value: !!binary b3RoZXID/3N0dWZm"
        actual = yaml_load_typed(schema=OptionalByteStruct, data=data)
        self.assertEqual(actual, OptionalByteStruct(b"other\x03\xffstuff"))

    @run_with_multiple_values(_all_typed_read_options())
    def test_read_variant(self, *, options):
        data = "value: foo"
        x = yaml_load_typed(schema=VariantStruct, data=data, **options)
        self.assertEqual(x, VariantStruct("foo"))
        self.assertEqual(type(x.value), str)

        data = "value: !!str bar"
        x = yaml_load_typed(schema=VariantStruct, data=data, **options)
        self.assertEqual(x, VariantStruct("bar"))
        self.assertEqual(type(x.value), str)

        data = "value: !!float 1.0"
        x = yaml_load_typed(schema=VariantStruct, data=data, **options)
        self.assertEqual(x, VariantStruct(1.0))
        self.assertEqual(type(x.value), float)

        data = "value: !FloatStruct { value: 1.0 }"
        x = yaml_load_typed(schema=VariantStruct, data=data, **options)
        self.assertEqual(x, VariantStruct(FloatStruct(1.0)))

        data = "value: !NumpyStruct { value: [1.0, 2.0] }"
        x = yaml_load_typed(schema=VariantStruct, data=data, **options)
        self.assertEqual(type(x.value), NumpyStruct)
        actual = x.value.value
        expected = np.array([1.0, 2.0])
        np.testing.assert_equal(actual, expected, verbose=True)

        data = "value: !FloatStruct {}"
        defaults = VariantStruct(FloatStruct(22.0))
        if options["allow_schema_with_no_yaml"]:
            x = yaml_load_typed(
                schema=VariantStruct, data=data, defaults=defaults, **options
            )
            self.assertEqual(x, defaults)
        else:
            with self.assertRaisesRegex(RuntimeError, ".*missing.*"):
                yaml_load_typed(
                    schema=VariantStruct,
                    data=data,
                    defaults=defaults,
                    **options,
                )

    @run_with_multiple_values(_all_typed_read_options())
    def test_read_generic_variant(self, *, options):
        """When the schema has a Union[List[T], U, ...], we must be careful to
        never call List[T]() like a constructor; we must call list() instead.

        This kind of problem cannot occur in the C++ type system, so this test
        case doesn't have any twin inside yaml_read_archive_test.cc.
        """
        schema = PrimitiveVariantStruct
        data = "value: [1.0, 2.0]"
        x = yaml_load_typed(schema=schema, data=data, **options)
        self.assertEqual(x.value, [1.0, 2.0])

    @run_with_multiple_values(_all_typed_read_options())
    def test_read_primitive_variant(self, *, options):
        schema = PrimitiveVariantStruct
        data = "value: !!bool true"
        x = yaml_load_typed(schema=schema, data=data, **options)
        self.assertEqual(x.value, True)
        data = "value: !!bool 'true'"
        x = yaml_load_typed(schema=schema, data=data, **options)
        self.assertEqual(x.value, True)

        data = "value: !!int 10"
        x = yaml_load_typed(schema=schema, data=data, **options)
        self.assertEqual(x.value, 10)
        data = "value: !!int '10'"
        x = yaml_load_typed(schema=schema, data=data, **options)
        self.assertEqual(x.value, 10)

        data = "value: !!float 1.0"
        x = yaml_load_typed(schema=schema, data=data, **options)
        self.assertEqual(x.value, 1.0)
        data = "value: !!float '1.0'"
        x = yaml_load_typed(schema=schema, data=data, **options)
        self.assertEqual(x.value, 1.0)

        data = "value: !!str foo"
        x = yaml_load_typed(schema=schema, data=data, **options)
        self.assertEqual(x.value, "foo")
        data = "value: !!str 'foo'"
        x = yaml_load_typed(schema=schema, data=data, **options)
        self.assertEqual(x.value, "foo")

        data = "value: !!binary A3Rlc3Rfc3RyAw=="
        x = yaml_load_typed(schema=schema, data=data, **options)
        self.assertEqual(x.value, b"\x03test_str\x03")

    @run_with_multiple_values(_all_typed_read_options())
    def test_read_variant_missing(self, *, options):
        if options["allow_schema_with_no_yaml"]:
            x = yaml_load_typed(schema=VariantStruct, data="{}", **options)
            self.assertTrue(math.isnan(x.value), msg=repr(x.value))
        else:
            with self.assertRaisesRegex(RuntimeError, ".*missing.*"):
                yaml_load_typed(schema=VariantStruct, data="{}", **options)

    @run_with_multiple_values(_all_typed_read_options())
    def test_read_variant_found_no_tag(self, *, options):
        data = "value:"
        with self.assertRaisesRegex(RuntimeError, "one of.*FloatStruct.*"):
            yaml_load_typed(schema=VariantStruct, data=data, **options)
        data = "value: [1.0, 2.0]"
        with self.assertRaisesRegex(RuntimeError, "str.*got.*list"):
            yaml_load_typed(schema=VariantStruct, data=data, **options)
        data = "value: { foo: bar }"
        with self.assertRaisesRegex(RuntimeError, "str.*got.*dict"):
            yaml_load_typed(schema=VariantStruct, data=data, **options)

    @run_with_multiple_values(_all_typed_read_options())
    def test_read_variant_found_unknown_tag(self, *, options):
        data = "value: !UnknownTag { foo: bar }"
        with self.assertRaisesRegex(RuntimeError, "UnknownTag.*match"):
            yaml_load_typed(schema=VariantStruct, data=data, **options)

    @run_with_multiple_values(_all_typed_read_options())
    def test_read_list_variant(self, *, options):
        data = dedent("""
        value:
        - foo
        - !!float 1.0
        - !FloatStruct { value: 2.0 }
        - !NumpyStruct { value: [3.0, 4.0] }
        """)
        x = yaml_load_typed(schema=ListVariantStruct, data=data, **options)
        self.assertEqual(len(x.value), 4)
        self.assertEqual(x.value[0], "foo")
        self.assertEqual(x.value[1], 1.0)
        self.assertEqual(x.value[2], FloatStruct(2.0))
        self.assertEqual(type(x.value[3]), NumpyStruct)

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
    def test_read_np_no_getattr(self, *, options):
        data = "value: [1.0]"
        expected = [1.0]
        x = yaml_load_typed(
            schema=RejectGetattrNumpyStruct, data=data, **options
        )
        np.testing.assert_equal(x._value(), np.array(expected), verbose=True)

    @run_with_multiple_values(_all_typed_read_options())
    def test_read_nested(self, *, options):
        data = dedent("""
        outer_value: 1.0
        inner_struct:
          inner_value: 2.0
        """)
        x = yaml_load_typed(schema=OuterStruct, data=data, **options)
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
        result = yaml_load_typed(
            schema=StringStruct, data=data, child_name="some_child_name"
        )
        self.assertEqual(result.value, "some_value")

        # When the requested child_name does not exist, that's an error.
        with self.assertRaisesRegex(KeyError, "wrong_child_name"):
            yaml_load_typed(
                schema=StringStruct, data=data, child_name="wrong_child_name"
            )

    def test_load_string_defaults(self):
        data = dedent("""
        value:
          some_key: 1.0
        """)
        defaults = MapStruct()

        # Merge the default map value(s).
        result = yaml_load_typed(schema=MapStruct, data=data, defaults=defaults)
        self.assertDictEqual(
            result.value, dict(nominal_float=nan, some_key=1.0)
        )

        # Replace the default map value(s).
        result = yaml_load_typed(
            schema=MapStruct,
            data=data,
            defaults=defaults,
            retain_map_defaults=False,
        )
        self.assertDictEqual(result.value, dict(some_key=1.0))

    def test_load_inferred_schema(self):
        data = dedent("""
        value:
          some_key: 1.0
        """)
        result = yaml_load_typed(data=data, defaults=MapStruct())
        self.assertIsInstance(result, MapStruct)
        self.assertDictEqual(
            result.value, dict(nominal_float=nan, some_key=1.0)
        )

        with self.assertRaisesRegex(Exception, "At least one"):
            yaml_load_typed(data=data)

    def test_load_string_options(self):
        data = dedent("""
        value: some_value
        extra_junk: will_be_ignored
        """)
        result = yaml_load_typed(
            schema=StringStruct, data=data, allow_yaml_with_no_schema=True
        )
        self.assertEqual(result.value, "some_value")

        # Cross-check that the option actually was important.
        with self.assertRaisesRegex(RuntimeError, ".*extra_junk.*"):
            yaml_load_typed(schema=StringStruct, data=data)

    def test_load_file(self):
        filename = FindResourceOrThrow(
            "drake/common/yaml/test/yaml_io_test_input_1.yaml"
        )
        result = yaml_load_typed(schema=StringStruct, filename=filename)
        self.assertEqual(result.value, "some_value_1")

    def test_read_bad_schema(self):
        # N.B. This test covers python-specific error handling, so does not
        # have any corrresponding cases in the C++ unit tests.
        with self.assertRaisesRegex(Exception, "should have been a dict"):
            yaml_load_typed(
                schema=typing.List[float], data="[1.0]", defaults=[]
            )


class TestYamlTypedWrite(unittest.TestCase):
    """Detailed tests for the yaml_dump_typed function.

    This test class is the Python flavor of the C++ test suite at
     drake/common/yaml/test/yaml_write_archive_test.cc
    and should be roughly kept in sync with the test cases in that file.
    """

    def test_write_float(self):
        cases = [
            (0.0, "0.0"),
            (1.0, "1.0"),
            (-1.0, "-1.0"),
            (0.009, "0.009"),
            (1.2, "1.2"),
            (-1.2, "-1.2"),
            (5.6e16, "5.6e+16"),
            (5.6e-12, "5.6e-12"),
            (-5.6e16, "-5.6e+16"),
            (-5.6e-12, "-5.6e-12"),
            # See https://yaml.org/spec/1.2.2/#10214-floating-point.
            (nan, ".nan"),
            (inf, ".inf"),
            (-inf, "-.inf"),
        ]
        for value, expected_str in cases:
            actual_doc = yaml_dump_typed(FloatStruct(value=value))
            expected_doc = f"value: {expected_str}\n"
            self.assertEqual(actual_doc, expected_doc)

    def test_write_string(self):
        # We'll use this abbreviation to help make our expected values clear.
        dq = '"'  # double quote
        cases = [
            # Plain string.
            ("a", "a"),
            # Needs quoting for special characters.
            ("'", "''''"),
            ('"', f"'{dq}'"),
            # Needs quoting to avoid being misinterpreted as another data type.
            ("1", "'1'"),
            ("1.0", "'1.0'"),
            (".NaN", "'.NaN'"),
            ("true", "'true'"),
            ("null", "'null'"),
            ("NO", "'NO'"),
            ("null", "'null'"),
            ("190:20:30", "'190:20:30'"),  # YAML has sexagesimal literals.
            # Similar to things that would be misinterpreted but actually a-ok.
            ("nonnull", "nonnull"),
            ("NaN", "NaN"),
            ("=1.0", "=1.0"),
            ("00:1A:2B:3C:4D:5E", "00:1A:2B:3C:4D:5E"),
        ]
        for value, expected_str in cases:
            actual_doc = yaml_dump_typed(StringStruct(value=value))
            expected_doc = f"value: {expected_str}\n"
            self.assertEqual(actual_doc, expected_doc)

    def test_write_all_scalars(self):
        x = AllScalarsStruct()
        x.some_bool = True
        x.some_float = 100.0
        x.some_int = 102
        x.some_str = "foo"
        x.some_path = Path("/test/path")
        x.some_bytes = b"\x05\x06\x07"
        actual_doc = yaml_dump_typed(x)
        expected_doc = dedent("""\
        some_bool: true
        some_bytes: !!binary |
          BQYH
        some_float: 100.0
        some_int: 102
        some_path: /test/path
        some_str: foo
        """)
        self.assertEqual(actual_doc, expected_doc)

    def test_write_bytes(self):
        cases = [
            (b"", '!!binary ""'),
            (b"all ascii", "!!binary |\n  YWxsIGFzY2lp"),
            (b"other\x03\xffstuff", "!!binary |\n  b3RoZXID/3N0dWZm"),
        ]
        for value, expected_str in cases:
            actual_doc = yaml_dump_typed(BytesStruct(value=value))
            expected_doc = f"value: {expected_str}\n"
            self.assertEqual(actual_doc, expected_doc)

    def test_write_path(self):
        cases = [
            # In contrast to C++, there is no "empty" Path; it defaults to '.'.
            (Path(""), "."),
            (Path("/absolute/path"), "/absolute/path"),
            (Path("relative/path"), "relative/path"),
            (Path("1234"), "'1234'"),
        ]
        for value, expected_str in cases:
            actual_doc = yaml_dump_typed(PathStruct(value=value))
            expected_doc = f"value: {expected_str}\n"
            self.assertEqual(actual_doc, expected_doc)

    def test_write_list_plain(self):
        # When the vector items are simple YAML scalars, we should use "flow"
        # style, where they all appear on a single line.
        cases = [
            ([], "value: []\n"),
            ([1.0, 2.0, 3.0], "value: [1.0, 2.0, 3.0]\n"),
        ]
        for value, expected_doc in cases:
            actual_doc = yaml_dump_typed(ListStruct(value=value))
            self.assertEqual(actual_doc, expected_doc)

    def test_write_list_nested(self):
        # When the vector items are not simple scalars, we should use "block"
        # style, where each gets its own line(s).
        cases = [
            (
                [FloatStruct(value=1.0), FloatStruct(value=2.0)],
                dedent("""\
                value:
                - !FloatStruct
                  value: 1.0
                - !FloatStruct
                  value: 2.0
                """),
            ),
            # Empty lists still use flow style.
            (
                [],
                dedent("""\
                value: []
                """),
            ),
        ]
        for value, expected_doc in cases:
            actual_doc = yaml_dump_typed(ListVariantStruct(value=value))
            self.assertEqual(actual_doc, expected_doc)

    def test_write_map(self):
        cases = [
            (
                dict(),
                dedent("""\
                value: {}
                """),
            ),
            (
                dict(foo=0.0),
                dedent("""\
                value:
                  foo: 0.0
                """),
            ),
            (
                dict(foo=0.0, bar=1.0),
                dedent("""\
                value:
                  bar: 1.0
                  foo: 0.0
                """),
            ),
        ]
        for value, expected_doc in cases:
            actual_doc = yaml_dump_typed(MapStruct(value=value))
            self.assertEqual(actual_doc, expected_doc)

    def test_write_bad_map_key(self):
        @dc.dataclass
        class BadMapStruct:
            value: typing.Dict[int, float]

        with self.assertRaisesRegex(Exception, "keys must be string"):
            yaml_dump_typed(BadMapStruct({1: 2}))

    def test_write_map_directly(self):
        cases = [
            (
                dict(),
                dedent("""\
                {}
                """),
            ),
            (
                dict(foo=0.0),
                dedent("""\
                foo: 0.0
                """),
            ),
            (
                dict(foo=0.0, bar=1.0),
                dedent("""\
                bar: 1.0
                foo: 0.0
                """),
            ),
        ]
        schema = typing.Dict[str, float]
        for value, expected_doc in cases:
            actual_doc = yaml_dump_typed(value, schema=schema)
            self.assertEqual(actual_doc, expected_doc)

    def test_write_optional(self):
        cases = [
            (1.0, "value: 1.0\n"),
            (None, "{}\n"),
        ]
        for value, expected_doc in cases:
            actual_doc = yaml_dump_typed(OptionalStruct(value=value))
            self.assertEqual(actual_doc, expected_doc)
        for value, expected_doc in cases:
            actual_doc = yaml_dump_typed(LegacyOptionalStruct(value=value))
            self.assertEqual(actual_doc, expected_doc)

    def test_write_optional_bytes(self):
        """Smoke test for compatibility for the non-json scalar: binary. This
        is trivial in python, but awkward in C++. To maintain parity with the
        C++ tests, we simply mirror the C++ test."""
        actual_doc = yaml_dump_typed(OptionalByteStruct(None))
        self.assertEqual(actual_doc, "{}\n")
        actual_doc = yaml_dump_typed(OptionalByteStruct(b"other\x03\xffstuff"))
        self.assertEqual(actual_doc, "value: !!binary |\n  b3RoZXID/3N0dWZm\n")

    def test_write_variant(self):
        cases = [
            (
                "",
                "value: ''\n",
            ),
            (
                "foo",
                "value: foo\n",
            ),
            (
                1.0,
                # TODO(jwnimmer-tri) Ideally we'd avoid the extra single quotes
                # in this output, but I haven't figured out how to teach pyyaml
                # to do that yet.
                "value: !!float '1.0'\n",
            ),
            (
                FloatStruct(1.0),
                dedent("""\
                value: !FloatStruct
                  value: 1.0
                """),
            ),
            (
                NumpyStruct(np.array([1.0, 2.0])),
                dedent("""\
                value: !NumpyStruct
                  value: [1.0, 2.0]
                """),
            ),
        ]
        for value, expected_doc in cases:
            with self.subTest(value=value):
                variant = VariantStruct(value=value)
                actual_doc = yaml_dump_typed(variant)
                self.assertEqual(actual_doc, expected_doc)

        # Check when the value in a Union is not one of the allowed types.
        with self.assertRaisesRegex(Exception, "did not match"):
            yaml_dump_typed(VariantStruct(value=MapStruct()))
        with self.assertRaisesRegex(Exception, "does not allow None"):
            yaml_dump_typed(VariantStruct(value=None))

    def test_write_nullable_variant(self):
        cases = [
            (
                None,
                "value: null\n",
            ),
            (
                FloatStruct(1.0),
                dedent("""\
                value: !FloatStruct
                  value: 1.0
                """),
            ),
        ]
        for value, expected_doc in cases:
            with self.subTest(value=value):
                variant = NullableVariantStruct(value=value)
                actual_doc = yaml_dump_typed(variant)
                self.assertEqual(actual_doc, expected_doc)

    def test_write_primitive_variant(self):
        cases = [
            (
                [1.0, 2.0],
                "value: [1.0, 2.0]\n",
            ),
            (
                True,
                # TODO(jwnimmer-tri) Ideally we'd avoid the extra single quotes
                # in this output, but I haven't figured out how to teach pyyaml
                # to do that yet.
                "value: !!bool 'true'\n",
            ),
            (
                10,
                # TODO(jwnimmer-tri) Ditto the above (re: quoting).
                "value: !!int '10'\n",
            ),
            (
                1.0,
                # TODO(jwnimmer-tri) Ditto the above (re: quoting).
                "value: !!float '1.0'\n",
            ),
            (
                "foo",
                # TODO(jwnimmer-tri) Ditto the above (re: quoting).
                "value: !!str 'foo'\n",
            ),
            (
                b"other\x03\xffstuff",
                "value: !!binary |\n  b3RoZXID/3N0dWZm\n",
            ),
        ]
        for value, expected_doc in cases:
            with self.subTest(value=value):
                variant = PrimitiveVariantStruct(value=value)
                actual_doc = yaml_dump_typed(variant)
                self.assertEqual(actual_doc, expected_doc)

    def test_write_numpy_vector(self):
        cases = [
            ([], "value: []\n"),
            ([1.0], "value: [1.0]\n"),
            ([1.0, 0.0], "value: [1.0, 0.0]\n"),
        ]
        for value, expected_doc in cases:
            actual_doc = yaml_dump_typed(NumpyStruct(value=np.array(value)))
            self.assertEqual(actual_doc, expected_doc)

    def test_write_numpy_matrix(self):
        x = NumpyStruct(
            value=np.array(
                [
                    [0.0, 1.0, 2.0, 3.0],
                    [4.0, 5.0, 6.0, 7.0],
                    [8.0, 9.0, 10.0, 11.0],
                ]
            )
        )
        self.assertEqual(
            yaml_dump_typed(x),
            dedent("""\
        value:
        - [0.0, 1.0, 2.0, 3.0]
        - [4.0, 5.0, 6.0, 7.0]
        - [8.0, 9.0, 10.0, 11.0]
        """),
        )

    def test_write_numpy_matrix00(self):
        x = NumpyStruct(value=np.ndarray(shape=(0, 0)))
        self.assertEqual(
            yaml_dump_typed(x),
            dedent("""\
        value: []
        """),
        )

    def test_write_nested(self):
        x = OuterStruct()
        x.outer_value = 1.0
        x.inner_struct.inner_value = 2.0

        saved = yaml_dump_typed(x, child_name="doc")
        expected = dedent("""\
        doc:
          outer_value: 1.0
          inner_struct:
            inner_value: 2.0
        """)
        self.assertEqual(saved, expected)

    def test_write_blank_inner(self):
        x = OuterWithBlankInner()
        x.outer_value = 1.0

        saved = yaml_dump_typed(x, child_name="doc")
        expected = dedent("""\
        doc:
          outer_value: 1.0
          inner_struct: {}
        """)
        self.assertEqual(saved, expected)

    def test_write_child_name(self):
        x = FloatStruct(value=1.0)
        dut = functools.partial(yaml_dump_typed, data=x)
        self.assertEqual(dut(child_name=None), "value: 1.0\n")
        self.assertEqual(dut(child_name="root"), "root:\n  value: 1.0\n")
        with self.assertRaisesRegex(Exception, "child_name must be a prim"):
            dut(child_name=[1, 2, 3])


class TestYamlTypedWriteDefaults(unittest.TestCase):
    """Detailed tests for the yaml_dump_typed use of ``defaults=...``, in
    particular the _erase_matching_maps function.

    This test class is the Python flavor of the C++ test suite at
     drake/common/yaml/test/yaml_write_archive_defaults_test.cc
    and should be roughly kept in sync with the test cases in that file.
    """

    def _save(self, data, defaults, child_name="doc"):
        return yaml_dump_typed(
            data=data, defaults=defaults, child_name=child_name
        )

    def test_dump_default_basic_example1(self):
        # Shows the typical use -- that only the novel data is output.
        # The inner_struct is the same for both x and y, so is not output.
        defaults = OuterStruct()
        defaults.outer_value = 1.0
        defaults.inner_struct.inner_value = 2.0
        data = copy.deepcopy(defaults)
        data.outer_value = 3.0
        self.assertEqual(
            self._save(data, defaults),
            dedent("""\
        doc:
          outer_value: 3.0
        """),
        )

    def test_dump_default_basic_example2(self):
        # Shows the typical use -- that only the novel data is output.
        # The outer_value is the same for both x and y, so is not output.
        defaults = OuterStruct()
        defaults.outer_value = 1.0
        defaults.inner_struct.inner_value = 2.0
        data = copy.deepcopy(defaults)
        data.inner_struct.inner_value = 3.0
        self.assertEqual(
            self._save(data, defaults),
            dedent("""\
        doc:
          inner_struct:
            inner_value: 3.0
        """),
        )

    def test_dump_default_basic_example3(self):
        # Shows the typical use -- emit the content with or without providing a
        # root_name.
        defaults = OuterStruct()
        defaults.outer_value = 1.0
        data = OuterStruct()
        data.outer_value = 3.0
        data.inner_struct.inner_value = defaults.inner_struct.inner_value

        # Emit using the default "doc" root name.
        self.assertEqual(
            self._save(data, defaults),
            dedent("""\
        doc:
          outer_value: 3.0
        """),
        )

        # Emit using an empty root name.
        self.assertEqual(
            self._save(data, defaults, None),
            dedent("""\
        outer_value: 3.0
        """),
        )

        # Emit with an empty root name without defaults.
        self.assertEqual(
            self._save(defaults, defaults, None),
            dedent("""\
        {}
        """),
        )

    def test_dump_default_different_map_order1(self):
        # Same as the BasicExample1 from above, except that the map order of
        # the defaults vs data differs.  The defaults still take effect.
        defaults = OuterStructOpposite()
        defaults.inner_struct.inner_value = 1.0
        defaults.outer_value = 2.0
        data = OuterStruct()
        data.outer_value = 3.0
        data.inner_struct.inner_value = defaults.inner_struct.inner_value

        self.assertEqual(
            self._save(data, defaults),
            dedent("""\
        doc:
          outer_value: 3.0
        """),
        )

    def test_dump_default_different_map_order2(self):
        # Same as the BasicExample2 from above, except that the map order of
        # the defaults vs data differs.  The defaults still take effect.
        defaults = OuterStructOpposite()
        defaults.inner_struct.inner_value = 1.0
        defaults.outer_value = 2.0
        data = OuterStruct()
        data.outer_value = defaults.outer_value
        data.inner_struct.inner_value = 3.0

        self.assertEqual(
            self._save(data, defaults),
            dedent("""\
        doc:
          inner_struct:
            inner_value: 3.0
        """),
        )

    def test_dump_default_nulls(self):
        # YAML nulls are handled reasonably, without throwing.
        defaults = OptionalStruct()
        defaults.value = None
        data = copy.deepcopy(defaults)

        self.assertEqual(
            self._save(data, defaults),
            dedent("""\
        doc: {}
        """),
        )

    def test_dump_default_different_lists(self):
        # Lists differing in their values are not erased.
        defaults = ListStruct()
        defaults.value = [0.0, 0.0, 0.0]
        data = ListStruct()
        data.value = [1.0, 2.0, 3.0]

        self.assertEqual(
            self._save(data, defaults),
            dedent("""\
        doc:
          value: [1.0, 2.0, 3.0]
        """),
        )

    def test_dump_default_different_size_lists(self):
        # Lists differing in size (but sharing a prefix) are not erased.
        defaults = ListStruct()
        defaults.value = [1.0, 2.0]
        data = ListStruct()
        data.value = [1.0, 2.0, 3.0]

        self.assertEqual(
            self._save(data, defaults),
            dedent("""\
        doc:
          value: [1.0, 2.0, 3.0]
        """),
        )

    def test_dump_default_different_variant_tag(self):
        # Variants differing by tag are not erased.
        defaults = VariantStruct()
        defaults.value = NumpyStruct()
        data = VariantStruct()
        data.value = FloatStruct(1.0)

        self.assertEqual(
            self._save(data, defaults),
            dedent("""\
        doc:
          value: !FloatStruct
            value: 1.0
        """),
        )

    def test_dump_default_different_map_keys(self):
        # Maps differing in key only (same value) are not erased.
        defaults = MapStruct()
        defaults.value["b"] = 1.0
        data = MapStruct()
        data.value["a"] = 1.0

        self.assertEqual(
            self._save(data, defaults),
            dedent("""\
        doc:
          value:
            a: 1.0
        """),
        )

    def test_dump_default_different_map_values(self):
        # Maps differing in value only (same key) are not erased.
        defaults = MapStruct()
        defaults.value["a"] = 2.0
        data = MapStruct()
        data.value["a"] = 1.0

        self.assertEqual(
            self._save(data, defaults),
            dedent("""\
        doc:
          value:
            a: 1.0
        """),
        )


class TestYamlTypedWriteAcceptance(unittest.TestCase):
    """Acceptance tests for the typed yaml_load function(s).

    This test class is the Python flavor of the C++ test suite at
     drake/common/yaml/test/yaml_io_test.cc
    and should be roughly kept in sync with the test cases in that file.
    """

    def test_save_string(self):
        data = StringStruct(value="save_string")
        result = yaml_dump_typed(data)
        self.assertEqual(result, "value: save_string\n")

    def test_save_string_child(self):
        child_name = "some_child"
        data = StringStruct(value="save_string_child")
        result = yaml_dump_typed(data, child_name=child_name)
        self.assertEqual(result, "some_child:\n  value: save_string_child\n")

    def test_save_string_defaults(self):
        # N.B. The MapStruct.value dict contains a "nominal_float" by default.
        defaults = MapStruct()
        data = MapStruct()
        data.value["save_string"] = 1.0
        assert len(data.value) == 2

        # Only the non-default map entry is saved.
        result = yaml_dump_typed(data, defaults=defaults)
        self.assertEqual(
            result,
            dedent("""\
            value:
              save_string: 1.0
            """),
        )

    # Inside the implementation of yaml_dump_typed, the code to save to a
    # string versus a file shares all of the same dumping logic; only at the
    # last moment do we choose to write the output to a string or a file.
    # Therefore, we don't need to repeat all of the schema-specific test cases
    # for files. Instead, we can just spot-check a few calls to probe the file
    # handling and argument passing.

    def test_save_file(self):
        filename = Path(os.environ["TEST_TMPDIR"]) / "save_file.yaml"
        data = StringStruct(value="save_file")
        yaml_dump_typed(filename=filename, data=data)
        readback = filename.read_text(encoding="utf-8")
        self.assertEqual(readback, "value: save_file\n")

    def test_save_file_all_args(self):
        # N.B. The MapStruct.value dict contains a "nominal_float" by default.
        defaults = MapStruct()
        data = MapStruct()
        data.value["save_file"] = 1.0
        assert len(data.value) == 2

        filename = Path(os.environ["TEST_TMPDIR"]) / "save_file_all_args.yaml"
        yaml_dump_typed(
            filename=filename,
            data=data,
            child_name="some_child",
            defaults=defaults,
        )
        readback = filename.read_text(encoding="utf-8")
        self.assertEqual(
            readback,
            dedent("""\
            some_child:
              value:
                save_file: 1.0
            """),
        )

    def test_write_bad_schema(self):
        # N.B. This test covers python-specific error handling, so does not
        # have any corrresponding cases in the C++ unit tests.
        with self.assertRaisesRegex(Exception, "should have been a dict"):
            yaml_dump_typed([1.0], schema=typing.List[float])


class TestYamlTypedReadPybind11(unittest.TestCase):
    """Tests for (de)serializing into pybind11 objects."""

    def test_missing_serialize_binding(self):
        # For testing the error message in case of missing C++ bindings, we
        # just need any bound C++ class that doesn't have a Serialize().
        # We'll use Value to avoid dependencies on non-'common' code.
        invalid_cxx_class = Value[str]
        with self.assertRaisesRegex(RuntimeError, ".*lacks.*__fields__.*"):
            yaml_dump_typed(invalid_cxx_class())
        with self.assertRaisesRegex(RuntimeError, ".*lacks.*__fields__.*"):
            yaml_load_typed(schema=invalid_cxx_class, data="{}")

    def test_mydata2(self):
        data = dedent("""\
        some_bool: true
        some_int: 1
        some_uint64: 1
        some_float: 1.0
        some_double: 1.0
        some_string: one
        some_eigen:
        - [1.0]
        some_optional: 1.0
        some_vector: [1.0]
        some_map:
          one: 1.0
        some_variant: !MyData1
          quux: 1.0
        """)
        x = yaml_load_typed(schema=MyData2, data=data)
        self.assertEqual(x.some_bool, True)
        self.assertEqual(x.some_int, 1)
        self.assertEqual(x.some_uint64, 1)
        self.assertEqual(x.some_float, 1.0)
        self.assertEqual(x.some_double, 1.0)
        self.assertEqual(x.some_string, "one")
        self.assertEqual(x.some_eigen, [1.0])
        self.assertEqual(x.some_optional, 1.0)
        self.assertEqual(x.some_vector, [1.0])
        self.assertEqual(x.some_map, dict(one=1.0))
        self.assertEqual(x.some_variant.quux, 1.0)
        self.assertEqual(yaml_dump_typed(x), data)
