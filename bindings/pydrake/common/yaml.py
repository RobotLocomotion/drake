import collections.abc
import copy
import dataclasses
import functools
import math
from pathlib import Path
import types
import typing

import numpy as np
import yaml
import yaml.representer

from pydrake.common import pretty_class_name


class _SchemaLoader(yaml.loader.SafeLoader):
    """Customizes SafeLoader for the purposes of this module."""

    @staticmethod
    def _handle_multi_variant(loader, tag, node):
        """Parses a variant-like YAML node (akin to std::variant<> in C++).

        Returns a dict with the inner mapping, plus one extra item with key
        `_tag` that holds the YAML tag.

        For example, a !Gaussian YAML node is returned as:
        {
          "_tag": "!Gaussian",
          "mean": 2.0,
          "std": 4.0,
        }.
        """
        assert type(loader) is _SchemaLoader, loader
        result = loader.construct_mapping(node)
        result.update({"_tag": tag})
        return result


_SchemaLoader.add_multi_constructor("", _SchemaLoader._handle_multi_variant)


def yaml_load_data(data, *, private=False):
    """Loads and returns the given `data` str as a yaml object, while also
    accounting for variant-like type tags.  Any tags are reported as an
    extra "_tag" field in the returned dictionary.

    (Alternatively, `data` may be a file-like stream instead of a str.)

    By default, removes any root-level keys that begin with an underscore, so
    that yaml anchors and templates are invisibly easy to use.  Callers that
    wish to receive the private data may pass `private=True`.

    This function returns the raw, untyped data (dict, list, str, float, etc.)
    without any schema checking nor default values. To load with respect to
    a schema with defaults, see ``yaml_load_typed()``.
    """
    result = yaml.load(data, Loader=_SchemaLoader)
    if not private:
        try:
            all_keys = list(result.keys())
        except AttributeError:
            all_keys = []
        for key in all_keys:
            if key.startswith("_"):
                del result[key]
    return result


def yaml_load_file(filename, *, private=False):
    """Loads and returns the given `filename` as a yaml object, while also
    accounting for variant-like type tags.

    See yaml_load_data for full details; this function differs only by reading
    the yaml data from a file instead of memory.
    """
    with open(filename, "r") as data:
        return yaml_load_data(data, private=private)


def yaml_load(*, data=None, filename=None, private=False):
    """Loads either a `data` str xor a `filename` (exactly one must be
    specified) and returns the value as a yaml object.

    This is sugar for calling either yaml_load_data or yaml_load_file;
    refer to those functions for additional details.

    This function returns the raw, untyped data (dict, list, str, float, etc.)
    without any schema checking nor default values. To load with respect to
    a schema with defaults, see ``yaml_load_typed()``.
    """
    has_data = data is not None
    has_filename = filename is not None
    if has_data and has_filename:
        raise RuntimeError(
            "Exactly one of `data=...` or `filename=...` must be provided, "
            "but both were non-None"
        )
    if not has_data and not has_filename:
        raise RuntimeError(
            "Exactly one of `data=...` or `filename=...` must be provided, "
            "but both were None"
        )
    if data:
        return yaml_load_data(data, private=private)
    else:
        return yaml_load_file(filename, private=private)


# This is a magic tribool value described at
#   https://github.com/yaml/pyyaml/pull/256
# and must be set this way for post- and pre- pyyaml#256 yaml dumpers to give
# the same output.
_FLOW_STYLE = None


class _SchemaDumper(yaml.dumper.SafeDumper):
    """Customizes SafeDumper for the purposes of this module."""

    DRAKE_EXPLICIT_TAG_PREFIX = "tag:drake.mit.edu/explicit:"

    class ExplicitScalar(typing.NamedTuple):
        """Wrapper type used when dumping a document. When this type is dumped,
        it will always emit the tag, e.g., `!!int 10`. (By default, tags for
        scalars are not emitted by pyyaml.)
        """

        value: typing.Union[bool, int, float, str]
        schema: type  # One of either bool, int, float, or str.

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # Override a superclass class variable with a custom instance variable.
        prefixes = copy.copy(yaml.dumper.SafeDumper.DEFAULT_TAG_PREFIXES)
        prefixes[_SchemaDumper.DRAKE_EXPLICIT_TAG_PREFIX] = "!!"
        self.DEFAULT_TAG_PREFIXES = prefixes

    def _represent_explicit_scalar(self, explicit_scalar):
        assert isinstance(explicit_scalar, _SchemaDumper.ExplicitScalar)
        value = explicit_scalar.value
        schema = explicit_scalar.schema
        assert schema in _PRIMITIVE_YAML_TYPES
        node = self.yaml_representers[type(value)](self, value)
        prefix = _SchemaDumper.DRAKE_EXPLICIT_TAG_PREFIX
        node.tag = f"{prefix}{schema.__name__}"
        return node

    def _represent_dict(self, data):
        """Permits a reverse of _SchemaLoader."""
        if "_tag" in data:
            tag = data["_tag"]
            data = copy.copy(data)
            del data["_tag"]
            return self.represent_mapping(tag, data)
        else:
            return super().represent_dict(data)

    def _represent_undefined(self, data):
        if getattr(type(data), "__module__", "").startswith("pydrake"):
            raise yaml.representer.RepresenterError(
                "yaml_dump does not operate on pydrake objects; "
                "use yaml_dump_typed instead",
                data,
            )
        return super().represent_undefined(data)


_SchemaDumper.add_representer(None, _SchemaDumper._represent_undefined)
_SchemaDumper.add_representer(dict, _SchemaDumper._represent_dict)
_SchemaDumper.add_representer(
    _SchemaDumper.ExplicitScalar, _SchemaDumper._represent_explicit_scalar
)


class _DrakeFlowSchemaDumper(_SchemaDumper):
    """Implements the 'drake flow' emitter styling, which follows the
    conventions from yaml_write_archive.cc:

    - For sequences: if all children are scalars, then formats the sequence
      onto a single line; otherwise, format as a bulleted list. This exact
      logic is already implemented in PyYAML when using _DEFAULT_FLOW_STYLE.

    - For mappings: If there are no children, then formats this map onto a
      single line; otherwise, format over multiple lines.
    """

    def serialize_node(self, node, parent, index):
        if isinstance(node, yaml.MappingNode):
            node.flow_style = len(node.value) == 0
        return super().serialize_node(node, parent, index)


def yaml_dump(data, *, filename=None):
    """Dumps a yaml object to a string or `filename` (if specified).

    This is the counterpart to `yaml_load(..., private=True)`, and will permit
    re-expressing the same tagged data.
    """
    if filename is not None:
        with open(filename, "w") as f:
            yaml.dump(
                data, f, Dumper=_SchemaDumper, default_flow_style=_FLOW_STYLE
            )
    else:
        return yaml.dump(
            data, Dumper=_SchemaDumper, default_flow_style=_FLOW_STYLE
        )


_LoadYamlOptions = collections.namedtuple(
    "LoadYamlOptions",
    [
        "allow_yaml_with_no_schema",
        "allow_schema_with_no_yaml",
        "retain_map_defaults",
    ],
)


def _enumerate_field_types(schema):
    """Returns a Mapping[str, type] of the schema-based field names and types
    of the given type `schema`.
    """
    assert isinstance(schema, type)

    # Dataclasses offer a public API for introspection.
    if dataclasses.is_dataclass(schema):
        return dict(
            [(field.name, field.type) for field in dataclasses.fields(schema)]
        )

    # Drake's DefAttributesUsingSerialize offers (hidden) introspection.
    fields = getattr(schema, "__fields__", None)
    if fields is not None:
        return dict([(field.name, field.type) for field in fields])

    # Detect when the user forgot to use DefAttributesUsingSerialize.
    if getattr(type(schema), "__name__", None) == "pybind11_type":
        raise NotImplementedError(
            f"The bound C++ type {schema} cannot be used as a schema because"
            f" it lacks a __fields__ attribute."
            f" Use DefAttributesUsingSerialize to add that attribute."
        )

    raise NotImplementedError(
        f"Schema objects of type {schema} are not yet supported"
    )


def _is_union(generic_base):
    """Given a generic_base type (from typing.get_origin), returns True iff it
    is a sum type (i.e., a Union).
    """
    return generic_base in [typing.Union, types.UnionType]


def _get_nested_optional_type(schema):
    """If the given schema (i.e., type) is equivalent to an Optional[Foo], then
    returns Foo. Otherwise, returns None.
    """
    generic_base = typing.get_origin(schema)
    if _is_union(generic_base):
        generic_args = typing.get_args(schema)
        NoneType = type(None)
        if len(generic_args) == 2 and generic_args[-1] == NoneType:
            (nested_type, _) = generic_args
            return nested_type
    return None


def _create_from_schema(*, schema, forthcoming_value):
    """Given a schema (i.e., a type), returns a default-constructed instance
    of that schema type, modulo a few exceptional cases that are specific to
    our parsing semantics as detailed below.

    When schema is a sum type (e.g., typing.Union[...]), returns None.

    When schema is a numpy type, returns a 1-d ndarray with the same size as
    the ``forthcoming_value`` if provided, otherwise an empty array.
    """
    if _is_union(typing.get_origin(schema)):
        return None
    if schema == np.ndarray:
        size = len(forthcoming_value)
        return np.array([math.nan] * size)
    return schema()


# For details, see:
#  https://yaml.org/spec/1.2.2/#scalars
#  https://yaml.org/spec/1.2.2/#json-schema
_PRIMITIVE_JSON_TYPES = (bool, int, float, str)
_PRIMITIVE_YAML_TYPES = _PRIMITIVE_JSON_TYPES + (bytes,)


def _convert_yaml_primitive_to_schema_type(*, yaml_value, value_schema):
    """Given a primitive yaml value parsed from a document and the declared
    schema type we are reading it into, converts the value to the requested
    schema type or else returns None when the types are not compatible.

    We need to be very careful that whatever the user wrote it their document
    is authentically represented by our return value, so we need to reject
    information-losing conversions like the plain scalar `3.0e+4` in the yaml
    to a `str` in the schema, since the str would be '30000.0` instead of what
    the user wrote in their document.
    """
    yaml_value_type = type(yaml_value)
    # Allow exact matches.
    if yaml_value_type is value_schema:
        return yaml_value
    # Allow numeric promotion from int -> float.
    if yaml_value_type is int and value_schema is float:
        return float(yaml_value)
    # Allow numeric demotion from float -> int iff no information loss.
    if yaml_value_type is float and value_schema is int:
        if yaml_value.is_integer():
            return int(yaml_value)
    # Allow parsing strings to numbers using Python's number parsers.
    if yaml_value_type is str and value_schema in (float, int):
        return value_schema(yaml_value)
    # Allow parsing strings to bools iff they are in YAML 1.2 canonical form.
    if yaml_value_type is str and value_schema is bool:
        if yaml_value == "true":
            return True
        if yaml_value == "false":
            return False
    # Don't allow any other primitive conversions.
    return None


def _merge_yaml_dict_item_into_target(
    *, options, name, yaml_value, target, value_schema
):
    """Parses the given `yaml_value` into an object of type `value_schema`,
    writing the result to the field named `name` of the given `target` object.
    """
    # The target can be either a dictionary or a dataclass.
    assert target is not None
    if isinstance(target, collections.abc.Mapping):
        getter = functools.partial(target.__getitem__, name)
        setter = functools.partial(target.__setitem__, name)
    else:
        getter = functools.partial(getattr, target, name)
        setter = functools.partial(setattr, target, name)

    # Handle all of the plain YAML scalars:
    #  https://yaml.org/spec/1.2.2/#scalars
    #  https://yaml.org/spec/1.2.2/#json-schema
    #  https://yaml.org/type/binary.html
    if value_schema in _PRIMITIVE_YAML_TYPES:
        if type(yaml_value) in (list, dict):
            raise RuntimeError(
                f"Expected a {value_schema} value for '{name}' but instead got"
                f" non-scalar yaml data of type {type(yaml_value)}"
            )
        new_value = _convert_yaml_primitive_to_schema_type(
            yaml_value=yaml_value,
            value_schema=value_schema,
        )
        if new_value is None:
            raise RuntimeError(
                f"Expected a {value_schema} value for '{name}' but instead got"
                f" yaml data of type {type(yaml_value)} ({yaml_value!r})"
            )
        setter(new_value)
        return

    # Handle nullable types (std::optional<T> or typing.Optional[T]).
    nested_optional_type = _get_nested_optional_type(value_schema)
    if nested_optional_type is not None:
        # If the yaml was null, the Python field will be None.
        if yaml_value is None:
            setter(None)
            return
        # Create a non-null default value, if necessary.
        old_value = getter()
        if old_value is None:
            setter(
                _create_from_schema(
                    schema=nested_optional_type, forthcoming_value=yaml_value
                )
            )
        # Now we can parse Optional[Foo] like a plain Foo.
        _merge_yaml_dict_item_into_target(
            options=options,
            name=name,
            yaml_value=yaml_value,
            target=target,
            value_schema=nested_optional_type,
        )
        return

    # Handle pathlib.Path.
    if value_schema == Path:
        if not isinstance(yaml_value, str):
            raise RuntimeError(
                f"Expected a !!str value for '{name}: Path' but instead got"
                f" yaml data of type {type(yaml_value)}"
            )
        new_value = Path(yaml_value)
        setter(new_value)
        return

    # Handle NumPy types.
    if value_schema == np.ndarray:
        new_value = np.array(yaml_value, dtype=float)
        setter(new_value)
        return

    # Check if the field is generic like list[str]; if yes, the generic_base
    # will be, e.g., `list` and generic_args will be, e.g., `[str]`.
    generic_base = typing.get_origin(value_schema)
    generic_args = typing.get_args(value_schema)

    # Handle YAML sequences:
    #  https://yaml.org/spec/1.2.2/#sequence
    #
    # In Drake's YamlLoad convention, merging a sequence denotes *overwriting*
    # what was there.
    if generic_base in (list, typing.List):
        (value_type,) = generic_args
        new_value = []
        for sub_yaml_value in yaml_value:
            sub_target = {
                "_": _create_from_schema(
                    schema=value_type, forthcoming_value=sub_yaml_value
                )
            }
            _merge_yaml_dict_item_into_target(
                options=options,
                name="_",
                yaml_value=sub_yaml_value,
                target=sub_target,
                value_schema=value_type,
            )
            new_value.append(sub_target["_"])
        setter(new_value)
        return

    # Handle YAML maps:
    #  https://yaml.org/spec/1.2.2/#mapping
    #
    # In Drake's YamlLoad convention, merging a mapping denotes *updating*
    # what was there iff retain_map_defaults was set.
    if generic_base in (dict, collections.abc.Mapping):
        (key_type, value_type) = generic_args
        # This requirement matches what we have in C++. Allowing sequences
        # or maps as keys would mean we're no longer JSON-compatible.
        assert key_type is str
        if options.retain_map_defaults:
            old_value = getter()
            new_value = copy.deepcopy(old_value)
        else:
            new_value = dict()
        for sub_key, sub_yaml_value in yaml_value.items():
            # In case the sub_key does not exist in the map, insert it.
            if sub_key not in new_value:
                new_value[sub_key] = _create_from_schema(
                    schema=value_type, forthcoming_value=sub_yaml_value
                )
            _merge_yaml_dict_item_into_target(
                options=options,
                name=sub_key,
                yaml_value=sub_yaml_value,
                target=new_value,
                value_schema=value_type,
            )
        setter(new_value)
        return

    # Handle schema sum types (std::variant<...> or typing.Union[...]).
    if _is_union(generic_base):
        # The YAML data might be a scalar value (as opposed to a mapping).
        yaml_value_type = type(yaml_value)
        if yaml_value_type in list(_PRIMITIVE_YAML_TYPES) + [type(None)]:
            if yaml_value_type not in generic_args:
                raise RuntimeError(
                    f"The schema sum type for '{name}' cannot accept a yaml "
                    f"value '{yaml_value}' of type {yaml_value_type}; only "
                    f"one of {generic_args} are acceptable"
                )
            setter(yaml_value)
            return
        # A mapping can optionally specify a type tag to choose which Union[]
        # type to parse into. When none is provided, the default is to use the
        # first option listed in the sum type (to match what C++ does).
        if yaml_value is not None and "_tag" in yaml_value:
            # Pop the type tag out of the yaml_value dictionary.
            refined_yaml_value = copy.copy(yaml_value)
            _tag = refined_yaml_value.pop("_tag")
            assert _tag.startswith("!"), yaml_value
            tag = _tag[1:]
            # Find which Union[] type argument matches the tag.
            refined_value_schema = None
            for candidate in generic_args:
                candidate_name = pretty_class_name(candidate)
                if "[" in candidate_name:
                    # When matching vs template type, compare the base name.
                    candidate_name = candidate_name.split("[", 1)[0]
                if candidate_name == tag:
                    refined_value_schema = candidate
                    break
            else:
                raise RuntimeError(
                    f"The yaml type tag value '{tag}' did not match any of the"
                    f" allowed type options for '{name}' ({generic_args})"
                )
        else:
            refined_yaml_value = yaml_value
            refined_value_schema = generic_args[0]
        # Self-call, but now with an updated value and type.
        refined_value_schema_origin = typing.get_origin(refined_value_schema)
        if refined_value_schema_origin is None:
            refined_value_schema_origin = refined_value_schema
        if not isinstance(getter(), refined_value_schema_origin):
            setter(
                _create_from_schema(
                    schema=refined_value_schema_origin,
                    forthcoming_value=yaml_value,
                )
            )
        _merge_yaml_dict_item_into_target(
            options=options,
            name=name,
            yaml_value=refined_yaml_value,
            target=target,
            value_schema=refined_value_schema,
        )
        return

    # By this point, we've handled all known cases of generic types.
    if generic_base is not None:
        raise NotImplementedError(
            f"The generic type {generic_base} of {value_schema} is "
            "not yet supported"
        )

    # If the value_schema is neither primitive nor generic, then we'll assume
    # it's a directly-nested subclass.
    old_value = getter()
    new_value = copy.deepcopy(old_value)
    _merge_yaml_dict_into_target(
        options=options,
        yaml_dict=yaml_value,
        target=new_value,
        target_schema=value_schema,
    )
    setter(new_value)


def _merge_yaml_dict_into_target(*, options, yaml_dict, target, target_schema):
    """Merges the given yaml_dict into the given target (of given type).
    The target must be an instance of some dataclass or pybind11 class.
    The yaml_dict must be typed like the result of calling yaml_load (i.e.,
    raw strings, dictionaries, lists, etc.).
    """
    assert isinstance(yaml_dict, collections.abc.Mapping), yaml_dict
    assert target is not None
    static_field_map = _enumerate_field_types(target_schema)
    schema_names = list(static_field_map.keys())
    schema_optionals = set(
        [
            name
            for name, sub_schema in static_field_map.items()
            if _get_nested_optional_type(sub_schema) is not None
        ]
    )
    yaml_names = list(yaml_dict.keys())
    extra_yaml_names = [name for name in yaml_names if name not in schema_names]
    missing_yaml_names = [
        name
        for name, sub_schema in static_field_map.items()
        if name not in yaml_names and name not in schema_optionals
    ]
    if extra_yaml_names and not options.allow_yaml_with_no_schema:
        raise RuntimeError(
            f"The fields {extra_yaml_names} were unknown to the schema"
        )
    if missing_yaml_names and not options.allow_schema_with_no_yaml:
        raise RuntimeError(
            f"The fields {missing_yaml_names} were missing in the yaml data"
        )
    for name, sub_schema in static_field_map.items():
        if name in yaml_dict:
            sub_value = yaml_dict[name]
        elif name in schema_optionals:
            # For Optional fields that are missing from the yaml data, we must
            # match the C++ heuristic: when "allow no yaml" is set, we'll leave
            # the existing value unchanged. Otherwise, we need to affirmatively
            # to set the target to the None value.
            if options.allow_schema_with_no_yaml:
                continue
            sub_value = None
        else:
            # Errors for non-Optional missing yaml data have already been
            # implemented above (see "missing_yaml_names"), so we should just
            # skip over those fields here. They will remain unchanged.
            continue
        _merge_yaml_dict_item_into_target(
            options=options,
            name=name,
            yaml_value=sub_value,
            target=target,
            value_schema=sub_schema,
        )


def yaml_load_typed(
    *,
    schema=None,
    data=None,
    filename=None,
    child_name=None,
    defaults=None,
    allow_yaml_with_no_schema=False,
    allow_schema_with_no_yaml=True,
    retain_map_defaults=True,
):
    """Loads either a ``data`` str or a ``filename`` against the given
    ``schema`` type and returns an instance of that type.

    This mimics the C++ function ``drake::common::yaml::LoadYamlFile``.
    This is the complementary operation to ``yaml_dump_typed``.

    Args:
        schema: The type to load as. This either must be a ``dataclass``, a C++
            class bound using pybind11 and ``DefAttributesUsingSerialize``, or
            a ``Mapping[str, ...]`` where the mapping's value type is one of
            those two categories. If a non-None ``defaults`` is provided, then
            ``schema`` can be ``None`` and will use ``type(defaults)`` in that
            case.
        data: The string of YAML data to be loaded. Exactly one of either
            ``data`` or ``filename`` must be provided.
        filename: The filename of YAML data to be loaded. Exactly one of either
            ``data`` or ``filename`` must be provided.
        child_name: If provided, loads data from given-named child of the
            document's root instead of the root itself.
        defaults: If provided, then the object being read into will be
            initialized using this value instead of the schema's default
            constructor.
        allow_yaml_with_no_schema: Allows yaml Maps to have extra key-value
            pairs that are specified by the schema being parsed into. In other
            words, the schema argument provides only an incomplete schema for
            the YAML data. This allows for parsing only a subset of the YAML
            data.
        allow_schema_with_no_yaml: Allows the schema to provide more key-value
            pairs than are present in the YAML data. In other words, objects
            can have default values that are left intact unless the YAML data
            provides a value.
        retain_map_defaults: If set to true, when parsing a Mapping the loader
           will merge the YAML data into the destination, instead of replacing
           the dict contents entirely. In other words, a Mapping field in a
           schema can have default values that are left intact unless the YAML
           data provides a value *for that specific key*.
    """

    # Infer the schema when possible.
    if schema is None:
        if defaults is None:
            raise ValueError(
                "At least one of schema= and defaults= must be provided"
            )
        schema = type(defaults)

    # Choose the allow/retain setting in case none were provided.
    options = _LoadYamlOptions(
        allow_yaml_with_no_schema=allow_yaml_with_no_schema,
        allow_schema_with_no_yaml=allow_schema_with_no_yaml,
        retain_map_defaults=retain_map_defaults,
    )

    # Create the result object.
    if defaults is not None:
        result = copy.deepcopy(defaults)
    else:
        result = schema()

    # Parse the YAML document.
    document = yaml_load(data=data, filename=filename)
    if child_name is not None:
        root_node = document[child_name]
    else:
        root_node = document
    if not isinstance(root_node, collections.abc.Mapping):
        raise RuntimeError(
            f"YAML root was a {type(root_node)} but should have been a dict"
        )

    # Merge the document into the result.
    _merge_yaml_dict_into_target(
        options=options,
        yaml_dict=root_node,
        target=result,
        target_schema=schema,
    )
    return result


def _yaml_dump_get_attribute(*, obj, name):
    """Given an object ``obj`` and attribute name ``name``, gets and returns
    the value of the given attribute, akin to ``getattr`. The difference is
    that for bound C++ types sometimes the Serialize function does non-standard
    tricks, so the yaml dictionary key name in __fields__ differs from the
    attribute name. This function provides a protocol for the bound type to
    communicate that distinction during yaml_dump_typed.
    """
    if hasattr(obj, "_rewrite_yaml_dump_attr_name"):
        name = obj._rewrite_yaml_dump_attr_name(name)
    return getattr(obj, name)


def _yaml_dump_typed_item(*, obj, schema):
    """Given an object ``obj`` and its type ``schema``, returns the plain YAML
    object that should be serialized. Objects that are already primitive types
    (str, float, etc.) are returned unchanged. Bare collection types (List and
    Mapping) are processed recursively. Structs (dataclasses) are processed
    using their schema. The result is "plain" in the sense that's it's always
    just a tree of primitives, lists, and dicts -- no user-defined types.
    """
    assert schema is not None

    # Handle all of the plain YAML scalars:
    #  https://yaml.org/spec/1.2.2/#scalars
    #  https://yaml.org/spec/1.2.2/#json-schema
    if schema in _PRIMITIVE_YAML_TYPES:
        return schema(obj)

    # Check if the field is generic like list[str]; if yes, the generic_base
    # will be, e.g., `list` and generic_args will be, e.g., `[str]`.
    generic_base = typing.get_origin(schema)
    generic_args = typing.get_args(schema)

    # Handle YAML sequences:
    #  https://yaml.org/spec/1.2.2/#sequence
    if generic_base in (list, typing.List):
        (item_schema,) = generic_args
        return [
            _yaml_dump_typed_item(obj=item, schema=item_schema) for item in obj
        ]

    # Handle YAML maps:
    #  https://yaml.org/spec/1.2.2/#mapping
    if generic_base in (dict, collections.abc.Mapping):
        (key_schema, value_schema) = generic_args
        if key_schema is not str:
            # This requirement matches what we have in C++. Allowing sequences
            # or maps as keys would mean we're no longer JSON-compatible.
            raise RuntimeError(f"Dict keys must be strings, not {key_schema}")
        result = dict()
        for key in sorted(obj):
            value = obj[key]
            value_plain = _yaml_dump_typed_item(obj=value, schema=value_schema)
            result[key] = value_plain
        return result

    # Handle nullable types (std::optional<T> or typing.Optional[T]).
    optional_schema = _get_nested_optional_type(schema)
    if optional_schema is not None:
        if obj is None:
            return None
        return _yaml_dump_typed_item(obj=obj, schema=optional_schema)

    # Handle schema sum types (std::variant<...> or typing.Union[...]).
    if _is_union(generic_base):
        if obj is None:
            if type(None) in generic_args:
                return None
            raise RuntimeError(f"The {schema} does not allow None as a value")
        match = None
        for i, one_schema in enumerate(generic_args):
            one_schema_origin = typing.get_origin(one_schema)
            if one_schema_origin is None:
                one_schema_origin = one_schema
            if isinstance(obj, one_schema_origin):
                match = i
                break
        if match is None:
            raise RuntimeError(
                f"A value of type {type(obj)} did not match any {schema}"
            )
        union_schema = generic_args[i]
        result = _yaml_dump_typed_item(obj=obj, schema=union_schema)
        if i != 0:
            if union_schema in _PRIMITIVE_JSON_TYPES:
                result = _SchemaDumper.ExplicitScalar(
                    value=result, schema=union_schema
                )
            elif union_schema in _PRIMITIVE_YAML_TYPES:
                # The correct tag will be automatically applied by pyyaml with
                # no special effort on our part.
                pass
            else:
                class_name_with_args = pretty_class_name(union_schema)
                class_name = class_name_with_args.split("[", 1)[0]
                result["_tag"] = "!" + class_name
        return result

    # Handle pathlib.Path types.
    if schema == Path:
        return str(obj)

    # Handle NumPy types.
    if schema == np.ndarray:
        # TODO(jwnimmer-tri) We should use the numpy.typing module here to
        # statically specify a shape and/or dtype in the schema. For now,
        # we only support floats with no restrictions on the shape.
        assert obj.dtype == np.dtype(np.float64)
        list_value = obj.tolist()
        list_schema = float
        for _ in obj.shape:
            list_schema = typing.List[list_schema]
        return _yaml_dump_typed_item(obj=list_value, schema=list_schema)

    # If no special case matched, then we'll assume it's a nested class and
    # dump its fields one by one.
    result = dict()
    for name, item_schema in _enumerate_field_types(schema).items():
        item_obj = _yaml_dump_get_attribute(obj=obj, name=name)
        item_plain = _yaml_dump_typed_item(obj=item_obj, schema=item_schema)
        if item_plain is None:
            if _get_nested_optional_type(item_schema) is not None:
                # When an Optional member field is set to None, then don't emit
                # "{name}: null"; instead, just skip it entirely.
                continue
        result[name] = item_plain
    return result


def _erase_matching_maps(*, node, defaults):
    """This logic mimics the C++ YamlWriteArchive function of the same name."""
    # Remove from `node` any key-value pair that is identical within both
    # `node` and `defaults`.
    keys_to_prune = []
    for key in node:
        if key not in defaults:
            # Don't prune and don't recurse.
            continue
        sub_node = node[key]
        sub_defaults = defaults[key]
        if sub_node is sub_defaults or sub_node == sub_defaults:
            # Found a match. Prune and don't recurse. (We check both 'is' and
            # '==' because `math.nan` does not compare equal to itself.)
            keys_to_prune.append(key)
            continue
        if not isinstance(sub_node, dict):
            continue
        if sub_node.get("_tag") != sub_defaults.get("_tag"):
            # The maps are tagged differently, so we should not subtract their
            # children, since they may have different semantics.
            continue
        # Recurse into children with the same key name.
        _erase_matching_maps(node=sub_node, defaults=sub_defaults)
    for key in keys_to_prune:
        del node[key]


def yaml_dump_typed(
    data, *, filename=None, schema=None, child_name=None, defaults=None
):
    """Dumps an object to a YAML string or ``filename`` (if specified), using
    the ``schema`` in order to support non-primitive types.

    This mimics the C++ function ``drake::common::yaml::SaveYamlFile``.
    This is the complementary operation to ``yaml_load_typed``.

    Args:
        data: The object to be dumped.
        filename: If provided, the YAML filename to be written to. When None,
            this function will return a YAML string instead of writing to a
            file.
        schema: If provided, the type to dump as. When None, the default is
            ``type(data)``. This either must be a ``dataclass``, a C++ class
            bound using pybind11 and ``DefAttributesUsingSerialize``, or a
            ``Mapping[str, ...]`` where the mapping's value type is one of
            those two categories.
        child_name: If provided, dumps using given name as the document root,
            with the ``data`` nested underneath.
        defaults: If provided, then only data that differs from the given
            ``defaults`` will be dumped.
    """
    # Sanity checks.
    assert data is not None
    if child_name is not None:
        if type(child_name) not in _PRIMITIVE_JSON_TYPES:
            raise RuntimeError(
                "The child_name must be a primitive type, "
                f"not a {type(child_name)}"
            )

    # If no schema was provided, then choose one.
    if schema is None:
        schema = type(data)

    # Convert to a tree of primitives, lists, and dicts.
    root = _yaml_dump_typed_item(obj=data, schema=schema)

    # If a baseline value was provided, then subtract it from the result.
    if defaults is not None:
        plain_defaults = _yaml_dump_typed_item(obj=defaults, schema=schema)
        _erase_matching_maps(node=root, defaults=plain_defaults)

    # If a child_name was provided, then weave it into the root.
    if child_name is not None:
        root = {child_name: root}

    # To align with the capabilities of yaml_load_typed, we limit the root to
    # be a mapping node (not scalar nor list).
    if not isinstance(root, collections.abc.Mapping):
        raise RuntimeError(
            f"YAML root was a {type(root)} but should have been a dict"
        )

    # Write the data to disk xor return a string, based on the presence of a
    # filename. Use layout options to match the C++ (SaveYamlFile) style.
    dump_func = functools.partial(
        yaml.dump,
        Dumper=_DrakeFlowSchemaDumper,
        default_flow_style=_FLOW_STYLE,
        sort_keys=False,
    )
    if filename is not None:
        with open(filename, "w", encoding="utf-8") as f:
            dump_func(root, f)
    else:
        return dump_func(root)


__all__ = [
    "yaml_dump",
    "yaml_dump_typed",
    "yaml_load",
    "yaml_load_data",
    "yaml_load_file",
    "yaml_load_typed",
]
