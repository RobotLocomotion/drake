import collections.abc
import copy
import dataclasses
import functools
import typing

import numpy as np
import yaml


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
        assert type(loader) == _SchemaLoader, loader
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
        keys_to_remove = [
            key for key in result.keys()
            if key.startswith("_")
        ]
        for key in keys_to_remove:
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
    if sum(bool(x) for x in [data, filename]) != 1:
        raise RuntimeError("Must specify exactly one of data= and filename=")
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

    def _represent_dict(self, data):
        """Permits a reverse of _SchemaLoader."""
        if "_tag" in data:
            tag = data["_tag"]
            data = copy.copy(data)
            del data["_tag"]
            return self.represent_mapping(tag, data)
        else:
            return super().represent_dict(data)


_SchemaDumper.add_representer(dict, _SchemaDumper._represent_dict)


def yaml_dump(data, *, filename=None):
    """Dumps a yaml object to a string or `filename` (if specified).

    This is the counterpart to `yaml_load(..., private=True)`, and will permit
    re-expressing the same tagged data.
    """
    if filename is not None:
        with open(filename, "w") as f:
            yaml.dump(data, f, Dumper=_SchemaDumper,
                      default_flow_style=_FLOW_STYLE)
    else:
        return yaml.dump(data, Dumper=_SchemaDumper,
                         default_flow_style=_FLOW_STYLE)


_LoadYamlOptions = collections.namedtuple("LoadYamlOptions", [
    "allow_yaml_with_no_schema",
    "allow_schema_with_no_yaml",
    "retain_map_defaults",
])


def _enumerate_field_types(obj):
    """Returns a Mapping[str, type] of field names and their types of the
    given concrete object `obj`.
    """
    # Dataclasses offer a public API for introspection.
    if dataclasses.is_dataclass(obj):
        return dict([
            (field.name, field.type)
            for field in dataclasses.fields(obj)])

    # Drake's DefAttributesUsingSerialize offers (hidden) introspection.
    fields = getattr(obj, "__fields__", None)
    if fields is not None:
        return dict([
            (field.name, field.type)
            for field in fields])

    # XXX
    # Otherwise, just fall back to scraping the properties and their _current_
    # runtime type (not the static schema type).
    obj_type = type(obj)
    raise NotImplementedError(f"{[obj_type, obj]}")
    result = dict()
    for name in dir(obj_type):
        if name.startswith("_"):
            continue
        if isinstance(getattr(obj_type, name), property):
            result[name] = type(getattr(obj, name))
    return result


def _get_nested_optional_type(schema):
    """If the given schema is equivalent to an Optional[Foo], then returns Foo.
    Otherwise, returns None.
    """
    generic_base = typing.get_origin(schema)
    if generic_base == typing.Union:
        generic_args = typing.get_args(schema)
        NoneType = type(None)
        if len(generic_args) == 2 and generic_args[-1] == NoneType:
            (nested_type, _) = generic_args
            return nested_type
    return None


def _merge_yaml_dict_item_into_target(*, options, name, yaml_value,
                                      target, value_schema):
    """XXX
    """
    # The target can be either a dictionary or a dataclass.
    if isinstance(target, collections.abc.Mapping):
        old_value = target[name]
        setter = functools.partial(target.__setitem__, name)
    else:
        old_value = getattr(target, name)
        setter = functools.partial(setattr, target, name)

    # Handle all of the plain YAML scalars:
    #  https://yaml.org/spec/1.2.2/#scalars
    #  https://yaml.org/spec/1.2.2/#json-schema
    if value_schema in (bool, int, float, str):
        new_value = value_schema(yaml_value)
        setter(new_value)
        return

    # Handle nullable types (std::optional<T> or typing.Optional[T]).
    nested_optional_type = _get_nested_optional_type(value_schema)
    if nested_optional_type is not None:
        if yaml_value is None:
            setter(None)
            return
        if old_value is None:
            setter(nested_optional_type())
        _merge_yaml_dict_item_into_target(
            options=options, name=name, yaml_value=yaml_value, target=target,
            value_schema=nested_optional_type)
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
            sub_target = {"_": value_type()}
            _merge_yaml_dict_item_into_target(
                options=options, name="_", yaml_value=sub_yaml_value,
                target=sub_target, value_schema=value_type)
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
        assert key_type == str
        if options.retain_map_defaults:
            new_value = copy.deepcopy(old_value)
        else:
            new_value = dict()
        for sub_key, sub_yaml_value in yaml_value.items():
            if sub_key not in new_value:
                new_value[sub_key] = value_type()
            _merge_yaml_dict_item_into_target(
                options=options, name=sub_key, yaml_value=sub_yaml_value,
                target=new_value, value_schema=value_type)
        setter(new_value)
        return

    # Handle schema sum types (std::variant<...> or typing.Union[...]).
    if generic_base is typing.Union:
        raise NotImplementedError("Union[]")

    # By this point, we've handled all known cases of generic types.
    if generic_base is not None:
        raise NotImplementedError(generic_base, value_schema)

    # If the value_schema is neither primitive nor generic, then we'll assume
    # it's a directly-nested subclass.
    new_value = copy.deepcopy(old_value)
    _merge_yaml_dict_into_target(
        options=options, yaml_dict=yaml_value,
        target=new_value, target_schema=value_schema)
    setter(new_value)


def _merge_yaml_dict_into_target(*, options, yaml_dict,
                                 target, target_schema):
    """Merges the given yaml_dict into the given target (of given type).
    The yaml_dict is a result of calling yaml_load. OK to be a child node.
    The target is an instance of some dataclass or pybind11 class.
    """
    assert isinstance(yaml_dict, collections.abc.Mapping), yaml_dict
    if "_tag" in yaml_dict:
        raise NotImplementedError(yaml_dict)
    static_field_map = _enumerate_field_types(target_schema)
    schema_names = list(static_field_map.keys())
    schema_optionals = set([
        name for name, sub_schema in static_field_map.items()
        if _get_nested_optional_type(sub_schema) is not None
    ])
    yaml_names = list(yaml_dict.keys())
    extra_yaml_names = [
        name for name in yaml_names
        if name not in schema_names
    ]
    missing_yaml_names = [
        name for name, sub_schema in static_field_map.items()
        if name not in yaml_names and name not in schema_optionals
    ]
    if extra_yaml_names and not options.allow_yaml_with_no_schema:
        raise RuntimeError(
            f"The fields {extra_yaml_names} were unknown to the schema")
    if missing_yaml_names and not options.allow_schema_with_no_yaml:
        raise RuntimeError(
            f"The fields {missing_yaml_names} were missing in the yaml data")
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
            options=options, name=name, yaml_value=sub_value,
            target=target, value_schema=sub_schema)


def yaml_load_typed(*, schema,
                    data=None,
                    filename=None,
                    child_name=None,
                    defaults=None,
                    allow_yaml_with_no_schema=None,
                    allow_schema_with_no_yaml=None,
                    retain_map_defaults=None):
    """Loads either a `data` str xor a `filename` (exactly one must be
    specified) against the given `schema` type and returns an instance
    of that type.

    This mimics the C++ function drake::common::yaml::LoadYamlFile.
    """
    # Choose the allow/retain setting in case none were provided.
    if allow_yaml_with_no_schema is None:
        allow_yaml_with_no_schema = False
    if allow_schema_with_no_yaml is None:
        allow_cpp_with_no_yaml = defaults is not None
    if retain_map_defaults is None:
        retain_map_defaults = defaults is not None
    options = _LoadYamlOptions(
        allow_yaml_with_no_schema=allow_yaml_with_no_schema,
        allow_schema_with_no_yaml=allow_schema_with_no_yaml,
        retain_map_defaults=retain_map_defaults)

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

    # Merge the document into the result.
    _merge_yaml_dict_into_target(
        options=options, yaml_dict=root_node,
        target=result, target_schema=schema)
    return result


__all__ = [
    "yaml_dump",
    "yaml_load",
    "yaml_load_data",
    "yaml_load_file",
    "yaml_load_typed",
]
