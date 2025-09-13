#pragma once

// GENERATED FILE DO NOT EDIT
// This file contains docstrings for the Python bindings that were
// automatically extracted by mkdoc.py.

#include <array>
#include <utility>

#if defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

// #include "drake/common/yaml/yaml_io.h"
// #include "drake/common/yaml/yaml_io_options.h"
// #include "drake/common/yaml/yaml_node.h"
// #include "drake/common/yaml/yaml_read_archive.h"
// #include "drake/common/yaml/yaml_write_archive.h"

// Symbol: pydrake_doc_common_yaml
constexpr struct /* pydrake_doc_common_yaml */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::yaml
    struct /* yaml */ {
      // Symbol: drake::yaml::LoadYamlFile
      struct /* LoadYamlFile */ {
        // Source: drake/common/yaml/yaml_io.h:36
        const char* doc =
R"""(Loads data from a YAML-formatted file.

Refer to yaml_serialization "YAML Serialization" for background and
examples.

Parameter ``filename``:
    Filename to be read from.

Parameter ``child_name``:
    (optional) If provided, loads data from given-named child of the
    document's root instead of the root itself.

Parameter ``defaults``:
    (optional) If provided, then the structure being read into will be
    initialized using this value instead of the default constructor,
    and also (unless the ``options`` argument is provided and
    specifies otherwise) any member fields that are not mentioned in
    the YAML will retain their default values.

Parameter ``options``:
    (optional, advanced) If provided, overrides the nominal parsing
    options. Most users should not specify this; the default is
    usually correct.

Returns:
    the loaded user data.

Template parameter ``Serializable``:
    must implement a implementing_serialize "Serialize" function and
    be default constructible.)""";
      } LoadYamlFile;
      // Symbol: drake::yaml::LoadYamlOptions
      struct /* LoadYamlOptions */ {
        // Source: drake/common/yaml/yaml_io_options.h:12
        const char* doc =
R"""(Configuration for LoadYamlFile() and LoadYamlString() to govern when
certain conditions are errors or not. Refer to the member fields for
details.)""";
        // Symbol: drake::yaml::LoadYamlOptions::allow_cpp_with_no_yaml
        struct /* allow_cpp_with_no_yaml */ {
          // Source: drake/common/yaml/yaml_io_options.h:24
          const char* doc =
R"""(Allows Serializables to provide more key-value pairs than are present
in the YAML data. In other words, the structs have default values that
are left intact unless the YAML data provides a value.)""";
        } allow_cpp_with_no_yaml;
        // Symbol: drake::yaml::LoadYamlOptions::allow_yaml_with_no_cpp
        struct /* allow_yaml_with_no_cpp */ {
          // Source: drake/common/yaml/yaml_io_options.h:19
          const char* doc =
R"""(Allows yaml Maps to have extra key-value pairs that are not Visited by
the Serializable being parsed into. In other words, the Serializable
types provide an incomplete schema for the YAML data. This allows for
parsing only a subset of the YAML data.)""";
        } allow_yaml_with_no_cpp;
        // Symbol: drake::yaml::LoadYamlOptions::retain_map_defaults
        struct /* retain_map_defaults */ {
          // Source: drake/common/yaml/yaml_io_options.h:30
          const char* doc =
R"""(If set to true, when parsing a std::map the Archive will merge the
YAML data into the destination, instead of replacing the std::map
contents entirely. In other words, a visited std::map can have default
values that are left intact unless the YAML data provides a value *for
that specific key*.)""";
        } retain_map_defaults;
      } LoadYamlOptions;
      // Symbol: drake::yaml::LoadYamlString
      struct /* LoadYamlString */ {
        // Source: drake/common/yaml/yaml_io.h:63
        const char* doc =
R"""(Loads data from a YAML-formatted string.

Refer to yaml_serialization "YAML Serialization" for background and
examples.

Parameter ``data``:
    the YAML document as a string.

Parameter ``child_name``:
    (optional) If provided, loads data from given-named child of the
    document's root instead of the root itself.

Parameter ``defaults``:
    (optional) If provided, then the structure being read into will be
    initialized using this value instead of the default constructor,
    and also (unless the ``options`` argument is provided and
    specifies otherwise) any member fields that are not mentioned in
    the YAML will retain their default values.

Parameter ``options``:
    (optional, advanced) If provided, overrides the nominal parsing
    options. Most users should not specify this; the default is
    usually correct.

Returns:
    the loaded user data.

Template parameter ``Serializable``:
    must implement a implementing_serialize "Serialize" function and
    be default constructible.)""";
      } LoadYamlString;
      // Symbol: drake::yaml::SaveJsonFile
      struct /* SaveJsonFile */ {
        // Source: drake/common/yaml/yaml_io.h:132
        const char* doc =
R"""(Saves data as a JSON-formatted file.

Refer to yaml_serialization "YAML Serialization" for background.

Note that there is no matching ``LoadJsonFile`` function, because we
haven't found any specific need for it yet in C++.

Parameter ``data``:
    User data to be serialized.

Returns:
    the JSON data as a string.

Template parameter ``Serializable``:
    must implement a implementing_serialize "Serialize" function.)""";
      } SaveJsonFile;
      // Symbol: drake::yaml::SaveJsonString
      struct /* SaveJsonString */ {
        // Source: drake/common/yaml/yaml_io.h:147
        const char* doc =
R"""(Saves data as a JSON-formatted string.

Refer to yaml_serialization "YAML Serialization" for background.

Note that there is no matching ``LoadJsonString`` function, because we
haven't found any specific need for it yet in C++.

Parameter ``data``:
    User data to be serialized.

Returns:
    the JSON data as a string.

Template parameter ``Serializable``:
    must implement a implementing_serialize "Serialize" function.)""";
      } SaveJsonString;
      // Symbol: drake::yaml::SaveYamlFile
      struct /* SaveYamlFile */ {
        // Source: drake/common/yaml/yaml_io.h:89
        const char* doc =
R"""(Saves data as a YAML-formatted file.

Refer to yaml_serialization "YAML Serialization" for background.

The YAML will consist of a single document with a mapping node at the
root. If a ``child_name`` is **not** provided (the default), then the
serialized data will appear directly within that top-level mapping
node. If a ``child_name`` **is** provided, then the top-level mapping
node will contain only one entry, whose key is ``child_name`` and
value is the serialized ``data``.

Parameter ``filename``:
    Filename to be written to.

Parameter ``data``:
    User data to be serialized.

Parameter ``child_name``:
    (optional) If provided, the YAML document will be ``{child_name: {
    data }}`` rather than just ``{ data }``.

Parameter ``defaults``:
    (optional) If provided, then only data that differs from the given
    defaults will be serialized.

Template parameter ``Serializable``:
    must implement a implementing_serialize "Serialize" function.)""";
      } SaveYamlFile;
      // Symbol: drake::yaml::SaveYamlString
      struct /* SaveYamlString */ {
        // Source: drake/common/yaml/yaml_io.h:114
        const char* doc =
R"""(Saves data as a YAML-formatted string.

Refer to yaml_serialization "YAML Serialization" for background.

The YAML will consist of a single document with a mapping node at the
root. If a ``child_name`` is **not** provided (the default), then the
serialized data will appear directly within that top-level mapping
node. If a ``child_name`` **is** provided, then the top-level mapping
node will contain only one entry, whose key is ``child_name`` and
value is the serialized ``data``.

Parameter ``data``:
    User data to be serialized.

Parameter ``child_name``:
    (optional) If provided, the YAML document will be ``{child_name: {
    data }}`` rather than just ``{ data }``.

Parameter ``defaults``:
    (optional) If provided, then only data that differs from the given
    defaults will be serialized.

Returns:
    the YAML document as a string.

Template parameter ``Serializable``:
    must implement a implementing_serialize "Serialize" function.)""";
      } SaveYamlString;
    } yaml;
  } drake;
} pydrake_doc_common_yaml;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
