#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <utility>

#include "drake/common/yaml/yaml_io_options.h"
#include "drake/common/yaml/yaml_read_archive.h"
#include "drake/common/yaml/yaml_write_archive.h"

namespace drake {
namespace yaml {

/** Loads data from a YAML-formatted file.

Refer to @ref yaml_serialization "YAML Serialization" for background and
examples.

@param filename Filename to be read from.
@param child_name (optional) If provided, loads data from given-named child of
  the document's root instead of the root itself.
@param defaults (optional) If provided, then the structure being read into
  will be initialized using this value instead of the default constructor,
  and also (unless the `options` argument is provided and specifies otherwise)
  any member fields that are not mentioned in the YAML will retain their
  default values.
@param options (optional, advanced) If provided, overrides the nominal parsing
  options.  Most users should not specify this; the default is usually correct.

@returns the loaded user data.

@tparam Serializable must implement a @ref implementing_serialize "Serialize"
  function and be default constructible. */
template <typename Serializable>
static Serializable LoadYamlFile(
    const std::string& filename,
    const std::optional<std::string>& child_name = std::nullopt,
    const std::optional<Serializable>& defaults = std::nullopt,
    const std::optional<LoadYamlOptions>& options = std::nullopt);

/** Loads data from a YAML-formatted string.

Refer to @ref yaml_serialization "YAML Serialization" for background and
examples.

@param data the YAML document as a string.
@param child_name (optional) If provided, loads data from given-named child of
  the document's root instead of the root itself.
@param defaults (optional) If provided, then the structure being read into
  will be initialized using this value instead of the default constructor,
  and also (unless the `options` argument is provided and specifies otherwise)
  any member fields that are not mentioned in the YAML will retain their
  default values.
@param options (optional, advanced) If provided, overrides the nominal parsing
  options.  Most users should not specify this; the default is usually correct.

@returns the loaded user data.

@tparam Serializable must implement a @ref implementing_serialize "Serialize"
  function and be default constructible. */
template <typename Serializable>
static Serializable LoadYamlString(
    const std::string& data,
    const std::optional<std::string>& child_name = std::nullopt,
    const std::optional<Serializable>& defaults = std::nullopt,
    const std::optional<LoadYamlOptions>& options = std::nullopt);

/** Saves data as a YAML-formatted file.

Refer to @ref yaml_serialization "YAML Serialization" for background.

The YAML will consist of a single document with a mapping node at the root.
If a `child_name` is **not** provided (the default), then the serialized data
will appear directly within that top-level mapping node.
If a `child_name` **is** provided, then the top-level mapping node will contain
only one entry, whose key is `child_name` and value is the serialized `data`.

@param filename Filename to be written to.
@param data User data to be serialized.
@param child_name (optional) If provided, the YAML document will be
  `{child_name: { data }}` rather than just `{ data }`.
@param defaults (optional) If provided, then only data that differs from
  the given defaults will be serialized.

@tparam Serializable must implement a @ref implementing_serialize "Serialize"
  function. */
template <typename Serializable>
void SaveYamlFile(const std::string& filename, const Serializable& data,
                  const std::optional<std::string>& child_name = std::nullopt,
                  const std::optional<Serializable>& defaults = std::nullopt);

/** Saves data as a YAML-formatted string.

Refer to @ref yaml_serialization "YAML Serialization" for background.

The YAML will consist of a single document with a mapping node at the root.
If a `child_name` is **not** provided (the default), then the serialized data
will appear directly within that top-level mapping node.
If a `child_name` **is** provided, then the top-level mapping node will contain
only one entry, whose key is `child_name` and value is the serialized `data`.

@param data User data to be serialized.
@param child_name (optional) If provided, the YAML document will be
  `{child_name: { data }}` rather than just `{ data }`.
@param defaults (optional) If provided, then only data that differs from
  the given defaults will be serialized.

@returns the YAML document as a string.

@tparam Serializable must implement a @ref implementing_serialize "Serialize"
  function. */
template <typename Serializable>
std::string SaveYamlString(
    const Serializable& data,
    const std::optional<std::string>& child_name = std::nullopt,
    const std::optional<Serializable>& defaults = std::nullopt);

/** Saves data as a JSON-formatted file.

Refer to @ref yaml_serialization "YAML Serialization" for background.

Note that there is no matching `LoadJsonFile` function, because we haven't
found any specific need for it yet in C++.

@param data User data to be serialized.
@returns the JSON data as a string.

@tparam Serializable must implement a @ref implementing_serialize "Serialize"
  function. */
template <typename Serializable>
void SaveJsonFile(const std::string& filename, const Serializable& data);

/** Saves data as a JSON-formatted string.

Refer to @ref yaml_serialization "YAML Serialization" for background.

Note that there is no matching `LoadJsonString` function, because we haven't
found any specific need for it yet in C++.

@param data User data to be serialized.
@returns the JSON data as a string.

@tparam Serializable must implement a @ref implementing_serialize "Serialize"
  function. */
template <typename Serializable>
std::string SaveJsonString(const Serializable& data);

namespace internal {

void WriteFile(std::string_view function_name, const std::string& filename,
               const std::string& data);

template <typename Serializable>
static Serializable LoadNode(internal::Node node,
                             const std::optional<Serializable>& defaults,
                             const std::optional<LoadYamlOptions>& options) {
  // Reify our optional arguments.
  Serializable result = defaults.value_or(Serializable{});
  LoadYamlOptions new_options = options.value_or(LoadYamlOptions{});
  if (defaults.has_value() && !options.has_value()) {
    // Do not overwrite existing values.
    new_options.allow_cpp_with_no_yaml = true;
    new_options.retain_map_defaults = true;
  }
  // Parse and return.
  internal::YamlReadArchive(std::move(node), new_options).Accept(&result);
  return result;
}

}  // namespace internal

// (Implementation of a function declared above.  This cannot be defined
// inline because we need internal::LoadNode to be declared.)
template <typename Serializable>
static Serializable LoadYamlFile(
    const std::string& filename, const std::optional<std::string>& child_name,
    const std::optional<Serializable>& defaults,
    const std::optional<LoadYamlOptions>& options) {
  internal::Node node =
      internal::YamlReadArchive::LoadFileAsNode(filename, child_name);
  return internal::LoadNode(std::move(node), defaults, options);
}

// (Implementation of a function declared above.  This cannot be defined
// inline because we need internal::LoadNode to be declared.)
template <typename Serializable>
static Serializable LoadYamlString(
    const std::string& data, const std::optional<std::string>& child_name,
    const std::optional<Serializable>& defaults,
    const std::optional<LoadYamlOptions>& options) {
  internal::Node node =
      internal::YamlReadArchive::LoadStringAsNode(data, child_name);
  return internal::LoadNode(std::move(node), defaults, options);
}

// (Implementation of a function declared above.  This cannot be defined
// inline because we need SaveYamlString to be declared.)
template <typename Serializable>
void SaveYamlFile(const std::string& filename, const Serializable& data,
                  const std::optional<std::string>& child_name,
                  const std::optional<Serializable>& defaults) {
  internal::WriteFile("SaveYamlFile", filename,
                      SaveYamlString(data, child_name, defaults));
}

// (Implementation of a function declared above.  This could be defined
// inline, but we keep it with the others for consistency.)
template <typename Serializable>
std::string SaveYamlString(const Serializable& data,
                           const std::optional<std::string>& child_name,
                           const std::optional<Serializable>& defaults) {
  internal::YamlWriteArchive archive;
  archive.Accept(data);
  if (defaults.has_value()) {
    internal::YamlWriteArchive defaults_archive;
    defaults_archive.Accept(defaults.value());
    archive.EraseMatchingMaps(defaults_archive);
  }
  return archive.EmitString(child_name.value_or(std::string()));
}

// (Implementation of a function declared above.  This could be defined
// inline, but we keep it with the others for consistency.)
template <typename Serializable>
void SaveJsonFile(const std::string& filename, const Serializable& data) {
  internal::WriteFile("SaveJsonFile", filename, SaveJsonString(data));
}

// (Implementation of a function declared above.  This could be defined
// inline, but we keep it with the others for consistency.)
template <typename Serializable>
std::string SaveJsonString(const Serializable& data) {
  internal::YamlWriteArchive archive;
  archive.Accept(data);
  return archive.ToJson();
}

}  // namespace yaml
}  // namespace drake
