#pragma once

#include <optional>
#include <string>

#include "drake/common/yaml/yaml_write_archive.h"

namespace drake {
namespace yaml {

// TODO(jwnimmer-tri) Add YAML helpers for loading.

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

@tparam Serializable must implement a
  @ref serialize_tips "Serialize" function. */
template <typename Serializable>
void SaveYamlFile(
    const std::string& filename,
    const Serializable& data,
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

@tparam Serializable must implement a
  @ref serialize_tips "Serialize" function. */
template <typename Serializable>
std::string SaveYamlString(
    const Serializable& data,
    const std::optional<std::string>& child_name = std::nullopt,
    const std::optional<Serializable>& defaults = std::nullopt);

namespace internal {

void WriteFile(const std::string& filename, const std::string& data);

}  // namespace internal

// (Implementation of a function declared above.)
template <typename Serializable>
void SaveYamlFile(
    const std::string& filename,
    const Serializable& data,
    const std::optional<std::string>& child_name,
    const std::optional<Serializable>& defaults) {
  internal::WriteFile(filename,
      SaveYamlString(data, child_name, defaults));
}

// (Implementation of a function declared above.)
template <typename Serializable>
std::string SaveYamlString(
    const Serializable& data,
    const std::optional<std::string>& child_name,
    const std::optional<Serializable>& defaults) {
  YamlWriteArchive archive;
  archive.Accept(data);
  if (defaults.has_value()) {
    YamlWriteArchive defaults_archive;
    defaults_archive.Accept(defaults.value());
    archive.EraseMatchingMaps(defaults_archive);
  }
  return archive.EmitString(child_name.value_or(std::string()));
}

}  // namespace yaml
}  // namespace drake
