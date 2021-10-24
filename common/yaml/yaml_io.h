#pragma once

#include <optional>
#include <string>
#include <utility>

#include "drake/common/yaml/yaml_read_archive.h"
#include "drake/common/yaml/yaml_write_archive.h"

namespace drake {
namespace yaml {

// XXX Cite the directory overview

/// Parses the contents of the given YAML filename into a Serializable struct.
///
/// If an optional child_name is provided, then parses the given-named child
/// of the root node instead of the root node itself.
///
/// If an optional Serializable defaults is provided, then those defaults will
/// remain unchanged for any members that are not mentioned in the YAML [1].
///
/// If an optional Options is provided, then those settings will be used
/// during parsing.
///
/// [1] N.B. The std::map collections merge the contents of the node into the
/// defaults, keeping anything in the default that is unchanged.  Collections
/// like std::vector are entirely reset, even if they already had some values
/// values already in place.
template <typename Serializable>
static Serializable LoadYamlFile(
    const std::string& filename,
    const std::optional<std::string>& child_name = std::nullopt,
    const std::optional<Serializable>& defaults = std::nullopt,
    const std::optional<YamlReadArchive::Options>& options = std::nullopt);

/// Like LoadYamlFile, but loads from a YAML string instead of a file.
template <typename Serializable>
static Serializable LoadYamlString(
    const std::string& data,
    const std::optional<std::string>& child_name = std::nullopt,
    const std::optional<Serializable>& defaults = std::nullopt,
    const std::optional<YamlReadArchive::Options>& options = std::nullopt);

/// Saves data from the Serializable struct into a YAML file.
template <typename Serializable>
void SaveYamlFile(
    const std::string& filename,
    const Serializable& data);

/// Saves data from the Serializable struct into a YAML file.
/// The data will appear under the `child_name` key in a top-level mapping.
template <typename Serializable>
void SaveYamlFile(
    const std::string& filename,
    const std::string& child_name,
    const Serializable& data);

/// Saves data from the Serializable struct into a YAML string.
template <typename Serializable>
std::string SaveYamlString(
    const Serializable& data);

/// Saves data from the Serializable struct into a YAML string.
/// The data will appear under the `child_name` key in a top-level mapping.
template <typename Serializable>
std::string SaveYamlString(
    const std::string& child_name,
    const Serializable& data);

// ============================= Implementations =============================

namespace internal {

template <typename Serializable>
static Serializable LoadNode(
    internal::Node node,
    const std::optional<Serializable>& defaults,
    const std::optional<YamlReadArchive::Options>& options) {
  // Reify our optional arguments.
  Serializable result = defaults.value_or(Serializable{});
  YamlReadArchive::Options new_options = options.value_or(
      YamlReadArchive::Options{});
  if (defaults.has_value() && !options.has_value()) {
    // Do not overwrite existing values.
    new_options.allow_cpp_with_no_yaml = true;
    new_options.retain_map_defaults = true;
  }
  // Parse and return.
  YamlReadArchive(std::move(node), new_options).Accept(&result);
  return result;
}

}  // namespace internal

template <typename Serializable>
static Serializable LoadYamlFile(
    const std::string& filename,
    const std::optional<std::string>& child_name,
    const std::optional<Serializable>& defaults,
    const std::optional<YamlReadArchive::Options>& options) {
  internal::Node node = YamlReadArchive::LoadFileAsNode(filename, child_name);
  return internal::LoadNode(std::move(node), defaults, options);
}

template <typename Serializable>
static Serializable LoadYamlString(
    const std::string& data,
    const std::optional<std::string>& child_name,
    const std::optional<Serializable>& defaults,
    const std::optional<YamlReadArchive::Options>& options) {
  internal::Node node = YamlReadArchive::LoadStringAsNode(data, child_name);
  return internal::LoadNode(std::move(node), defaults, options);
}

template <typename Serializable>
void SaveYamlFile(
    const std::string& filename,
    const Serializable& data) {
  SaveYamlFile<Serializable>(filename, std::string{}, data);
}

template <typename Serializable>
void SaveYamlFile(
    const std::string& filename,
    const std::string& child_name,
    const Serializable& data) {
  YamlWriteArchive::WriteFile(filename, SaveYamlString(child_name, data));
}

template <typename Serializable>
std::string SaveYamlString(
    const Serializable& data) {
  return SaveYamlString<Serializable>(std::string{}, data);
}

template <typename Serializable>
std::string SaveYamlString(
    const std::string& child_name,
    const Serializable& data) {
  YamlWriteArchive archive;
  archive.Accept(data);
  return archive.EmitString(child_name);
}

}  // namespace yaml
}  // namespace drake
