#pragma once

#include <optional>
#include <string>

#include "drake/common/yaml/yaml_read_archive.h"
#include "drake/common/yaml/yaml_write_archive.h"

namespace drake {
namespace yaml {

// TODO(jwnimmer-tri) Create some directory-overview documentation about our
// YAML file format, using the example content immediately below.

/// Sample data:
/// @code{yaml}
/// doc:
///   foo: 1.0
///   bar: [2.0, 3.0]
/// @endcode
///
/// Sample code:
/// @code{cpp}
/// struct MyData {
///   double foo{NAN};
///   std::vector<double> bar;
///
///   template <typename Archive>
///   void Serialize(Archive* a) {
///     a->Visit(DRAKE_NVP(foo));
///     a->Visit(DRAKE_NVP(bar));
///   }
/// };
///
/// MyData LoadData(const std::string& filename) {
///   return YamlReadArchive::LoadFile<MyData>(filename);
/// }
/// @endcode
///
/// Structures can be arbitrarily nested, as long as each `struct` has a
/// `Serialize` method.  Many common built-in types (int, double, std::string,
/// std::vector, std::array, std::optional, std::variant, Eigen::Matrix) may
/// also be used.
///
/// YAML's "merge keys" (https://yaml.org/type/merge.html) are supported.
///
/// When reading into a std::variant<>, we match its YAML tag to the shortened
/// C++ class name of the variant selection.  For example, to read into this
/// sample struct:
///
/// @code
/// struct Foo {
///   std::string data;
/// };
/// struct Bar {
///   std::variant<std::string, double, Foo> value;
/// };
/// @endcode
///
/// Some valid YAML examples are:
///
/// @code
/// # For the first type declared in the variant<>, the tag is optional.
/// bar:
///   value: hello
///
/// # YAML has built-in tags for string, float, int.
/// bar2:
///   value: !!str hello
///
/// # For any other type within the variant<>, the tag is required.
/// bar3:
///   value: !!float 1.0
///
/// # User-defined types use a single exclamation point.
/// bar4:
///   value: !Foo
///     data: hello
/// @endcode

/// Parses the contents of the given YAML filename into a Serializable struct.
///
/// If an optional child_name is provided, then parses the given-named child
/// of the root node instead of the root node itself.
///
/// If an optional Serializable defaults is provided, then those defaults will
/// remain unchanged for any members that are not mentioned in the YAML [1].
///
/// If an optional Options is provided, then those settings will be using
/// during parsing.
///
/// [1] N.B. The std::map collections merge the contents of the node into the
/// defaults, keeping anything in the default that is unchanged.  Collections
/// like std::vector are entirely reset, even if they already had some values
/// values already in place.
template <typename Serializable>
static Serializable LoadFile(
    const std::string& filename,
    const std::optional<std::string>& child_name = std::nullopt,
    const std::optional<Serializable>& defaults = std::nullopt,
    const std::optional<YamlReadArchive::Options>& options = std::nullopt) {
  YAML::Node node = YamlReadArchive::LoadFileAsNode(filename, child_name);

  // ... XXX ...
  Serializable result = defaults.value_or(Serializable{});
  YamlReadArchive::Options new_options = options.value_or(
      YamlReadArchive::Options{});
  if (defaults.has_value() && !options.has_value()) {
    // Do not overwrite existing values.
    new_options.allow_cpp_with_no_yaml = true;
    new_options.retain_map_defaults = true;
  }

  // Parse and return.
  YamlReadArchive(node, options).Accept(&result);
  return result;
}

/// TODO(jwnimmer-tri) Write me.
template <typename Serializable>
std::string SaveString(const Serializable& data) {
  YamlWriteArchive archive;
  archive.Accept(data);
  return archive.EmitString();
}

}  // namespace yaml
}  // namespace drake
