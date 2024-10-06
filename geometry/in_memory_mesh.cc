#include "drake/geometry/in_memory_mesh.h"

#include <algorithm>
#include <vector>

#include <fmt/format.h>
#include <fmt/ranges.h>

#include "drake/common/drake_assert.h"

// Drake currently supports a wide range of fmt versions (8..11), which vary
// heavily in terms of how they format maps (i.e., range-of-pairs) and variant.
// When formatting the supported_files, we can't use fmt's built-in std::map
// formatter because as of fmt 11 it forces the "debug presentation format"
// which when combined with our DRAKE_FORMATTER_AS on FileSource ends up
// re-quoting the formatted FileSource as if it were a string. Instead, we'll
// use a helper struct that more directly controls the format of a map entry.
// Once we drop Jammy and fmt8, we can clean this up even further by removing
// DRAKE_FORMATTER_AS on FileSource since variant<> will be natively formattable
// in that version.
namespace drake {
namespace {
struct FormattableSupportingFileMapEntry {
  std::string to_string() const {
    DRAKE_DEMAND(key != nullptr && value != nullptr);
    return fmt::format(
#if FMT_VERSION >= 90000
        "{:?}: {}",  // Use '?' specifier to format the key as a string literal.
#else
        // TODO(jwnimmer-tri) On Ubuntu Jammy we use fmt8 which does not offer
        // the capability to represent strings as literals, so we'll just toss
        // quotes around and call it a day. Most filenames won't have quotes in
        // them anyway.
        "\"{}\": {}",
#endif
        *key, *value);
  }
  const std::string* key{};
  const FileSource* value{};
};
}  // namespace
}  // namespace drake

DRAKE_FORMATTER_AS(, drake, FormattableSupportingFileMapEntry, x, x.to_string())

namespace drake {
namespace geometry {

std::string InMemoryMesh::to_string() const {
  std::vector<FormattableSupportingFileMapEntry> supporting_encoded;
  std::transform(supporting_files.cbegin(), supporting_files.cend(),
                 std::back_inserter(supporting_encoded),
                 [](const auto& key_value) {
                   return FormattableSupportingFileMapEntry{&key_value.first,
                                                            &key_value.second};
                 });
  return fmt::format("InMemoryMesh(mesh_file={}{})", mesh_file,
                     supporting_files.empty()
                         ? std::string{}
                         : fmt::format(", supporting_files={{{}}}",
                                       fmt::join(supporting_encoded, ", ")));
}
}  // namespace geometry
}  // namespace drake
