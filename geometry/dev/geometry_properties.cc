#include "drake/geometry/dev/geometry_properties.h"

namespace drake {
namespace geometry {
namespace dev {

// NOTE: Because we actually use string as the lookup, we don't want to keep
// creating a new string every time the default group name is accessed.
// NOLINTNEXTLINE(runtime/string)
const std::string GeometryProperties::kDefaultGroup("__^default^__");

std::ostream& operator<<(std::ostream& out, const GeometryProperties& props) {
  const size_t count = props.values_.size();
  size_t i = 0;
  for (const auto& group_pair : props.values_) {
    const std::string& group_name = group_pair.first;
    const GeometryProperties::PropertyGroup& group_properties =
        group_pair.second;
    out << "[" << group_name << "]";
    for (const auto& property_pair : group_properties) {
      const std::string& property_name = property_pair.first;
      out << "\n  " << property_name;
      // TODO(SeanCurtis-TRI): How do I print the value in an AbstractValue?
    }
    if (i < count - 1) out << "\n";
    ++i;
  }
  return out;
}

}  // namespace dev
}  // namespace geometry
}  // namespace drake
