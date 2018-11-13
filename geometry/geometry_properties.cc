#include "drake/geometry/geometry_properties.h"

namespace drake {
namespace geometry {

const GeometryProperties::Group& GeometryProperties::GetPropertiesInGroup(
    const std::string& group_name) const {
  const auto iter = values_.find(group_name);
  if (iter != values_.end()) {
    return iter->second;
  }
  throw std::logic_error(
      fmt::format("GetPropertiesInGroup(): Can't retrieve properties for a "
                  "group that doesn't exist: '{}'",
                  group_name));
}

std::set<std::string> GeometryProperties::GetGroupNames() const {
  std::set<std::string> group_names;
  for (const auto& pair : values_) {
    group_names.insert(pair.first);
  }
  return group_names;
}

bool GeometryProperties::HasProperty(const std::string& group_name,
                                     const std::string& name) const {
  const auto iter = values_.find(group_name);
  if (iter != values_.end()) {
    const Group& group = iter->second;
    return group.count(name) > 0;
  }
  return false;
}

std::ostream& operator<<(std::ostream& out, const GeometryProperties& props) {
  int i = 0;
  for (const auto& group_pair : props.values_) {
    const std::string& group_name = group_pair.first;
    const GeometryProperties::Group& group_properties =
        group_pair.second;
    out << "[" << group_name << "]";
    for (const auto& property_pair : group_properties) {
      const std::string& property_name = property_pair.first;
      out << "\n  " << property_name << ": "
          << property_pair.second->GetNiceTypeName();
      // TODO(SeanCurtis-TRI): How do I print the value in an AbstractValue?
    }
    if (i < props.num_groups() - 1) out << "\n";
    ++i;
  }
  return out;
}

}  // namespace geometry
}  // namespace drake
