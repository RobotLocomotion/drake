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

void GeometryProperties::AddPropertyAbstract(
    const std::string& group_name, const std::string& name,
    const AbstractValue& value) {
  auto iter = values_.find(group_name);
  if (iter == values_.end()) {
    auto result = values_.insert({group_name, Group{}});
    DRAKE_DEMAND(result.second);
    iter = result.first;
  }

  Group& group = iter->second;
  auto value_iter = group.find(name);
  if (value_iter == group.end()) {
    group[name] = value.Clone();
    return;
  }
  throw std::logic_error(fmt::format(
      "AddProperty(): Trying to add property '{}' to group '{}'; "
      "a property with that name already exists",
      name, group_name));
}

bool GeometryProperties::HasProperty(const std::string& group_name,
                                     const std::string& name) const {
  return GetPropertyAbstractMaybe(group_name, name, false) != nullptr;
}

const AbstractValue& GeometryProperties::GetPropertyAbstract(
    const std::string& group_name, const std::string& name) const {
  const AbstractValue* abstract = GetPropertyAbstractMaybe(
      group_name, name, true);
  if (!abstract) {
    throw std::logic_error(
        fmt::format("GetProperty(): There is no property '{}' in group '{}'.",
                    name, group_name));
  }
  return *abstract;
}

const AbstractValue* GeometryProperties::GetPropertyAbstractMaybe(
    const std::string& group_name, const std::string& name,
    bool throw_for_bad_group) const {
  const auto iter = values_.find(group_name);
  if (iter == values_.end()) {
    if (throw_for_bad_group) {
      throw std::logic_error(
          fmt::format(
              "GetProperty(): Trying to read property '{}' from group "
              "'{}'. But the group does not exist.",
              name, group_name));
    } else {
      return nullptr;
    }
  }
  const Group& group = iter->second;
  const auto value_iter = group.find(name);
  if (value_iter != group.end()) {
    return value_iter->second.get();
  } else {
    return nullptr;
  }
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
