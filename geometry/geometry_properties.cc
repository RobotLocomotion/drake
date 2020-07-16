#include "drake/geometry/geometry_properties.h"

#include <fmt/format.h>

namespace drake {
namespace geometry {

using std::string;

GeometryProperties& GeometryProperties::AddAbstract(
    const string& name, const AbstractValue& value) {
  const auto iter = values_.find(name);
  if (iter != values_.end()) {
    throw std::logic_error(
        fmt::format("Add(): Trying to add property '{}'; a property with "
                    "that name already exists",
                    name));
  }
  values_[name] = value.Clone();
  return *this;
}

GeometryProperties& GeometryProperties::UpdateAbstract(
    const string& name, const AbstractValue& value) {
  const auto iter = values_.find(name);
  if (iter != values_.end() &&
      values_.at(name)->type_info() != value.type_info()) {
    throw std::logic_error(fmt::format(
        "Update(): Trying to update property '{}'; the property already exists "
        "and is of different type. New type {}, existing type {}",
        name, value.GetNiceTypeName(), values_.at(name)->GetNiceTypeName()));
  }

  values_[name] = value.Clone();
  return *this;
}

bool GeometryProperties::HasProperty(const string& name) const {
  return GetAbstractMaybe(name) != nullptr;
}

const AbstractValue& GeometryProperties::GetAbstract(const string& name) const {
  const AbstractValue* abstract = GetAbstractMaybe(name);
  if (!abstract) {
    throw std::logic_error(
        fmt::format("Get(): There is no property '{}'.", name));
  }
  return *abstract;
}

bool GeometryProperties::Remove(const string& name) {
  const auto iter = values_.find(name);
  if (iter == values_.end()) return false;
  values_.erase(iter);
  return true;
}

bool GeometryProperties::HasGroup(const string& group_name) const {
  return GetGroupNames().count(group_name) > 0;
}

int GeometryProperties::num_groups() const {
  return static_cast<int>(GetGroupNames().size());
}

GeometryProperties::Group GeometryProperties::GetPropertiesInGroup(
    const string& group_name) const {
  Group group;
  for (const auto& pair : values_) {
    const string& name = pair.first;
    const string pair_group = extract_group(name);
    if (pair_group == group_name) {
      group[extract_property(name)] = pair.second->Clone();
    }
  }

  return group;
}

std::set<string> GeometryProperties::GetGroupNames() const {
  std::set<string> group_names;
  group_names.insert("");  // Always include the "default" group.
  for (const auto& pair : values_) {
    const string& name = pair.first;
    group_names.insert(extract_group(name));
  }
  return group_names;
}

string GeometryProperties::extract_group(const string& name) {
  size_t index = name.find_first_of("/");
  if (index == string::npos) {
    return "";
  }
  return name.substr(0, index);
}

string GeometryProperties::extract_property(const string& name) {
  size_t index = name.find_first_of("/");
  if (index == string::npos) {
    return name;
  }
  return name.substr(index + 1, string::npos);
}

const AbstractValue* GeometryProperties::GetAbstractMaybe(
    const string& name) const {
  const auto iter = values_.find(name);
  if (iter == values_.end()) {
    return nullptr;
  }
  return iter->second.get();
}

std::ostream& operator<<(std::ostream& out, const GeometryProperties& props) {
  out << NiceTypeName::Get(props);
  for (const auto& property_pair : props.values_) {
    const string& property_name = property_pair.first;
    out << "\n  " << property_name << ": "
        << property_pair.second->GetNiceTypeName();
    // TODO(SeanCurtis-TRI): How do I print the value in an AbstractValue?
  }
  return out;
}

}  // namespace geometry
}  // namespace drake
