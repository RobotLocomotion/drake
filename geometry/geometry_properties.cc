#include "drake/geometry/geometry_properties.h"

#include <sstream>

#include <fmt/format.h>

namespace drake {
namespace geometry {

using std::string;

GeometryProperties::~GeometryProperties() = default;

const GeometryProperties::Group& GeometryProperties::GetPropertiesInGroup(
    const string& group_name) const {
  const auto iter = values_.find(group_name);
  if (iter != values_.end()) {
    return iter->second;
  }
  throw std::logic_error(
      fmt::format("GetPropertiesInGroup(): Can't retrieve properties for a "
                  "group that doesn't exist: '{}'",
                  group_name));
}

std::set<string> GeometryProperties::GetGroupNames() const {
  std::set<string> group_names;
  for (const auto& pair : values_) {
    group_names.insert(pair.first);
  }
  return group_names;
}

void GeometryProperties::AddPropertyAbstract(const string& group_name,
                                             const string& name,
                                             const AbstractValue& value) {
  WritePropertyAbstract(
      group_name, name, value, [&group_name, &name](const Group& group) {
        const auto value_iter = group.find(name);
        if (value_iter != group.end()) {
          throw std::logic_error(
              fmt::format("AddProperty(): Trying to add property ('{}', '{}'); "
                          "a property with that name already exists",
                          group_name, name));
        }
      });
}

void GeometryProperties::UpdatePropertyAbstract(const string& group_name,
                                                const string& name,
                                                const AbstractValue& value) {
  WritePropertyAbstract(
      group_name, name, value,
      [&group_name, &name, &value](const Group& group) {
        const auto value_iter = group.find(name);
        if (value_iter != group.end() &&
            group.at(name)->type_info() != value.type_info()) {
          throw std::logic_error(
              fmt::format("UpdateProperty(): Trying to update property ('{}', "
                          "'{}'); The property already exists and is of "
                          "different type. New type {}, existing type {}",
                          group_name, name, value.GetNiceTypeName(),
                          group.at(name)->GetNiceTypeName()));
        }
      });
}

bool GeometryProperties::HasProperty(const string& group_name,
                                     const string& name) const {
  return GetPropertyAbstractMaybe(group_name, name, false) != nullptr;
}

const AbstractValue& GeometryProperties::GetPropertyAbstract(
    const string& group_name, const string& name) const {
  const AbstractValue* abstract =
      GetPropertyAbstractMaybe(group_name, name, true);
  if (!abstract) {
    throw std::logic_error(fmt::format(
        "GetProperty(): There is no property ('{}', '{}')", group_name, name));
  }
  return *abstract;
}

bool GeometryProperties::RemoveProperty(const string& group_name,
                                        const string& name) {
  const auto iter = values_.find(group_name);
  if (iter == values_.end()) return false;
  Group& group = iter->second;
  const auto value_iter = group.find(name);
  if (value_iter == group.end()) return false;
  group.erase(value_iter);
  return true;
}

void GeometryProperties::WritePropertyAbstract(
    const string& group_name, const string& name, const AbstractValue& value,
    const std::function<void(const Group&)>& throw_if_invalid) {
  auto iter = values_.find(group_name);
  if (iter == values_.end()) {
    auto result = values_.insert({group_name, Group{}});
    DRAKE_DEMAND(result.second);
    iter = result.first;
  }

  Group& group = iter->second;
  throw_if_invalid(group);

  group[name] = value.Clone();
}

const AbstractValue* GeometryProperties::GetPropertyAbstractMaybe(
    const string& group_name, const string& name,
    bool throw_for_bad_group) const {
  const auto iter = values_.find(group_name);
  if (iter == values_.end()) {
    if (throw_for_bad_group) {
      throw std::logic_error(fmt::format(
          "GetProperty(): Trying to read property ('{}', '{}'), but the group "
          "does not exist.",
          group_name, name));
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

std::string GeometryProperties::to_string() const {
  std::stringstream out;
  int i = 0;
  for (const auto& group_pair : values_) {
    const string& group_name = group_pair.first;
    const GeometryProperties::Group& group_properties = group_pair.second;
    out << "[" << group_name << "]";
    for (const auto& property_pair : group_properties) {
      const string& property_name = property_pair.first;
      out << "\n  " << property_name << ": "
          << property_pair.second->GetNiceTypeName();
      // TODO(SeanCurtis-TRI): How do I print the value in an AbstractValue?
    }
    if (i < num_groups() - 1) out << "\n";
    ++i;
  }
  return out.str();
}

}  // namespace geometry
}  // namespace drake
