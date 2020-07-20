#include "drake/geometry/geometry_properties.h"

#include <fmt/format.h>

namespace drake {
namespace geometry {

using std::string;

std::ostream& operator<<(std::ostream& out, const PropertyName& property) {
  out << property.group() << "/" << property.property();
  return out;
}

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
  AddAbstract({group_name, name}, value);
}

GeometryProperties& GeometryProperties::AddAbstract(
    const PropertyName& property, const AbstractValue& value) {
  WriteAbstract(
      property, value, [&property](const Group& group) {
        const auto value_iter = group.find(property.property());
        if (value_iter != group.end()) {
          throw std::logic_error(
              fmt::format("Add(): Trying to add property {}; a property with "
                          "that name already exists",
                          property));
        }
      });
  return *this;
}

void GeometryProperties::UpdatePropertyAbstract(const string& group_name,
                                                const string& name,
                                                const AbstractValue& value) {
  UpdateAbstract({group_name, name}, value);
}

GeometryProperties& GeometryProperties::UpdateAbstract(
    const PropertyName& property, const AbstractValue& value) {
  WriteAbstract(
      property, value, [&property, &value](const Group& group) {
        const std::string& name = property.property();
        const auto value_iter = group.find(name);
        if (value_iter != group.end() &&
            group.at(name)->type_info() != value.type_info()) {
          throw std::logic_error(
              fmt::format("Update(): Trying to update property {}; The "
                          "property already exists and is of different type. "
                          "New type {}, existing type {}",
                          property, value.GetNiceTypeName(),
                          group.at(name)->GetNiceTypeName()));
        }
      });
  return *this;
}

bool GeometryProperties::HasProperty(const string& group_name,
                                     const string& name) const {
  return HasProperty({group_name, name});
}

bool GeometryProperties::HasProperty(const PropertyName& property) const {
  return GetAbstractMaybe(property, false) != nullptr;
}

const AbstractValue& GeometryProperties::GetPropertyAbstract(
    const string& group_name, const string& name) const {
  return GetAbstract({group_name, name});
}

const AbstractValue& GeometryProperties::GetAbstract(
    const PropertyName& property) const {
  const AbstractValue* abstract = GetAbstractMaybe(property, true);
  if (!abstract) {
    throw std::logic_error(
        fmt::format("Get(): There is no property {}", property));
  }
  return *abstract;
}

bool GeometryProperties::RemoveProperty(const string& group_name,
                                        const string& name) {
  return Remove({group_name, name});
}

bool GeometryProperties::Remove(const PropertyName& property) {
  const auto iter = values_.find(property.group());
  if (iter == values_.end()) return false;
  Group& group = iter->second;
  const auto value_iter = group.find(property.property());
  if (value_iter == group.end()) return false;
  group.erase(value_iter);
  return true;
}

void GeometryProperties::WriteAbstract(
    const PropertyName& property, const AbstractValue& value,
    const std::function<void(const Group&)>& throw_if_invalid) {
  auto iter = values_.find(property.group());
  if (iter == values_.end()) {
    auto result = values_.insert({property.group(), Group{}});
    DRAKE_DEMAND(result.second);
    iter = result.first;
  }

  Group& group = iter->second;
  throw_if_invalid(group);

  group[property.property()] = value.Clone();
}

const AbstractValue* GeometryProperties::GetAbstractMaybe(
    const PropertyName& property, bool throw_for_bad_group) const {
  const auto iter = values_.find(property.group());
  if (iter == values_.end()) {
    if (throw_for_bad_group) {
      throw std::logic_error(fmt::format(
          "Get(): Trying to read property {}, but the group does not exist.",
          property));
    } else {
      return nullptr;
    }
  }
  const Group& group = iter->second;
  const auto value_iter = group.find(property.property());
  if (value_iter != group.end()) {
    return value_iter->second.get();
  } else {
    return nullptr;
  }
}

std::ostream& operator<<(std::ostream& out, const GeometryProperties& props) {
  int i = 0;
  for (const auto& group_pair : props.values_) {
    const string& group_name = group_pair.first;
    const GeometryProperties::Group& group_properties = group_pair.second;
    out << "[" << group_name << "]";
    for (const auto& property_pair : group_properties) {
      const string& property_name = property_pair.first;
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
