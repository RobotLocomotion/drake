#pragma once

#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "fmt/ostream.h"

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/value.h"

namespace drake {
namespace geometry {
namespace dev {

/** The base class for defining geometry properties. Ultimately, this will be
 sub-classed into domain-specific properties (e.g., proximity queries, sensor
 simulation, external visualization, etc.) This provides the basic underlying
 functionality of all the sub-classes but is *not* intended to be used
 directly.

 Generically, a property set can contain multiple property *groups*. Each group
 has a unique name and contains a number of key-value pairs (the properties).
 All property values must either be cloneable or copy-constructible, otherwise
 the type is unconstrained.

 Ultimately, geometry properties are defined during scene initialization and
 consumed by some downstream entity. Each downstream entity that consumes
 geometry properties should document what it expects to find in the geometry
 properties and what kind of default values it provides. It becomes the
 responsibility of the initializer to make sure that the geometry properties
 are provided in the correct form. This includes, key-value pairs placed into
 a *correctly*-spelled group, property keys being likewise correctly spelled,
 and values of the right type. Correct spelling includes correct case.

 To build a set of properties to apply to a geometry:

   1. Create a named group. Properties can only be assigned to a group that has
      already been added. There is a *default* group included in every
      GeometryProperty instance; it can be accessed with the name returned by
      the default_group_name() method.
   2. Add a *new* property to the group via AddProperty(). This is used when
      there it is known that no other property in the specified group has the
      name of the new property.
   3. Set a property (possibly overwriting a previously existing value) via the
      SetProperty() method.

 To read properties from a property set:

   1. Optionally, test to see if the desired property group exists in the
      property set via has_group(). Property access generally requires a group
      name and referencing a group that is not in the set may cause an exception
      to be thrown (depending on the API being exercised; see below).
   2. Optionally, test to see if a group has a property via the HasProperty()
      method. This isn't strictly required if the `OrDefault` interface is used.
   3. Acquire a property value via the GetProperty() or GetPropertyOrDefault()
      methods. The first form fails if the property (or group) doesn't exist.
      The second form takes an alternative default value which gets returned in
      the case where the property or group is missing.
      NOTE: Getting a property requires a compile-time declaration of the *type*
      of value being read. If the stored value is of a different type, an
      exception will be thrown.

 <h2>Common workflows</h2>

 The following examples outline a number of ways to create and consume geometry
 properties.  By design, %GeometryProperties cannot be constructed,
 copied, or moved directly. Only derived classes can do so. So, for these
 examples we've introduced the artificial `DerivedProperties` class which is a
 simple sub-class of GeometryProperties.

 The string-based structure of %GeometryProperties provides a great deal of
 flexibility at the cost to spelling sensitivity. It would be easy to create
 typos that would then "hide" properties in some location a consumer wouldn't
 look. In these examples, we avoid using string literals as group or property
 names (at least in the cases where the same name is used multiple times) to
 help avoid the possibility of typo-induced errors. That is not required and
 certainly not the only way to avoid such bugs.

 <h3>Creating properties</h3>

 <h4>Creating properties for a single group</h4>

 This is a simple example in which a single group is added with properties
 of various types.

 ```
 const std::string group_name("my_group");
 DerivedProperties properties;
 properties.AddGroup(group_name);
 properties.AddProperty(group_name, "int value", 7);
 properties.AddProperty(group_name, "double value", 7.);
 properties.AddProperty(group_name, "string value", "7");
 ```

 <h4>Creating properties in the default group</h4>

 Similar to the previous examples, the properties are added to the default
 group. In this case, "common" property names are avoided as they can more
 easily lead to name collisions in the common group.

 ```
 DerivedProperties properties;
 properties.AddProperty(properties.default_group_name(), "PRE int value", 7);
 properties.AddProperty(properties.default_group_name(), "PRE double value",
                        7.);
 properties.AddProperty(properties.default_group_name(), "PRE string value",
                        "7");
 ```

 <h4>Aggregate properties in a struct</h4>

 In some cases, there are a set of properties that will *always* be required
 or always be read together. In these cases, it makes sense to aggregate them
 into a struct and store that as a single value. This reduces the number of
 lookups required.

 It's worth noting, that if the data value is a struct, calls to
 GetPropertyOrDefault() still operate as an "all-or-nothing" basis.
 If the property *struct* exists, it will be returned, if it's missing the
 default struct will be returned. There is no concept of a "partial" struct
 in which some undefined values in the struct will be replaced with their
 corresponding values in the default struct.

 ```
 struct MyData {
    int i{};
    double d{};
    std::string s;
 };

 DerivedProperties properties;
 const std::string group_name("my_group");
 properties.AddGroup(group_name);
 MyData data{7, 7., "7"};
 properties.AddProperty(group_name, "data value", data);
 ```

 <h3>Reading properties</h3>

 <h4>Targeted look up for *required* properties</h4>

 In the case, the consumer of the properties is looking for one or more specific
 properties. It will ignore any other properties. More particularly, if those
 properties are missing, it is considered a runtime error and an exception is
 thrown. The consumer relies on the %GeometryProperties to throw that exception.
 NOTE: in this case, it is necessary to specify the return type template value
 (`Eigen::Vector3d`) in the call.

 ```
 const DerivedProperties& properties = FunctionThatGivesProperties();
 // Looking for a Vector3d of rgb colors named "rgb".
 const Eigen::Vector3d rgb =
     properties.GetProperty<Eigen::Vector3d>("MyGroup", "rgb");
 ```

 <h4>Targeted look up with default property values</h4>

 As with the previous case, the consumer is looking for one more more specific
 properties. However, in this case, the consumer provides a default value to
 use in case the target property is not defined. In this invocation, the
 template parameter need not be explicitly declared -- the inferred return type
 will be the same as the default value.

 ```
 const DerivedProperties& properties = FunctionThatGivesProperties();
 // Looking for a Vector3d of rgb colors named "rgb".
 const Eigen::Vector3d default_color{0.9, 0.9, 0.9};
 const Eigen::Vector3d rgb =
     properties.GetPropertyOrDefault("MyGroup", "rgb", default_color);
 ```

 <h4>Iterating through provided properties</h4>

 Another alternative is to iterate through the properties that *have* been
 provided. This might be done for several reasons, e.g.:
   -# the consumer wants to validate the set properties, giving the user
      feedback if an unsupported property has been provided, and/or
   -# the consumer has a default value for all properties pre-defined and merely
      wants to update those values that have been explicitly called out -- this
      allows the creator of the properties to provide sparse specifications,
      only noting those values that deviate from the consumer's default values.

 ```
 const DerivedProperties& properties = FunctionThatGivesProperties();
 for (const auto& pair : properties.GetGroupProperties("MyGroup") {
   const std::string& name = pair.first;
   if (name == "rgb") {
     // Throws an exception if the named parameter is of the wrong type.
     const Eigen::Vector3d& rgb =
         pair.second->GetValueOrThrow<Eigen::Vector3d>();
   }
 }
 ```
 */
class GeometryProperties {
 public:
  using PropertyGroup =
      std::unordered_map<std::string, copyable_unique_ptr<AbstractValue>>;

  /** Adds a *new* group to the property set (if it doesn't already exist). If
   the group already exists, there is no change to the property set.  */
  void AddGroup(const std::string& group_name) {
    if (values_.count(group_name) == 0) {
      values_.emplace(std::make_pair(group_name, PropertyGroup{}));
    }
  }

  /** Reports if the given named group is part of this property set.  */
  bool has_group(const std::string& group_name) const {
    return values_.count(group_name) == 1;
  }

  /** Reports the number of property groups in this set.  */
  int num_groups() const { return static_cast<int>(values_.size()); }

  /** Retrieves the indicated property group.
   @throws std::logic_error if there is no group with the given name.  */
  const PropertyGroup& get_group(const std::string& group_name) const {
    auto iter = values_.find(group_name);
    if (iter != values_.end()) {
      return iter->second;
    }

    throw std::logic_error(
        fmt::format("There is no property group with the name {}", group_name));
  }

  /** Returns all of the defined group names. */
  std::set<std::string> GroupNames() const {
    std::set<std::string> group_names;
    for (const auto& pair : values_) {
      group_names.insert(pair.first);
    }
    return group_names;
  }

  /** Adds a property with the given `name` and `value` to the the named
   property group. This will *not* overwrite the previous property if it already
   exists.
   @returns true if the property value is added.
   @throws  std::logic_error if `group` is not a valid property group.
   @tparam ValueType the type of data to store with the attribute -- must be
                     copy constructible or cloneable (see Value).  */
  template <typename ValueType>
  bool AddProperty(const std::string& group_name, const std::string& name,
                   const ValueType& value) {
    auto iter = values_.find(group_name);
    if (iter != values_.end()) {
      PropertyGroup& group = iter->second;
      auto value_iter = group.find(name);
      if (value_iter == group.end()) {
        group[name] = std::make_unique<Value<ValueType>>(value);
        return true;
      }
      return false;
    }

    throw std::logic_error(fmt::format(
        "Trying to add property {} to group {}. But the group does not exist.",
        name, group_name));
  }

  /** Sets a named property in the given group. If the property didn't already
   exist, it will be added. If the property already existed, it will be
   replaced, even if the new value is of a different type than the previous
   value.
   @throws  std::logic_error if `group` is not a valid property group.
   @tparam ValueType the type of data to store with the attribute -- must be
                     copy constructible or cloneable (see Value).  */
  template <typename ValueType>
  void SetProperty(const std::string& group_name, const std::string& name,
                   const ValueType& value) {
    auto iter = values_.find(group_name);
    if (iter != values_.end()) {
      PropertyGroup& group = iter->second;
      group[name] = std::make_unique<Value<ValueType>>(value);
    } else {
      throw std::logic_error(
          fmt::format("Trying to add property {} to group {}. But the group "
                      "does not exist.",
                      name, group_name));
    }
  }

#ifndef DRAKE_DOXYGEN_CXX
  // Note: these two overloads of the property write methods exist to enable
  // calls like `properties.AddProperty("group", "property", "string literal");
  // Template matching would deduce that the `ValueType` in this case is a const
  // char* (which is copyable). By explicitly declaring this API, we can
  // implicitly convert the string literals to copyable std::strings. We omit
  // these from the doxygen because they provide no value there.
  bool AddProperty(const std::string& group_name, const std::string& name,
                   const char* value) {
    return AddProperty<std::string>(group_name, name, value);
  }

  void SetProperty(const std::string& group_name, const std::string& name,
                   const char* value) {
    SetProperty<std::string>(group_name, name, value);
  }
#endif

  /** Reports if the property exists in the group. This is false if either
   property name or group name does not exist.   */
  bool HasProperty(const std::string& group_name,
                   const std::string& name) const {
    const auto iter = values_.find(group_name);
    if (iter != values_.end()) {
      const PropertyGroup& group = iter->second;
      const auto value_iter = group.find(name);
      return value_iter != group.end();
    }
    return false;
  }

  /** Retrieves the typed value from the property set.
   @throws std::logic_error if 1. the group name is invalid,
                               2. the property name is invalid, or
                               3. the property type is not that specified.  */
  template <typename ValueType>
  const ValueType& GetProperty(const std::string& group_name,
                               const std::string& name) const {
    const auto iter = values_.find(group_name);
    if (iter != values_.end()) {
      const PropertyGroup& group = iter->second;
      const auto value_iter = group.find(name);
      if (value_iter != group.end()) {
        return value_iter->second->get_value<ValueType>();
      }
      throw std::logic_error(fmt::format("There is no property {} in group {}.",
                                         name, group_name));
    }

    throw std::logic_error(
        fmt::format("Trying to read property {} from group {}. But the group "
                    "does not exist.",
                    name, group_name));
  }

  /** Retrieves the typed value from the property set (if it exists), otherwise
   returns the given default value.  */
  template <typename ValueType>
  const ValueType& GetPropertyOrDefault(const std::string& group_name,
                                        const std::string& name,
                                        const ValueType& default_value) const {
    const auto iter = values_.find(group_name);
    if (iter != values_.end()) {
      const PropertyGroup& group = iter->second;
      const auto value_iter = group.find(name);
      if (value_iter != group.end()) {
        return value_iter->second->get_value<ValueType>();
      }
    }
    return default_value;
  }

  /** Retrieves the set of defined properties for the indicated `group`.
   @throws std::logic_error if the name does not refer to a valid group.   */
  const PropertyGroup& GetGroupProperties(const std::string& name) const {
    const auto iter = values_.find(name);
    if (iter != values_.end()) {
      return iter->second;
    } else {
      throw std::logic_error(fmt::format(
          "Can't retrieve properties for a group that doesn't exist: {}",
          name));
    }
  }

  /** Reports the number of properties in the named group.
   @throws std::logic_error if the group does not exist.  */
  int NumProperties(const std::string& group_name) const {
    auto iter = values_.find(group_name);
    if (iter != values_.end()) {
      const PropertyGroup& group = iter->second;
      return static_cast<int>(group.size());
    }

    throw std::logic_error(fmt::format(
        "Cannot report number of properties for a non-existant group: {}",
        group_name));
  }

  /** Reports the *total* number of properties in this set. */
  int NumProperties() const {
    size_t count = 0;
    for (const auto& pair : values_) {
      count += pair.second.size();
    }
    return static_cast<int>(count);
  }

  /** Returns the default group name; this protects against typos when the goal
   is to make sure that a property belongs to the default group.   */
  static const std::string& default_group_name() { return kDefaultGroup; }

 protected:
  /** @name  Construction, Copying, and Moving

   By design, constructing, moving, and copying are all rendered protected.
   Only derived classes can perform these operations -- this precludes
   interacting directly with this class whose sole purpose is to provide the
   inner machinery of the role-related derived types.  */
  //@{
  GeometryProperties() {
    AddGroup(kDefaultGroup);
  }
  GeometryProperties(const GeometryProperties&) = default;
  GeometryProperties& operator=(const GeometryProperties&) = default;
  GeometryProperties(GeometryProperties&&) = default;
  GeometryProperties& operator=(GeometryProperties&&) = default;
  //@}

 private:
  // The collection of property groups.
  std::unordered_map<std::string, PropertyGroup> values_;

  friend std::ostream& operator<<(std::ostream& out,
                                  const GeometryProperties& props);

  // An arbitrary name of the "default" nameless group.
  static const std::string kDefaultGroup;
};

}  // namespace dev
}  // namespace geometry
}  // namespace drake
