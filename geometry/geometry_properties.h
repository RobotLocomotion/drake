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
#include "drake/common/never_destroyed.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace geometry {

/** The base class for defining a set of geometry properties.

 A property is a value with associated semantics. The value 0.5 has no intrinsic
 interpretation. Labeling it "coefficient of friction" or "emission" can
 suggest its semantics. In the domain in which that label has meaning, the value
 can be used in a meaningful way. In %GeometryProperties the semantics of a
 property are defined by its name; if two properties have different names, then
 they are treated as having different semantics. To be a _set_ of properties,
 the elements of the set must be unique. In this case, uniqueness is defined by
 the "identifying name" of the property (i.e., the semantics) and not the value.
 (See below for elaboration on the concept of "identifying name".)

 %GeometryProperties provides a mechanism to facilitate working with the set
 of properties. Rather than defining the set as a single large set,
 %GeometryProperties encodes the set as the union of multiple disjoint subsets
 (called "groups" and each identified by a unique group name). The "identifying
 name" of a property is the pair of names: (group name, property name). This
 allows the same _property name_ to be used in multiple groups. What groups
 exist and what properties are contained in those groups is _specified_ by
 property _consumers_ and that specification is _instantiated_ on a per-geometry
 basis by _instantiators_.

 Generally, the value type of property can be arbitrarily complex (anything from
 a simple POD to a large struct). The only restriction is that all property
 value types must either be cloneable or copy-constructible.

 A set of geometry property values are defined when geometry is registered
 with SceneGraph by an instantiator and accessed by some downstream consumer
 entity. Each consumer specifies what properties it expects to find and what
 default values (if any) it provides. For example, the consumer could document
 that a particular property is always required and its absence would throw an
 exception. Alternatively, it could indicate that a property is optional
 and a default value will be used in its absence. It is the responsibility of
 the instantiator to make sure that the geometry property values are _correctly_
 defined according to the expected consumer's specification. Correctness
 includes such issues as key-value pairs placed into a _correctly_-spelled
 group, property keys being likewise correctly spelled, and values of the
 expected type. Correct spelling includes correct case.

 To define a set of property values to apply to a geometry:

   1. Create a named group. Properties can only be assigned to a group that has
      already been created. There is a _default_ group included in every
      %GeometryProperties instance; it can be accessed with the name returned by
      the default_group_name() method.
   2. Add a _new_ property to the group via AddProperty(). This is used when
      it is known the property name has _not_ been used in the corresponding
      property group already. Property names must be unique within a group. Once
      a property has been added, it cannot be modified.
   <!-- TODO(SeanCurtis-TRI): As need becomes apparent, add interface to replace
   values in previously defined values -- while preserving the original type.
   -->

 To read properties from a property set:

   1. Optionally, test to see if the desired property group exists in the
      property set via HasGroup(). Property access requires a group name and
      referencing a group that is not in the set may cause an exception
      to be thrown (depending on the API being exercised; see below).
   2. Optionally, test to see if a group has a property via the HasProperty()
      method. This isn't strictly required if the `OrDefault` interface is used.
   3. Acquire a property value via the GetProperty() or GetPropertyOrDefault()
      methods. The first form fails if the property (or group) doesn't exist.
      The second form takes an alternative default value which gets returned in
      the case where the property or group is missing.
      NOTE: Getting a property requires a compile-time declaration of the _type_
      of value being read. If the stored value is of a different type, an
      exception will be thrown.

 <h2>Common workflows</h2>

 The following examples outline a number of ways to create and consume geometry
 properties. By design, %GeometryProperties cannot be constructed,
 copied, or moved directly. Only derived classes can do so. This facilitates
 _strongly typed_ sets of properties associated with particular geometry roles.
 So, for these examples we'll exercise the derived class associated with
 proximity queries: ProximityProperties.

 The string-based structure of %GeometryProperties provides a great deal of
 flexibility at the cost of spelling sensitivity. It would be easy to introduce
 typos that would then "hide" property values in some location a consumer
 wouldn't look. In these examples, we avoid using string literals as group or
 property names (at least in the cases where the same name is used multiple
 times) to help avoid the possibility of typo-induced errors. That is not
 required and certainly not the only way to avoid such bugs.

 <h3>Creating properties</h3>

 <h4>Creating properties for a single group</h4>

 This is a simple example in which a single group is added with properties
 of various types.

 ```
 const std::string group_name("my_group");
 ProximityProperties properties;
 properties.AddGroup(group_name);
 properties.AddProperty(group_name, "count", 7);     // int type
 properties.AddProperty(group_name, "length", 7.);   // double type
 properties.AddProperty(group_name, "name", "7");    // std::string type
 ```

 <h4>Creating properties in the default group</h4>

 Similar to the previous examples, the properties are added to the default
 group. Just be aware that if multiple sites in your code add properties to
 the default group, the possibility that names get repeated increases. Property
 names _must_ be unique within a single group, including the default group.

 ```
 ProximityProperties properties;
 properties.AddProperty(ProximityProperties::default_group_name(), "count", 7);
 properties.AddProperty(ProximityProperties::default_group_name(), "length", 7.);
 properties.AddProperty(ProximityProperties::default_group_name(), "name", "7");
 ```

 <h4>Aggregate properties in a struct</h4>

 In some cases, there is a set of values that will _always_ be accessed
 together (specified with coordinated semantics). In these cases, it makes sense
 to aggregate them into a struct and store that as a single value. This reduces
 the number of lookups required.

 It's worth noting, that if the data value is a struct, calls to
 GetPropertyOrDefault() still operate as an "all-or-nothing" basis.
 If the property _struct_ exists, it will be returned, if it's missing the
 default struct will be returned. There is no concept of a "partial" struct
 in which some undefined values in the struct will be replaced with their
 corresponding values in the default struct.

 ```
 struct MyData {
    int i{};
    double d{};
    std::string s;
 };

 ProximityProperties properties;
 const std::string group_name("my_group");
 properties.AddGroup(group_name);
 MyData data{7, 7., "7"};
 properties.AddProperty(group_name, "data1", data);
 // These alternate forms are also acceptable (but not in succession, as the
 // property name has already been used by the first invocation).
 properties.AddProperty(group_name, "data2", MyData{6, 6., "6"});
 properties.AddProperty<MyData>(group_name, "data2", {6, 6., "6"});
 ```

 <h3>Reading properties</h3>

 This section describes how to read properties under several different
 scenarios: (a) when they are required, (b) when the consumer provides a
 default, and (c) when the consumer needs to inspect what properties are
 available.

 <h4>Look up specific, _required_ properties</h4>

 In this case, the consumer of the properties is looking for one or more
 specific properties. It will ignore any other properties. More particularly, if
 those properties are missing, it is considered a runtime error and an exception
 is thrown.

 The error can be handled in one of two ways: simply let the generic exception
 generated by %GeometryProperties propagate upward, or detect the missing
 property and throw an exception with a custom message. The example below shows
 both approaches.

 ```
 const ProximityProperties& properties = FunctionThatReturnsProperties();
 // Looking for a Vector3d of rgb colors named "rgb" - send generic error that
 // the property set is missing the required property.
 const Eigen::Vector3d rgb =
     properties.GetProperty<Eigen::Vector3d>("MyGroup", "rgb");

 // Explicitly detect missing property and throw exception with custom message.
 if (!properties.HasProperty("MyGroup", "rgb")) {
   throw std::logic_error(
       "ThisClass: Missing the necessary 'rgb' property; the object cannot be "
       "rendered");
 }
 // Otherwise acquire value, confident that no exception will be thrown.
 const Eigen::Vector3d rgb =
     properties.GetProperty<Eigen::Vector3d>("MyGroup", "rgb");
 ```

 @note calls to `GetProperty()` always require the return type template value
 (e.g., `Eigen::Vector3d`) to be specified in the call.

 <h4>Look up specific properties with default property values</h4>

 As with the previous case, the consumer is looking for one or more specific
 properties. However, in this case, the consumer provides a default value to
 use in case the target property is not defined. In this invocation, the
 template parameter need not be explicitly declared -- the inferred return type
 will be the same as the default value.

 ```
 const ProximityProperties& properties = FunctionThatReturnsProperties();
 // Looking for a Vector3d of rgb colors named "rgb".
 const Eigen::Vector3d default_color{0.9, 0.9, 0.9};
 const Eigen::Vector3d rgb =
     properties.GetPropertyOrDefault("MyGroup", "rgb", default_color);
 ```

 Alternatively, the default value can be provided in one of the following forms:

 ```
 properties.GetPropertyOrDefault("MyGroup", "rgb",
     Eigen::Vector3d{0.9, 0.9, 0.9});
 properties.GetPropertyOrDefault<Eigen::Vector3d>("MyGroup", "rgb",
     {0.9, 0.9, 0.9});
 ```

 <h4>Iterating through provided properties</h4>

 Another alternative is to iterate through the properties that _have_ been
 provided. This might be done for several reasons, e.g.:

   - the consumer wants to validate the set of properties, giving the user
     feedback if an unsupported property has been provided, and/or
   - the consumer has a default value for every property and allows the
     registering code to define only those properties that deviate from the
     specified default.

 Working with properties in this manner requires knowledge of how to work with
 systems::AbstractValue.

 ```
 const ProximityProperties& properties = FunctionThatReturnsProperties();
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
  /** The properties for a single group as a property name-value map.  */
  using Group =
      std::unordered_map<std::string,
                         copyable_unique_ptr<systems::AbstractValue>>;

  /** Adds a _new_ group to the property set (if it doesn't already exist). If
   the group already exists, there is no change to the property set.
   @throws std::logic_error if there is already a group with the given name.  */
  void AddGroup(const std::string& group_name) {
    if (values_.count(group_name) == 0) {
      values_.emplace(std::make_pair(group_name, Group{}));
    } else {
      throw std::logic_error(
          "AddGroup(): Cannot add group named '" + group_name + "'. "
          "A group with that name already exists");
    }
  }

  /** Reports if the given named group is part of this property set.  */
  bool HasGroup(const std::string& group_name) const {
    return values_.count(group_name) == 1;
  }

  /** Reports the number of property groups in this set.  */
  int num_groups() const { return static_cast<int>(values_.size()); }

  /** Retrieves the indicated property group.
   @throws std::logic_error if there is no group with the given name.  */
  const Group& GetGroup(const std::string& group_name) const {
    auto iter = values_.find(group_name);
    if (iter != values_.end()) {
      return iter->second;
    }

    throw std::logic_error(
        fmt::format("GetGroup(): There is no property group with the name {}",
                    group_name));
  }

  /** Returns all of the defined group names.  */
  std::set<std::string> GetGroupNames() const {
    std::set<std::string> group_names;
    for (const auto& pair : values_) {
      group_names.insert(pair.first);
    }
    return group_names;
  }

  /** Adds a property with the given `name` and `value` to the the named
   property group. This will _not_ overwrite the previous property if it already
   exists.

   @param group_name   The previously added group name.
   @param name         The name of the property -- must be unique in the group.
   @param value        The value to assign to the property.
   @throws std::logic_error if `group` is not a valid property group or
                               `name` already exists.
   @tparam ValueType   The type of data to store with the attribute -- must be
                       copy constructible or cloneable (see systems::Value).  */
  template <typename ValueType>
  void AddProperty(const std::string& group_name, const std::string& name,
                   const ValueType& value) {
    auto iter = values_.find(group_name);
    if (iter != values_.end()) {
      Group& group = iter->second;
      auto value_iter = group.find(name);
      if (value_iter == group.end()) {
        group[name] = std::make_unique<systems::Value<ValueType>>(value);
        return;
      }
      throw std::logic_error(fmt::format(
          "AddProperty(): Trying to add property `{}` to group `{}`; "
          "a property with that name already exists",
          name, group_name));
    }

    throw std::logic_error(
        fmt::format("AddProperty(): Trying to add property `{}` to group `{}`; "
                    "the group does not exist",
                    name, group_name));
  }

  /** Reports if the property exists in the group.

   @param group_name  The name of the group to which the tested property should
                      belong.
   @param name        The name of the property under question.
   @returns true iff the group exists and a property with the given `name`
                 exists in that group.  */
  bool HasProperty(const std::string& group_name,
                   const std::string& name) const {
    const auto iter = values_.find(group_name);
    if (iter != values_.end()) {
      const Group& group = iter->second;
      const auto value_iter = group.find(name);
      return value_iter != group.end();
    }
    return false;
  }

  /** Retrieves the typed value from this set of properties.

   @param group_name  The name of the group to which the property belongs.
   @param name        The name of the desired property.
   @throws std::logic_error
                            1. if the group name is invalid,
                            2. if the property name is invalid, or
                            3. if the property type is not that specified.
   @tparam ValueType  The expected type of the desired property.  */
  template <typename ValueType>
  const ValueType& GetProperty(const std::string& group_name,
                               const std::string& name) const {
    const auto iter = values_.find(group_name);
    if (iter != values_.end()) {
      const Group& group = iter->second;
      const auto value_iter = group.find(name);
      if (value_iter != group.end()) {
        const ValueType* value = value_iter->second->MaybeGetValue<ValueType>();
        if (value == nullptr) {
          throw std::logic_error(fmt::format(
              "GetProperty(): The property '{}' in group '{}' exists, "
              "but is of a different type. Requested {}, but found {}",
              group_name, name, NiceTypeName::Get<ValueType>(),
              value_iter->second->GetNiceTypeName()));
        }
        return *value;
      }
      throw std::logic_error(
          fmt::format("GetProperty(): There is no property {} in group {}.",
                      name, group_name));
    }

    throw std::logic_error(
        fmt::format("GetProperty(): Trying to read property {} from group {}. "
                    "But the group does not exist.",
                    name, group_name));
  }

  /** Retrieves the typed value from the set of properties (if it exists),
   otherwise returns the given default value. The given `default_value` is
   returned only if the group or property are missing. If the property exists
   and is of a _different_ type, an exception will be thrown. If it is of the
   expected type, the stored value will be returned.

   Generally, it is unnecessary to explicitly declare the `ValueType` of the
   property value; it will be inferred from the provided default value.
   Sometimes it is convenient to provide the default value in a form that can
   be implicitly converted to the final type. In that case, it is necessary
   to explicitly declare the desired `ValueType` so the compiler does not
   infer the wrong type, e.g.:

   ```
   // Note the _integer_ value as default value.
   const double my_value = properties.GetPropertyOrDefault<double>("g", "p", 2);
   ```

   @param group_name     The name of the group to which the property belongs.
   @param name           The name of the desired property.
   @param default_value  The alternate value to return if the property cannot
                         be acquired.
   @throws std::logic_error if a property of the given name exists but is not
                            of `ValueType`.  */
  // NOTE: The documentation that suggests capturing const T& as a return value
  // is based on the idea that some day copying won't happen; and code that
  // already uses references will immediately benefit.
  template <typename ValueType>
  ValueType GetPropertyOrDefault(const std::string& group_name,
                                 const std::string& name,
                                 ValueType default_value) const {
    const auto iter = values_.find(group_name);
    if (iter != values_.end()) {
      const Group& group = iter->second;
      const auto value_iter = group.find(name);
      if (value_iter != group.end()) {
        const ValueType* value = value_iter->second->MaybeGetValue<ValueType>();
        if (value == nullptr) {
          throw std::logic_error(fmt::format(
              "GetPropertyOrDefault(): The property '{}' in group '{}' exists, "
              "but is of a different type. Requested {}, but found {}",
              group_name, name, NiceTypeName::Get<ValueType>(),
              value_iter->second->GetNiceTypeName()));
        }
        // This incurs the cost of copying a stored value.
        return *value;
      }
    }
    return default_value;
  }

  /** Retrieves the set of defined properties for the indicated `group`.

   @param group_name  The name of the group to retrieve.
   @throws std::logic_error if the name does not refer to a valid group.  */
  const Group& GetPropertiesInGroup(const std::string& group_name) const {
    const auto iter = values_.find(group_name);
    if (iter != values_.end()) {
      return iter->second;
    } else {
      throw std::logic_error(
          fmt::format("GetPropertiesInGroup(): Can't retrieve properties for a "
                      "group that doesn't exist: {}",
                      group_name));
    }
  }

  /** Returns the default group name. There is no guarantee as to _what_ string
   corresponds to the default group. Therefore it should always be accessed via
   this method.  */
  static const std::string& default_group_name() {
    static const never_destroyed<std::string> kDefaultGroup("__default__");
    return kDefaultGroup.access();
  }

#ifndef DRAKE_DOXYGEN_CXX
  // Note: these overloads of the property access methods exist to enable
  // calls like `properties.AddProperty("group", "property", "string literal");
  // Template matching would deduce that the `ValueType` in this case is a const
  // char* (which is not copyable). By explicitly declaring this API, we can
  // implicitly convert the string literals to copyable std::strings. We assume
  // that the user is never actually trying to store const char*. We omit
  // these from the doxygen because they provide no value there.
  void AddProperty(const std::string& group_name, const std::string& name,
                   const char* value) {
    AddProperty<std::string>(group_name, name, value);
  }

  std::string GetPropertyOrDefault(const std::string& group_name,
                                   const std::string& name,
                                   const char* default_value) const {
    return GetPropertyOrDefault(group_name, name, std::string(default_value));
  }
#endif

 protected:
  /** @name  Construction, Copying, and Moving

   By design, constructing, moving, and copying are all rendered protected.
   Only derived classes can perform these operations -- this precludes
   interacting directly with this class whose sole purpose is to provide the
   inner machinery of the role-related derived types.  */
  //@{
  GeometryProperties() {
    AddGroup(default_group_name());
  }
  GeometryProperties(const GeometryProperties&) = default;
  GeometryProperties& operator=(const GeometryProperties&) = default;
  GeometryProperties(GeometryProperties&&) = default;
  GeometryProperties& operator=(GeometryProperties&&) = default;
  //@}

 private:
  // The collection of property groups.
  std::unordered_map<std::string, Group> values_;

  friend std::ostream& operator<<(std::ostream& out,
                                  const GeometryProperties& props);
};

}  // namespace geometry
}  // namespace drake
