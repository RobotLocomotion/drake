#pragma once

#include <functional>
#include <memory>
#include <set>
#include <string>
#include <string_view>
#include <typeinfo>
#include <unordered_map>

#include <Eigen/Dense>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/fmt.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/value.h"
#include "drake/geometry/rgba.h"

namespace drake {
namespace geometry {

/** The base class for defining a set of geometry properties.

 Each property consists of a `(group, property)` name-pair and a typed value.
 The name pair allows for reuse of common property names (e.g., "diffuse") to be
 differentiated in interpretation by associating them with different groups. The
 only restriction on the value type is that it must be either cloneable or
 copy-constructible.

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
 expected type. Correct spelling includes correct case. The instantiator uses
 the AddProperty() method to add new properties to the set.

 To read the property (`some_group`, `some_property`) from a property set:

   1. Optionally test to see if the property exists by confirming the group
      `some_group` is in the set via HasGroup() and that the property
      `some_property` is in `some_group` via HasProperty(). Attempting to access
      a property with a non-existent (group, property) pair may lead to an
      exception (see API documentation below).
   2. Acquire a property value via the GetProperty() or GetPropertyOrDefault()
      methods.
      NOTE: Reading a property requires a compile-time declaration of the _type_
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
 typos that would then "hide" property values in some group a consumer
 wouldn't look. In these examples, we avoid using string literals as group or
 property names (at least in the cases where the same name is used multiple
 times) to help avoid the possibility of typo-induced errors. That is not
 required and certainly not the only way to avoid such bugs.

 <h3>Creating properties</h3>

 <h4>Creating properties in a new group</h4>

 This is a simple example in which a single group is added with properties
 of various types.

 ```
 const std::string group_name("my_group");
 ProximityProperties properties;
 // This first invocation implicitly creates the group "my_group".
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
 properties.AddProperty(ProximityProperties::default_group_name(), "width", 7.);
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
 MyData data{7, 7., "7"};
 properties.AddProperty(group_name, "data1", data);
 // These alternate forms are also acceptable (but not in succession, as the
 // property name has already been used by the first invocation).
 properties.AddProperty(group_name, "data2", MyData{6, 6., "6"});
 properties.AddProperty<MyData>(group_name, "data2", {6, 6., "6"});
 ```

 <h3>Reading properties</h3>

 This section describes how to read properties under several different
 scenarios: (a) when specific properties are required, (b) when the consumer
 provides a default value for missing properties, and (c) when the consumer
 needs to inspect what properties are available.

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
 const IllustrationProperties& properties = FunctionThatReturnsProperties();
 // Looking for a Rgba of rgba colors named "rgba" - send generic error that
 // the property set is missing the required property.
 const Rgba rgba =
     properties.GetProperty<Rgba>("MyGroup", "rgba");

 // Explicitly detect missing property and throw exception with custom message.
 if (!properties.HasProperty("MyGroup", "rgba")) {
   throw std::logic_error(
       "ThisClass: Missing the necessary 'rgba' property; the object cannot be "
       "rendered");
 }
 // Otherwise acquire value, confident that no exception will be thrown.
 const Rgba rgba =
     properties.GetProperty<Rgba>("MyGroup", "rgba");
 ```

 @note calls to `GetProperty()` always require the return type template value
 (e.g., `Rgba`) to be specified in the call.

 <h4>Look up specific properties with default property values</h4>

 As with the previous case, the consumer is looking for one or more specific
 properties. However, in this case, the consumer provides a default value to
 use in case the target property is not defined. In this invocation, the
 template parameter need not be explicitly declared -- the inferred return type
 will be the same as the default value.

 ```
 const IllustrationProperties& properties = FunctionThatReturnsProperties();
 // Looking for a Rgba of rgba colors named "rgba".
 const Rgba default_color{0.9, 0.9, 0.9};
 const Rgba rgba =
     properties.GetPropertyOrDefault("MyGroup", "rgba", default_color);
 ```

 Alternatively, the default value can be provided in one of the following forms:

 ```
 properties.GetPropertyOrDefault("MyGroup", "rgba",
     Rgba{0.9, 0.9, 0.9});
 properties.GetPropertyOrDefault<Rgba>("MyGroup", "rgba",
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
 AbstractValue.

 ```
 const IllustrationProperties& properties = FunctionThatReturnsProperties();
 for (const auto& pair : properties.GetGroupProperties("MyGroup") {
   const std::string& name = pair.first;
   if (name == "rgba") {
     // Throws an exception if the named parameter is of the wrong type.
     const Rgba& rgba =
         pair.second->GetValueOrThrow<Rgba>();
   }
 }
 ```
 */
class GeometryProperties {
 public:
  virtual ~GeometryProperties();

  /** The properties for a single group as a property name-value map.  */
  using Group =
      std::unordered_map<std::string, copyable_unique_ptr<AbstractValue>>;

  /** Reports if the given named group is part of this property set.  */
  bool HasGroup(const std::string& group_name) const {
    return values_.contains(group_name);
  }

  /** Reports the number of property groups in this set.  */
  int num_groups() const { return static_cast<int>(values_.size()); }

  /** Retrieves the indicated property group. The returned group is valid for
   as long as this instance.
   @throws std::exception if there is no group with the given name.  */
  const Group& GetPropertiesInGroup(const std::string& group_name) const;

  /** Returns all of the defined group names.  */
  std::set<std::string> GetGroupNames() const;

  /** Adds the named property (`group_name`, `name`) with the given `value`.
   Adds the group if it doesn't already exist.

   @param group_name   The group name.
   @param name         The name of the property -- must be unique in the group.
   @param value        The value to assign to the property.
   @throws std::exception if the property already exists.
   @tparam ValueType   The type of data to store with the attribute -- must be
                       copy constructible or cloneable (see Value).  */
  template <typename ValueType>
  void AddProperty(const std::string& group_name, const std::string& name,
                   const ValueType& value) {
    if constexpr (std::is_same_v<ValueType, Eigen::Vector4d>) {
      AddPropertyAbstract(group_name, name, Value(ToRgba(value)));
    } else {
      AddPropertyAbstract(group_name, name, Value(value));
    }
  }

  /** Updates the named property (`group_name`, `name`) with the given `value`.
   If the property doesn't already exist, it is equivalent to calling
   `AddProperty`. If the property does exist, its value (which must have the
   same type as `value`) will be replaced.

   @param group_name   The group name.
   @param name         The name of the property -- must be unique in the group.
   @param value        The value to assign to the property.
   @throws std::exception if the property exists with a different type.
   @tparam ValueType   The type of data to store with the attribute -- must be
                       copy constructible or cloneable (see Value).  */
  template <typename ValueType>
  void UpdateProperty(const std::string& group_name, const std::string& name,
                      const ValueType& value) {
    UpdatePropertyAbstract(group_name, name, Value(value));
  }

  /** Adds the named property (`group_name`, `name`) with the given type-erased
   `value`. Adds the group if it doesn't already exist.

   @param group_name   The group name.
   @param name         The name of the property -- must be unique in the group.
   @param value        The value to assign to the property.
   @throws std::exception if the property already exists.  */
  void AddPropertyAbstract(const std::string& group_name,
                           const std::string& name, const AbstractValue& value);

  /** Updates the named property (`group_name`, `name`) with the given
   type-erased `value`. If the property doesn't already exist, it is equivalent
   to calling `AddPropertyAbstract`. If the property does exist, its value
   (which must have the same type as `value`) will be replaced.

   @param group_name   The group name.
   @param name         The name of the property -- must be unique in the group.
   @param value        The value to assign to the property.
   @throws std::exception if the property exists with a different type.  */
  void UpdatePropertyAbstract(const std::string& group_name,
                              const std::string& name,
                              const AbstractValue& value);

  /** Reports if the property (`group_name`, `name`) exists in the group.

   @param group_name  The name of the group to which the tested property should
                      belong.
   @param name        The name of the property under question.
   @returns true iff the group exists and a property with the given `name`
                 exists in that group.  */
  bool HasProperty(const std::string& group_name,
                   const std::string& name) const;

  /** Retrieves the typed value for the property (`group_name`, `name`) from
   this set of properties.

   @param group_name  The name of the group to which the property belongs.
   @param name        The name of the desired property.
   @throws std::exception if a) the group name is invalid,
                          b) the property name is invalid, or
                          c) the property type is not that specified.
   @tparam ValueType  The expected type of the desired property.
   @returns const ValueType& of stored value.
            If ValueType is Eigen::Vector4d, the return type will be a copy
            translated from Rgba.
   */
  template <typename ValueType>
  decltype(auto) GetProperty(const std::string& group_name,
                             const std::string& name) const {
    const AbstractValue& abstract = GetPropertyAbstract(group_name, name);
    if constexpr (std::is_same_v<ValueType, Eigen::Vector4d>) {
      const Rgba color = GetValueOrThrow<Rgba>(
          "GetProperty", group_name, name, abstract, typeid(Eigen::Vector4d));
      return ToVector4d(color);
    } else {
      return GetValueOrThrow<ValueType>("GetProperty", group_name, name,
                                        abstract);
    }
  }

  /** Retrieves the type-erased value for the property (`group_name`, `name`)
   from this set of properties.

   @param group_name  The name of the group to which the property belongs.
   @param name        The name of the desired property.
   @throws std::exception if a) the group name is invalid, or
                          b) the property name is invalid. */
  const AbstractValue& GetPropertyAbstract(const std::string& group_name,
                                           const std::string& name) const;

  /** Retrieves the typed value for the property (`group_name`, `name`) from the
   set of properties (if it exists), otherwise returns the given default value.
   The given `default_value` is returned only if the property is missing. If the
   property exists and is of a _different_ type, an exception will be thrown. If
   it is of the expected type, the stored value will be returned.

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
   @throws std::exception if a property of the given name exists but is not
                          of `ValueType`.  */
  template <typename ValueType>
  ValueType GetPropertyOrDefault(const std::string& group_name,
                                 const std::string& name,
                                 ValueType default_value) const {
    const AbstractValue* abstract =
        GetPropertyAbstractMaybe(group_name, name, false);
    if (!abstract) {
      return default_value;
    } else {
      if constexpr (std::is_same_v<ValueType, Eigen::Vector4d>) {
        const Rgba color =
            GetValueOrThrow<Rgba>("GetPropertyOrDefault", group_name, name,
                                  *abstract, typeid(Eigen::Vector4d));
        return ToVector4d(color);
      } else {
        // This incurs the cost of copying a stored value.
        return GetValueOrThrow<ValueType>("GetPropertyOrDefault", group_name,
                                          name, *abstract);
      }
    }
  }

  /** Returns the default group name. There is no guarantee as to _what_ string
   corresponds to the default group. Therefore it should always be accessed via
   this method.  */
  static const std::string& default_group_name() {
    static const never_destroyed<std::string> kDefaultGroup("__default__");
    return kDefaultGroup.access();
  }

  /** Removes the (`group_name`, `name`) property (if it exists). Upon
   completion the property will not be in the set.
   @returns `true` if the property existed prior to the call.  */
  bool RemoveProperty(const std::string& group_name, const std::string& name);

#ifndef DRAKE_DOXYGEN_CXX
  // Note: these overloads of the property access methods exist to enable
  // calls like `properties.AddProperty("group", "property", "string literal");`
  // Template matching would deduce that the `ValueType` in this case is a const
  // char* (which is not copyable). By explicitly declaring this API, we can
  // implicitly convert the string literals to copyable std::strings. We assume
  // that the user is never actually trying to store const char*. We omit
  // these from the doxygen because they provide no value there.
  void AddProperty(const std::string& group_name, const std::string& name,
                   const char* value) {
    AddProperty<std::string>(group_name, name, value);
  }

  void UpdateProperty(const std::string& group_name, const std::string& name,
                      const char* value) {
    UpdateProperty<std::string>(group_name, name, value);
  }

  std::string GetPropertyOrDefault(const std::string& group_name,
                                   const std::string& name,
                                   const char* default_value) const {
    return GetPropertyOrDefault(group_name, name, std::string(default_value));
  }
#endif

  /** Converts the GeometryProperties to a string representation.  */
  std::string to_string() const;

 protected:
  /** Constructs a property set with the default group. Only invoked by final
   subclasses.  */
  GeometryProperties() { values_.emplace(default_group_name(), Group{}); }

  // Final subclasses are allowed to make copy/move/assign public.
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GeometryProperties);

 private:
  // Conditionally writes the property (group_name, name) with the given value.
  // The caller provides a test function that should throw if assigning `value`
  // to the specified property would fail. The function takes the `Group`
  // associated with `group_name`.
  void WritePropertyAbstract(
      const std::string& group_name, const std::string& name,
      const AbstractValue& value,
      const std::function<void(const Group&)>& throw_if_invalid);

  // The collection of property groups.
  std::unordered_map<std::string, Group> values_;

  // Return value or nullptr if it does not exist.
  // If `throw_for_bad_group` is true, an error will be thrown if `group_name`
  // does not exist.
  const AbstractValue* GetPropertyAbstractMaybe(const std::string& group_name,
                                                const std::string& name,
                                                bool throw_for_bad_group) const;

  // Get the wrapped value from an AbstractValue, or throw an error message
  // that is easily traceable to this class.
  template <typename ValueType>
  static const ValueType& GetValueOrThrow(
      std::string_view method, const std::string& group_name,
      const std::string& name, const AbstractValue& abstract,
      const std::type_info& requested_type = typeid(ValueType)) {
    const ValueType* value = abstract.maybe_get_value<ValueType>();
    if (value == nullptr) {
      throw std::logic_error(fmt::format(
          "{}(): The property ('{}', '{}') exists, but is of a different type. "
          "Requested '{}', but found '{}'",
          method, group_name, name, NiceTypeName::Get(requested_type),
          abstract.GetNiceTypeName()));
    }
    return *value;
  }

  // TODO(eric.cousineau): Deprecate this, saying: "Use Rgba instead of
  // Vector4d to define diffuse color."
  static Eigen::Vector4d ToVector4d(const Rgba& color) { return color.rgba(); }

  // TODO(eric.cousineau): Deprecate this, saying: "Use Rgba instead of
  // Vector4d to define diffuse color."
  static Rgba ToRgba(const Eigen::Vector4d& value) {
    return Rgba(value(0), value(1), value(2), value(3));
  }
};

}  // namespace geometry
}  // namespace drake

DRAKE_FORMATTER_AS(, drake::geometry, GeometryProperties, x, x.to_string())
