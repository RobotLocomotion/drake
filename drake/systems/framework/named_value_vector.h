#pragma once

#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "drake/systems/framework/vector_interface.h"

#include <Eigen/Dense>

namespace drake {
namespace systems {

/// NamedValueVector is a low-performance convenience implementation of
/// VectorInterface that labels each element in a column vector with a string.
template <typename ScalarType>
class NamedValueVector : public VectorInterface<ScalarType> {
 public:
  /// Constructs a vector with one ScalarType element per name in names. Throws
  /// an std::runtime_error if names are not unique.
  explicit NamedValueVector(const std::vector<std::string>& names)
      : values_(VectorX<ScalarType>::Zero(names.size(), 1)),
        names_(MakeNameMap(names)) {
    if (names_.size() != names.size()) {
      throw std::runtime_error("NamedValueVector has non-unique names.");
    }
  }

  /// Constructs a vector with the names and values in named_values. Throws an
  /// std::runtime_error if names are not unique.
  NamedValueVector(
      const std::vector<std::pair<std::string, ScalarType>>& named_values)
      : NamedValueVector(GetKeys(named_values)) {
    for (const auto& named_value : named_values) {
      const std::string& name = named_value.first;
      const ScalarType& value = named_value.second;
      values_(names_.find(name)->second) = value;
    }
  }

  ~NamedValueVector() override {}

  void set_value(const VectorX<ScalarType>& value) override {
    if (value.rows() != values_.rows()) {
      throw std::runtime_error("Cannot set a NamedValueVector of size " +
                               std::to_string(values_.rows()) +
                               " with a value of size " +
                               std::to_string(value.rows()));
    }
    values_ = value;
  }

  const VectorX<ScalarType>& get_value() const override { return values_; }

  Eigen::VectorBlock<VectorX<ScalarType>> get_mutable_value() override {
    return values_.head(values_.rows());
  }

  bool has_named_value(const std::string& name) const {
    return names_.find(name) != names_.end();
  }

  /// Sets the element with the given name. Throws std::runtime_error if
  /// the name doesn't exist.
  void set_named_value(const std::string& name, ScalarType value) {
    if (has_named_value(name)) {
      values_(names_.find(name)->second) = value;
    } else {
      throw std::runtime_error("Name " + name + " does not exist.");
    }
  }

  /// Returns the value with a given name, or nullptr if it doesn't exist.
  const ScalarType* get_named_value(const std::string& name) const {
    if (has_named_value(name)) {
      return &values_(names_.find(name)->second);
    }
    return nullptr;
  }

 private:
  static std::map<std::string, size_t> MakeNameMap(
      const std::vector<std::string>& names) {
    std::map<std::string, size_t> name_map;
    for (int i = 0; i < names.size(); i++) {
      name_map[names[i]] = i;
    }
    return name_map;
  }

  static std::vector<std::string> GetKeys(
      const std::vector<std::pair<std::string, ScalarType>>& named_values) {
    std::vector<std::string> keys;
    for (const auto& named_value : named_values) {
      keys.push_back(named_value.first);
    }
    return keys;
  }

  // The column vector backing store.
  VectorX<ScalarType> values_;
  // A map from a name, to the corresponding index in values_.
  const std::map<std::string, size_t> names_;
};

}  // namespace systems
}  // namespace drake
