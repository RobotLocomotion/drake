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
template <typename T>
class NamedValueVector : public VectorInterface<T> {
 public:
  /// Constructs a vector with one T element per name in names. Throws
  /// an std::runtime_error if names are not unique.
  explicit NamedValueVector(const std::vector<std::string>& names)
      : values_(VectorX<T>::Zero(names.size(), 1 /* column */)),
        names_(MakeNameMap(names)) {
    if (names_.size() != names.size()) {
      throw std::runtime_error("NamedValueVector has non-unique names.");
    }
  }

  /// Constructs a vector with the names and values in named_values. Throws an
  /// std::runtime_error if names are not unique.
  explicit NamedValueVector(
      const std::vector<std::pair<std::string, T>>& named_values)
      : NamedValueVector(GetKeys(named_values)) {
    for (int i = 0; i < named_values.size(); ++i) {
      values_[i] = named_values[i].second;
    }
  }

  ~NamedValueVector() override {}

  ptrdiff_t size() const override { return values_.rows(); }

  void set_value(const Eigen::Ref<const VectorX<T>>& value) override {
    if (value.rows() != size()) {
      throw std::out_of_range(
          "Cannot set a NamedValueVector of size " + std::to_string(size()) +
          " with a value of size " + std::to_string(value.rows()));
    }
    values_ = value;
  }

  const Eigen::VectorBlock<const VectorX<T>> get_value() const override {
    return values_.head(size());
  }

  Eigen::VectorBlock<VectorX<T>> get_mutable_value() override {
    return values_.head(size());
  }

  bool has_named_value(const std::string& name) const {
    return names_.find(name) != names_.end();
  }

  /// Sets the element with the given name. Throws std::runtime_error if
  /// the name doesn't exist.
  void set_named_value(const std::string& name, T value) {
    auto it = names_.find(name);
    if (it != names_.end()) {
      values_(it->second) = value;
    } else {
      throw std::runtime_error("Name " + name + " does not exist.");
    }
  }

  /// Returns the value with a given name, or nullptr if it doesn't exist.
  const T* get_named_value(const std::string& name) const {
    auto it = names_.find(name);
    if (it != names_.end()) {
      return &values_(it->second);
    }
    return nullptr;
  }

 private:
  // Given a list of names, returns a map from each name to its index in the
  // vector. If a name appears multiple times, the last index is used.
  static std::map<std::string, size_t> MakeNameMap(
      const std::vector<std::string>& names) {
    std::map<std::string, size_t> name_map;
    for (int i = 0; i < names.size(); i++) {
      name_map[names[i]] = i;
    }
    return name_map;
  }

  // Extracts the first element of each pair in named_values, preserving order.
  static std::vector<std::string> GetKeys(
      const std::vector<std::pair<std::string, T>>& named_values) {
    std::vector<std::string> keys;
    for (const auto& named_value : named_values) {
      keys.push_back(named_value.first);
    }
    return keys;
  }

  // The column vector of values.
  VectorX<T> values_;
  // A map from a name, to the corresponding index in values_.
  const std::map<std::string, size_t> names_;
};

}  // namespace systems
}  // namespace drake
