#pragma once

#include <map>
#include <stdexcept>
#include <string>
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
      : values_(Eigen::Matrix<ScalarType, Eigen::Dynamic, 1>::Zero(names.size(),
                                                                   1)),
        names_(MakeNameMap(names)) {
    if (names_.size() != names.size()) {
      throw std::runtime_error("NamedValueVector has non-unique names.");
    }
  }

  /// Constructs a vector with one ScalarType element per name in names, and
  /// the values in values.
  NamedValueVector(const std::vector<std::string>& names,
                   const std::vector<ScalarType>& values)
      : NamedValueVector(names) {
    if (names.size() != values.size()) {
      throw std::runtime_error("Cannot initialize a NamedValueVector with " +
                               std::to_string(names.size()) + "names " +
                               "and " + std::to_string(values.size()) +
                               "values.");
    }
    for (int i = 0; i < values.size(); i++) {
      values_(i) = values[i];
    }
  }

  virtual ~NamedValueVector() {}

  void set_value(
      const Eigen::Matrix<ScalarType, Eigen::Dynamic, 1>& value) override {
    if (value.rows() != values_.rows()) {
      throw std::runtime_error("Cannot set a NamedValueVector of size " +
                               std::to_string(values_.rows()) +
                               " with a value of size " +
                               std::to_string(value.rows()));
    }
    values_ = value;
  }

  const Eigen::Matrix<ScalarType, Eigen::Dynamic, 1>& get_value()
      const override {
    return values_;
  }

  Eigen::Matrix<ScalarType, Eigen::Dynamic, 1>* get_mutable_value() override {
    return &values_;
  }

  /// Sets the element with the given name.  Does nothing if the name doesn't
  /// exist.
  void set_named_value(const std::string& name, ScalarType value) {
    if (names_.find(name) != names_.end()) {
      values_(names_.find(name)->second) = value;
    }
  }

  /// Returns the value with a given name, or nullptr if it doesn't exist.
  const ScalarType* get_named_value(const std::string& name) const {
    if (names_.find(name) != names_.end()) {
      return &values_(names_.find(name)->second);
    }
    return nullptr;
  }

 private:
  /// Assumes values_ has already been initialized.
  std::map<std::string, size_t> MakeNameMap(
      const std::vector<std::string>& names) {
    std::map<std::string, size_t> name_map;
    for (int i = 0; i < names.size(); i++) {
      name_map[names[i]] = i;
      values_(i) = 0;
    }
    return name_map;
  }

  // The column vector backing store.
  Eigen::Matrix<ScalarType, Eigen::Dynamic, 1> values_;
  // A map from a name, to the corresponding index in values_.
  const std::map<std::string, size_t> names_;
};

}  // namespace systems
}  // namespace drake
