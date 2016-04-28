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
  /// Constructs a vector with one ScalarType element per name in names.
  explicit NamedValueVector(const std::vector<std::string>& names)
      : values_(names.size(), 1) {
    for (int i = 0; i < names.size(); i++) {
      names_[names[i]] = &values_(i, 0);
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
      values_(i, 0) = values[i];
    }
  }

  void Initialize(
      const Eigen::Matrix<ScalarType, Eigen::Dynamic, 1>& value) override {
    if (value.rows() != values_.rows()) {
      throw std::runtime_error("Cannot initialize a NamedValueVector of size " +
                               std::to_string(values_.rows()) +
                               "with a value of size " +
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

  void set_named_value(const std::string& name, ScalarType value) {
    *names_[name] = value;
  }

  const ScalarType& get_named_value(const std::string& name) {
    return *names_[name];
  }

 private:
  // The column vector backing store.
  Eigen::Matrix<ScalarType, Eigen::Dynamic, 1> values_;
  // A map from a name, to the corresponding element in values_.
  std::map<std::string, ScalarType*> names_;
};

}  // namespace systems
}  // namespace drake
