#pragma once

#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <drake_vendor/tinyxml2.h>

#include "drake/common/diagnostic_policy.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace multibody {
namespace internal {

// Parses a string containing values of type T in the XML attribute format into
// a vector of T.
template <typename T>
std::vector<T> ConvertToVector(const std::string& str) {
  std::istringstream ss(str);
  // Every real number in a URDF file needs to be parsed assuming that the
  // decimal point separator is the period, as specified in XML Schema
  // definition of xs:double. The call to imbue ensures that a suitable
  // locale is used, instead of using the current C++ global locale.
  // Related PR: https://github.com/ros/urdfdom_headers/pull/42 .
  ss.imbue(std::locale::classic());

  T val{};
  std::vector<T> out;
  while (ss >> val) {
    out.push_back(val);
  }
  return out;
}

// Parses a string attribute of @p node named @p attribute_name into @p val.
// If the attribute is not present, @p val will be cleared.
//
// @returns false if the attribute is not present
bool ParseStringAttribute(const tinyxml2::XMLElement* node,
                          const char* attribute_name, std::string* val);

// Parses a scalar attribute of @p node named @p attribute_name into @p val.
//
// @returns false if the attribute is not present, or if no numeric values were
// found. Otherwise, returns true and sets @p val to the first value obtained.
//
// If the attribute doesn't contain exactly one numeric value, the function
// emits an error on @p policy, which may result in the policy throwing.
//
// If @p policy is empty, then the default policy will be used, which results
// in a `throw` on error.
template <typename T>
bool ParseScalarAttribute(const tinyxml2::XMLElement* node,
                          const char* attribute_name, T* val,
                          std::optional<const drake::internal::DiagnosticPolicy>
                              policy = std::nullopt) {
  if (!policy.has_value()) {
    policy.emplace();
  }
  const char* attr = node->Attribute(attribute_name);
  if (attr) {
    std::vector<T> vals = ConvertToVector<T>(attr);
    if (vals.size() != 1) {
      policy->Error(
          fmt::format("Expected single value for attribute '{}' got '{}'",
                      attribute_name, attr));
    }
    if (!vals.empty()) {
      *val = vals[0];
    }
    return !vals.empty();
  }
  return false;
}

// Parses an attribute of @p node named @p attribute_name consisting of scalar
// values into @p val.
//
// @returns false if the attribute is not present, and the output is left
// untouched.
// @tparam rows is indicates the size of the vector `val`.
//
// @throws std::exception if the attribute doesn't contain `rows` numeric
// values.
template <int rows>
bool ParseVectorAttribute(const tinyxml2::XMLElement* node,
                          const char* attribute_name,
                          Eigen::Matrix<double, rows, 1>* val) {
  const char* attr = node->Attribute(attribute_name);
  if (attr) {
    std::vector<double> vals = ConvertToVector<double>(attr);
    if (vals.size() != rows) {
      throw std::invalid_argument(
          fmt::format("Expected {} values for attribute {} got {}", rows,
                      attribute_name, attr));
    }
    *val = Eigen::Matrix<double, rows, 1>(vals.data());
    return true;
  }
  return false;
}

// Parses "xyz" and "rpy" attributes from @p node and returns a
// RigidTransformd created from them.  If either the "xyz" or "rpy"
// attributes are omitted they will be initialized with zero values.
//
// @throws std::exception if the "xyz" or "rpy" attributes are
// malformed.
math::RigidTransformd OriginAttributesToTransform(
    const tinyxml2::XMLElement* node);

// Parses a three vector value from parameter @p node, which is an
// XML node. The value is specified by an attribute within the XML
// whose name is specified by parameter @p attribute_name.  This
// method also supports a three vector specified by a single scalar
// value, which it automatically converts into a three vector by
// using the same scalar value for all three dimensions.
//
// @param[in] node A pointer to the XML element node that contains an
// attribute with a three vector or a scalar value.
// @param[in] attribute_name The name of the attribute containing the three
// vector or scalar value.
// @param[out] val The three vector where the results should be stored.
//
// @returns false if the attribute is not present
//
// @throws std::exception If any problem is encountered parsing the
// three vector value.
bool ParseThreeVectorAttribute(const tinyxml2::XMLElement* node,
                               const char* attribute_name,
                               Eigen::Vector3d* val);

}  // namespace internal
}  // namespace multibody
}  // namespace drake
