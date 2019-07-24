#include "drake/multibody/parsing/detail_tinyxml.h"

#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "drake/math/rotation_matrix.h"

namespace drake {
namespace multibody {
namespace internal {

namespace {
std::vector<double> ConvertToDoubles(const std::string& str) {
  std::istringstream ss(str);
  // Every real number in a URDF file needs to be parsed assuming that the
  // decimal point separator is the period, as specified in XML Schema
  // definition of xs:double. The call to imbue ensures that a suitable
  // locale is used, instead of using the current C++ global locale.
  // Related PR: https://github.com/ros/urdfdom_headers/pull/42 .
  ss.imbue(std::locale::classic());

  double val{};
  std::vector<double> out;
  while (ss >> val) {
    out.push_back(val);
  }
  return out;
}
}  // namespace

bool ParseStringAttribute(const tinyxml2::XMLElement* node,
                          const char* attribute_name, std::string* val) {
  const char* attr = node->Attribute(attribute_name);
  if (attr) {
    *val = attr;
    return true;
  }
  val->clear();
  return false;
}

bool ParseScalarAttribute(const tinyxml2::XMLElement* node,
                          const char* attribute_name, double* val) {
  const char* attr = node->Attribute(attribute_name);
  if (attr) {
    std::vector<double> vals = ConvertToDoubles(attr);
    if (vals.size() != 1) {
      throw std::invalid_argument(
          std::string("Expected single value for attribute ") + attribute_name +
          " got " + attr);
    }
    *val = vals[0];
    return true;
  }
  return false;
}

bool ParseVectorAttribute(const tinyxml2::XMLElement* node,
                          const char* attribute_name,
                          Eigen::Vector3d* val) {
  const char* attr = node->Attribute(attribute_name);
  if (attr) {
    std::vector<double> vals = ConvertToDoubles(attr);
    if (vals.size() != 3) {
      throw std::invalid_argument(
          std::string("Expected three values for attribute ") + attribute_name +
          " got " + attr);
    }
    *val = Eigen::Vector3d(vals.data());
    return true;
  }
  return false;
}

bool ParseVectorAttribute(const tinyxml2::XMLElement* node,
                          const char* attribute_name,
                          Eigen::Vector4d* val) {
  const char* attr = node->Attribute(attribute_name);
  if (attr) {
    std::vector<double> vals = ConvertToDoubles(attr);
    if (vals.size() != 4) {
      throw std::invalid_argument(
          std::string("Expected four values for attribute ") + attribute_name +
          " got " + attr);
    }
    *val = Eigen::Vector4d(vals.data());
    return true;
  }
  return false;
}

math::RigidTransformd OriginAttributesToTransform(
    const tinyxml2::XMLElement* node) {

  Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
  Eigen::Vector3d xyz = Eigen::Vector3d::Zero();

  ParseVectorAttribute(node, "xyz", &xyz);
  ParseVectorAttribute(node, "rpy", &rpy);

  return {math::RollPitchYawd(rpy), xyz};
}

bool ParseThreeVectorAttribute(const tinyxml2::XMLElement* node,
                               const char* attribute_name,
                               Eigen::Vector3d* val) {
  if (!node || !attribute_name) {
    throw std::invalid_argument(
        "ERROR: ParseThreeVectorAttribute: Parameter"
        "\"node\" and/or parameter \"attribute_name\" is null.");
  }

  const char* attr = node->Attribute(attribute_name);
  if (attr == nullptr) {
    return false;
  }

  std::string attr_str(attr);
  if (attr_str.find(' ') != std::string::npos) {
    ParseVectorAttribute(node, attribute_name, val);
  } else {
    double scalar_val;
    ParseScalarAttribute(node, attribute_name, &scalar_val);
    val->fill(scalar_val);
  }
  return true;
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
