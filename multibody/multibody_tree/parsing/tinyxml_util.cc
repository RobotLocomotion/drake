#include "drake/multibody/multibody_tree/parsing/tinyxml_util.h"

#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "drake/math/rotation_matrix.h"

namespace drake {
namespace multibody {
namespace parsing {
namespace detail {
namespace {

std::vector<double> ConvertToDoubles(const std::string& str) {
  std::istringstream ss(str);

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

Eigen::Isometry3d OriginAttributesToTransform(
    const tinyxml2::XMLElement* node) {

  Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
  Eigen::Vector3d xyz = Eigen::Vector3d::Zero();

  ParseVectorAttribute(node, "xyz", &xyz);
  ParseVectorAttribute(node, "rpy", &rpy);

  const drake::math::RollPitchYaw<double> roll_pitch_yaw(rpy);
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.matrix() << roll_pitch_yaw.ToMatrix3ViaRotationMatrix(), xyz, 0, 0, 0, 1;
  return T;
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

}  // namespace detail
}  // namespace parsing
}  // namespace multibody
}  // namespace drake
