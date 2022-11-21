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
