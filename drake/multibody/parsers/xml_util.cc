#include "drake/multibody/parsers/xml_util.h"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_path.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/thirdParty/bsd/tinydir/tinydir.h"

using std::string;
using std::stringstream;
using tinyxml2::XMLElement;

void ParseThreeVectorValue(const char* strval, Eigen::Vector3d* val) {
  if (val == nullptr) {
    throw std::invalid_argument(
        "ERROR: ParseThreeVectorValue: Parameter \"val\" is null.");
  }

  // Handles the case where strval is a nullptr.
  if (!strval) {
    throw std::invalid_argument(
        "ERROR: ParseThreeVectorValue: Parameter \"strval\" is null.");
  }

  // Handles the case where strval is an empty strval.
  if (string(strval) == "") {
    throw std::invalid_argument(
        "ERROR: ParseThreeVectorValue: Parameter \"strval\" is empty.");
  }

  stringstream ss(strval);
  string token;
  ss >> token;

  (*val)[0] = StringToDouble(token);

  // Handles the case where strval is a single scalar value.
  if (!ss.good()) {
    (*val)[1] = (*val)[0];
    (*val)[2] = (*val)[0];
    return;
  }

  ss >> token;
  (*val)[1] = StringToDouble(token);

  // Handles the case where strval is a 2 vector.
  if (!ss.good()) {
    throw std::invalid_argument(
        "ERROR: ParseThreeVectorValue: A 2 vector was supplied.");
  }

  ss >> token;
  (*val)[2] = StringToDouble(token);

  // Handles the case where strval is vector that's longer than 3.
  if (ss.good()) {
    throw std::invalid_argument(
        "ERROR: ParseThreeVectorValue: A vector with more than three "
        "elements was supplied.");
  }
}

double StringToDouble(const string& str) {
  std::size_t num_chars = 0;
  double result = std::stod(str, &num_chars);

  // Verifies that there are no additional characters after the double value.
  if (str.size() != num_chars) {
    throw std::invalid_argument(
        "ERROR: Double value contained additional characters after the "
        "number.");
  }

  return result;
}

void ParseThreeVectorValue(const tinyxml2::XMLElement* node,
                           Eigen::Vector3d* val) {
  if (node) {
    ParseThreeVectorValue(node->FirstChild()->Value(), val);
  } else {
    throw std::invalid_argument(
        "ERROR: ParseThreeVectorValue: Parameter \"node\" is null.");
  }
}

void ParseThreeVectorValue(const tinyxml2::XMLElement* node,
                           const char* element_name, Eigen::Vector3d* val) {
  if (!node || !element_name) {
    throw std::invalid_argument(
        "ERROR: ParseThreeVectorValue: Parameter \"node\" and/or parameter "
        "\"element_name\" is null.");
  } else {
    const tinyxml2::XMLElement* child_node =
        node->FirstChildElement(element_name);
    if (child_node == nullptr) {
      throw std::invalid_argument(
          "ERROR: ParseThreeVectorValue: Element \"" +
          std::string(element_name) + "\" not found.");
    }
    ParseThreeVectorValue(child_node, val);
  }
}

void ParseThreeVectorAttribute(const tinyxml2::XMLElement* node,
                               const char* attribute_name,
                               Eigen::Vector3d* val) {
  if (!node || !attribute_name) {
    throw std::invalid_argument(
        "ERROR: ParseThreeVectorAttribute: Parameter"
        "\"node\" and/or parameter \"attribute_name\" is null.");
  } else {
    const char* scale = node->Attribute(attribute_name);
    if (scale == nullptr) {
      throw std::invalid_argument(
          "ERROR: ParseThreeVectorAttribute: Attribute \"" +
          std::string(attribute_name) + "\" not found.");
    }
    ParseThreeVectorValue(scale, val);
  }
}

// only writes values if they exist
bool parseVectorAttribute(const tinyxml2::XMLElement* node,
                          const char* attribute_name,
                          // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                          Eigen::Vector3d& val) {
  const char* attr = node->Attribute(attribute_name);
  if (attr) {
    stringstream s(attr);
    s >> val(0) >> val(1) >> val(2);
    return true;
  }
  return false;
}

bool parseVectorAttribute(const tinyxml2::XMLElement* node,
                          const char* attribute_name,
                          // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                          Eigen::Vector4d& val) {
  const char* attr = node->Attribute(attribute_name);
  if (attr) {
    stringstream s(attr);
    s >> val(0) >> val(1) >> val(2) >> val(3);
    return true;
  }
  return false;
}

bool parseVectorValue(tinyxml2::XMLElement* node, const char* element_name,
                      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                      Eigen::Vector3d& val) {
  XMLElement* elnode = node->FirstChildElement(element_name);
  if (elnode && elnode->FirstChild()) {
    stringstream s(elnode->FirstChild()->Value());
    s >> val(0) >> val(1) >> val(2);
    return true;
  }
  return false;
}

bool parseVectorValue(tinyxml2::XMLElement* node, const char* element_name,
                      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                      Eigen::Vector4d& val) {
  XMLElement* elnode = node->FirstChildElement(element_name);
  if (elnode && elnode->FirstChild()) {
    stringstream s(elnode->FirstChild()->Value());
    s >> val(0) >> val(1) >> val(2) >> val(3);
    return true;
  }
  return false;
}

bool parseStringValue(tinyxml2::XMLElement* node, const char* element_name,
                      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                      std::string& val) {
  XMLElement* elnode = node->FirstChildElement(element_name);
  if (elnode && elnode->FirstChild()) {
    val = elnode->FirstChild()->Value();
    return true;
  }
  return false;
}

void originAttributesToTransform(
    tinyxml2::XMLElement* node,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    Eigen::Isometry3d& T) {
  Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
  Eigen::Vector3d xyz = Eigen::Vector3d::Zero();

  parseVectorAttribute(node, "xyz", xyz);
  parseVectorAttribute(node, "rpy", rpy);

  T.matrix() << drake::math::rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
}

void poseValueToTransform(tinyxml2::XMLElement* node, const PoseMap& pose_map,
                          // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                          Eigen::Isometry3d& T,
                          const Eigen::Isometry3d& T_default_frame) {
  Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
  Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
  const char* strval = node->FirstChild()->Value();
  if (strval) {
    stringstream s(strval);
    s >> xyz(0) >> xyz(1) >> xyz(2) >> rpy(0) >> rpy(1) >> rpy(2);
  }

  T.matrix() << drake::math::rpy2rotmat(rpy), xyz, 0, 0, 0, 1;

  const char* attr = node->Attribute("frame");
  if (attr && strlen(attr) > 0) {
    string frame;
    stringstream s(attr);
    s >> frame;
    // This could cause problems with the default pose assumptions in
    // the sdf parser, which I simply had to guess.
    DRAKE_ASSERT(0 && "this has not been tested yet");
    Eigen::Isometry3d T_frame =
        pose_map.at(frame);  // will throw an exception if the frame is not
                             // found.  that is the desired behavior.
    T = T_frame * T;
  } else {
    T = T_default_frame * T;
  }
}
