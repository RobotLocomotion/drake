#ifndef DRAKE_URDFPARSINGUTIL_H_H
#define DRAKE_URDFPARSINGUTIL_H_H

#include <string>
#include <fstream>
#include <sstream>
#include "drake/thirdParty/tinyxml2/tinyxml2.h"

bool parseScalarValue(tinyxml2::XMLElement* node, double &val)
{
  const char* strval = node->FirstChild()->Value();
  if (strval) {
    std::stringstream s(strval);
    s >> val;
    return true;
  }
  return false;
}

bool parseScalarAttribute(tinyxml2::XMLElement* node, const char* attribute_name, double& val)
{
  const char* attr = node->Attribute(attribute_name);
  if (attr) {
    std::stringstream s(attr);
    s >> val;
    return true;
  }
  return false;
}

// only writes values if they exist
bool parseVectorAttribute(tinyxml2::XMLElement* node, const char* attribute_name, Eigen::Vector3d &val)
{
  const char* attr = node->Attribute(attribute_name);
  if (attr) {
    std::stringstream s(attr);
    s >> val(0) >> val(1) >> val(2);
    return true;
  }
  return false;
}

bool parseVectorAttribute(tinyxml2::XMLElement* node, const char* attribute_name, Eigen::Vector4d &val)
{
  const char* attr = node->Attribute(attribute_name);
  if (attr) {
    std::stringstream s(attr);
    s >> val(0) >> val(1) >> val(2) >> val(3);
    return true;
  }
  return false;
}

void poseAttributesToTransform(tinyxml2::XMLElement* node, Eigen::Isometry3d& T)
{
  Eigen::Vector3d rpy=Eigen::Vector3d::Zero(), xyz=Eigen::Vector3d::Zero();

  parseVectorAttribute(node,"xyz",xyz);
  parseVectorAttribute(node,"rpy",rpy);

  T.matrix() << rpy2rotmat(rpy), xyz, 0,0,0,1;
}

#endif //DRAKE_URDFPARSINGUTIL_H_H
