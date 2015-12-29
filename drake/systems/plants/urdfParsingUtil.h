#ifndef DRAKE_URDFPARSINGUTIL_H_H
#define DRAKE_URDFPARSINGUTIL_H_H

#include <string>
#include <fstream>
#include <sstream>
#include "tinyxml.h"

bool parseScalarValue(TiXmlElement* node, double &val)
{
  const char* strval = node->FirstChild()->Value();
  if (strval) {
    std::stringstream s(strval);
    s >> val;
    return true;
  }
  return false;
}

bool parseScalarAttribute(TiXmlElement* node, const char* attribute_name, double& val)
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
bool parseVectorAttribute(TiXmlElement* node, const char* attribute_name, Eigen::Vector3d &val)
{
  const char* attr = node->Attribute(attribute_name);
  if (attr) {
    std::stringstream s(attr);
    s >> val(0) >> val(1) >> val(2);
    return true;
  }
  return false;
}

bool parseVectorAttribute(TiXmlElement* node, const char* attribute_name, Eigen::Vector4d &val)
{
  const char* attr = node->Attribute(attribute_name);
  if (attr) {
    std::stringstream s(attr);
    s >> val(0) >> val(1) >> val(2) >> val(3);
    return true;
  }
  return false;
}

void poseAttributesToTransform(TiXmlElement* node, Eigen::Matrix4d& T)
{
  double x = 0.0, y = 0.0, z = 0.0, roll = 0.0, pitch = 0.0, yaw = 0.0;

  const char* attr = node->Attribute("xyz");
  if (attr) {
    std::stringstream s(attr);
    s >> x >> y >> z;
  }

  attr = node->Attribute("rpy");
  if (attr) {
    std::stringstream s(attr);
    s >> roll >> pitch >> yaw;
  }

  T << cos(yaw) * cos(pitch), cos(yaw) * sin(pitch) * sin(roll) - sin(yaw) * cos(roll), cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(roll), x, sin(yaw) * cos(pitch), sin(yaw) * sin(pitch) * sin(roll) + cos(yaw) * cos(roll), sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll), y, -sin(
          pitch), cos(pitch) * sin(roll), cos(pitch) * cos(roll), z, 0, 0, 0, 1;
}

#endif //DRAKE_URDFPARSINGUTIL_H_H
