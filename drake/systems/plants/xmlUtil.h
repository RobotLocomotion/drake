#ifndef DRAKE_SYSTEMS_PLANTS_XMLUTIL_H_
#define DRAKE_SYSTEMS_PLANTS_XMLUTIL_H_

#include <string>
#include <map>
#include <Eigen/Dense>

#include "spruce.hh"
#include "drake/systems/plants/pose_map.h"
#include "drake/thirdParty/tinyxml2/tinyxml2.h"
#include "drake/drakeXMLUtil_export.h"

template <typename Scalar>
bool parseScalarValue(tinyxml2::XMLElement* node, Scalar& val) {
  const char* strval = node->FirstChild()->Value();
  if (strval) {
    std::stringstream s(strval);
    s >> val;
    return true;
  }
  return false;
}

template <typename Scalar>
bool parseScalarValue(tinyxml2::XMLElement* node, const char* element_name,
                      Scalar& val) {
  tinyxml2::XMLElement* elnode = node->FirstChildElement(element_name);
  if (elnode) return parseScalarValue(elnode, val);
  return false;
}

template <typename Scalar>
bool parseScalarAttribute(tinyxml2::XMLElement* node,
                          const char* attribute_name, Scalar& val) {
  const char* attr = node->Attribute(attribute_name);
  if (attr) {
    std::stringstream s(attr);
    s >> val;
    return true;
  }
  return false;
}

// only writes values if they exist
DRAKEXMLUTIL_EXPORT bool parseVectorAttribute(const tinyxml2::XMLElement* node,
                                              const char* attribute_name,
                                              Eigen::Vector3d& val);
DRAKEXMLUTIL_EXPORT bool parseVectorAttribute(const tinyxml2::XMLElement* node,
                                              const char* attribute_name,
                                              Eigen::Vector4d& val);
DRAKEXMLUTIL_EXPORT bool parseVectorValue(tinyxml2::XMLElement* node,
                                          const char* element_name,
                                          Eigen::Vector3d& val);
DRAKEXMLUTIL_EXPORT bool parseVectorValue(tinyxml2::XMLElement* node,
                                          const char* element_name,
                                          Eigen::Vector4d& val);
DRAKEXMLUTIL_EXPORT bool parseStringValue(tinyxml2::XMLElement* node,
                                          const char* element_name,
                                          std::string& val);

DRAKEXMLUTIL_EXPORT void originAttributesToTransform(tinyxml2::XMLElement* node,
                                                     Eigen::Isometry3d& T);

DRAKEXMLUTIL_EXPORT void poseValueToTransform(
    tinyxml2::XMLElement* node, const PoseMap& pose_map, Eigen::Isometry3d& T,
    const Eigen::Isometry3d& T_default_frame = Eigen::Isometry3d::Identity());

typedef std::map<std::string, std::string> PackageMap;
DRAKEXMLUTIL_EXPORT void populatePackageMap(PackageMap& package_map);
DRAKEXMLUTIL_EXPORT std::string resolveFilename(const std::string& filename,
                                                const PackageMap& package_map,
                                                const std::string& root_dir);

#endif  // DRAKE_SYSTEMS_PLANTS_XMLUTIL_H_
