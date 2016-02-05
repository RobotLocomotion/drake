#ifndef DRAKE_URDFPARSINGUTIL_H_H
#define DRAKE_URDFPARSINGUTIL_H_H

#include <string>
#include <map>
#include <Eigen/Dense>

#include "spruce.hh"
#include "drake/thirdParty/tinyxml2/tinyxml2.h"
#include "drake/drakeXMLUtil_export.h"

DRAKEXMLUTIL_EXPORT bool parseScalarValue(tinyxml2::XMLElement* node, double &val);
DRAKEXMLUTIL_EXPORT bool parseScalarAttribute(tinyxml2::XMLElement* node, const char* attribute_name, double& val);

// only writes values if they exist
DRAKEXMLUTIL_EXPORT bool parseVectorAttribute(tinyxml2::XMLElement* node, const char* attribute_name, Eigen::Vector3d &val);
DRAKEXMLUTIL_EXPORT bool parseVectorAttribute(tinyxml2::XMLElement* node, const char* attribute_name, Eigen::Vector4d &val);

DRAKEXMLUTIL_EXPORT void poseAttributesToTransform(tinyxml2::XMLElement* node, Eigen::Isometry3d& T);

DRAKEXMLUTIL_EXPORT void populatePackageMap(std::map<std::string,std::string>& package_map);
DRAKEXMLUTIL_EXPORT std::string resolveFilename(const std::string& filename, const std::map<std::string,std::string>& package_map, const std::string& root_dir);


#endif //DRAKE_URDFPARSINGUTIL_H_H
