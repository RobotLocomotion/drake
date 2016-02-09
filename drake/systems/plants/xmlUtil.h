#ifndef DRAKE_URDFPARSINGUTIL_H_H
#define DRAKE_URDFPARSINGUTIL_H_H

#include <string>
#include <map>
#include <Eigen/Dense>

#include "spruce.hh"
#include "drake/thirdParty/tinyxml2/tinyxml2.h"
#include "drake/drakeXMLUtil_export.h"

DRAKEXMLUTIL_EXPORT bool parseScalarValue(tinyxml2::XMLElement* node, double &val);
DRAKEXMLUTIL_EXPORT bool parseScalarValue(tinyxml2::XMLElement* node, const char* element_name, double &val);
DRAKEXMLUTIL_EXPORT bool parseScalarAttribute(tinyxml2::XMLElement* node, const char* attribute_name, double& val);

// only writes values if they exist
DRAKEXMLUTIL_EXPORT bool parseVectorAttribute(tinyxml2::XMLElement* node, const char* attribute_name, Eigen::Vector3d &val);
DRAKEXMLUTIL_EXPORT bool parseVectorAttribute(tinyxml2::XMLElement* node, const char* attribute_name, Eigen::Vector4d &val);
DRAKEXMLUTIL_EXPORT bool parseVectorValue(tinyxml2::XMLElement* node, const char* element_name, Eigen::Vector3d &val);

DRAKEXMLUTIL_EXPORT void originAttributesToTransform(tinyxml2::XMLElement *node, Eigen::Isometry3d &T);

typedef std::map<std::string, Eigen::Isometry3d, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Eigen::Isometry3d> > > PoseMap;
DRAKEXMLUTIL_EXPORT void poseValueToTransform(tinyxml2::XMLElement *node, const PoseMap& pose_map, Eigen::Isometry3d &T, const Eigen::Isometry3d& T_default_frame = Eigen::Isometry3d::Identity());

DRAKEXMLUTIL_EXPORT bool parseStringValue(tinyxml2::XMLElement* node, const char* element_name, std::string &val);

DRAKEXMLUTIL_EXPORT void populatePackageMap(std::map<std::string,std::string>& package_map);
DRAKEXMLUTIL_EXPORT std::string resolveFilename(const std::string& filename, const std::map<std::string,std::string>& package_map, const std::string& root_dir);


#endif //DRAKE_URDFPARSINGUTIL_H_H
