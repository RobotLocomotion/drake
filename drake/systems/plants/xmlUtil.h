#pragma once

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

/**
 * Parses a three vector value from parameter \p strval. If \p strval points to
 * a single scalar value, it automatically converts it into a three vector by
 * using the same scalar value for all three dimensions.
 *
 * @param[in] strval A pointer to the character array describing a three vector
 * or a scalar value.
 * @param[out] val The three vector into which the results should be stored.
 * @return Whether the three vector was successfully parsed from \p strval.
 */
DRAKEXMLUTIL_EXPORT
bool parse_three_vector_value(const char* strval, Eigen::Vector3d& val);

/**
 * Parses a three vector value from an XML node. It also supports a single
 * scalar value, which it automatically converts to a three vector.
 *
 * @param[in] node A pointer to the XML element node that contains either a
 * three vector or a scalar value.
 * @param[out] val The three vector into which the results should be stored.
 * @return Whether the three vector was successfully parsed from the XML element
 * node.
 */
DRAKEXMLUTIL_EXPORT
bool parse_three_vector_value(tinyxml2::XMLElement* node, Eigen::Vector3d& val);

/**
 * Parses a three vector value from an XML node that contains a child XML node,
 * which contains the three vector. It also supports a single scalar value,
 * which it automatically converts to a three vector.
 *
 * @param[in] node A pointer to the XML element node that contains either a
 * three vector or a scalar value.
 * @param[in] element_name The name of the child XML element containing the
 * scale three vector or scalar value.
 * @param[out] val The three vector where the results should be stored.
 * @return Whether the three vector was successfully parsed from the XML element
 * node.
 */
DRAKEXMLUTIL_EXPORT
bool parse_three_vector_value(tinyxml2::XMLElement* node,
  const char* element_name, Eigen::Vector3d& val);

/**
 * Parses a three vector value from an XML node's attribute. It also supports a
 * single scalar value, which it automatically converts to a three vector.
 *
 * @param[in] node A pointer to the XML element node that contains an attribute
 * with a three vector or a scalar value.
 * @param[in] element_name The name of the attribute containing the three vector
 * or scalar value.
 * @param[out] val The three vector where the results should be stored.
 * @return Whether the three vector was successfully parsed from the XML element
 * node.
 */
DRAKEXMLUTIL_EXPORT
bool parse_three_vector_attribute(tinyxml2::XMLElement* node,
                               const char* element_name, Eigen::Vector3d& val);

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
