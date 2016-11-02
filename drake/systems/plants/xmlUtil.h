#pragma once

#include <map>
#include <string>

#include <Eigen/Dense>

#include "drake/common/drake_export.h"
#include "drake/systems/plants/pose_map.h"
#include "drake/thirdParty/zlib/tinyxml2/tinyxml2.h"

template <typename Scalar>
// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
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
                      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                      Scalar& val) {
  tinyxml2::XMLElement* elnode = node->FirstChildElement(element_name);
  if (elnode) return parseScalarValue(elnode, val);
  return false;
}

template <typename Scalar>
bool parseScalarAttribute(tinyxml2::XMLElement* node,
                          // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
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
 * Parses a three vector value from parameter \p strval. There are two formats
 * of \p strval that can be successfully parsed. The first format is
 * "val1 val2 val3" where val1, val2, and val3 are double values. The second
 * valid format is "val" where val is a double type. In this case, this method
 * automatically converts the val1 scalar value into a three vector by
 * using the same scalar value for all three dimensions.
 *
 * @param[in] strval A pointer to the character array describing a three vector
 * or a scalar value.
 * @param[out] val The three vector into which the results should be stored.
 * @return Whether the three vector was successfully parsed from \p strval.
 * @throws std::invalid_argument If any problem is encountered parsing the three
 * vector value.
 */
DRAKE_EXPORT
void ParseThreeVectorValue(const char* strval, Eigen::Vector3d* val);

/**
 * Parses a three vector value from parameter \p node, which is an XML node.
 * It also supports a single scalar value, which it automatically converts to a
 * three vector by using the same scalar value for all three dimensions.
 *
 * @param[in] node A pointer to the XML element node that contains either a
 * three vector or a scalar value.
 * @param[out] val The three vector into which the results should be stored.
 * @return Whether the three vector was successfully parsed from the XML element
 * node.
 * @throws std::invalid_argument If any problem is encountered parsing the three
 * vector value.
 */
DRAKE_EXPORT
void ParseThreeVectorValue(const tinyxml2::XMLElement* node,
                           Eigen::Vector3d* val);

/**
 * Parses a three vector value from parameter \p node, which is an XML node.
 * The value is contained in an element within \p node, as specified by
 * parameter \p element_name. This method also supports a three vector specified
 * by a single scalar value, which it automatically converts into a three vector
 * by using the same scalar value for all three dimensions.
 *
 * @param[in] node A pointer to the XML element node that contains either a
 * three vector or a scalar value.
 * @param[in] element_name The name of the child XML element containing the
 * scale three vector or scalar value.
 * @param[out] val The three vector where the results should be stored.
 * @return Whether the three vector was successfully parsed from the XML element
 * node.
 * @throws std::invalid_argument If any problem is encountered parsing the three
 * vector value.
 */
DRAKE_EXPORT
void ParseThreeVectorValue(const tinyxml2::XMLElement* node,
                           const char* element_name, Eigen::Vector3d* val);

/**
 * Parses a three vector value from parameter \p node, which is an XML node. The
 * value is specified by an attribute within the XML whose name is apecified by
 * parameter \p attribute_name.This method also supports a three vector
 * specified
 * by a single scalar value, which it automatically converts into a three vector
 * by using the same scalar value for all three dimensions.
 *
 * @param[in] node A pointer to the XML element node that contains an attribute
 * with a three vector or a scalar value.
 * @param[in] attribute_name The name of the attribute containing the three
 * vector or scalar value.
 * @param[out] val The three vector where the results should be stored.
 * @return Whether the three vector was successfully parsed from the XML element
 * node.
 * @throws std::invalid_argument If any problem is encountered parsing the three
 * vector value.
 */
DRAKE_EXPORT
void ParseThreeVectorAttribute(const tinyxml2::XMLElement* node,
                               const char* attribute_name,
                               Eigen::Vector3d* val);

/**
 * Converts a string to a double value.
 *
 * @param[in] str A pointer to a string containing a representation of a double
 * value.
 * @return The corresponding double value that was represented in \p str.
 * @throws std::invalid_argument If any problem is encountered while parsing the
 * double value represented within \p str.
 */
DRAKE_EXPORT
double StringToDouble(const std::string& str);

// only writes values if they exist
DRAKE_EXPORT bool parseVectorAttribute(
    const tinyxml2::XMLElement* node,
    const char* attribute_name,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    Eigen::Vector3d& val);
DRAKE_EXPORT bool parseVectorAttribute(
    const tinyxml2::XMLElement* node,
    const char* attribute_name,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    Eigen::Vector4d& val);
DRAKE_EXPORT bool parseVectorValue(
    tinyxml2::XMLElement* node,
    const char* element_name,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    Eigen::Vector3d& val);
DRAKE_EXPORT bool parseVectorValue(
    tinyxml2::XMLElement* node,
    const char* element_name,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    Eigen::Vector4d& val);
DRAKE_EXPORT bool parseStringValue(
    tinyxml2::XMLElement* node,
    const char* element_name,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    std::string& val);

DRAKE_EXPORT void originAttributesToTransform(
    tinyxml2::XMLElement* node,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    Eigen::Isometry3d& T);

DRAKE_EXPORT void poseValueToTransform(
    tinyxml2::XMLElement* node, const PoseMap& pose_map,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    Eigen::Isometry3d& T,
    const Eigen::Isometry3d& T_default_frame = Eigen::Isometry3d::Identity());

typedef std::map<std::string, std::string> PackageMap;
// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
DRAKE_EXPORT void populatePackageMap(PackageMap& package_map);

/// Resolves the fully-qualified name of a file. If @p filename starts with
/// "package:", the ROS packages specified in @p package_map are searched.
/// Otherwise, @p filename is appended to the end of @p root_dir and checked
/// for existence. If the file does not exist or is not found, a warning is
/// printed to `std::cerr` and an empty string is returned.
///
/// @param[in] filename The name of the file to find.
///
/// @param[in] package_map A map where the keys are ROS package names and the
/// values are the paths to the packages. This is only used if @p filename
/// starts with "package:".
///
/// @param[in] root_dir The root directory to look in. This is only used when
/// assuming @p filename does not start with "package:".
///
/// @return The file's fully-qualified name or an empty string if the file is
/// not found or does not exist.
DRAKE_EXPORT std::string resolveFilename(const std::string& filename,
                                                const PackageMap& package_map,
                                                const std::string& root_dir);
