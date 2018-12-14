#pragma once

#include <functional>
#include <map>
#include <string>
#include <utility>

#include <Eigen/Dense>
#include <tinyxml2.h>

#include "drake/geometry/geometry_instance.h"
#include "drake/multibody/parsing/package_map.h"
#include "drake/multibody/plant/coulomb_friction.h"

namespace drake {
namespace multibody {
namespace detail {

/// A map from the name of a material to its color. The color is specified in
/// RGBA (Red, Green, Blue, Alpha) format.
// TODO(sammy-tri) Add support for texture-based materials, see #2588.
typedef std::map<std::string, Eigen::Vector4d> MaterialMap;

/// Parses a "material" element in @p node and adds the result to @p
/// materials.
///
/// @throws std::runtime_error if the material is missing required attributes
/// or if it was already defined with different properties.
void ParseMaterial(const tinyxml2::XMLElement* node, MaterialMap* materials);

/// Parses a "visual" element in @p node.
///
/// @param[in] parent_element_name The name of the parent link element, used
/// to construct default geometry names and for error reporting.
/// @param[in,out] materials The MaterialMap is used to look up materials
/// which are referenced by name only in the visual element.  New materials
/// which are specified by both color and name will be added to the map and
/// can be used by later visual elements.  Material definitions may be
/// repeated if the material properties are identical.
geometry::GeometryInstance ParseVisual(
    const std::string& parent_element_name,
    const PackageMap& package_map,
    const std::string& root_dir, const tinyxml2::XMLElement* node,
    MaterialMap* materials);

/// Parses a "collision" element in @p node.
///
/// @param[in] parent_element_name The name of the parent link element, used
/// to construct default geometry names and for error reporting.
/// @param[out] friction Coulomb friction for the associated geometry.
geometry::GeometryInstance ParseCollision(
    const std::string& parent_element_name,
    const PackageMap& package_map,
    const std::string& root_dir, const tinyxml2::XMLElement* node,
    CoulombFriction<double>* friction);

}  /// namespace detail
}  /// namespace multibody
}  /// namespace drake
