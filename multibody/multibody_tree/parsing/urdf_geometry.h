#pragma once

#include <functional>
#include <map>
#include <string>
#include <utility>

#include <Eigen/Dense>
#include <tinyxml2.h>

#include "drake/geometry/geometry_instance.h"
#include "drake/multibody/multibody_tree/multibody_plant/coulomb_friction.h"
#include "drake/multibody/multibody_tree/parsing/package_map.h"

namespace drake {
namespace multibody {
namespace parsing {
namespace detail {

/// A map from the name of a material to its color. The color is specified in
/// RGBA (Red, Green, Blue, Alpha) format.
// TODO(liang.fok) Add support for texture-based materials, see #2588.
typedef std::map<std::string, Eigen::Vector4d, std::less<std::string>,
                 Eigen::aligned_allocator<
                   std::pair<const std::string,
                             Eigen::Vector4d>>> MaterialMap;

/// Parses a "material" element in @p node and adds the result to @p
/// materials.
///
/// @throws std::runtime_error if the material is missing required attributes
/// or if it was already defined with different properties.
void ParseMaterial(tinyxml2::XMLElement* node, MaterialMap* materials);

/// Parses a "visual" element in @p node.
geometry::GeometryInstance ParseVisual(
    const std::string& element_name, const PackageMap& package_map,
    const std::string& root_dir, tinyxml2::XMLElement* node,
    MaterialMap* materials);

/// Parses a "collision" element in @p node.
///
/// @param[out] friction Coulomb friction for the associated geometry.
geometry::GeometryInstance ParseCollision(
    const std::string& element_name, const PackageMap& package_map,
    const std::string& root_dir, tinyxml2::XMLElement* node,
    multibody_plant::CoulombFriction<double>* friction);

}  /// namespace detail
}  /// namespace parsing
}  /// namespace multibody
}  /// namespace drake
