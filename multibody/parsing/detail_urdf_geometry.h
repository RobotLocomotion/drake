#pragma once

#include <functional>
#include <map>
#include <optional>
#include <string>
#include <utility>

#include <Eigen/Dense>
#include <tinyxml2.h>

#include "drake/geometry/geometry_instance.h"
#include "drake/multibody/parsing/package_map.h"
#include "drake/multibody/plant/coulomb_friction.h"

namespace drake {
namespace multibody {
namespace internal {

/// A URDF-defined material consists of:
///
///    - A normalized RGBA color (Red, Green, Blue, and Alpha values in the
///      range [0, 1]) and/or
///    - the absolute path to an image file to apply as a diffuse texture map.
///
/// If both pieces of information are provided in the URDF file, both will be
/// reported and it is up to the downstream consumer of this data to devise a
/// policy for reconciling the data.
struct UrdfMaterial {
  std::optional<Eigen::Vector4d> rgba;
  std::optional<std::string> diffuse_map;
};

/// A map from the name of a material to its definition.
typedef std::map<std::string, UrdfMaterial> MaterialMap;

/// Utility function that extracts the material defined by a <material> @p node.
/// Returns the specified material. If the material has a name associated with
/// it, the material will be reconciled with the given set of @p materials
/// (added as appropriate).
///
/// It has the following error conditions (which throw exceptions):
///    1. a material name is required, but none is given.
///    2. a material with a new name (i.e. it does not appear in @p materials,
///       has no color or texture information.
///    3. the named material conflicts with a previous material with the same
///       name.
///
/// If none of the error conditions apply, and the material has no color or
/// texture information, it is defined as the color black.
///
/// @param[in] node The <material> XML node.
/// @param[in] name_required If true, throws if the tag doesn't have the name
/// attribute.
/// @param[in] package_map A map where the keys are ROS package names and the
/// values are the paths to the packages. This is only used if @p filename
/// starts with "package:"or "model:".
/// @param[in] root_dir The absolute path to the root directory of the URDF.
/// @param[in|out] materials The set of parsed materials; the new material will
/// be added if it can be (throws otherwise).
UrdfMaterial ParseMaterial(const tinyxml2::XMLElement* node, bool name_required,
                           const PackageMap& package_map,
                           const std::string& root_dir,
                           MaterialMap* materials);

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

}  /// namespace internal
}  /// namespace multibody
}  /// namespace drake
