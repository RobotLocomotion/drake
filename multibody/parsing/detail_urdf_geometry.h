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
namespace internal {

// TODO(sammy-tri) Add support for texture-based materials, see #2588.
/** A map from the name of a material to its color. The color is specified in
 RGBA (Red, Green, Blue, Alpha) format. */
typedef std::map<std::string, Eigen::Vector4d> MaterialMap;

/** Parses a "material" element in @p node and adds the result to @p materials.

 @throws std::runtime_error if the material is missing required attributes
 or if it was already defined with different properties. */
void ParseMaterial(const tinyxml2::XMLElement* node, MaterialMap* materials);

/** Parses a "visual" element in @p node.

 @param[in] parent_element_name The name of the parent link element, used
 to construct default geometry names and for error reporting.
 @param[in,out] materials The MaterialMap is used to look up materials
 which are referenced by name only in the visual element.  New materials
 which are specified by both color and name will be added to the map and
 can be used by later visual elements.  Material definitions may be
 repeated if the material properties are identical. */
geometry::GeometryInstance ParseVisual(
    const std::string& parent_element_name,
    const PackageMap& package_map,
    const std::string& root_dir, const tinyxml2::XMLElement* node,
    MaterialMap* materials);

/** Parses a <collision> element in @p node.

 Reads the definition of a collision geometry (shape, pose, properties, etc.)

 For properties, this function specifically looks for the <drake:drake> child
 tag to find drake-specific geometry collision (or "proximity") properties. It
 looks for the following tags with the mapping to properties as show below.

 All of the tags should be of the form:

   `<tag value="double"/>`

 Mapping from URDF tag to geometry property.
 | Tag                                | Group        | Property                  | Notes                                          |
 | :--------------------------------: | :----------: | :-----------------------: | :--------------------------------------------: |
 | drake:hydroelastic_resolution_hint | hydroelastic | resolution_hint           | Required for hydroelastic contact.             |
 | drake:elastic_modulus              | material     | elastic_modulus           | ∞ for rigid hydroleastic models; < ∞ for soft. |
 | drake:dissipation                  | material     | hunt_crossley_dissipation |                                                |
 | drake:mu_dynamic                   | material     | coulomb_friction          | See note below on friction.                    |

 <h3>Coefficients of friction</h3>

 Parsing coefficients of friction has a relatively complicated protocol:

   1. If the `<drake:mu_dynamic>` tag is present, it will be used to instantiate
      a property with a value of type CoulombFriction<double> (where both
      coefficients are equal to the parsed value).
        a. If the value is negative, an exception will be thrown.
   2. If there is no `<drake:mu_dynamic>` tag, it will look for two tags:
      `<drake_compliance><dynamic_friction>` and
      `<drake_compliance><static_friction>`. The `CoulombFriction<double> value
      will be instantiated from the two values. Exceptions will be thrown in the
      following cases:
        a. The tags `<dynamic_friction>` or `<static_friction>` exist but its
        contents isn't real valued.
        b. Only one of the two tags is defined.
    3. If there is no `<drake_compliance>` definition of friction coefficients,
       a default value will be created.

 As long as no exception is thrown, the resulting GeometryInstance will have
 proximity properties that are guaranteed to include ('material',
 'coulomb_friction').

 @param[in] parent_element_name The name of the parent link element, used
 to construct default geometry names and for error reporting.
 @param[in] package_map The map used to resolve paths.
 @param[in] root_dir The root directory of the containing URDF file.
 @param[in] node The node corresponding to the <collision> tag. */
geometry::GeometryInstance ParseCollision(
    const std::string& parent_element_name,
    const PackageMap& package_map,
    const std::string& root_dir, const tinyxml2::XMLElement* node);

}  /// namespace internal
}  /// namespace multibody
}  /// namespace drake
