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

/* A URDF-defined material consists of:

    - A normalized RGBA color (Red, Green, Blue, and Alpha values in the
      range [0, 1]) and/or
    - the absolute path to an image file to apply as a diffuse texture map.

 If both pieces of information are provided in the URDF file, both will be
 reported and it is up to the downstream consumer of this data to devise a
 policy for reconciling the data. */
struct UrdfMaterial {
  std::optional<Eigen::Vector4d> rgba;
  std::optional<std::string> diffuse_map;
};

/* A map from the name of a material to its definition.  */
typedef std::map<std::string, UrdfMaterial> MaterialMap;

/* Adds a material to the supplied `materials` map.

 If the input material is missing an rgba value, a default rgba value will be
 assigned (a fully transparent black).

 @param[in] material_name A human-understandable name of the material.
 @param[in] material The definition of the URDF material to associate with the
 name.
 @param[in] abort_if_name_clash If true, this method will abort if
 @p material_name is already in @p materials regardless of whether the
 material values are the same. If false, this method will abort if
 @p material_name is already in @p materials and material values don't
 match.
 @param[out] materials A pointer to the map in which to store the material.
 This cannot be nullptr.

 @returns The material with the given name stored in @p materials.
*/
UrdfMaterial AddMaterialToMaterialMap(const std::string& material_name,
                                      UrdfMaterial material,
                                      bool abort_if_name_clash,
                                      MaterialMap* materials);

/* Returns the material specified by a <material> @p node. If the material has
 a name associated with it, the material will be reconciled with the given
 set of @p materials (and added to the set as appropriate).

 It has the following error-throwing conditions:
    1. `name_required` is true, but the tag has no name attribute.
    2. a material with a new name (i.e. it does not appear in @p materials)
       has no color or texture information.
    3. the named material conflicts with a previous material in @p materials
       with the same name.
    4. The file referenced by a texture does not exist.

 If none of the error conditions apply, and the material has no color or
 texture information, it is defined as fully transparent black.

 @param[in] node The <material> XML node.
 @param[in] name_required If true, throws if the tag doesn't have the name
 attribute.
 @param[in] package_map A map where the keys are ROS package names and the
 values are the paths to the packages. This is only used if @p filename
 starts with "package:"or "model:".
 @param[in] root_dir The absolute path to the root directory of the URDF.
 @param[in|out] materials The set of parsed materials; a *new* material will
 be conditionally added (throwing if not possible).
 @pre @p node is a <material> node.
 @note This capability is not specified by the official URDF specification (see:
       http://wiki.ros.org/urdf/XML/link), but is needed by certain URDFs
       released by companies and organizations like Robotiq and ROS Industrial
       (for example, see this URDF by Robotiq: http://bit.ly/28P0pmo).  */
UrdfMaterial ParseMaterial(const tinyxml2::XMLElement* node, bool name_required,
                           const PackageMap& package_map,
                           const std::string& root_dir,
                           MaterialMap* materials);

/* Parses a "visual" element in @p node.

 <!-- TODO(SeanCurtis-TRI): Ultimately, a module for what we parse should be
  written outside of this _internal_ namespace. This should go there and
  merely reference it.  -->

 <h2>Targeting Renderers</h2>

 In addition to the standard SDF <visual> hierarchy, Drake offers an additional
 tag `<drake:accepting_renderer>`:

 ```
    <visual>
      <geometry ... />
      <drake:accepting_renderer name="renderer_name" />
      ...
    </visual>
 ```

 The new tag serves as a list of renderers for which this visual is targeted.

  - The _value_ of the tag is the name of the renderer.
  - If the _value_ is empty, that is a parsing error.
  - If no instance of `<drake:accepting_renderer>` every renderer will be given
    the chance to reify this visual geometry.
  - Multiple instances of this tag are allowed. Each instance adds a renderer to
    the list of targeted renderers.

 This feature is one way to provide multiple visual representations of a body.

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

/* @anchor urdf_contact_material
 Parses a <collision> element in @p node.

 Reads the definition of a collision geometry (shape, pose, properties, etc.)

 For properties, this function specifically looks for the
 `<drake:proximity_properties>` child tag to find drake-specific geometry
 collision (or "proximity") properties. It looks for the following tags with the
 mapping to properties as show below.

 All of the tags should be of the form:

 @code{xml}
 <tag value="double"/>
 @endcode

 There are not default values for these tags (except for friction -- see below);
 if the tag is missing, the corresponding property will be missing from the
 property set.

 Mapping from URDF tag to geometry property. See
 @ref YET_TO_BE_WRITTEN_HYDROELATIC_GEOMETRY_MODULE for details on the semantics
 of these properties.
 | Tag                              | Group        | Property                  | Notes                                                                                                                            |
 | :------------------------------: | :----------: | :-----------------------: | :------------------------------------------------------------------------------------------------------------------------------: |
 | drake:mesh_resolution_hint       | hydroelastic | resolution_hint           | Required for shapes that require tessellation to support hydroelastic contact.                                                   |
 | drake:hydroelastic_modulus       | hydroelastic | hydroelastic_modulus      | Finite positive value. Required for soft hydroelastic representations.                                                           |
 | drake:hunt_crossley_dissipation  | material     | hunt_crossley_dissipation |                                                                                                                                  |
 | drake:mu_dynamic                 | material     | coulomb_friction          | See note below on friction.                                                                                                      |
 | drake:mu_static                  | material     | coulomb_friction          | See note below on friction.                                                                                                      |
 | drake:rigid_hydroelastic         | hydroelastic | compliance_type           | Requests a rigid hydroelastic representation. Cannot be combined *with* soft_hydroelastic.                                       |
 | drake:soft_hydroelastic          | hydroelastic | compliance_type           | Requests a soft hydroelastic representation. Cannot be combined *with* rigid_hydroelastic. Requires a value for hydroelastic_modulus. |

 <h3>Coefficients of friction</h3>

 Parsing coefficients of friction has a relatively complicated protocol:

   1. If one of `<drake:mu_dynamic>` *or* `<drake:mu_static>` is present, the
      property of type CoulombFriction<double> will be instantiated with both
      values initialized to the single value. An exception will be thrown
        - if the value is negative.
   2. If both `<drake:mu_dynamic>` and `<drake:mu_static>` tags are present, the
      CoulombFriction<double> will contain both values. An exception will be
      thrown if:
        - either value is negative, or
        - `mu_dynamic` is greater than `mu_static`.
   3. If *both* tags are missing, the parser will look for two tags:
      `<drake_compliance><dynamic_friction>` and
      `<drake_compliance><static_friction>` with the same error conditions on
      values.
   4. If no meaningful friction coefficients are found, a default value will be
      created (see default_friction()).
 As long as no exception is thrown, the returned geometry::GeometryInstance
 will contain a valid instance of geometry::ProximityProperties with (at least)
 the ('material', 'coulomb_friction') property.

 @param[in] parent_element_name The name of the parent link element, used
 to construct default geometry names and for error reporting.
 @param[in] package_map The map used to resolve paths.
 @param[in] root_dir The root directory of the containing URDF file.
 @param[in] node The node corresponding to the <collision> tag. */
geometry::GeometryInstance ParseCollision(
    const std::string& parent_element_name,
    const PackageMap& package_map,
    const std::string& root_dir, const tinyxml2::XMLElement* node);

}  // namespace internal
}  // namespace multibody
}  // namespace drake
