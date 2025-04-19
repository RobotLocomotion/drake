#pragma once

#include <memory>
#include <optional>
#include <string>
#include <utility>

#include <sdf/Collision.hh>
#include <sdf/Geometry.hh>
#include <sdf/Visual.hh>

#include "drake/common/diagnostic_policy.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/detail_sdf_diagnostic.h"
#include "drake/multibody/parsing/package_map.h"
#include "drake/multibody/plant/coulomb_friction.h"

namespace drake {
namespace multibody {
namespace internal {

// We convert <drake:visual> to <visual> but want to record its unique origin.
// We do so by adding an attribute to the <visual>. If present, it came from a
// drake-namespaced element, originally. (The value of the attribute is
// irrelevant.)
constexpr char kIsDrakeNamespaceAttr[] = "drake-namespaced";

/* Used for resolving URIs / filenames.  */
using ResolveFilename =
    std::function<std::string(const SDFormatDiagnostic&, std::string)>;

/* Given an sdf::Geometry object representing a <geometry> element from an SDF
 file, this method makes a new drake::geometry::Shape object from this
 specification.
 If no recognizable geometry is specified or the geometry is recognized but
 malformed, nullptr is returned. In the case of a malformed specification, an
 error is also emitted on `diagnostic`. */
std::unique_ptr<geometry::Shape> MakeShapeFromSdfGeometry(
    const SDFormatDiagnostic& diagnostic, const sdf::Geometry& sdf_geometry,
    ResolveFilename resolve_filename);

/* Given an sdf::Visual object representing a <visual> element from an SDF
 file, this method makes a new drake::geometry::GeometryInstance object from
 this specification at a pose `X_LG` relative to its parent link.

 This method returns null when:
   1. the given SDF specification corresponds to an uninterpreted geometry type:
      - `sdf::GeometryType::EMPTY` (`<empty/>` SDF tag.)
      - `sdf::GeometryType::HEIGHTMAP` (`<heightmap/>` SDF tag.)
   2. the <visual> has had both of its Drake roles disabled.
   3. the <visual> specification was in some way malformed.

 In the case of a malformed specification, an error is also emitted to
 `diagnostic`.

 <!-- TODO(SeanCurtis-TRI): Ultimately, a module for what we parse should be
  written outside of this _internal_ namespace. This should go there and
  merely reference it.  -->

 <h2>Controlling Drake visual roles</h2>

 SDFormat <visual> tags get illustration and perception roles by default. We
 can opt out of those roles and even use different geometries for different
 roles. See, @ref tag_drake_perception_properties,
 @ref tag_drake_illustration_properties, and @ref tag_drake_visual in
 parsing_doxygen.h for more details.

 <h2>Targeting Renderers</h2>

 In addition to the standard SDF <visual> hierarchy, Drake offers an additional
 tag `<drake:accepting_renderer>`:

 ```
    <visual>
      <geometry ... />
      <drake:accepting_renderer>renderer_name</drake:accepting_renderer>
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
 */
std::unique_ptr<geometry::GeometryInstance> MakeGeometryInstanceFromSdfVisual(
    const SDFormatDiagnostic& diagnostic, const sdf::Visual& sdf_visual,
    ResolveFilename resolve_filename, const math::RigidTransformd& X_LG);

/* The visual properties implied by the <visual> tag. It is possible that zero,
 one, or two sets of properties are defined. */
struct VisualProperties {
  std::optional<geometry::IllustrationProperties> illustration;
  std::optional<geometry::PerceptionProperties> perception;
};

// clang-format off
/* Extracts the material properties from the given sdf::Visual object.
 The sdf::Visual object represents a corresponding <visual> tag from an SDF
 file. The material properties are placed into both a
 geometry::IllustrationProperties and geometry::PerceptionProperties as follows:

 <!-- NOTE: Lines longer than 80 columns required for the doxygen tables. -->
 | Group |    Name     |      Type       | Description |
 | :---: | :---------: | :-------------: | :---------- |
 | phong | diffuse     | Vector4<double> | The normalized rgba values for the diffuse color |
 | phong | ambient     | Vector4<double> | The normalized rgba values for the ambient color |
 | phong | specular    | Vector4<double> | The normalized rgba values for the specular color |
 | phong | emissive    | Vector4<double> | The normalized rgba values for the emissive color |
 | phong | diffuse_map | string          | A resolvable URI to a png image that will be applied as a diffuse map. |

 These are properties to be used in the
 <a href="https://en.wikipedia.org/wiki/Phong_reflection_model">Phong
 lighting model</a> and are taken from the similarly named property tags (see
 below). If any of `ambient`, `diffuse`, `specular`, or `emissive` tags are
 missing, that property will be omitted from the property set and the impact
 of those missing properties will depend on the downstream consumers documented
 handling of missing parameters. If any of the tags are malformed, sdformat
 will treat those properties as "missing" and they will not be included in
 the property set.

 <!-- TODO(SeanCurtis-TRI): Consider changing the behavior for unspecified tags
 to provide the material value as specified in the current (12/11/18) sdf
 specification (http://sdformat.org/spec?ver=1.6&elem=material).
 This is captured in the issue:
 https://github.com/RobotLocomotion/drake/issues/10193 -->

 The material properties come from the child <material> tag. E.g.,
 @code{xml}
 <visual>
   <geometry>
   ...
   </geometry>
   <material>
     <ambient>r_a g_a b_a a_a</ambient>
     <diffuse>r_d g_d b_d a_d</diffuse>
     <specular>r_s g_s b_s a_s</specular>
     <emissive>r_e g_e b_e a_e</emissive>
   </material>
 </visual>
 @endcode

 An instance of geometry::IllustrationProperties will be returned. If there is
 no material tag, no material property tags, or no successfully parsed material
 property tags, the property set will be empty. If the material is malformed
 an error will be emitted. If the error policy is not set to throw, an
 std::nullopt will be returned. */
// clang-format on
VisualProperties MakeVisualPropertiesFromSdfVisual(
    const SDFormatDiagnostic& diagnostic, const sdf::Visual& sdf_visual,
    ResolveFilename resolve_filename);

/* Computes the pose `X_LC` of frame C (the "canonical frame" of the geometry)
 relative to the link L containing the collision, given an `sdf_collision`
 stemming from the parsing of a `<collision>` element in an SDF file and its
 pose `X_LG`, where `G` represents the frame for the geometry of that collision
 element.  */
math::RigidTransformd MakeGeometryPoseFromSdfCollision(
    const sdf::Collision& sdf_collision, const math::RigidTransformd& X_LG);

// clang-format off
/* @anchor sdf_contact_material
 Parses the drake-relevant collision properties from a <collision> element.

 Specifically, looks for <drake:proximity_properties> tag to find drake-specific
 geometry collision (or "proximity") properties. The set of tags are enumerated
 in the table below. Each tag should be of the form:

 @code{xml}
   <tag>real_value</tag>
 @endcode

 It returns a valid instance of ProximityProperties. There are not default
 values for these tags (except for friction -- see below); if the tag is
 missing, the corresponding property will be missing from the property set.
 If the collision element is malformed it emits an error. If the error
 policy is set to not throw an std::nullopt is returned instead.

 Mapping from SDF tag to geometry property. See @ref hydroelastic_user_guide
 for details on the semantics of these properties.

 | Tag                              | Group        | Property                  | Notes                                                                                                                            |
 | :------------------------------: | :----------: | :-----------------------: | :------------------------------------------------------------------------------------------------------------------------------: |
 | drake:mesh_resolution_hint       | hydroelastic | resolution_hint           | Required for shapes that require tessellation to support hydroelastic contact.                                                   |
 | drake:hydroelastic_modulus       | hydroelastic | hydroelastic_modulus      | Finite positive value. Required for compliant hydroelastic representations.                                                      |
 | drake:hydroelastic_margin        | hydroelastic | margin                    | Finite nonnegative value. Optional.
 | drake:hunt_crossley_dissipation  | material     | hunt_crossley_dissipation |                                                                                                                                  |
 | drake:relaxation_time            | material     | relaxation_time           | Required when using a linear model of dissipation, for instance with the SAP solver.                                             |
 | drake:mu_dynamic                 | material     | coulomb_friction          | See note below on friction.                                                                                                      |
 | drake:mu_static                  | material     | coulomb_friction          | See note below on friction.                                                                                                      |
 | drake:rigid_hydroelastic         | hydroelastic | compliance_type           | Requests a rigid hydroelastic representation. Cannot be combined *with* soft_hydroelastic.                                       |
 | drake:compliant_hydroelastic     | hydroelastic | compliance_type           | Requests a compliant hydroelastic representation. Cannot be combined *with* rigid_hydroelastic. Requires a value for hydroelastic_modulus. |

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
   3. If *both* tags are missing, the parser will look for two coefficients
      in the SDF tag path: `<surface><friction><ode><mu>` and
      `<surface><friction><ode><mu2>`.
        a. See MakeCoulombFrictionFromSdfCollisionOde() for failure modes.
   4. If no meaningful friction coefficients are found, a default value will be
      created (see default_friction()).
 As long as no exception is thrown, the resulting ProximityProperties will have
 the ('material', 'coulomb_friction') property.  */
// clang-format on
std::optional<geometry::ProximityProperties>
MakeProximityPropertiesForCollision(const SDFormatDiagnostic& diagnostic,
                                    const sdf::Collision& sdf_collision);

/* Parses friction coefficients from `sdf_collision`.
 This method looks for the definitions specific to ODE, as given by the SDF
 specification in `<collision><surface><friction><ode>`. Drake understands
 `<mu>` as the static coefficient of friction and `<mu2>` as the dynamic
 coefficient of friction. Consider the example below:
 ```xml
   <collision>
     <surface>
       <friction>
         <ode>
           <mu>0.8</mu>
           <mu2>0.3</mu2>
         </ode>
       </friction>
     </surface>
   </collision>
 ```
 If mu or mu2 are not found, an error is emitted. If the error policy is not
 set to throw an std::nullopt is returned. */
std::optional<CoulombFriction<double>> MakeCoulombFrictionFromSdfCollisionOde(
    const SDFormatDiagnostic& diagnostic, const sdf::Collision& sdf_collision);

// Returns proximity properties from <collision>. Supports only
// the following children elements of <drake:proximity_properties>:
//  - <drake:mu_dynamic>
//  - <drake:hunt_crossley_dissipation>
//  - <drake:relaxation_time>.
//
// Returns nullopt if nothing specified or on error.
std::optional<geometry::ProximityProperties>
MakeProximityForDeformableCollision(const SDFormatDiagnostic& diagnostic,
                                    const sdf::Collision& collision);

}  // namespace internal
}  // namespace multibody
}  // namespace drake
