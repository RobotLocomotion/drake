#pragma once

#include <memory>
#include <string>

#include <sdf/sdf.hh>

#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/package_map.h"
#include "drake/multibody/plant/coulomb_friction.h"

namespace drake {
namespace multibody {
namespace internal {

// Used for resolving URIs / filenames.
using ResolveFilename = std::function<std::string (std::string)>;

/** Given an sdf::Geometry object representing a <geometry> element from an SDF
 file, this method makes a new drake::geometry::Shape object from this
 specification.
 For `sdf_geometry.Type() == sdf::GeometryType::EMPTY`, corresponding to the
 <empty/> SDF tag, it returns `nullptr`.  */
std::unique_ptr<geometry::Shape> MakeShapeFromSdfGeometry(
    const sdf::Geometry& sdf_geometry, ResolveFilename resolve_filename);

/** Given an sdf::Visual object representing a <visual> element from an SDF
 file, this method makes a new drake::geometry::GeometryInstance object from
 this specification.
 This method returns nullptr when the given SDF specification corresponds
 to a geometry of type `sdf::GeometryType::EMPTY` (`<empty/>` SDF tag.)  */
std::unique_ptr<geometry::GeometryInstance> MakeGeometryInstanceFromSdfVisual(
    const sdf::Visual& sdf_visual, ResolveFilename resolve_filename,
    const math::RigidTransformd& X_LG);

/** Extracts the material properties from the given sdf::Visual object.
 The sdf::Visual object represents a corresponding <visual> tag from an SDF
 file. The material properties are placed into a
 geometry::IllustrationProperties as follows:

 <!-- NOTE: Lines longer than 80 columns required for the doxygen tables. -->
 | Group |   Name   |      Type       | Description |
 | :---: | :------: | :-------------: | :---------- |
 | phong | diffuse  | Vector4<double> | The normalized rgba values for the diffuse color |
 | phong | ambient  | Vector4<double> | The normalized rgba values for the ambient color |
 | phong | specular | Vector4<double> | The normalized rgba values for the specular color |
 | phong | emissive | Vector4<double> | The normalized rgba values for the emissive color |

 These are properties to be used in the
 <a href="https://en.wikipedia.org/wiki/Phong_reflection_model">Phong
 lighting model</a> and are taken from the similarly named property tags (see
 below). If any of `ambient`, `diffuse`, `specular`, or `emissive` tags are
 missing, that property will be omitted from the property set and the impact
 of those missing properties will depend on the downstream consumers documented
 handling of missing parameters.

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

 An instance of geometry::IllustrationProperties will always be returned. If
 there is no material tag, no material property tags, or no successfully
 parsed material property tags, the property set will be empty.  */
geometry::IllustrationProperties MakeVisualPropertiesFromSdfVisual(
    const sdf::Visual& sdf_visual);

/** Given `sdf_collision` stemming from the parsing of a `<collision>` element
 in an SDF file, this method makes the pose `X_LG` of frame G for the geometry
 of that collision element in the frame L of the link it belongs to.  */
math::RigidTransformd MakeGeometryPoseFromSdfCollision(
    const sdf::Collision& sdf_collision, const math::RigidTransformd& X_LG);

/** Parses the drake-relevant collision properties from a <collision> element.

 Specifically, looks for <drake:proximity_properties> tag to find drake-specific
 geometry collision (or "proximity") properties. The set of tags are enumerated
 in the table below. Each tag should be of the form:

 @code{xml}
   <tag>real_value</tag>
 @endcode

 As long as no exceptions are thrown, the function is guaranteed to return
 a valid instance of ProximityProperties.

 Mapping from SDF tag to geometry property. See
 @ref YET_TO_BE_WRITTEN_HYDROELATIC_GEOMETRY_MODULE for details on the semantics
 of these properties.
 | Tag                              | Group        | Property                  | Notes                                           |
 | :------------------------------: | :----------: | :-----------------------: | :---------------------------------------------: |
 | drake:mesh_resolution_hint       | hydroelastic | resolution_hint           | Required for hydroelastic contact.              |
 | drake:elastic_modulus            | material     | elastic_modulus           | ∞ for rigid hydrolelastic models; < ∞ for soft. |
 | drake:hunt_crossley_dissipation  | material     | hunt_crossley_dissipation |                                                 |
 | drake:mu_dynamic                 | material     | coulomb_friction          | See note below on friction.                     |
 | drake:mu_static                  | material     | coulomb_friction          | See note below on friction.                     |

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
geometry::ProximityProperties MakeProximityPropertiesForCollision(
        const sdf::Collision& sdf_collision);

/** Parses friction coefficients from `sdf_collision`.
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
 If a `<surface>` is not found, it returns arbitrary default coefficients
 (typically mu = mu2 = 1). If `<surface>` is found, all other nested elements
 are required and an exception is thrown if not present.  */
CoulombFriction<double> MakeCoulombFrictionFromSdfCollisionOde(
    const sdf::Collision& sdf_collision);

}  // namespace internal
}  // namespace multibody
}  // namespace drake
