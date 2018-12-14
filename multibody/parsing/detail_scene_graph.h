#pragma once

#include <memory>
#include <string>

#include <sdf/sdf.hh>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/package_map.h"
#include "drake/multibody/plant/coulomb_friction.h"

namespace drake {
namespace multibody {
namespace detail {

/** Given an sdf::Geometry object representing a <geometry> element from an SDF
 file, this method makes a new drake::geometry::Shape object from this
 specification.
 For `sdf_geometry.Type() == sdf::GeometryType::EMPTY`, corresponding to the
 <empty/> SDF tag, it returns `nullptr`.  */
std::unique_ptr<geometry::Shape> MakeShapeFromSdfGeometry(
    const sdf::Geometry& sdf_geometry);

/** Given an sdf::Visual object representing a <visual> element from an SDF
 file, this method makes a new drake::geometry::GeometryInstance object from
 this specification.
 This method returns nullptr when the given SDF specification corresponds
 to a geometry of type `sdf::GeometryType::EMPTY` (<empty/> SDF tag.)  */
std::unique_ptr<geometry::GeometryInstance> MakeGeometryInstanceFromSdfVisual(
    const sdf::Visual& sdf_visual);

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
 ```xml
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
 ```

 An instance of geometry::IllustrationProperties will always be returned. If
 there is no material tag, no material property tags, or no successfully
 parsed material property tags, the property set will be empty.  */
geometry::IllustrationProperties MakeVisualPropertiesFromSdfVisual(
    const sdf::Visual& sdf_visual);

/** Given `sdf_collision` stemming from the parsing of a `<collision>` element
 in an SDF file, this method makes the pose `X_LG` of frame G for the geometry
 of that collision element in the frame L of the link it belongs to.  */
Eigen::Isometry3d MakeGeometryPoseFromSdfCollision(
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
 If a `<surface>` is not found, it returns the coefficients for a
 frictionless surface. If `<surface>` is found, all other nested elements
 are required and an exception is thrown if not present.  */
CoulombFriction<double> MakeCoulombFrictionFromSdfCollisionOde(
    const sdf::Collision& sdf_collision);

// TODO(sam.creasey) Making this operate specifically on sdf::Visual
// is overly specific since we're going to be parsing collision meshes
// at some point.

/** Given an sdf::Visual object representing a <visual> element from
 an SDF file, this method makes a new Visual object which resolves
 the uri for the mesh element, if present.  If the mesh element is
 not present, the new object will be identical to the original.
 See parsers::ResolveFilename() for more detail on this operation.

 @throws std::runtime_error if the <mesh> tag is present but
 missing <uri> or if the file referenced in <uri> can not be found.  */
sdf::Visual ResolveVisualUri(const sdf::Visual& original,
                             const multibody::PackageMap& package_map,
                             const std::string& root_dir);

}  // namespace detail
}  // namespace multibody
}  // namespace drake
