#pragma once

#include <memory>
#include <string>

#include <sdf/sdf.hh>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/multibody_tree/multibody_plant/coulomb_friction.h"
#include "drake/multibody/multibody_tree/parsing/package_map.h"

namespace drake {
namespace multibody {
namespace parsing {
namespace detail {

/// Given an sdf::Geometry object representing a <geometry> element from an SDF
/// file, this method makes a new drake::geometry::Shape object from this
/// specification.
/// For `sdf_geometry.Type() == sdf::GeometryType::EMPTY`, corresponding to the
/// <empty/> SDF tag, it returns `nullptr`.
std::unique_ptr<geometry::Shape> MakeShapeFromSdfGeometry(
    const sdf::Geometry& sdf_geometry);

/// Given an sdf::Visual object representing a <visual> element from an SDF
/// file, this method makes a new drake::geometry::GeometryInstance object from
/// this specification.
/// This method returns nullptr when the given SDF specification corresponds
/// to a geometry of type `sdf::GeometryType::EMPTY` (<empty/> SDF tag.)
std::unique_ptr<geometry::GeometryInstance> MakeGeometryInstanceFromSdfVisual(
    const sdf::Visual& sdf_visual);

/// Given an sdf::Visual object representing a <visual> element from an SDF
/// file, this method makes a new @ref drake::geometry::VisualMaterial
/// "VisualMaterial" object from the specification.
///
/// The visual material comes from the child <material> tag. E.g.,
/// ```xml
/// <visual>
///   <geometry>
///   ...
///   </geometry>
///   <material>
///     <ambient>r_a g_a b_a a_a</ambient>
///     <diffuse>r_d g_d b_d a_d</diffuse>
///     <specular>r_s g_s b_s a_s</specular>
///     <emissive>r_e g_e b_e a_e</emissive>
///   </material>
/// </visual>
/// ```
/// If there is no material tag, the supported material tags are missing, or
/// there is an error parsing the supported values, the instantiated
/// VisualMaterial will use default values. (See geometry::VisualMaterial for
/// description of the default color.)
///
/// @note Currently, only the diffuse value is included in the VisualMaterial.
geometry::VisualMaterial MakeVisualMaterialFromSdfVisual(
    const sdf::Visual& sdf_visual);

/// Given `sdf_collision` stemming from the parsing of a `<collision>` element
/// in an SDF file, this method makes the pose `X_LG` of frame G for the
/// geometry of that collision element in the frame L of the link it belongs to.
Eigen::Isometry3d MakeGeometryPoseFromSdfCollision(
    const sdf::Collision& sdf_collision);

/// Parses friction coefficients from `sdf_collision`.
/// This method looks for the definitions specific to ODE, as given by the SDF
/// specification in `<collision><surface><friction><ode>`. Drake understands
/// `<mu>` as the static coefficient of friction and `<mu2>` as the dynamic
/// coefficient of friction. Consider the example below:
/// ```xml
///   <collision>
///     <surface>
///       <friction>
///         <ode>
///           <mu>0.8</mu>
///           <mu2>0.3</mu2>
///         </ode>
///       </friction>
///     </surface>
///   </collision>
/// ```
/// If a `<surface>` is not found, it returns the coefficients for a
/// frictionless surface. If `<surface>` is found, all other nested elements
/// are required and an exception is thrown if not present.
multibody_plant::CoulombFriction<double> MakeCoulombFrictionFromSdfCollisionOde(
    const sdf::Collision& sdf_collision);

// TODO(sam.creasey) Making this operate specifically on sdf::Visual
// is overly specific since we're going to be parsing collision meshes
// at some point.

/// Given an sdf::Visual object representing a <visual> element from
/// an SDF file, this method makes a new Visual object which resolves
/// the uri for the mesh element, if present.  If the mesh element is
/// not present, the new object will be identical to the original.
/// See parsers::ResolveFilename() for more detail on this operation.
///
/// @throws std::runtime_error if the <mesh> tag is present but
/// missing <uri> or if the file referenced in <uri> can not be found.
sdf::Visual ResolveVisualUri(const sdf::Visual& original,
                             const parsing::PackageMap& package_map,
                             const std::string& root_dir);

}  // namespace detail
}  // namespace parsing
}  // namespace multibody
}  // namespace drake
