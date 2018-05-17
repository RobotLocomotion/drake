#pragma once

#include <memory>

#include <sdf/sdf.hh>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/multibody_tree/multibody_plant/coulomb_friction.h"

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

/// Given `sdf_collision` stemming from the parsing of a `<collision>` element
/// in an SDF file, this method makes the pose `X_LG` of frame G for the
/// geometry of that collision element in the frame L of the link it belongs to.
Eigen::Isometry3d MakeGeometryPoseFromSdfCollision(
    const sdf::Collision& sdf_collision);

/// Parses friction coefficients from `sdf_collision`.
/// This method looks for a custom <drake_friction> element within <collision>
/// as shown below: <pre>
///   <collision>
///     <drake_friction>
///       <static_friction>0.8</static_friction>
///       <dynamic_friction>0.3</dynamic_friction>
///     </drake_friction>
///   </collision>
/// </pre>
/// If a `<drake_collision>` is not found, it returns the coefficients for a
/// frictionless surface. After `<drake_collision>` is found,
/// `<static_friction>` and `<dynamic_friction>` are required to be present and
/// an exception is thrown if not.
multibody_plant::CoulombFriction<double> MakeCoulombFrictionFromSdfCollision(
    const sdf::Collision& sdf_collision);

/// Parses friction coefficients from `sdf_collision`.
/// This method looks for the definitions specific to ODE, as given by the SDF
/// specification in `<collision><surface><friction><ode>`. Drake understands
/// `<mu>` as the static coefficient of friction and `<mu2>` as the dynamic
/// coefficient of friction. Consider the example below: <pre>
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
/// </pre>
/// If a `<surface>` is not found, it returns the coefficients for a
/// frictionless surface. After `<surface>` is found, all other nested elements
/// are required and an exception is thrown if not present.
multibody_plant::CoulombFriction<double> MakeCoulombFrictionFromSdfCollisionOde(
    const sdf::Collision& sdf_collision);

}  // namespace detail
}  // namespace parsing
}  // namespace multibody
}  // namespace drake
