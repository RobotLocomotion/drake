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

multibody_plant::CoulombFriction<double> MakeCoulombFrictionFromSdfCollision(
    const sdf::Collision& sdf_collision);

multibody_plant::CoulombFriction<double> MakeCoulombFrictionFromSdfCollisionOde(
    const sdf::Collision& sdf_collision);

}  // namespace detail
}  // namespace parsing
}  // namespace multibody
}  // namespace drake
