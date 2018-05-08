#pragma once

#include <memory>

#include <sdf/sdf.hh>

#include "drake/geometry/scene_graph.h"

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

}  // namespace detail
}  // namespace parsing
}  // namespace multibody
}  // namespace drake
