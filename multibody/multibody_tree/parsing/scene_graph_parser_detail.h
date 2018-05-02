#pragma once

#include <memory>
#include <string>

#include <sdf/sdf.hh>

#include "drake/geometry/scene_graph.h"

namespace drake {
namespace multibody {
namespace parsing {
namespace detail {

std::unique_ptr<geometry::Shape> MakeShapeFromSdfGeometry(
    const sdf::Geometry& sdf_geometry);

/// This method helps in creating a GeometryInstance object from the shape
/// defintion in a sdf::Geometry object and the pose X_PG of that geometry in
/// a parent frame P.
std::unique_ptr<geometry::GeometryInstance> MakeGeometryInstanceFromSdfVisual(
    const sdf::Visual& sdf_visual);

}  // namespace detail
}  // namespace parsing
}  // namespace multibody
}  // namespace drake
