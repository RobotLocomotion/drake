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

}  // namespace detail
}  // namespace parsing
}  // namespace multibody
}  // namespace drake
