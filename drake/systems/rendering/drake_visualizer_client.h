#pragma once

#include "drake/lcmt_viewer_geometry_data.hpp"
#include "drake/multibody/shapes/visual_element.h"

/// @file Utilities for implementing the Drake Visualizer Interface.

namespace drake {
namespace systems {
namespace rendering {

/// Returns an lcmt_viewer_geometry_message with contents that mirror the
/// @p visual_element.
lcmt_viewer_geometry_data MakeGeometryData(
    const DrakeShapes::VisualElement& visual_element);

}  // namespace rendering
}  // namespace systems
}  // namespace drake
