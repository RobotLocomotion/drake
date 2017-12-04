#pragma once

#include "drake/lcmt_viewer_geometry_data.hpp"
#include "drake/multibody/shapes/visual_element.h"

/// @file Utilities for implementing the Drake Visualizer Interface. This is
/// the oldest supported interface in the RobotLocomotion/director GUI called
/// drake-visualizer. It uses LCM messages, and conventionally communicates on
/// LCM channels named DRAKE_VIEWER_DRAW and DRAKE_VIEWER_LOAD_ROBOT.
///
/// The Drake Visualizer Interface is distinct from, and should not be confused
/// with, the newer Remote Tree Viewer Interface, which was introduced in
/// https://github.com/RobotLocomotion/director/pull/420.

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
