#pragma once

#include "drake/lcmt_viewer_load_robot.hpp"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace multibody {

/// Creates and returns an lcmt_viewer_load_robot message containing the visual
/// geometries from the provided RigidBodyTree. Note that this includes any
/// visual geometries attached to the world body.
///
/// Instantiated templates for the following ScalarTypes are provided:
///
/// - double
///
/// They are already available to link against in the containing library.
template <typename T>
lcmt_viewer_load_robot CreateLoadRobotMessage(
    const RigidBodyTree<double>& tree);

}  // namespace multibody
}  // namespace drake
