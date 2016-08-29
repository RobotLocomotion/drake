#pragma once

#include <lcm/lcm-cpp.hpp>

#include "drake/lcmt_viewer_draw.hpp"
#include "drake/lcmt_viewer_geometry_data.hpp"
#include "drake/lcmt_viewer_load_robot.hpp"

#include "drake/systems/plants/RigidBodyTree.h"


namespace drake {
namespace systems {
namespace test {

/**
 * Tests BotVisualizerSystem by having it send load and draw messages for
 * visualizing @p tree. The messages actually received are compared against
 * @p expected_load_message and @p expected_draw_message. The LCM channel name
 * postfix to use is @p kChannelPostfix. This postfix is used to avoid
 * conflicts between concurrently running unit tests. The LCM subsystem to use
 * is provided by @p lcm.
 */
void DoBotVisualizerTest(const RigidBodyTree& tree,
    const drake::lcmt_viewer_load_robot& expected_load_message,
    const drake::lcmt_viewer_draw& expected_draw_message,
    const std::string& kChannelPostfix,
    ::lcm::LCM* lcm);

}  // namespace test
}  // namespace systems
}  // namespace drake