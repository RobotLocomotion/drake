#pragma once

#include "drake/lcmt_viewer_draw.hpp"
#include "drake/lcmt_viewer_load_robot.hpp"

namespace drake {
namespace systems {
namespace test {

bool CompareLoadMessage(
    const drake::lcmt_viewer_load_robot& received,
    const drake::lcmt_viewer_load_robot& expected);

bool CompareDrawMessage(
    const drake::lcmt_viewer_draw& received,
    const drake::lcmt_viewer_draw& expected);

}  // namespace test
}  // namespace systems
}  // namespace drake
