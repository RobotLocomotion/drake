#include "drake/multibody/rigid_body_plant/frame_visualizer.h"

#include <memory>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/lcm/drake_mock_lcm.h"
#include "drake/lcmt_viewer_draw.hpp"

namespace drake {
namespace systems {
namespace {

GTEST_TEST(FrameVisualizerTests, TestMessageGeneration) {
  RigidBodyTree<double> tree;

  for (size_t i = 0; i < tree.bodies.size(); ++i) {
    std::cout << tree.bodies[i]->get_name() << "\n";
  }

}

}  // namespace
}  // namespace systems
}  // namespace drake
