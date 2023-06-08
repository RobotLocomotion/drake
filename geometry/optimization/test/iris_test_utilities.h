#pragma once

#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/planning/robot_diagram_builder.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace geometry {
namespace optimization {
/* A toy robot with two revolute joints and some collision geometries.
 */
class Toy2DofRobotTest : public ::testing::Test {
 public:
  Toy2DofRobotTest();

 protected:
  planning::RobotDiagramBuilder<double> builder_;
  std::vector<multibody::BodyIndex> body_indices_;
  geometry::GeometryId world_left_wall_;
  geometry::GeometryId world_right_wall_;
  geometry::GeometryId world_top_wall_;
  geometry::GeometryId world_bottom_wall_;
  std::vector<geometry::GeometryId> body0_spheres_;
  std::vector<geometry::GeometryId> body1_spheres_;
};
}  // namespace optimization
}  // namespace geometry
}  // namespace drake
