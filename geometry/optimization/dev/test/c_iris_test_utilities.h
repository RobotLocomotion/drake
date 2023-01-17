#pragma once

#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace geometry {
namespace optimization {

// Create a toy robot with different collision geometries.
// world - weld - body0 - revolute - body1 - prismatic - body2
//                 |
//              revolute - body3
class CIrisToyRobotTest : public ::testing::Test {
 public:
  CIrisToyRobotTest();

 protected:
  std::unique_ptr<systems::Diagram<double>> diagram_;
  multibody::MultibodyPlant<double>* plant_;
  geometry::SceneGraph<double>* scene_graph_;
  std::vector<multibody::BodyIndex> body_indices_;
  geometry::GeometryId world_box_;
  geometry::GeometryId world_cylinder_;
  geometry::GeometryId body0_box_;
  geometry::GeometryId body0_sphere_;
  geometry::GeometryId body1_convex_;
  geometry::GeometryId body1_capsule_;
  geometry::GeometryId body2_capsule_;
  geometry::GeometryId body2_sphere_;
  geometry::GeometryId body3_box_;
  geometry::GeometryId body3_cylinder_;
};

// Create a robot with only polytopic collision geometry.
// world - revolute - body0 - revolute - body1 - revolute - body2 - revolute -
// body3
class CIrisRobotPolytopicGeometryTest : public ::testing::Test {
 public:
  CIrisRobotPolytopicGeometryTest();

 protected:
  std::unique_ptr<systems::Diagram<double>> diagram_;
  multibody::MultibodyPlant<double>* plant_;
  geometry::SceneGraph<double>* scene_graph_;
  std::vector<multibody::BodyIndex> body_indices_;
  std::vector<geometry::GeometryId> world_boxes_;
  geometry::GeometryId world_convex_;
  std::vector<geometry::GeometryId> body_boxes_;
};

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
