#include "drake/manipulation/util/multibody_plant_posture_visualizer.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/parser.h"
namespace drake {
namespace manipulation {
GTEST_TEST(MultibodyPlantPostureVisualizer, Test1) {
  const std::string file_path =
      "drake/manipulation/models/iiwa_description/sdf/iiwa14_no_collision.sdf";
  auto plant = std::make_unique<multibody::MultibodyPlant<double>>();
  auto scene_graph = std::make_unique<geometry::SceneGraph<double>>();
  plant->RegisterAsSourceForSceneGraph(scene_graph.get());
  multibody::Parser(plant.get())
      .AddModelFromFile(FindResourceOrThrow(file_path));
  plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("iiwa_link_0"));

  plant->Finalize(scene_graph.get());

  MultibodyPlantPostureVisualizer visualizer(*plant, std::move(scene_graph));
  Eigen::Matrix<double, 7, 1> q;
  q << 0.4, 0.4, 0.4, -0.4, 0.4, 0.4, 0.4;
  visualizer.VisualizePosture(q);
}

GTEST_TEST(MultibodyPlantPostureVisualizer, Test2) {
  const std::string file_path =
      "drake/manipulation/models/iiwa_description/sdf/iiwa14_no_collision.sdf";
  MultibodyPlantPostureVisualizer visualizer(file_path);
  // Now the IIWA robot base it not welded to the ground, so it has 14
  // generalized positions, 7 for the base, and 7 for the joints.
  Eigen::Matrix<double, 14, 1> q;
  q << 1, 0, 0, 0, 0, 0, 0, 0.2, -0.3, 0.5, 0.6, -0.1, 0.4, 0.5;
  visualizer.VisualizePosture(q);
}
}  // namespace manipulation
}  // namespace drake
