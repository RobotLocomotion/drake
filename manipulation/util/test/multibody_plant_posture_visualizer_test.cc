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
}  // namespace manipulation
}  // namespace drake
