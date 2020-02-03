#include "drake/systems/rendering/frame_visualizer.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/parsing/parser.h"

namespace drake {
namespace systems {
namespace rendering {
namespace {

using multibody::Frame;
using multibody::MultibodyPlant;

const char kIiwaFilePath[] =
    "drake/manipulation/models/iiwa_description/sdf/iiwa14_no_collision.sdf";

GTEST_TEST(FrameVisualizerTest, Test) {
  MultibodyPlant<double> plant(0.0);
  drake::multibody::Parser(&plant).AddModelFromFile(
      FindResourceOrThrow(kIiwaFilePath));
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("iiwa_link_0"));
  plant.Finalize();
  auto plant_context = plant.CreateDefaultContext();
  Eigen::VectorXd x(plant.num_multibody_states());
  for (int i = 0; i < x.size(); i++) {
    x[i] = static_cast<double>(i) / 10.0;
  }
  plant.SetPositionsAndVelocities(plant_context.get(), x);

  const std::vector<const Frame<double>*> frames_to_visualize = {
      &plant.GetFrameByName("iiwa_link_1"),
      &plant.GetFrameByName("iiwa_link_7"),
  };
  std::vector<math::RigidTransform<double>> expected_X_WF;
  for (const auto* frame : frames_to_visualize) {
    expected_X_WF.push_back(plant.CalcRelativeTransform(
        *plant_context, plant.world_frame(), *frame));
  }

  FrameVisualizer<double> dut(&plant, frames_to_visualize);
  auto context = dut.CreateDefaultContext();
  dut.get_input_port(0).FixValue(
      context.get(), Value<Context<double>>{plant_context->Clone()});
  const auto& message = dut.get_output_port(0).Eval<lcmt_viewer_draw>(*context);

  // Check message output.
  EXPECT_EQ(message.num_links, static_cast<int>(frames_to_visualize.size()));
  EXPECT_EQ(frames_to_visualize.size(), message.link_name.size());
  EXPECT_EQ(frames_to_visualize.size(), message.robot_num.size());
  EXPECT_EQ(frames_to_visualize.size(), message.position.size());
  EXPECT_EQ(frames_to_visualize.size(), message.quaternion.size());

  for (size_t i = 0; i < frames_to_visualize.size(); i++) {
    const Eigen::Vector3f pos = expected_X_WF[i].translation().cast<float>();
    const Eigen::Quaternion<float> quat =
        expected_X_WF[i].rotation().ToQuaternion().cast<float>();
    EXPECT_EQ(message.link_name[i], frames_to_visualize[i]->name());
    for (int j = 0; j < 3; j++) {
      EXPECT_EQ(message.position[i][j], pos[j]);
    }
    EXPECT_EQ(message.quaternion[i][0], quat.w());
    EXPECT_EQ(message.quaternion[i][1], quat.x());
    EXPECT_EQ(message.quaternion[i][2], quat.y());
    EXPECT_EQ(message.quaternion[i][3], quat.z());
    EXPECT_EQ(message.robot_num[i], frames_to_visualize[i]->model_instance());
  }
}

}  // namespace
}  // namespace rendering
}  // namespace systems
}  // namespace drake
