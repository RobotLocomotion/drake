#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace systems {
namespace rendering {
namespace {

GTEST_TEST(MultibodyPositionToGeometryPoseTest, InputOutput) {
  multibody::MultibodyPlant<double> mbp;
  geometry::SceneGraph<double> scene_graph;
  mbp.RegisterAsSourceForSceneGraph(&scene_graph);
  multibody::Parser(&mbp).AddModelFromFile(
      FindResourceOrThrow("drake/manipulation/models/iiwa_description/iiwa7"
                          "/iiwa7_no_collision.sdf"));
  mbp.Finalize(&scene_graph);

  const MultibodyPositionToGeometryPose<double> dut(mbp);

  EXPECT_EQ(dut.get_input_port().get_index(), 0);
  EXPECT_EQ(dut.get_output_port().get_index(), 0);
  EXPECT_TRUE(dut.HasAnyDirectFeedthrough());

  auto context = dut.CreateDefaultContext();

  const Eigen::VectorXd position =
      Eigen::VectorXd::LinSpaced(mbp.num_positions(), 0.123, 0.456);
  context->FixInputPort(dut.get_input_port().get_index(), position);

  const auto& output =
      dut.get_output_port().Eval<geometry::FramePoseVector<double>>(*context);
  EXPECT_EQ(output.source_id(), mbp.get_source_id());
  for (multibody::BodyIndex i(0); i < mbp.num_bodies(); i++) {
    if (i == mbp.world_body().index()) {
      // The world geometry will not appear in the poses.
      continue;
    }
    const optional<geometry::FrameId> id = mbp.GetBodyFrameIdIfExists(i);
    EXPECT_TRUE(id.has_value());
    EXPECT_TRUE(output.has_id(id.value()));
  }
  EXPECT_EQ(output.size(), mbp.num_bodies()-1);
}

}  // namespace
}  // namespace rendering
}  // namespace systems
}  // namespace drake
