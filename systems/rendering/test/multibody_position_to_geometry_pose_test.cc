#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace systems {
namespace rendering {
namespace {

using geometry::SceneGraph;
using multibody::BodyIndex;
using multibody::MultibodyPlant;
using multibody::Parser;
using std::make_unique;
using std::move;

GTEST_TEST(MultibodyPositionToGeometryPoseTest, BadConstruction) {
  {
    MultibodyPlant<double> mbp(0.0);
    mbp.Finalize();

    DRAKE_EXPECT_THROWS_MESSAGE(
        MultibodyPositionToGeometryPose<double>{mbp}, std::logic_error,
        "MultibodyPositionToGeometryPose requires a MultibodyPlant that has "
        "been registered with a SceneGraph");
  }

  {
    MultibodyPlant<double> mbp(0.0);
    SceneGraph<double> scene_graph;
    mbp.RegisterAsSourceForSceneGraph(&scene_graph);
    Parser(&mbp).AddModelFromFile(
        FindResourceOrThrow("drake/manipulation/models/iiwa_description/iiwa7"
                            "/iiwa7_no_collision.sdf"));
    DRAKE_EXPECT_THROWS_MESSAGE(MultibodyPositionToGeometryPose<double>{mbp},
                                std::logic_error,
                                "MultibodyPositionToGeometryPose requires a "
                                "MultibodyPlant that has been finalized");
  }
}

GTEST_TEST(MultibodyPositionToGeometryPoseTest, Ownership) {
  auto mbp = make_unique<MultibodyPlant<double>>(0.0);
  auto raw_ptr = mbp.get();
  SceneGraph<double> scene_graph;
  mbp->RegisterAsSourceForSceneGraph(&scene_graph);
  Parser(mbp.get()).AddModelFromFile(
      FindResourceOrThrow("drake/manipulation/models/iiwa_description/iiwa7"
                          "/iiwa7_no_collision.sdf"));
  mbp->Finalize();

  const MultibodyPositionToGeometryPose<double> dut(move(mbp));
  EXPECT_EQ(dut.get_input_port().size(),
            dut.multibody_plant().num_positions());

  EXPECT_EQ(&dut.multibody_plant(), raw_ptr);
  EXPECT_TRUE(dut.owns_plant());

  auto context = dut.CreateDefaultContext();

  const Eigen::VectorXd position =
      Eigen::VectorXd::LinSpaced(raw_ptr->num_positions(), 0.123, 0.456);
  dut.get_input_port().FixValue(context.get(), position);

  const auto& output =
      dut.get_output_port().Eval<geometry::FramePoseVector<double>>(*context);
  for (BodyIndex i(0); i < raw_ptr->num_bodies(); ++i) {
    if (i == raw_ptr->world_body().index()) {
      // The world geometry will not appear in the poses.
      continue;
    }
    const std::optional<geometry::FrameId> id =
        raw_ptr->GetBodyFrameIdIfExists(i);
    EXPECT_TRUE(id.has_value());
    EXPECT_TRUE(output.has_id(id.value()));
  }
  EXPECT_EQ(output.size(), raw_ptr->num_bodies() - 1);
}

GTEST_TEST(MultibodyPositionToGeometryPoseTest, InputOutput) {
  MultibodyPlant<double> mbp(0.0);
  SceneGraph<double> scene_graph;
  mbp.RegisterAsSourceForSceneGraph(&scene_graph);
  Parser(&mbp).AddModelFromFile(
      FindResourceOrThrow("drake/manipulation/models/iiwa_description/iiwa7"
                          "/iiwa7_no_collision.sdf"));
  mbp.Finalize();

  const MultibodyPositionToGeometryPose<double> dut(mbp);
  EXPECT_FALSE(dut.owns_plant());
  EXPECT_EQ(dut.get_input_port().size(), mbp.num_positions());

  EXPECT_EQ(dut.get_input_port().get_index(), 0);
  EXPECT_EQ(dut.get_output_port().get_index(), 0);
  EXPECT_TRUE(dut.HasAnyDirectFeedthrough());

  auto context = dut.CreateDefaultContext();

  const Eigen::VectorXd position =
      Eigen::VectorXd::LinSpaced(mbp.num_positions(), 0.123, 0.456);
  dut.get_input_port().FixValue(context.get(), position);

  const auto& output =
      dut.get_output_port().Eval<geometry::FramePoseVector<double>>(*context);
  for (BodyIndex i(0); i < mbp.num_bodies(); ++i) {
    if (i == mbp.world_body().index()) {
      // The world geometry will not appear in the poses.
      continue;
    }
    const std::optional<geometry::FrameId> id = mbp.GetBodyFrameIdIfExists(i);
    EXPECT_TRUE(id.has_value());
    EXPECT_TRUE(output.has_id(id.value()));
  }
  EXPECT_EQ(output.size(), mbp.num_bodies() - 1);
}

// Confirm that we can pass in the larger state vector and it does not
// affect our results.
GTEST_TEST(MultibodyPositionToGeometryPoseTest, FullStateInput) {
  auto mbp = make_unique<MultibodyPlant<double>>(0.0);
  SceneGraph<double> scene_graph;
  mbp->RegisterAsSourceForSceneGraph(&scene_graph);
  Parser(mbp.get()).AddModelFromFile(
      FindResourceOrThrow("drake/manipulation/models/iiwa_description/iiwa7"
                          "/iiwa7_no_collision.sdf"));
  mbp->Finalize();

  const Eigen::VectorXd state =
      Eigen::VectorXd::LinSpaced(mbp->num_multibody_states(), 0.123, 0.456);

  const MultibodyPositionToGeometryPose<double> position_sys(*mbp, false);
  EXPECT_EQ(position_sys.get_input_port().size(), mbp->num_positions());
  auto position_context = position_sys.CreateDefaultContext();
  position_sys.get_input_port().FixValue(position_context.get(),
                                         state.head(mbp->num_positions()));
  const auto& position_output =
      position_sys.get_output_port().Eval<geometry::FramePoseVector<double>>(
          *position_context);

  const MultibodyPositionToGeometryPose<double> state_sys(*mbp, true);
  EXPECT_EQ(state_sys.get_input_port().size(), mbp->num_multibody_states());
  auto state_context = state_sys.CreateDefaultContext();
  state_sys.get_input_port().FixValue(state_context.get(), state);
  const auto& state_output =
      state_sys.get_output_port().Eval<geometry::FramePoseVector<double>>(
          *state_context);

  EXPECT_EQ(position_output.size(), state_output.size());
  for (const auto& id : position_output.frame_ids()) {
    EXPECT_TRUE(
        position_output.value(id).IsExactlyEqualTo(state_output.value(id)));
  }

  // Test the ownership constructor also has the right size.
  const MultibodyPositionToGeometryPose<double> owned_sys(move(mbp), true);
  EXPECT_EQ(owned_sys.get_input_port().size(),
            owned_sys.multibody_plant().num_multibody_states());
}

}  // namespace
}  // namespace rendering
}  // namespace systems
}  // namespace drake
