#include "drake/visualization/meshcat_pose_sliders.h"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/test_utilities/meshcat_environment.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/test_utilities/initialization_test_system.h"

namespace drake {
namespace visualization {
namespace {

using Eigen::Vector3d;
using math::RigidTransformd;
using math::RollPitchYawd;
using systems::Context;

// Test the narrowest constructor, where all optionals are null.
GTEST_TEST(MeshcatPoseSlidersTest, NarrowConstructor) {
  auto meshcat = geometry::GetTestEnvironmentMeshcat();

  const MeshcatPoseSliders<double> dut(meshcat);
  auto context = dut.CreateDefaultContext();

  // Sliders start at their default context value.
  EXPECT_EQ(meshcat->GetSliderValue("roll"), 0);
  EXPECT_EQ(meshcat->GetSliderValue("pitch"), 0);
  EXPECT_EQ(meshcat->GetSliderValue("yaw"), 0);
  EXPECT_EQ(meshcat->GetSliderValue("x"), 0);
  EXPECT_EQ(meshcat->GetSliderValue("y"), 0);
  EXPECT_EQ(meshcat->GetSliderValue("z"), 0);

  EXPECT_TRUE(
      dut.get_output_port().Eval<RigidTransformd>(*context).IsExactlyEqualTo(
          RigidTransformd()));

  // Slider limits obey the default limits.
  const double kPiRounded = 3.14;
  meshcat->SetSliderValue("roll", -9999);
  EXPECT_EQ(meshcat->GetSliderValue("roll"), -kPiRounded);
  meshcat->SetSliderValue("yaw", 9999);
  EXPECT_EQ(meshcat->GetSliderValue("yaw"), kPiRounded);
  EXPECT_TRUE(
      dut.get_output_port().Eval<RigidTransformd>(*context).IsExactlyEqualTo(
          RigidTransformd(RollPitchYawd(-kPiRounded, 0, kPiRounded),
                          Vector3d::Zero())));
}

// Test the constructor with all vectors passed in.
GTEST_TEST(MeshcatPoseSlidersTest, WideConstructor) {
  auto meshcat = geometry::GetTestEnvironmentMeshcat();

  const RigidTransformd initial_pose(RollPitchYawd(0.1, 0.2, 0.3),
                                     Vector3d(0.4, 0.5, 0.6));
  const Vector6d lower_limit = Vector6d::Constant(-0.8);
  const Vector6d upper_limit = Vector6d::Constant(0.8);
  const Vector6d step =
      (Vector6d() << 0.1, 0.1, 0.1, 0.1, 0.01, 0.1).finished();
  const std::vector<std::string> decrement_keycodes = {"KeyA", "KeyB", "KeyC",
                                                       "KeyD", "KeyE", "KeyF"};
  const std::vector<std::string> increment_keycodes = {"KeyG", "KeyH", "KeyI",
                                                       "KeyJ", "KeyK", "KeyL"};
  const std::string prefix = "prefix";
  const Vector6<bool> visible =
      (Vector6<bool>() << false, false, true, true, true, false).finished();

  const MeshcatPoseSliders<double> dut(meshcat, initial_pose, lower_limit,
                                       upper_limit, step, decrement_keycodes,
                                       increment_keycodes, prefix, visible);
  auto context = dut.CreateDefaultContext();

  // Sliders start at their initial value.
  EXPECT_NEAR(meshcat->GetSliderValue("prefix_yaw"), 0.3, 1e-14);
  EXPECT_EQ(meshcat->GetSliderValue("prefix_x"), 0.4);
  EXPECT_EQ(meshcat->GetSliderValue("prefix_y"), 0.5);
  EXPECT_TRUE(
      dut.get_output_port().Eval<RigidTransformd>(*context).IsNearlyEqualTo(
          initial_pose, 1e-14));

  // Sliders obey their configured limits.
  meshcat->SetSliderValue("prefix_yaw", -9999);
  EXPECT_EQ(meshcat->GetSliderValue("prefix_yaw"), -0.8);
  meshcat->SetSliderValue("prefix_x", 9999);
  EXPECT_EQ(meshcat->GetSliderValue("prefix_x"), 0.8);
  EXPECT_TRUE(
      dut.get_output_port().Eval<RigidTransformd>(*context).IsNearlyEqualTo(
          RigidTransformd(RollPitchYawd(0.1, 0.2, -0.8),
                          Vector3d(0.8, 0.5, 0.6)),
          1e-14));

  // The step parameter rounds correctly.
  meshcat->SetSliderValue("prefix_x", 0.1111111111);
  EXPECT_EQ(meshcat->GetSliderValue("prefix_x"), 0.1);
  meshcat->SetSliderValue("prefix_y", 0.2222222222);
  EXPECT_EQ(meshcat->GetSliderValue("prefix_y"), 0.22);
}

// Confirm that the bounds are respected when they aren't in the usual range of
// RigidTransform.ToRollPitchYaw().
GTEST_TEST(MeshcatPoseSlidersTest, ShiftedLimits) {
  auto meshcat = geometry::GetTestEnvironmentMeshcat();
  const double kInf = std::numeric_limits<double>::infinity();

  const RigidTransformd initial_pose(RollPitchYawd(0.1, 0.2, 0.3),
                                     Vector3d(0.4, 0.5, 0.6));
  Vector6d lower_limit = Vector6d::Constant(-0.8);
  lower_limit.head<3>() << M_PI, -kInf, 4 * M_PI;
  Vector6d upper_limit = Vector6d::Constant(-0.8);
  upper_limit.head<3>() << 3 * M_PI, -2 * M_PI, kInf;

  const MeshcatPoseSliders<double> dut(meshcat, initial_pose, lower_limit,
                                       upper_limit);
  auto context = dut.CreateDefaultContext();

  // Sliders start at their initial value.
  EXPECT_NEAR(meshcat->GetSliderValue("roll"), 6.38, 1e-14);
  EXPECT_NEAR(meshcat->GetSliderValue("pitch"), -12.37, 1e-14);
  EXPECT_NEAR(meshcat->GetSliderValue("yaw"), 12.87, 1e-14);
}

// Tests that the Delete function removes the sliders.
GTEST_TEST(MeshcatPoseSlidersTest, DeleteFunction) {
  auto meshcat = geometry::GetTestEnvironmentMeshcat();

  auto dut = std::make_unique<MeshcatPoseSliders<double>>(meshcat);

  EXPECT_EQ(meshcat->GetSliderValue("roll"), 0.0);

  // Remove them; confirm that they are gone.
  dut->Delete();
  EXPECT_THROW(meshcat->GetSliderValue("roll"), std::exception);

  // A second call is harmless.
  dut->Delete();

  // The destructor does not crash.
  EXPECT_NO_THROW(dut.reset());
}

// Tests that the destructor removes the sliders.
GTEST_TEST(MeshcatPoseSlidersTest, Destructor) {
  auto meshcat = geometry::GetTestEnvironmentMeshcat();

  auto dut = std::make_unique<MeshcatPoseSliders<double>>(meshcat);
  EXPECT_EQ(meshcat->GetSliderValue("roll"), 0.0);

  // Delete the entire object; confirm that the sliders are gone.
  EXPECT_NO_THROW(dut.reset());
  EXPECT_THROW(meshcat->GetSliderValue("roll"), std::exception);
}

// Helper system that records the pose on a forced publish event.
class PosePublishRecorder : public systems::LeafSystem<double> {
 public:
  PosePublishRecorder() {
    DeclareAbstractInputPort("pose", Value<RigidTransformd>());
    DeclareForcedPublishEvent(&PosePublishRecorder::Publish);
  }

  RigidTransformd pose() const { return pose_; }

 private:
  systems::EventStatus Publish(const Context<double>& context) const {
    pose_ = this->get_input_port().Eval<RigidTransformd>(context);
    return systems::EventStatus::Succeeded();
  }

  mutable RigidTransformd pose_{};
};

// Tests the "Run" sugar function.
GTEST_TEST(MeshcatPoseSlidersTest, Run) {
  auto meshcat = geometry::GetTestEnvironmentMeshcat();
  const RigidTransformd initial_pose(RollPitchYawd(0.1, 0.2, 0.3),
                                     Vector3d(0.4, 0.5, 0.6));

  systems::DiagramBuilder<double> builder;
  auto* dut = builder.AddSystem<MeshcatPoseSliders>(meshcat, initial_pose);
  auto* recorder = builder.AddSystem<PosePublishRecorder>();
  builder.Connect(dut->get_output_port(), recorder->get_input_port());

  auto init_system = builder.AddSystem<systems::InitializationTestSystem>();

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  // Run for a while.
  const double timeout = 1.0;
  RigidTransformd X = dut->Run(*diagram, *context, timeout);
  EXPECT_TRUE(X.IsExactlyEqualTo(initial_pose));
  // Check that the recorder's Publish method was called.
  EXPECT_TRUE(recorder->pose().IsExactlyEqualTo(initial_pose));

  // Confirm that initialization events were triggered.
  EXPECT_TRUE(init_system->get_pub_init());
  EXPECT_TRUE(init_system->get_dis_update_init());
  EXPECT_TRUE(init_system->get_unres_update_init());

  // Note: the stop button is deleted on timeout, so we cannot easily check
  // that it was created correctly here.

  // Set a non-default slider position.
  meshcat->SetSliderValue("roll", 0.25);
  const RigidTransformd expected_pose(RollPitchYawd(0.25, 0.2, 0.3),
                                      Vector3d(0.4, 0.5, 0.6));

  // Run for a while (with a non-default stop_button_keycode).
  X = dut->Run(*diagram, *context, timeout, "KeyP");
  EXPECT_TRUE(X.IsExactlyEqualTo(expected_pose));

  // Check that the slider's transform had any effect, i.e., that the
  // PosePublishRecorder::Publish was called again.
  EXPECT_TRUE(recorder->pose().IsExactlyEqualTo(expected_pose));
}

/* Tests the "SetPose" function. */
GTEST_TEST(MeshcatPoseSlidersTest, SetPose) {
  auto meshcat = geometry::GetTestEnvironmentMeshcat();
  MeshcatPoseSliders<double> dut(meshcat);
  auto context = dut.CreateDefaultContext();

  EXPECT_TRUE(dut.get_output_port()
                  .Eval<RigidTransformd>(*context)
                  .IsExactlyIdentity());
  EXPECT_EQ(meshcat->GetSliderValue("roll"), 0);
  EXPECT_EQ(meshcat->GetSliderValue("pitch"), 0);

  /* Setting the pose should update both the output value and sliders. */
  const RigidTransformd X(RollPitchYawd(0.1, 0.2, 0.3),
                          Vector3d(0.4, 0.5, 0.6));
  dut.SetPose(X);
  EXPECT_TRUE(
      dut.get_output_port().Eval<RigidTransformd>(*context).IsExactlyEqualTo(
          X));
  EXPECT_EQ(meshcat->GetSliderValue("roll"), 0.1);
  EXPECT_EQ(meshcat->GetSliderValue("pitch"), 0.2);

  // Deleting should remove the sliders, but the positions should remain.
  dut.Delete();
  EXPECT_TRUE(
      dut.get_output_port().Eval<RigidTransformd>(*context).IsExactlyEqualTo(
          X));
  EXPECT_THROW(meshcat->GetSliderValue("roll"), std::exception);
  EXPECT_THROW(meshcat->GetSliderValue("pitch"), std::exception);
}

/* Tests that the optional input port can be used to set the (initial) pose. */
GTEST_TEST(MeshcatPoseSlidersTest, InputPort) {
  auto meshcat = geometry::GetTestEnvironmentMeshcat();
  MeshcatPoseSliders<double> dut(meshcat);

  const RigidTransformd X(RollPitchYawd(0.1, 0.2, 0.3),
                          Vector3d(0.4, 0.5, 0.6));

  systems::Simulator<double> simulator(dut);
  auto& context = simulator.get_mutable_context();
  dut.get_input_port().FixValue(&context, X);
  simulator.AdvanceTo(1.0);

  EXPECT_EQ(meshcat->GetSliderValue("roll"), 0.1);
  EXPECT_EQ(meshcat->GetSliderValue("pitch"), 0.2);
  EXPECT_TRUE(
      dut.get_output_port().Eval<RigidTransformd>(context).IsExactlyEqualTo(X));
}

/* Tests that the optional input port is called (via the initialization event)
via the Run method. */
GTEST_TEST(MeshcatPoseSlidersTest, RunWithFixedInputPort) {
  auto meshcat = geometry::GetTestEnvironmentMeshcat();
  MeshcatPoseSliders<double> dut(meshcat);

  const RigidTransformd X(RollPitchYawd(0.1, 0.2, 0.3),
                          Vector3d(0.4, 0.5, 0.6));

  auto context = dut.CreateDefaultContext();
  dut.get_input_port().FixValue(context.get(), X);

  const double timeout = 1.0;
  RigidTransformd X_out = dut.Run(dut, *context, timeout);

  EXPECT_EQ(meshcat->GetSliderValue("roll"), 0.1);
  EXPECT_EQ(meshcat->GetSliderValue("pitch"), 0.2);
  EXPECT_TRUE(
      dut.get_output_port().Eval<RigidTransformd>(*context).IsExactlyEqualTo(
          X));
  EXPECT_TRUE(X_out.IsExactlyEqualTo(X));
}

GTEST_TEST(MeshcatPoseSlidersTest, Graphviz) {
  auto meshcat = geometry::GetTestEnvironmentMeshcat();
  MeshcatPoseSliders<double> dut(meshcat);
  EXPECT_THAT(dut.GetGraphvizString(), testing::HasSubstr("meshcat_out ->"));
}

}  // namespace
}  // namespace visualization
}  // namespace drake
