#include "drake/systems/rendering/render_pose_to_geometry_pose.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

#include "drake/systems/framework/test_utilities/scalar_conversion.h"

namespace drake {
namespace systems {
namespace rendering {
namespace {

GTEST_TEST(RenderPoseToGeometryPoseTest, InputOutput) {
  const auto source_id = geometry::SourceId::get_new_id();
  const auto frame_id = geometry::FrameId::get_new_id();
  const RenderPoseToGeometryPose<double> dut(source_id, frame_id);

  EXPECT_EQ(dut.get_output_port().get_index(), 0);

  const Eigen::Quaternion<double> rotation(0.5, 0.5, 0.5, 0.5);
  const Eigen::Translation3d translation(1.0, 2.0, 3.0);
  const PoseVector<double> input(rotation, translation);
  auto context = dut.CreateDefaultContext();
  dut.get_input_port().FixValue(context.get(), input);

  const auto& output =
      dut.get_output_port().Eval<geometry::FramePoseVector<double>>(
          *context);
  EXPECT_EQ(output.size(), 1);
  ASSERT_TRUE(output.has_id(frame_id));
  EXPECT_TRUE(CompareMatrices(
      output.value(frame_id).GetAsMatrix4(),
      input.get_transform().GetAsMatrix4()));

  EXPECT_TRUE(dut.HasAnyDirectFeedthrough());
}

GTEST_TEST(RenderPoseToGeometryPoseTest, ToAutoDiff) {
  const RenderPoseToGeometryPose<double> dut({}, {});
  EXPECT_TRUE(is_autodiffxd_convertible(dut, [&](const auto& converted) {
    EXPECT_EQ(1, converted.num_input_ports());
    EXPECT_EQ(1, converted.num_output_ports());
  }));
}

GTEST_TEST(RenderPoseToGeometryPoseTest, ToSymbolic) {
  const RenderPoseToGeometryPose<double> dut({}, {});
  EXPECT_TRUE(is_symbolic_convertible(dut, [&](const auto& converted) {
    EXPECT_EQ(1, converted.num_input_ports());
    EXPECT_EQ(1, converted.num_output_ports());
  }));
}

#pragma GCC diagnostic pop

}  // namespace
}  // namespace rendering
}  // namespace systems
}  // namespace drake
