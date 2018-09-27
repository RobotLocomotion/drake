#include "drake/systems/rendering/pose_vector_to_geometry.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"

namespace drake {
namespace systems {
namespace rendering {
namespace {

GTEST_TEST(PoseVectorToGeometryTest, InputOutput) {
  const geometry::SourceId source_id;
  const geometry::FrameId frame_id;
  const PoseVectorToGeometry<double> dut(source_id, frame_id);

  EXPECT_EQ(dut.get_output_port().get_index(), 0);

  const Eigen::Quaternion<double> rotation(0.5, 0.5, 0.5, 0.5);
  const Eigen::Translation3d translation(1.0, 2.0, 3.0);
  const PoseVector<double> input(rotation, translation);
  auto context = dut.CreateDefaultContext();
  context->FixInputPort(dut.get_input_port().get_index(), input);

  const auto& output =
      dut.get_output_port().Eval<geometry::FramePoseVector<double>>(
          *context);
  EXPECT_EQ(output.source_id(), source_id);
  EXPECT_EQ(output.size(), 1);
  ASSERT_TRUE(output.has_id(frame_id));
  EXPECT_TRUE(CompareMatrices(
      output.value(frame_id).matrix(),
      input.get_isometry().matrix()));
}

GTEST_TEST(PoseVectorToGeometryTest, DirectFeedthrough) {
  const PoseVectorToGeometry<double> dut({}, {});
  EXPECT_TRUE(dut.HasAnyDirectFeedthrough());
}

GTEST_TEST(PoseVectorToGeometryTest, ToAutoDiff) {
  const PoseVectorToGeometry<double> dut({}, {});
  EXPECT_TRUE(is_autodiffxd_convertible(dut, [&](const auto& converted) {
    EXPECT_EQ(1, converted.get_num_input_ports());
    EXPECT_EQ(1, converted.get_num_output_ports());
  }));
}

GTEST_TEST(PoseVectorToGeometryTest, ToSymbolic) {
  const PoseVectorToGeometry<double> dut({}, {});
  EXPECT_TRUE(is_symbolic_convertible(dut));
}

}  // namespace
}  // namespace rendering
}  // namespace systems
}  // namespace drake
