#include "drake/perception/point_cloud_to_lcm.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/lcmt_point_cloud.hpp"
#include "drake/perception/point_cloud.h"

using drake::Vector3;
using Eigen::Vector3f;

namespace drake {
namespace perception {
namespace {

class PointCloudToLcmTest : public testing::Test {
 protected:
  lcmt_point_cloud Convert(
      const PointCloud& cloud,
      const double time = 1.0,
      const std::string& frame_name = "world") {
    PointCloudToLcm dut(frame_name);
    auto context = dut.CreateDefaultContext();
    context->SetTime(time);
    dut.get_input_port().FixValue(context.get(), Value<PointCloud>(cloud));
    const lcmt_point_cloud output =
        dut.get_output_port().Eval<lcmt_point_cloud>(*context);
    return output;
  }

  void CheckEqual(
      const lcmt_point_cloud& a,
      const lcmt_point_cloud& b) {
    EXPECT_EQ(a.utime, b.utime);
    EXPECT_EQ(a.frame_name, b.frame_name);
    EXPECT_EQ(a.width, b.width);
    EXPECT_EQ(a.height, b.height);
    ASSERT_EQ(a.fields.size(), a.num_fields);
    ASSERT_EQ(b.fields.size(), b.num_fields);
    ASSERT_EQ(a.num_fields, b.num_fields);
    for (int i = 0; i < a.num_fields; ++i) {
      EXPECT_EQ(a.fields[i].name, b.fields[i].name);
      EXPECT_EQ(a.fields[i].byte_offset, b.fields[i].byte_offset);
      EXPECT_EQ(a.fields[i].datatype, b.fields[i].datatype);
      EXPECT_EQ(a.fields[i].count, b.fields[i].count);
    }
    EXPECT_EQ(a.is_bigendian, b.is_bigendian);
    EXPECT_EQ(a.point_step, b.point_step);
    EXPECT_EQ(a.row_step, b.row_step);
    EXPECT_EQ(a.is_dense, b.is_dense);
    EXPECT_EQ(a.filler_size, b.filler_size);
    EXPECT_EQ(a.filler, b.filler);
    EXPECT_EQ(a.data_size, b.data_size);
    EXPECT_EQ(a.data, b.data);
  }
};

// Check an empty cloud.
TEST_F(PointCloudToLcmTest, Empty) {
  PointCloud cloud;
  lcmt_point_cloud expected{};
  expected.utime = 1'000'000;
  expected.frame_name = "world";
  expected.width = 0;
  expected.height = 1;
  expected.num_fields = 3;
  expected.fields.resize(3);
  expected.fields.at(0).name = "x";
  expected.fields.at(0).byte_offset = 0;
  expected.fields.at(0).datatype = lcmt_point_cloud_field::FLOAT32;
  expected.fields.at(0).count = 1;
  expected.fields.at(1).name = "y";
  expected.fields.at(1).byte_offset = 4;
  expected.fields.at(1).datatype = lcmt_point_cloud_field::FLOAT32;
  expected.fields.at(1).count = 1;
  expected.fields.at(2).name = "z";
  expected.fields.at(2).byte_offset = 8;
  expected.fields.at(2).datatype = lcmt_point_cloud_field::FLOAT32;
  expected.fields.at(2).count = 1;
  expected.point_step = 12;
  expected.is_dense = true;
  expected.filler_size = 14;
  expected.filler.resize(14, 0);

  lcmt_point_cloud actual = Convert(cloud);
  CheckEqual(actual, expected);
  EXPECT_EQ(actual.getEncodedSize() % 16, 0);
}

// Convert a small, XYZ-only cloud with a few non-finite points.
TEST_F(PointCloudToLcmTest, XyzOnly) {
  constexpr float kInf = std::numeric_limits<float>::infinity();

  // Create the input.
  PointCloud cloud(5);
  cloud.mutable_xyz(0) = Vector3f(1.0, 2.0, 3.0);
  cloud.mutable_xyz(1) = Vector3f(NAN, 0.0, 0.0);
  cloud.mutable_xyz(2) = Vector3f(0.0, kInf, 0.0);
  cloud.mutable_xyz(3) = Vector3f(0.0, 0.0, kInf);
  cloud.mutable_xyz(4) = Vector3f(4.0, 5.0, 6.0);

  const auto output = Convert(cloud);

  // Confirm all message fields.
  const int num_fields = 3;
  const int num_points = 2;
  const int num_bytes = 4 * num_fields * num_points;
  EXPECT_EQ(output.utime, 1'000'000);
  EXPECT_EQ(output.frame_name, "world");
  EXPECT_EQ(output.is_bigendian, false);
  EXPECT_EQ(output.is_dense, true);
  ASSERT_EQ(output.num_fields, num_fields);
  EXPECT_EQ(output.fields[0].name, "x");
  EXPECT_EQ(output.fields[0].datatype, lcmt_point_cloud_field::FLOAT32);
  EXPECT_EQ(output.fields[0].byte_offset, 0);
  EXPECT_EQ(output.fields[0].count, 1);
  EXPECT_EQ(output.fields[1].name, "y");
  EXPECT_EQ(output.fields[1].datatype, lcmt_point_cloud_field::FLOAT32);
  EXPECT_EQ(output.fields[1].byte_offset, 4);
  EXPECT_EQ(output.fields[1].count, 1);
  EXPECT_EQ(output.fields[2].name, "z");
  EXPECT_EQ(output.fields[2].datatype, lcmt_point_cloud_field::FLOAT32);
  EXPECT_EQ(output.fields[2].byte_offset, 8);
  EXPECT_EQ(output.fields[2].count, 1);
  EXPECT_EQ(output.width, num_points);
  EXPECT_EQ(output.height, 1);
  EXPECT_EQ(output.point_step, 4 * num_fields);
  EXPECT_EQ(output.row_step, num_bytes);
  ASSERT_EQ(output.data_size, num_bytes);
  std::vector<float> float_data(num_bytes / 4);
  std::memcpy(float_data.data(), output.data.data(), num_bytes);
  EXPECT_EQ(float_data[0], 1.0);
  EXPECT_EQ(float_data[1], 2.0);
  EXPECT_EQ(float_data[2], 3.0);
  EXPECT_EQ(float_data[3], 4.0);
  EXPECT_EQ(float_data[4], 5.0);
  EXPECT_EQ(float_data[5], 6.0);
}

// Convert a small, XYZ+RGB cloud with one non-finite point.
TEST_F(PointCloudToLcmTest, XyzRgb) {
  // Create the input.
  PointCloud cloud(3, pc_flags::kXYZs | pc_flags::kRGBs);
  cloud.mutable_xyz(0) = Vector3f(1.0, 2.0, 3.0);
  cloud.mutable_xyz(1) = Vector3f(NAN, 0.0, 0.0);
  cloud.mutable_xyz(2) = Vector3f(4.0, 5.0, 6.0);
  cloud.mutable_rgb(0) = Vector3<uint8_t>(0, 127, 255);
  cloud.mutable_rgb(1) = Vector3<uint8_t>(0, 0, 0);
  cloud.mutable_rgb(2) = Vector3<uint8_t>(1, 128, 255);
  // const int finite_count = 2;

  // Feed through the device under test.
  PointCloudToLcm dut("world");
  auto context = dut.CreateDefaultContext();
  context->SetTime(1.0);
  dut.get_input_port().FixValue(context.get(), Value<PointCloud>(cloud));
  const auto& output = dut.get_output_port().Eval<lcmt_point_cloud>(*context);

  // Confirm all message fields.
#if 0
  EXPECT_EQ(output.header.seq, 0);
  EXPECT_EQ(output.header.utime, 1'000'000);
  EXPECT_EQ(output.header.frame_name, "world");
  ASSERT_EQ(output.num_points, finite_count);
  ASSERT_EQ(output.points.size(), 3);
  ASSERT_EQ(output.points[0].size(), finite_count);
  ASSERT_EQ(output.points[1].size(), finite_count);
  ASSERT_EQ(output.points[2].size(), finite_count);
  EXPECT_EQ(output.points[0][0], 1.0);
  EXPECT_EQ(output.points[1][0], 2.0);
  EXPECT_EQ(output.points[2][0], 3.0);
  EXPECT_EQ(output.points[0][1], 4.0);
  EXPECT_EQ(output.points[1][1], 5.0);
  EXPECT_EQ(output.points[2][1], 6.0);
  ASSERT_EQ(output.num_channels, 3);
  ASSERT_EQ(output.channel_names.size(), 3);
  ASSERT_EQ(output.channels.size(), 3);
  ASSERT_EQ(output.channels[0].size(), finite_count);
  ASSERT_EQ(output.channels[1].size(), finite_count);
  ASSERT_EQ(output.channels[2].size(), finite_count);
  EXPECT_EQ(output.channel_names[0], "r");
  EXPECT_EQ(output.channel_names[1], "g");
  EXPECT_EQ(output.channel_names[2], "b");
  EXPECT_EQ(output.channels[0][0] * 255.0f, 0);
  EXPECT_EQ(output.channels[1][0] * 255.0f, 127);
  EXPECT_EQ(output.channels[2][0] * 255.0f, 255);
  EXPECT_EQ(output.channels[0][1] * 255.0f, 1);
  EXPECT_EQ(output.channels[1][1] * 255.0f, 128);
  EXPECT_EQ(output.channels[2][1] * 255.0f, 255);
#else
  unused(output);
#endif
}

}  // namespace
}  // namespace perception
}  // namespace drake
