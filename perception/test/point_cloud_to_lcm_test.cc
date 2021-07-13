#include "drake/perception/point_cloud_to_lcm.h"

#include <array>
#include <limits>

#include <gtest/gtest.h>

#include "drake/common/bit_cast.h"
#include "drake/common/test_utilities/limit_malloc.h"
#include "drake/lcmt_point_cloud.hpp"
#include "drake/perception/point_cloud.h"

using drake::Vector3;
using drake::internal::bit_cast;
using Eigen::Vector3f;

namespace drake {
namespace perception {
namespace {

constexpr float kInf = std::numeric_limits<float>::infinity();

std::vector<lcmt_point_cloud_field> MakeExpectedFields(int num_fields) {
  DRAKE_DEMAND((num_fields == 3) || (num_fields == 4) || (num_fields == 7));
  std::vector<lcmt_point_cloud_field> result;
  result.resize(num_fields);
  result.at(0).name = "x";
  result.at(0).byte_offset = 0;
  result.at(0).datatype = lcmt_point_cloud_field::FLOAT32;
  result.at(0).count = 1;
  result.at(1).name = "y";
  result.at(1).byte_offset = 4;
  result.at(1).datatype = lcmt_point_cloud_field::FLOAT32;
  result.at(1).count = 1;
  result.at(2).name = "z";
  result.at(2).byte_offset = 8;
  result.at(2).datatype = lcmt_point_cloud_field::FLOAT32;
  result.at(2).count = 1;
  if (num_fields > 3) {
    result.at(3).name = "rgb";
    result.at(3).byte_offset = 12;
    result.at(3).datatype = lcmt_point_cloud_field::UINT32;
    result.at(3).count = 1;
  }
  if (num_fields > 4) {
    result.at(4).name = "normal_x";
    result.at(4).byte_offset = 16;
    result.at(4).datatype = lcmt_point_cloud_field::FLOAT32;
    result.at(4).count = 1;
    result.at(5).name = "normal_y";
    result.at(5).byte_offset = 20;
    result.at(5).datatype = lcmt_point_cloud_field::FLOAT32;
    result.at(5).count = 1;
    result.at(6).name = "normal_z";
    result.at(6).byte_offset = 24;
    result.at(6).datatype = lcmt_point_cloud_field::FLOAT32;
    result.at(6).count = 1;
  }
  return result;
}

float PackRgb(uint8_t r, uint8_t g, uint8_t b) {
  return bit_cast<float>(std::array<uint8_t, 4>{ r, g, b, 0 });
}

class PointCloudToLcmTest : public testing::Test {
 protected:
  lcmt_point_cloud Convert(
      const PointCloud& cloud,
      const double time = 1.0,
      const std::string& frame_name = "world") {
    const PointCloudToLcm dut(frame_name);
    auto context = dut.CreateDefaultContext();
    context->SetTime(time);
    dut.get_input_port().FixValue(context.get(), Value<PointCloud>(cloud));
    const lcmt_point_cloud output =
        dut.get_output_port().Eval<lcmt_point_cloud>(*context);
    return output;
  }

  // Checks that all fields of a and b match exactly, except for the filler.
  // We check that any filler used is set to zero, but we don't check how much
  // filler a vs b each use.
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
    EXPECT_EQ(a.flags, b.flags);
    EXPECT_EQ(a.point_step, b.point_step);
    EXPECT_EQ(a.row_step, b.row_step);
    EXPECT_EQ(a.data_size, b.data_size);
    EXPECT_EQ(a.data, b.data);
    EXPECT_EQ(a.filler_size, a.filler.size());
    for (const auto& dummy : a.filler) {
      EXPECT_EQ(dummy, 0);
    }
    EXPECT_EQ(b.filler_size, b.filler.size());
    for (const auto& dummy : b.filler) {
      EXPECT_EQ(dummy, 0);
    }
  }
};

// Check an empty XYZ-only cloud.
TEST_F(PointCloudToLcmTest, Empty) {
  PointCloud cloud;

  lcmt_point_cloud expected{};
  expected.utime = 1'000'000;
  expected.frame_name = "world";
  expected.width = 0;
  expected.height = 1;
  constexpr int num_fields = 3;
  expected.num_fields = num_fields;
  expected.fields = MakeExpectedFields(num_fields);
  expected.point_step = num_fields * sizeof(float);
  expected.flags = lcmt_point_cloud::IS_STRICTLY_FINITE;

  lcmt_point_cloud actual = Convert(cloud);
  CheckEqual(actual, expected);
  EXPECT_EQ(actual.getEncodedSize() % 16, 0);
}

// Check a small, XYZ-only cloud with only two finite points.
TEST_F(PointCloudToLcmTest, XyzOnly) {
  PointCloud cloud(5);
  cloud.mutable_xyz(0) = Vector3f(1.0f, 2.0f, 3.0f);
  cloud.mutable_xyz(1) = Vector3f(NAN,  0.0f, 0.0f);
  cloud.mutable_xyz(2) = Vector3f(0.0f, kInf, 0.0f);
  cloud.mutable_xyz(3) = Vector3f(0.0f, 0.0f, kInf);
  cloud.mutable_xyz(4) = Vector3f(4.0f, 5.0f, 6.0f);
  constexpr int num_valid_points = 2;

  lcmt_point_cloud expected{};
  expected.utime = 1'000'000;
  expected.frame_name = "world";
  expected.width = num_valid_points;
  expected.height = 1;
  constexpr int num_fields = 3;
  expected.num_fields = num_fields;
  expected.fields = MakeExpectedFields(num_fields);
  constexpr int point_step = num_fields * sizeof(float);
  expected.point_step = point_step;
  expected.row_step = point_step * num_valid_points;
  expected.flags = lcmt_point_cloud::IS_STRICTLY_FINITE;
  constexpr int data_size = point_step * num_valid_points;
  expected.data_size = data_size;
  const float data[6] = {
    1.0f, 2.0f, 3.0f,
    4.0f, 5.0f, 6.0f,
  };
  static_assert(sizeof(data) == data_size);
  expected.data.resize(data_size);
  std::memcpy(expected.data.data(), data, data_size);

  lcmt_point_cloud actual = Convert(cloud);
  CheckEqual(actual, expected);
  EXPECT_EQ(actual.getEncodedSize() % 16, data_size % 16);
}

// Check a small, XYZ+RGB cloud with only two finite points.
TEST_F(PointCloudToLcmTest, XyzRgb) {
  PointCloud cloud(3, pc_flags::kXYZs | pc_flags::kRGBs);
  cloud.mutable_xyz(0) = Vector3f(1.0, 2.0, 3.0);
  cloud.mutable_xyz(1) = Vector3f(NAN, 0.0, 0.0);
  cloud.mutable_xyz(2) = Vector3f(4.0, 5.0, 6.0);
  cloud.mutable_rgb(0) = Vector3<uint8_t>(0, 127, 255);
  cloud.mutable_rgb(1) = Vector3<uint8_t>(0, 0, 0);
  cloud.mutable_rgb(2) = Vector3<uint8_t>(1, 128, 255);
  constexpr int num_valid_points = 2;

  lcmt_point_cloud expected{};
  expected.utime = 1'000'000;
  expected.frame_name = "world";
  expected.width = num_valid_points;
  expected.height = 1;
  constexpr int num_fields = 4;
  expected.num_fields = num_fields;
  expected.fields = MakeExpectedFields(num_fields);
  constexpr int point_step = num_fields * sizeof(float);
  expected.point_step = point_step;
  expected.row_step = point_step * num_valid_points;
  expected.flags = lcmt_point_cloud::IS_STRICTLY_FINITE;
  constexpr int data_size = point_step * num_valid_points;
  expected.data_size = data_size;
  const float data[8] = {
    1.0f, 2.0f, 3.0f, PackRgb(0, 127, 255),
    4.0f, 5.0f, 6.0f, PackRgb(1, 128, 255),
  };
  static_assert(sizeof(data) == data_size);
  expected.data.resize(data_size);
  std::memcpy(expected.data.data(), data, data_size);

  lcmt_point_cloud actual = Convert(cloud);
  CheckEqual(actual, expected);
  EXPECT_EQ(actual.getEncodedSize() % 16, data_size % 16);
}

// Check a small, XYZ+RGB+Normal cloud.
TEST_F(PointCloudToLcmTest, XyzRgbNormal) {
  PointCloud cloud(2, pc_flags::kXYZs | pc_flags::kRGBs | pc_flags::kNormals);
  cloud.mutable_xyz(0) = Vector3f(1.0, 2.0, 3.0);
  cloud.mutable_xyz(1) = Vector3f(4.0, 5.0, 6.0);
  cloud.mutable_rgb(0) = Vector3<uint8_t>(0, 127, 255);
  cloud.mutable_rgb(1) = Vector3<uint8_t>(1, 128, 255);
  cloud.mutable_normal(0) = Vector3f(1.0, 0.0, 0.0);
  cloud.mutable_normal(1) = Vector3f(0.0, -1.0, 0.0);
  constexpr int num_valid_points = 2;

  lcmt_point_cloud expected{};
  expected.utime = 1'500'000;
  expected.frame_name = "frame";
  expected.width = num_valid_points;
  expected.height = 1;
  constexpr int num_fields = 7;
  expected.num_fields = num_fields;
  expected.fields = MakeExpectedFields(num_fields);
  constexpr int point_step = num_fields * 4;
  expected.point_step = point_step;
  expected.row_step = point_step * num_valid_points;
  expected.flags = lcmt_point_cloud::IS_STRICTLY_FINITE;
  constexpr int data_size = point_step * num_valid_points;
  expected.data_size = data_size;
  const float data[14] = {
    1.0f, 2.0f, 3.0f, PackRgb(0, 127, 255), 1.0, 0.0, 0.0,
    4.0f, 5.0f, 6.0f, PackRgb(1, 128, 255), 0.0, -1.0, 0.0,
  };
  static_assert(sizeof(data) == data_size);
  expected.data.resize(data_size);
  std::memcpy(expected.data.data(), data, data_size);

  lcmt_point_cloud actual = Convert(cloud, 1.5, "frame");
  CheckEqual(actual, expected);
  EXPECT_EQ(actual.getEncodedSize() % 16, data_size % 16);
}

TEST_F(PointCloudToLcmTest, StorageReuse) {
  PointCloud cloud(2, pc_flags::kXYZs | pc_flags::kRGBs | pc_flags::kNormals);
  cloud.mutable_xyz(0) = Vector3f(1.0, 2.0, 3.0);
  cloud.mutable_xyz(1) = Vector3f(4.0, 5.0, 6.0);
  cloud.mutable_rgb(0) = Vector3<uint8_t>(0, 127, 255);
  cloud.mutable_rgb(1) = Vector3<uint8_t>(1, 128, 255);
  cloud.mutable_normal(0) = Vector3f(1.0, 0.0, 0.0);
  cloud.mutable_normal(1) = Vector3f(0.0, -1.0, 0.0);

  // Convert one cloud; this ends up resizing storage to be large enough.
  const PointCloudToLcm dut;
  auto context = dut.CreateDefaultContext();
  context->SetTime(1.0);
  auto& fixed_value = dut.get_input_port().FixValue(
      context.get(), Value<PointCloud>(cloud));
  dut.get_output_port().Eval<lcmt_point_cloud>(*context);

  // Invalidate the output message and then recompute it, without making any
  // additional allocations.
  fixed_value.GetMutableData();
  {
    test::LimitMalloc guard;
    dut.get_output_port().Eval<lcmt_point_cloud>(*context);
  }
}


}  // namespace
}  // namespace perception
}  // namespace drake
