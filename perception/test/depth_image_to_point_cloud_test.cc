#include "drake/perception/depth_image_to_point_cloud.h"

#include <cmath>
#include <random>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/rigid_transform.h"
#include "drake/systems/sensors/camera_info.h"

namespace drake {
namespace perception {
namespace {

const int kWidth = 640;
const int kHeight = 480;

class DepthImageToPointCloudTest : public ::testing::Test {
 public:
  static systems::sensors::ImageDepth32F MakeImage(
      int width, int height, const Eigen::VectorXf& data) {
    systems::sensors::ImageDepth32F image(width, height);
    for (int v = 0; v < image.height(); ++v) {
      for (int u = 0; u < image.width(); ++u) {
        *image.at(u, v) = data(v * image.width() + u);
      }
    }
    return image;
  }

 protected:
  void SetUp() override {
    // Taken from RgbdCamera's depth camera info
    const double kFocalX = 579.411;
    const double kFocalY = 579.411;
    const double kCenterX = 320;
    const double kCenterY = 240;
    camera_info_ = std::make_unique<systems::sensors::CameraInfo>(
        kWidth, kHeight, kFocalX, kFocalY, kCenterX, kCenterY);
    converter_ = std::make_unique<DepthImageToPointCloud>(*camera_info_.get());
    context_ = converter_->CreateDefaultContext();
    output_ = converter_->point_cloud_output_port().Allocate();
    systems::sensors::ImageDepth32F image;
    input_ =
        systems::AbstractValue::Make<systems::sensors::ImageDepth32F>(image);
  }

  std::unique_ptr<DepthImageToPointCloud> converter_;
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::AbstractValue> output_;
  std::unique_ptr<systems::AbstractValue> input_;
  std::unique_ptr<systems::sensors::CameraInfo> camera_info_;
};

// Verifies that the system computes the correct point cloud for a given depth
// image.
TEST_F(DepthImageToPointCloudTest, ConversionAndNanValueTest) {
  const float kDistanceTolerance = 1e-6;
  const float kDepth = 0.65;
  const int kTooClosePointCloudIndex = 2;
  const int kTooFarPointCloudIndex = 658;

  Eigen::VectorXf test_data = kDepth * Eigen::VectorXf::Ones(kWidth * kHeight);
  test_data(kTooClosePointCloudIndex) =
      systems::sensors::InvalidDepth::kTooClose;
  test_data(kTooFarPointCloudIndex) = systems::sensors::InvalidDepth::kTooFar;

  systems::sensors::ImageDepth32F image =
      DepthImageToPointCloudTest::MakeImage(kWidth, kHeight, test_data);

  context_->FixInputPort(
      0, systems::AbstractValue::Make<systems::sensors::ImageDepth32F>(image));

  converter_->point_cloud_output_port().Calc(*context_, output_.get());

  auto output_cloud = output_->GetValueOrThrow<perception::PointCloud>();
  Eigen::VectorXf output_depth = output_cloud.xyzs().row(2);

  // kTooClose is treated as kTooFar. For the detail, refer to the document of
  // RgbdCamera::ConvertDepthImageToPointCloud.
  for (int i = 0; i < output_depth.size(); ++i) {
    if (i == kTooClosePointCloudIndex) {
      ASSERT_EQ(output_depth(i), systems::sensors::InvalidDepth::kTooFar);
    } else if (i == kTooFarPointCloudIndex) {
      ASSERT_EQ(output_depth(i), systems::sensors::InvalidDepth::kTooFar);
    } else {
      ASSERT_NEAR(output_depth(i), test_data(i), kDistanceTolerance);
    }
  }

  // Now apply a camera_pose input.
  Eigen::Vector3d offset(1.3, 4.5, -4);
  math::RigidTransformd pose(offset);
  context_->FixInputPort(
      converter_->camera_pose_input_port().get_index(),
      systems::AbstractValue::Make<math::RigidTransformd>(pose));

  converter_->point_cloud_output_port().Calc(*context_, output_.get());
  output_cloud = output_->GetValueOrThrow<perception::PointCloud>();
  output_depth = output_cloud.xyzs().row(2);

  for (int i = 0; i < output_cloud.size(); ++i) {
    if (i == kTooClosePointCloudIndex) {
      ASSERT_EQ(output_depth(i), systems::sensors::InvalidDepth::kTooFar);
    } else if (i == kTooFarPointCloudIndex) {
      ASSERT_EQ(output_depth(i), systems::sensors::InvalidDepth::kTooFar);
    } else {
      ASSERT_NEAR(output_depth(i), test_data(i) + offset(2),
                  kDistanceTolerance);
    }
  }
}

// Note that we use ASSERTs instead of EXPECTs when we need to traverse many
// pixels in an image or many points in a point cloud. This is to prevent
// outputting massive amount of error messages and making debug hard when
// test failed.

class DepthImageToPointCloudConversionTest : public ::testing::Test {
 public:
  static constexpr float kFocal = 500.f;
  static constexpr int kDepthWidth = 60;
  static constexpr int kDepthHeight = 40;

  DepthImageToPointCloudConversionTest()
      : camera_info_(kDepthWidth, kDepthHeight, kFocal, kFocal,
                     kDepthWidth * 0.5, kDepthHeight * 0.5),
        depth_image_(kDepthWidth, kDepthHeight, 1) {}

  // kTooClose is treated as kTooFar. For the detail, refer to the document of
  // RgbdCamera::ConvertDepthImageToPointCloud.
  void VerifyTooFarTooClose() {
    for (int v = 0; v < depth_image_.height(); ++v) {
      for (int u = 0; u < depth_image_.width(); ++u) {
        const int i = v * depth_image_.width() + u;
        Eigen::Vector3f actual = actual_point_cloud_.col(i);
        ASSERT_EQ(actual(0), systems::sensors::InvalidDepth::kTooFar);
        ASSERT_EQ(actual(1), systems::sensors::InvalidDepth::kTooFar);
        ASSERT_EQ(actual(2), systems::sensors::InvalidDepth::kTooFar);
      }
    }
  }

 protected:
  // This must be called by all the test cases first.
  void Init(float depth_value) {
    std::fill(depth_image_.at(0, 0),
              depth_image_.at(0, 0) + depth_image_.size(), depth_value);
  }

  const systems::sensors::CameraInfo camera_info_;
  systems::sensors::ImageDepth32F depth_image_;
  Eigen::Matrix3Xf actual_point_cloud_;
};

// Verifies computed point cloud when pixel values in depth image are valid.
TEST_F(DepthImageToPointCloudConversionTest, ValidValueTest) {
  constexpr float kDepthValue = 1.f;
  Init(kDepthValue);

  DepthImageToPointCloud::Convert(depth_image_, camera_info_,
                                  &actual_point_cloud_);

  // This tolerance was determined empirically using Drake's supported
  // platforms.
  constexpr float kDistanceTolerance = 1e-8;
  for (int v = 0; v < depth_image_.height(); ++v) {
    for (int u = 0; u < depth_image_.width(); ++u) {
      const int i = v * depth_image_.width() + u;
      Eigen::Vector3f actual = actual_point_cloud_.col(i);

      const double expected_x = kDepthValue * (u - kDepthWidth * 0.5) / kFocal;
      ASSERT_NEAR(actual(0), expected_x, kDistanceTolerance);
      const double expected_y = kDepthValue * (v - kDepthHeight * 0.5) / kFocal;
      ASSERT_NEAR(actual(1), expected_y, kDistanceTolerance);
      ASSERT_NEAR(actual(2), kDepthValue, kDistanceTolerance);
    }
  }
}

// Verifies computed point cloud when pixel values in depth image are NaN.
TEST_F(DepthImageToPointCloudConversionTest, NanValueTest) {
  Init(std::numeric_limits<float>::quiet_NaN());

  DepthImageToPointCloud::Convert(depth_image_, camera_info_,
                                  &actual_point_cloud_);

  for (int v = 0; v < depth_image_.height(); ++v) {
    for (int u = 0; u < depth_image_.width(); ++u) {
      const int i = v * depth_image_.width() + u;
      Eigen::Vector3f actual = actual_point_cloud_.col(i);
      ASSERT_TRUE(std::isnan(actual(0)));
      ASSERT_TRUE(std::isnan(actual(1)));
      ASSERT_TRUE(std::isnan(actual(2)));
    }
  }
}

// Verifies computed point cloud when pixel values in depth image are kTooFar.
TEST_F(DepthImageToPointCloudConversionTest, TooFarTest) {
  Init(systems::sensors::InvalidDepth::kTooFar);

  DepthImageToPointCloud::Convert(depth_image_, camera_info_,
                                  &actual_point_cloud_);

  VerifyTooFarTooClose();
}

// Verifies computed point cloud when pixel values in depth image are kTooClose.
TEST_F(DepthImageToPointCloudConversionTest, TooCloseTest) {
  Init(systems::sensors::InvalidDepth::kTooClose);

  DepthImageToPointCloud::Convert(depth_image_, camera_info_,
                                  &actual_point_cloud_);

  VerifyTooFarTooClose();
}

}  // namespace
}  // namespace perception
}  // namespace drake
