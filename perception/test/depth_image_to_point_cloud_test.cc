#include "drake/perception/depth_image_to_point_cloud.h"

#include <cmath>
#include <random>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
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
  const float kDistanceTolerance = 1e-8;
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
}

}  // namespace
}  // namespace perception
}  // namespace drake
