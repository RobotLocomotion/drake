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
    output_ = converter_->get_output_port().Allocate();
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

// Tests that the system computes the correct point cloud for a given depth
// image where all pixels have a constant value.
TEST_F(DepthImageToPointCloudTest, ConvertConstantDepth) {
  const double kDepth = 0.65;
  Eigen::VectorXf test_data = kDepth * Eigen::VectorXf::Ones(kWidth * kHeight);
  systems::sensors::ImageDepth32F image =
      DepthImageToPointCloudTest::MakeImage(kWidth, kHeight, test_data);

  context_->FixInputPort(
      0, systems::AbstractValue::Make<systems::sensors::ImageDepth32F>(image));

  converter_->get_output_port().Calc(*context_, output_.get());

  auto output_cloud = output_->GetValueOrThrow<perception::PointCloud>();
  Eigen::VectorXf output_depth = output_cloud.xyzs().row(2);

  EXPECT_TRUE(CompareMatrices(output_depth, test_data));
}

}  // namespace
}  // namespace perception
}  // namespace drake
