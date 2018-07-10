#include "drake/perception/depth_image_to_point_cloud.h"

#include <cmath>
#include <random>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/systems/sensors/camera_info.h"

namespace drake {
namespace perception {
namespace {
class DepthImageToPointCloudTest : public ::testing::Test {
 public:
  static systems::sensors::ImageDepth32F make_image(
      int width, int height, const Eigen::VectorXf& data) {
    systems::sensors::ImageDepth32F image(width, height);
    for (int v = 0; v < image.height(); ++v) {
      for (int u = 0; u < image.width(); ++u) {
        *image.at(u, v) = data(v * image.width() + u);
      }
    }
    return image;
  }

  static Eigen::VectorXf generate_bounded_random_sample(
      std::default_random_engine* generator, float min, float max, int length) {
    std::uniform_real_distribution<float> distribution(min, max);
    Eigen::VectorXf return_vector = Eigen::VectorXf::Zero(length);
    for (int i = 0; i < return_vector.size(); ++i) {
      return_vector(i) = distribution(*generator);
    }
    return return_vector;
  }

  static systems::sensors::ImageDepth32F convert_point_cloud_to_depth_image(
      const PointCloud& point_cloud,
      systems::sensors::CameraInfo* camera_info) {
    const int height = camera_info->height();
    const int width = camera_info->width();
    const float cx = camera_info->center_x();
    const float cy = camera_info->center_y();
    const float fx_inv = 1.f / camera_info->focal_x();
    const float fy_inv = 1.f / camera_info->focal_y();

    systems::sensors::ImageDepth32F image(width, height);
    for (int i = 0; i < point_cloud.size(); ++i) {
      const float x = point_cloud.xyz(i)(0);
      const float y = point_cloud.xyz(i)(1);
      const float z = point_cloud.xyz(i)(2);
      int u = round(cx + x / (fx_inv * z));
      int v = round(cy + y / (fy_inv * z));
      *image.at(u, v) = z;
    }

    return image;
  }

 protected:
  void SetUp() override {
    // Taken from RgbdCamera's depth camera info
    const int width = 640;
    const int height = 480;
    const double focal_x = 579.411;
    const double focal_y = 579.411;
    const double center_x = 320;
    const double center_y = 240;
    camera_info_ = std::make_unique<systems::sensors::CameraInfo>(
        width, height, focal_x, focal_y, center_x, center_y);
    converter_ = std::make_unique<DepthImageToPointCloud>(camera_info_.get());
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
  const double depth = 0.65;
  const int width = 640;
  const int height = 480;
  Eigen::VectorXf test_data = depth * Eigen::VectorXf::Ones(width * height);
  systems::sensors::ImageDepth32F image =
      DepthImageToPointCloudTest::make_image(width, height, test_data);

  context_->FixInputPort(
      0, systems::AbstractValue::Make<systems::sensors::ImageDepth32F>(image));

  converter_->get_output_port().Calc(*context_, output_.get());

  auto output_cloud = output_->GetValueOrThrow<perception::PointCloud>();
  Eigen::VectorXf output_depth = output_cloud.xyzs().row(2);

  EXPECT_TRUE(CompareMatrices(output_depth, test_data));
}

// Tests that the system computes the correct point cloud for a given depth
// image where pixels have a value from an increasing sequence.
TEST_F(DepthImageToPointCloudTest, ConvertIncreasingDepth) {
  const int width = 640;
  const int height = 480;
  const double depth_min = 0.51;
  const double depth_max = 1.0;
  Eigen::VectorXf test_data =
      Eigen::VectorXf::LinSpaced(width * height, depth_min, depth_max);
  systems::sensors::ImageDepth32F image =
      DepthImageToPointCloudTest::make_image(width, height, test_data);

  context_->FixInputPort(
      0, systems::AbstractValue::Make<systems::sensors::ImageDepth32F>(image));

  converter_->get_output_port().Calc(*context_, output_.get());

  auto output_cloud = output_->GetValueOrThrow<perception::PointCloud>();
  Eigen::VectorXf output_depth = output_cloud.xyzs().row(2);

  EXPECT_TRUE(CompareMatrices(output_depth, test_data));
}

// Tests that the system computes the correct point cloud for a given depth
// image where all pixels have random values drawn from a uniform distribution.
TEST_F(DepthImageToPointCloudTest, ConvertRandomDepthAndCheckProjection) {
  const float tolerance = 1e-7;
  const int width = 640;
  const int height = 480;
  const float depth_min = 0.51;
  const float depth_max = 1.0;

  std::default_random_engine generator(321);
  Eigen::VectorXf test_data =
      DepthImageToPointCloudTest::generate_bounded_random_sample(
          &generator, depth_min, depth_max, width * height);
  Eigen::VectorXf::LinSpaced(width * height, depth_min, depth_max);
  systems::sensors::ImageDepth32F image =
      DepthImageToPointCloudTest::make_image(width, height, test_data);

  context_->FixInputPort(
      0, systems::AbstractValue::Make<systems::sensors::ImageDepth32F>(image));

  converter_->get_output_port().Calc(*context_, output_.get());

  auto output_cloud = output_->GetValueOrThrow<perception::PointCloud>();

  systems::sensors::ImageDepth32F output_image =
      DepthImageToPointCloudTest::convert_point_cloud_to_depth_image(
          output_cloud, camera_info_.get());

  for (int v = 0; v < height; ++v) {
    for (int u = 0; u < width; ++u) {
      int i = v * output_image.width() + u;
      EXPECT_TRUE((fabs(*output_image.at(u, v) - test_data(i)) < tolerance));
    }
  }
}

}  // namespace
}  // namespace perception
}  // namespace drake
