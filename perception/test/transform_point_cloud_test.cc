#include "drake/perception/transform_point_cloud.h"

#include <cmath>
#include <random>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace perception {
namespace {

class TransformPointCloudTest : public ::testing::Test {
 protected:
  void SetUp() override {
    transformer_ = std::make_unique<TransformPointCloud>();
    context_ = transformer_->CreateDefaultContext();
    output_ = transformer_->point_cloud_output_port().Allocate();
    point_cloud_input_ =
        systems::AbstractValue::Make<PointCloud>(PointCloud(0));
    rigid_transform_input_ = 
        systems::AbstractValue::Make<RigidTransform>(RigidTransform());
  }

  std::unique_ptr<TransformPointCloud> transformer_;
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::AbstractValue> output_;
  std::unique_ptr<systems::AbstractValue> point_cloud_input_;
  std::unique_ptr<systems::AbstractValue> rigid_transform_input_;
};

// Verifies that the system applies the transform correctly to the point cloud.
TEST_F(TransformPointCloudTest, ApplyTransformTest) {
  Eigen::VectorXf test_data(10);
  for (int i = 0; i < output_depth.size(); ++i) {

  } 
 
 
  = kDepth * Eigen::VectorXf::Ones(kWidth * kHeight);
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
