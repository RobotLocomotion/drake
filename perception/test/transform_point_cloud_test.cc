#include "drake/perception/transform_point_cloud.h"

#include <cmath>
#include <random>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace perception {
namespace {

class TransformPointCloudTest : public ::testing::Test {
 public:
  static Matrix3X<float> GenerateBoundedRandomSample(
      std::default_random_engine* generator, float min, float max,
      int num_cols) {
    Matrix3X<float> return_matrix = Matrix3X<float>::Zero(3, num_cols);
    std::uniform_real_distribution<float> distribution(min, max);
    for (int i = 0; i < num_cols; ++i) {
      return_matrix.col(i) =
          Vector3<float>(distribution(*generator), distribution(*generator),
                         distribution(*generator));
    }
    return return_matrix;
  }

 protected:
  void SetUp() override {
    transformer_ = std::make_unique<TransformPointCloud>();
    context_ = transformer_->CreateDefaultContext();
    output_ = transformer_->point_cloud_output_port().Allocate();
    point_cloud_input_ =
        systems::AbstractValue::Make<PointCloud>(PointCloud(0));
    rigid_transform_input_ =
        systems::AbstractValue::Make<math::RigidTransform<float>>(
            math::RigidTransform<float>());
  }

  std::unique_ptr<TransformPointCloud> transformer_;
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::AbstractValue> output_;
  std::unique_ptr<systems::AbstractValue> point_cloud_input_;
  std::unique_ptr<systems::AbstractValue> rigid_transform_input_;
};

// Verifies that the system applies the transform correctly to the point cloud.
TEST_F(TransformPointCloudTest, ApplyTransformTest) {
  const float kMin = -10.;
  const float kMax = 10.;
  const int kNumPoints = 5;
  const Vector3<float> kRpy(M_PI_4, -M_PI_2, 0.543);
  const math::RollPitchYaw<float> kRollPitchYaw(kRpy);
  const math::RotationMatrix<float> kR(kRollPitchYaw);
  const Vector3<float> kP(-.3, 5.4, -2.7);

  std::default_random_engine generator(321);

  MatrixX<float> test_data =
      TransformPointCloudTest::GenerateBoundedRandomSample(&generator, kMin,
                                                           kMax, kNumPoints);

  PointCloud cloud(kNumPoints);
  cloud.mutable_xyzs() = test_data;

  math::RigidTransform<float> transform(kR, kP);

  context_->FixInputPort(0, systems::AbstractValue::Make<PointCloud>(cloud));
  context_->FixInputPort(
      1, systems::AbstractValue::Make<math::RigidTransform<float>>(transform));

  transformer_->point_cloud_output_port().Calc(*context_, output_.get());

  auto output_cloud = output_->GetValueOrThrow<PointCloud>();

  Matrix4X<float> test_data_homogeneous(4, kNumPoints);
  test_data_homogeneous.block(0, 0, 3, kNumPoints) = test_data;
  test_data_homogeneous.row(3) = VectorX<float>::Ones(kNumPoints);
  Matrix4X<float> expected_output =
      transform.GetAsMatrix4() * test_data_homogeneous;

  EXPECT_TRUE(CompareMatrices(
      output_cloud.xyzs(), expected_output.block(0, 0, 3, kNumPoints), 1e-6));
}

}  // namespace
}  // namespace perception
}  // namespace drake
